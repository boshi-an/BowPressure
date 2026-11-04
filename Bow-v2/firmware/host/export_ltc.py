#!/usr/bin/env python3
"""Export compact CSV on a 25fps LTC timeline using bracketing edge anchors."""
import argparse
from bisect import bisect_right
from collections import defaultdict
import csv
import json
import os
import sqlite3
import tempfile
from pathlib import Path

DAY = 86400000000
FIELDS = ['record_type', 'epoch', 'ltc_segment', 'ltc_us', 'ltc_time', 'sync',
          'sample_mcu_us', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
          'H_mV', 'F_mV', 'detail']


def anchors_from(path):
    groups = defaultdict(list)
    with Path(path).open(newline='', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if row.get('record_type') != 'LTC_ANCHOR' or not row.get('ltc_anchor_mcu_us'):
                continue
            ep = int(row['epoch'])
            x, y = int(row['ltc_anchor_mcu_us']), int(row['ltc_anchor_day_us'])
            group = groups[ep]
            segment = 0
            if group:
                px, py, segment = group[-1]
                delta = (y-py) % DAY
                if 0 < x-px < DAY/2 and delta > 0 and .95 <= delta/(x-px) <= 1.05:
                    y = py+delta  # Midnight unwrapping, only with continuous codes.
                else:
                    segment += 1
            group.append((x, y, segment))
    return groups


def time_text(us):
    days, rem = divmod(us, DAY)
    seconds, micro = divmod(rem, 1000000)
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f'{days}+{h:02}:{m:02}:{s:02}.{micro:06}'


class Timeline:
    """Single mapping implementation shared by recording and offline export."""
    def __init__(self, anchor_file):
        self.groups = anchors_from(anchor_file)
        self.indices = {ep: [a[0] for a in group] for ep, group in self.groups.items()}

    def map(self, row):
        kind = row['record_type']
        out = dict(sync='unlocked', ltc_us='', ltc_time='', ltc_segment='')
        t = row.get('sample_mcu_us') if kind in ('IMU', 'ADC') else row.get('mcu_read_us_unwrapped')
        if kind == 'LTC_ANCHOR':
            t = row.get('ltc_anchor_mcu_us')
        if t not in (None, '') and row.get('epoch') not in (None, ''):
            t, ep = float(t), int(row['epoch'])
            out['sample_mcu_us'] = t
            group = self.groups.get(ep, [])
            i = bisect_right(self.indices.get(ep, []), t)-1
            value = None
            quality = 'interpolated'
            if i >= 0:
                a = group[i]
                if t == a[0]:
                    value = a[1]
                elif i+1 < len(group):
                    b = group[i+1]
                    if a[2] == b[2] and b[0] > a[0]:
                        value = round(a[1]+(t-a[0])*(b[1]-a[1])/(b[0]-a[0]))
                        if b[1]-a[1] != 40000:
                            quality = 'interpolated_gap'
            if value is None and group:
                # No bracket: use the closest anchor, and a neighbour in its
                # own continuous segment. Never fit across a timecode jump.
                candidates = [j for j in (i, i+1) if 0 <= j < len(group)]
                j = min(candidates, key=lambda k: abs(group[k][0]-t))
                a = group[j]
                neighbours = [k for k in (j-1, j+1) if 0 <= k < len(group)
                              and group[k][2] == a[2] and group[k][0] != a[0]]
                slope = 1.0
                quality = 'extrapolated_nominal'
                if neighbours:
                    k = min(neighbours, key=lambda k: abs(group[k][0]-t))
                    b = group[k]
                    slope = (b[1]-a[1])/(b[0]-a[0])
                    quality = 'extrapolated'
                value = round(a[1]+(t-a[0])*slope)
            if value is not None:
                out.update(ltc_segment=a[2], ltc_us=value, ltc_time=time_text(value), sync=quality)
        return out


def export(source, target):
    timeline = Timeline(source)
    synced = total = 0
    with Path(source).open(newline='', encoding='utf-8-sig') as src, \
            Path(target).open('x', newline='', encoding='utf-8') as dst:
        reader = csv.DictReader(src)
        writer = csv.DictWriter(dst, fieldnames=FIELDS)
        writer.writeheader()
        for row in reader:
            kind = row['record_type']
            out = dict(record_type=kind, epoch=row.get('epoch', ''))
            out.update(timeline.map(row))
            synced += out['ltc_us'] != ''
            if kind == 'IMU':
                out.update({k: row.get(k, '') for k in FIELDS[7:13]})
            elif kind == 'ADC':
                if row.get('channel') in ('H', 'F'):
                    out[row['channel']+'_mV'] = row.get('diff_mV', '')
            else:
                keys = ('timecode', 'locked', 'faults', 'alive', 'tx_drops', 'missed_slots',
                        'max_late_us', 'imu_status_hex', 'fifo_count', 'peak_count', 'message')
                out['detail'] = json.dumps({k: row[k] for k in keys if row.get(k)}, ensure_ascii=False)
            writer.writerow(out)
            total += 1
    return total, synced


def merge_directory(directory):
    """Rebuild merged.csv atomically; never modify any of the four inputs.

    Disk-backed sorting keeps memory bounded for long recordings. Rows remain
    separate events; no resampling/forward filling. Logs without MCU time sort last.
    """
    directory = Path(directory)
    sources = [directory/(name+'.csv') for name in ('adc', 'imu', 'ltc', 'mcu')]
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)
    timeline = Timeline(directory/'ltc.csv')
    fields = ['record_type', 'epoch', 'seq', 'ltc_us', 'ltc_time', 'ltc_segment', 'sync', 'sample_mcu_us']
    total = synced = 0
    with tempfile.TemporaryDirectory(prefix='.merge-', dir=directory) as work:
        db = sqlite3.connect(str(Path(work)/'sort.sqlite'))
        try:
            db.execute('CREATE TABLE rows (epoch INTEGER, t REAL, n INTEGER, data TEXT)')
            for source in sources:
                with source.open(newline='', encoding='utf-8-sig') as handle:
                    reader = csv.DictReader(handle)
                    fields.extend(k for k in (reader.fieldnames or []) if k not in fields)
                    for row in reader:
                        row.update(timeline.map(row))
                        total += 1
                        synced += row['ltc_us'] != ''
                        ep = int(row['epoch']) if row.get('epoch') else None
                        t = float(row['sample_mcu_us']) if row.get('sample_mcu_us') not in (None, '') else None
                        db.execute('INSERT INTO rows VALUES (?,?,?,?)', (ep, t, total, json.dumps(row)))
            db.commit()
            staged = Path(work)/'merged.csv'
            with staged.open('w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for (data,) in db.execute('SELECT data FROM rows ORDER BY epoch IS NULL, epoch, t IS NULL, t, n'):
                    writer.writerow(json.loads(data))
            os.replace(staged, directory/'merged.csv')
        finally:
            db.close()
    return total, synced


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path, help='Run directory, or a legacy shared CSV')
    p.add_argument('-o', '--output', type=Path, help='Only for legacy shared CSV input')
    a = p.parse_args()
    if a.source.is_dir():
        if a.output:
            p.error('目录模式固定生成同目录下的 merged.csv，无需 --output')
        n, synced = merge_directory(a.source)
        target = a.source/'merged.csv'
    else:
        if not a.output:
            p.error('旧版单CSV输入需要 --output；新录制请传入目录')
        n, synced = export(a.source, a.output)
        target = a.output
    print(f'已导出 {n} 行，LTC已映射 {synced} 行；未同步区间留空：{target.resolve()}')
