#!/usr/bin/env python3
"""Plot a shared recording CSV as ACC, GYRO, H, F; no filtering/resampling."""
import argparse
import csv
from contextlib import ExitStack
from itertools import chain
import math
from pathlib import Path

SERIES = {
    'ACC': ('ax_g', 'ay_g', 'az_g'),
    'GYRO': ('gx_dps', 'gy_dps', 'gz_dps'),
    'H': ('diff_mV',), 'F': ('diff_mV',),
}


def padded_limits(values):
    """120% of max-min, centered on the data range; finite fallback for DC."""
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    low, high = min(finite), max(finite)
    span = high - low
    if span == 0:
        margin = max(abs(low) * 0.1, 1e-6)
    else:
        margin = span * 0.1
    return low - margin, high + margin


def load_recording(path, epoch=None):
    data = {}
    skipped = 0
    required = {'record_type', 'epoch', 'mcu_read_us_unwrapped', 'channel',
                *SERIES['ACC'], *SERIES['GYRO'], 'diff_mV'}
    path = Path(path)
    paths = [path/'imu.csv', path/'adc.csv'] if path.is_dir() else [path]
    with ExitStack() as stack:
        readers = [csv.DictReader(stack.enter_context(p.open(newline='', encoding='utf-8-sig'))) for p in paths]
        columns = set().union(*(set(r.fieldnames or []) for r in readers))
        reader = chain.from_iterable(readers)
        missing = required - columns
        if missing:
            raise ValueError('缺少统一CSV列：' + ', '.join(sorted(missing)))
        for row in reader:
            kind = row['record_type']
            if kind not in ('IMU', 'ADC'):
                continue
            try:
                ep = int(row['epoch'])
                if epoch is not None and ep != epoch:
                    continue
                timestamp = row.get('sample_mcu_us')
                if not timestamp and kind == 'IMU':
                    timestamp = row.get('imu_sample_mcu_us')
                t = float(timestamp or row['mcu_read_us_unwrapped']) / 1e6
                if not math.isfinite(t):
                    raise ValueError()
                if kind == 'IMU':
                    groups = {'ACC': tuple(float(row[k]) for k in SERIES['ACC']),
                              'GYRO': tuple(float(row[k]) for k in SERIES['GYRO'])}
                else:
                    if row['channel'] not in ('H', 'F'):
                        raise ValueError()
                    groups = {row['channel']: (float(row['diff_mV']),)}
                if not all(math.isfinite(v) for group in groups.values() for v in group):
                    raise ValueError()
            except (ValueError, TypeError):
                skipped += 1
                continue
            target = data.setdefault(ep, {k: [] for k in SERIES})
            for group, values in groups.items():
                target[group].append((t, *values))
    if not data:
        raise ValueError('未找到有效的IMU/ADC数据' + (f'（epoch={epoch}）' if epoch is not None else ''))
    if len(data) > 1:
        raise ValueError(f'文件包含多个MCU运行段 {sorted(data)}，请用 --epoch 编号选择，避免跨复位拼接时间。')
    selected, groups = next(iter(data.items()))
    origin = min(r[0] for rows in groups.values() for r in rows)
    for name, rows in groups.items():
        groups[name] = [(r[0] - origin, *r[1:]) for r in rows]
    return selected, groups, skipped


def create_figure(groups, title):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(13, 9), layout='constrained')
    colors = ('tab:blue', 'tab:orange', 'tab:green')
    units = {'ACC': 'g', 'GYRO': 'deg/s', 'H': 'mV', 'F': 'mV'}
    for ax, (name, rows) in zip(axes, groups.items()):
        ax.set_ylabel(f'{name} ({units[name]})')
        ax.grid(True, alpha=0.25)
        if not rows:
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
            continue
        times = [r[0] for r in rows]
        for i in range(len(SERIES[name])):
            label = ('X', 'Y', 'Z')[i] if name in ('ACC', 'GYRO') else name
            ax.plot(times, [r[i+1] for r in rows], color=colors[i], linewidth=0.65, label=label)
        limits = padded_limits(v for r in rows for v in r[1:])
        if limits:
            ax.set_ylim(*limits)
        ax.legend(loc='upper right', ncol=3 if name in ('ACC', 'GYRO') else 1)
        ax.margins(x=0.01)
    axes[-1].set_xlabel('Time since first sample (s)')
    fig.suptitle(title)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, help='统一录制CSV')
    parser.add_argument('--epoch', type=int, help='CSV含MCU复位时选择运行段')
    parser.add_argument('--output', '-o', type=Path, help='输出PNG/PDF/SVG；默认与CSV同名的PNG')
    parser.add_argument('--show', action='store_true', help='保存后打开交互窗口')
    args = parser.parse_args()
    try:
        epoch, groups, skipped = load_recording(args.csv, args.epoch)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    try:
        import matplotlib
        if not args.show:
            matplotlib.use('Agg')
    except ImportError:
        parser.error('缺少matplotlib；运行 python -m pip install matplotlib')
    output = args.output or args.csv.with_name(args.csv.stem + (f'_epoch{epoch}' if args.epoch is not None else '') + '.png')
    if output.resolve() == args.csv.resolve():
        parser.error('输出路径不能覆盖输入CSV')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig = create_figure(groups, f'{args.csv.stem} | epoch {epoch}')
    fig.savefig(output, dpi=180)
    print(f'已保存：{output.resolve()}')
    print('样本数：' + ', '.join(f'{k}={len(v)}' for k, v in groups.items()))
    if skipped:
        print(f'跳过 {skipped} 行无效/非有限数据。')
    import matplotlib.pyplot as plt
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == '__main__':
    main()
