#!/usr/bin/env python3
"""Receive V2/V3: IMU3200Hz + H/F1000SPS each, raw data, no host smoothing."""
import argparse
import binascii
import csv
import math
import os
import sys
from contextlib import ExitStack
from pathlib import Path
import struct
import time
import serial
from diagnostics import CaptureDiagnostics, MCU_DEBUG_FIELDS, PACKET_NAMES, PIPE_FIELDS, format_mcu_debug
from serial_rx import SerialReceiver, RX_FIELDS
from imu_time import ImuClock, FIELDS as IMU_TIME_FIELDS
from sample_time import adc_time, FIELDS as SAMPLE_TIME_FIELDS
from serial.tools import list_ports

SYNC = b'\xa5\x5a'


def imu_units(raw):
    """v5 HIRES: signed 20-bit fields; fixed scaling regardless of FS registers."""
    return tuple(x / 16384 for x in raw[:3]), tuple(x * 4000 / 524288 for x in raw[3:])


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.header_errors = 0
        self.discarded = 0
        self.messages = []
        self.text = bytearray()
        self.seen_binary = False

    def discard(self, n):
        # Startup ASCII also occurs after a reset in an already-open session.
        # Only parse bytes rejected by the binary framing, never valid payloads.
        self.text.extend(self.buffer[:n])
        while b'\n' in self.text:
            line, _, rest = self.text.partition(b'\n')
            self.text = bytearray(rest)
            marker = line.find(b'#')
            if marker >= 0:
                line = line[marker:]
                if all(c in (9, 13) or 32 <= c < 127 for c in line):
                    self.messages.append(line.decode('ascii'))
        if len(self.text) > 512:
            self.text.clear()
        del self.buffer[:n]
        self.discarded += n

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= 2:
            where = self.buffer.find(SYNC)
            if where < 0:
                self.discard(len(self.buffer) - (self.buffer[-1] == 0xA5))
                break
            if where:
                self.discard(where)
            if len(self.buffer) < 32:
                break
            frame = bytes(self.buffer[:32])
            crc = struct.unpack_from('<H', frame, 30)[0]
            bad_header = frame[2] != 2 or frame[3] not in PACKET_NAMES
            if bad_header or binascii.crc_hqx(frame[:30], 0xFFFF) != crc:
                self.crc_errors += 1
                self.header_errors += bad_header
                self.discard(1)
                continue
            del self.buffer[:32]
            self.seen_binary = True
            frames.append(decode_frame(frame))
        return frames


def decode_frame(frame):
    kind = frame[3]
    seq, read_us = struct.unpack_from('<II', frame, 4)
    item = {'type': kind, 'seq': seq, 'read_us': read_us}
    if kind == 1:
        values = struct.unpack_from('<6hH', frame, 12)
        low = frame[26:29]
        raw = tuple(values[j] * 16 + ((low[j % 3] >> 4) if j < 3 else (low[j % 3] & 15))
                    for j in range(6))
        item.update(raw=raw, sensor_us16=values[6])
    elif kind == 2:
        raw, status, channel, gain, start, age = struct.unpack_from('<iHBBII', frame, 12)
        item.update(raw=raw, status=status, channel=channel, gain=gain, start_us=start, age_us=age)
    elif kind == 5:
        status, pin, count, irq, polls, observed, peak = struct.unpack_from('<BBHIIIH', frame, 12)
        item.update(imu_status=status, int1_pin=pin, fifo_count=count,
                    irq_count=irq, poll_count=polls, observed_us=observed, peak_count=peak)
    elif kind == 6:
        h, m, s, frame, fps, boundary, edge = struct.unpack_from('<6BI', frame, 12)
        item.update(timecode=f'{h:02}:{m:02}:{s:02}:{frame:02}', fps=fps,
                    boundary=boundary, edge_us=edge)
    elif kind == 4:
        h, m, s, f, fps, locked, edges, errors, overflows = struct.unpack_from('<6BIII', frame, 12)
        item.update(timecode=f'{h:02}:{m:02}:{s:02}:{f:02}', fps=fps,
                    locked=bool(locked), edges=edges, errors=errors, overflows=overflows)
    elif kind == 7:
        item.update(zip(MCU_DEBUG_FIELDS, struct.unpack_from('<IIIHI', frame, 12)))
    else:
        drop, late, max_late, faults, alive = struct.unpack_from('<IIIIH', frame, 12)
        item.update(tx_drops=drop, missed_slots=late, max_late_us=max_late, faults=faults, alive=alive)
    return item


class Tracking:
    def __init__(self, imu_rate_hz=3200):
        self.epoch = 0
        self.last_us = None
        self.extended_us = 0
        self.previous = {}
        self.seq_errors = 0
        self.first = {}
        self.last = {}
        self.counts = {'I': 0, 'H': 0, 'F': 0}
        self.total = dict(self.counts)
        self.imu_ts = None
        self.timing_errors = 0
        self.imu_clock = ImuClock(imu_rate_hz)
        self.imu_time = {}

    def update(self, f):
        now = f['read_us']
        delta = (now - self.last_us) & 0xFFFFFFFF if self.last_us is not None else 0
        if self.last_us is not None and delta > 0x80000000:
            self.epoch += 1  # MCU reset; do not interpret as a 71-minute wrap
            self.previous.clear()
            self.imu_ts = None
            self.first.clear()
            self.last.clear()
            self.counts = dict.fromkeys(self.counts, 0)
            self.extended_us = now
        elif self.last_us is None:
            self.extended_us = now
        else:
            self.extended_us += delta
        self.last_us = now
        typ = f['type']
        old = self.previous.get(typ)
        if old is not None and (f['seq'] - old) & 0xFFFFFFFF != 1:
            self.seq_errors += 1
        self.previous[typ] = f['seq']
        dt = None
        if typ == 1:
            key = 'I'
            if self.imu_ts is not None:
                dt = (f['sensor_us16'] - self.imu_ts) & 0xFFFF
                if not self.imu_clock.valid_interval(dt):
                    self.timing_errors += 1
            self.imu_ts = f['sensor_us16']
            self.imu_time = self.imu_clock.update(f['sensor_us16'], self.extended_us, f['seq'], self.epoch)
        elif typ == 2:
            key = 'H' if f['channel'] == 0 else 'F'
        else:
            return None
        self.counts[key] += 1
        self.total[key] += 1
        self.first.setdefault(key, self.extended_us)
        self.last[key] = self.extended_us
        return dt

    def rates(self):
        result = {}
        for k, n in self.counts.items():
            span = self.last.get(k, 0) - self.first.get(k, 0)
            result[k] = (n - 1) * 1e6 / span if n > 1 and span > 0 else 0
        return result


def format_display(latest):
    def value(key, digits, signed=False):
        if key not in latest:
            return '--'
        return format(latest[key], f'{"+" if signed else ""}.{digits}f')
    return (f'|a|={value("a", 4)} g  |v_ang|={value("w", 3)} °/s  '
            f'H={value("H", 6, True)} mV  F={value("F", 6, True)} mV' +
            (f'  LTC={latest["ltc"]}' if 'ltc' in latest else ''))


CSV_FIELDS = [
    'host_unix_s', 'record_type', 'epoch', 'seq', 'mcu_read_us', 'mcu_read_us_unwrapped',
    'channel', 'sensor_us16', 'sensor_dt_us',
    'ax_raw', 'ay_raw', 'az_raw', 'gx_raw', 'gy_raw', 'gz_raw',
    'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
    'conversion_start_us', 'ready_observed_after_us', 'raw24', 'diff_mV', 'gain',
    'avdd_V', 'status_hex', 'fault_mask_hex',
    'timecode', 'fps', 'locked', 'edge_count', 'decode_errors', 'edge_overflows',
    'tx_drops', 'missed_slots', 'max_late_us', 'faults', 'alive',
    'imu_status_hex', 'int1_pin', 'fifo_count', 'irq_count', 'poll_count',
    'observed_us', 'peak_count', 'crc_errors', 'seq_errors', 'imu_dt_errors',
    'invalid_payloads', 'message', 'ltc_anchor_mcu_us', 'ltc_anchor_day_us',
] + IMU_TIME_FIELDS + SAMPLE_TIME_FIELDS + RX_FIELDS + PIPE_FIELDS + list(MCU_DEBUG_FIELDS)


def csv_record(f, tracking, dt, avdd, host_time):
    """One received packet per row; unrelated columns remain empty, never held."""
    typ = f['type']
    row = dict(host_unix_s=host_time,
               record_type=PACKET_NAMES[typ],
               epoch=tracking.epoch, seq=f['seq'], mcu_read_us=f['read_us'],
               mcu_read_us_unwrapped=tracking.extended_us)
    if typ == 1:
        a, g = imu_units(f['raw'])
        row.update(zip(('ax_raw', 'ay_raw', 'az_raw', 'gx_raw', 'gy_raw', 'gz_raw'), f['raw']))
        row.update(zip(('ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'), (*a, *g)))
        row.update(sensor_us16=f['sensor_us16'], sensor_dt_us=dt)
        row.update(tracking.imu_time)
        row.update(sample_mcu_us=tracking.imu_time['imu_sample_mcu_us'],
                   sample_time_basis='imu_clock_map_estimate')
    elif typ == 2:
        row.update(channel='H' if f['channel'] == 0 else 'F',
                   conversion_start_us=f['start_us'], ready_observed_after_us=f['age_us'],
                   raw24=f['raw'], diff_mV=f['raw'] * avdd * 1000 / (8388608 * f['gain']),
                   gain=f['gain'], avdd_V=avdd, status_hex=f"0x{f['status']:04X}",
                   fault_mask_hex=f"0x{(~(f['status'] >> 8)) & 0xFE:02X}")
        row.update(adc_time(f['read_us'], tracking.extended_us, f['start_us'], f['age_us']))
    elif typ == 6:
        h, m, s, frame = map(int, f['timecode'].split(':'))
        row.update(timecode=f['timecode'], fps=f['fps'])
        if f['fps'] == 25 and f['boundary'] == 1 and h < 24 and m < 60 and s < 60 and frame < 25:
            row.update(ltc_anchor_mcu_us=tracking.extended_us-((f['read_us']-f['edge_us']) & 0xffffffff),
                       ltc_anchor_day_us=(((h*60+m)*60+s)*1000000+(frame+1)*40000) % 86400000000)
    elif typ == 3:
        row.update({k: f[k] for k in ('tx_drops', 'missed_slots', 'max_late_us', 'faults', 'alive')})
    elif typ == 4:
        row.update(timecode=f['timecode'], fps=f['fps'], locked=int(f['locked']),
                   edge_count=f['edges'], decode_errors=f['errors'], edge_overflows=f['overflows'])
    elif typ == 7:
        row.update({key: f[key] for key in MCU_DEBUG_FIELDS})
    else:
        row.update(imu_status_hex=f"0x{f['imu_status']:02X}")
        row.update({k: f[k] for k in ('int1_pin', 'fifo_count', 'irq_count', 'poll_count', 'observed_us', 'peak_count')})
    return row


def main(argv=None, control=None, report=print, on_sample=None, output_provider=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port')
    p.add_argument('--list', action='store_true')
    p.add_argument('--baud', type=int, default=3000000)
    p.add_argument('--duration', type=float)
    p.add_argument('--output', type=Path, help='New run directory: adc.csv, imu.csv, ltc.csv, mcu.csv')
    p.add_argument('--csv', type=Path, help='NEW shared CSV: IMU, ADC H/F, LTC, status and FIFO diagnostics')
    p.add_argument('--ltc-csv', type=Path, help='On exit, export compact LTC-timed CSV; requires --csv raw recording')
    p.add_argument('--ltc-lines', action='store_true', help='print every received LTC frame')
    p.add_argument('--avdd', type=float, default=3.3)
    p.add_argument('--lines', action='store_true', help='refresh four values on every data frame; may slow terminal')
    p.add_argument('--diagnostics', action='store_true', help='also show startup, rates and diagnostic summary')
    p.add_argument('--raw-capture', action='store_true',
                   help='keep a bounded copy of serial input in RAM and save it in the run directory on stop')
    args = p.parse_args(argv)
    if args.list:
        for port in list_ports.comports():
            report(port.device, port.description)
        return 0
    if not args.port:
        p.error('--port required')
    if args.duration is not None and args.duration <= 0:
        p.error('--duration must be positive')
    if not 2.7 <= args.avdd <= 3.6:
        p.error('--avdd must be 2.7..3.6V')
    if args.output and (args.csv or args.ltc_csv):
        p.error('--output cannot be combined with --csv or --ltc-csv')
    if args.raw_capture and not (args.output or output_provider):
        p.error('--raw-capture requires --output or a GUI recording')
    if args.output and args.output.exists():
        p.error('--output must be a new directory; existing recordings are never overwritten')
    if args.csv and args.csv.exists():
        p.error('CSV must be a new file; existing recordings are never overwritten')
    if args.ltc_csv and not args.csv:
        p.error('--ltc-csv requires --csv for raw recording')
    if args.ltc_csv and args.ltc_csv.exists():
        p.error('LTC CSV must be a new file')
    if args.ltc_csv and args.ltc_csv.resolve() == args.csv.resolve():
        p.error('Raw CSV and LTC CSV must have different paths')
    imu_rate_hz = 3200 if args.baud == 3_000_000 else 1600
    decoder, tracking = Decoder(), Tracking(imu_rate_hz)
    latest, health = {}, {}
    last_arrival = {}
    no_diag_warned = False
    adc_faults = saturated = invalid = 0
    started = last_display = time.monotonic()
    rx = None
    rx_generation = 0
    rx_last_overflow = 0
    saving = False
    diagnostics = CaptureDiagnostics() if args.diagnostics else None
    reported_silence = False
    raw_capture = None
    raw_directory = args.output
    if args.raw_capture:
        from raw_capture import RawCapture
        raw_capture = RawCapture()

    def log(message, **fields):
        if args.diagnostics:
            report(message)
        if writer and saving:
            fields.setdefault('host_monotonic_ns', time.monotonic_ns())
            writer.writerow(dict(record_type='LOG', host_unix_s=time.time(), message=message, **fields))

    def save_raw_capture():
        if raw_capture is not None and raw_directory is not None:
            result = raw_capture.dump(raw_directory)
            message = (f"# RAW saved={result['bytes_saved']}/{result['bytes_seen']} B"
                       f" start_byte={result['start_byte_offset']} truncated={result['truncated']}")
            report(message)
            if writer:
                writer.writerow(dict(record_type='LOG', host_unix_s=time.time(),
                                     host_monotonic_ns=time.monotonic_ns(), message=message))

    def report_rx(final=False):
        nonlocal rx_last_overflow, reported_silence
        if rx is None:
            return
        stats = rx.snapshot(reset_interval=True)
        overflow = stats['rx_overflow_events'] != rx_last_overflow
        message = (f"# RX{' FINAL' if final else ''}{' OVERFLOW' if overflow else ''} "
                   f"queue={stats['rx_queue_bytes']}/{stats['rx_queue_capacity']} B "
                   f"({100*stats['rx_queue_bytes']/stats['rx_queue_capacity']:.1f}%) "
                   f"peak={stats['rx_queue_peak_bytes']} B "
                   f"overflows={stats['rx_overflow_events']} dropped={stats['rx_dropped_bytes']} B")
        if diagnostics:
            idle = stats['rx_read_idle_us']
            silence = idle is None or idle >= 1000000
            if silence != reported_silence and not final:
                text = 'no serial data for at least 1 s' if silence else 'serial data resumed'
                log(f'# STREAM {text}', **stats)
                reported_silence = silence
            idle_ms = f'{idle/1000:.3f}' if idle is not None else 'no-data'
            message += (f" read={stats['rx_received_bytes']} B"
                        f" handed_to_decoder={stats['rx_dequeued_bytes']} B idle_ms={idle_ms}"
                        f" max_read_gap_ms={stats['rx_interval_max_read_gap_us']/1000:.3f}"
                        f" driver_pending_peak={stats['rx_interval_driver_pending_peak_bytes']} B"
                        f" max_queue_wait_ms={stats['rx_interval_queue_max_residence_us']/1000:.3f}")
            stats.update(diagnostics.snapshot(decoder, time.monotonic_ns()), crc_errors=decoder.crc_errors,
                         seq_errors=tracking.seq_errors, invalid_payloads=invalid)
            report(f"# PIPE{' FINAL' if final else ''} window_ms={stats['diag_interval_us']/1000:.1f}"
                   f" decoded={stats['decoded_packets']} saved={stats['saved_packets']}"
                   f" rejected_bytes_total={stats['decoder_discarded_bytes']} crc_or_header_total={decoder.crc_errors}"
                   f" decode_ms={stats['decode_us']/1000:.1f} process_ms={stats['process_us']/1000:.1f}"
                   f" csv_write_ms={stats['csv_write_us']/1000:.1f} csv_flush_ms={stats['csv_flush_us']/1000:.1f}")
        report(message)
        if writer and saving:
            writer.writerow(dict(record_type='LOG', host_unix_s=time.time(),
                                 message=message, **stats))
        rx_last_overflow = stats['rx_overflow_events']

    try:
        with ExitStack() as stack:
            outputs = stack.enter_context(ExitStack())
            writer = handle = None
            if args.output:
                from run_output import RunOutput
                writer = handle = outputs.enter_context(RunOutput(args.output))
                report(f'分项CSV保存到：{args.output.resolve()}；结束时生成 merged.csv（LTC时间映射）')
            if args.csv:
                args.csv.parent.mkdir(parents=True, exist_ok=True)
                handle = outputs.enter_context(args.csv.open('x', newline='', encoding='utf-8'))
                writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
                writer.writeheader()
                report(f'全部数据保存到：{args.csv.resolve()}')
            port = serial.Serial(port=None, baudrate=args.baud, timeout=.05)
            port.dtr = port.rts = False
            port.port = args.port
            port.open()
            stack.callback(save_raw_capture)
            stack.callback(port.close)
            stack.callback(lambda: report_rx(final=True))
            rx = stack.enter_context(SerialReceiver(port, raw_capture=raw_capture if args.output else None))
            if args.diagnostics:
                report(f'读取 {args.port} {args.baud} 8N1；IMU{imu_rate_hz}Hz High Resolution，H/F目标各1000SPS；无电脑端滤波。')
            while args.duration is None or time.monotonic() - started < args.duration:
                action = control() if control else 'recording'
                if action == 'stopping':
                    break
                if writer is None and output_provider and action == 'recording':
                    destination = output_provider()
                    if destination:
                        from run_output import RunOutput
                        writer = handle = outputs.enter_context(RunOutput(destination))
                        if raw_capture is not None:
                            raw_directory = Path(destination)
                            rx.set_raw_capture(raw_capture)
                        report(f'开始保存到：{destination}')
                saving = action not in ('paused', 'previewing')
                try:
                    generation, chunk = rx.get()
                except (OSError, serial.SerialException) as exc:
                    log(f'# STREAM serial read failed: {exc}', **rx.snapshot())
                    raise
                if generation is not None and generation != rx_generation:
                    decoder.discard(len(decoder.buffer))
                    decoder.text.clear()
                    rx_generation = generation
                if rx.overflow_events() != rx_last_overflow:
                    report_rx()
                decode_started_ns = time.monotonic_ns() if diagnostics else 0
                frames = decoder.feed(chunk)
                if diagnostics:
                    process_started_ns = time.monotonic_ns()
                    diagnostics.decode_ns += process_started_ns - decode_started_ns
                    diagnostics.decoded += len(frames)
                    gaps = []
                for f in frames:
                    typ = f['type']
                    if typ == 2 and (f['channel'] not in (0, 1) or f['gain'] != 32 or not -8388608 <= f['raw'] < 8388608):
                        invalid += 1
                        continue
                    old_epoch = tracking.epoch
                    previous_mcu_us = tracking.extended_us
                    dt = tracking.update(f)
                    if tracking.epoch != old_epoch:
                        latest.clear(); health.clear(); last_arrival.clear()
                        no_diag_warned = False
                        if diagnostics:
                            diagnostics.reset_epoch()
                            log(f'# STREAM MCU timestamp discontinuity epoch={tracking.epoch}'
                                f' mcu_us={previous_mcu_us}->{tracking.extended_us}',
                                epoch=tracking.epoch, mcu_read_us_unwrapped=tracking.extended_us)
                    if diagnostics:
                        gaps.extend(diagnostics.observe(f, tracking.epoch, tracking.extended_us, previous_mcu_us))
                    if typ in (1, 2):
                        last_arrival['I' if typ == 1 else ('H' if f['channel'] == 0 else 'F')] = time.monotonic()
                    host_time = time.time()
                    if typ == 1:
                        raw = f['raw']
                        a, g = imu_units(raw)
                        latest['a'] = math.sqrt(sum(x*x for x in a))
                        latest['w'] = math.sqrt(sum(x*x for x in g))
                    elif typ == 2:
                        ch = 'H' if f['channel'] == 0 else 'F'
                        mv = f['raw'] * args.avdd * 1000 / (8388608 * f['gain'])
                        fault = (~(f['status'] >> 8)) & 0xFE
                        adc_faults += bool(fault)
                        saturated += abs(f['raw']) >= 8388607
                        latest[ch] = mv
                    elif typ == 5:
                        latest['imu_diag'] = f
                    elif typ == 6:
                        pass  # Precision anchor saved to CSV; not a health packet.
                    elif typ == 4:
                        latest['ltc'] = f['timecode'] if f['locked'] else 'UNLOCKED'
                        latest['ltc_received'] = time.monotonic()
                        if args.ltc_lines:
                            report(f"LTC {latest['ltc']} {f['fps']}fps edges={f['edges']} "
                                  f"errors={f['errors']} overflow={f['overflows']}")
                    elif typ == 7:
                        if diagnostics:
                            report(format_mcu_debug(f))
                    else:
                        health = f
                    row = None
                    if (writer and saving) or (on_sample and typ in (1, 2)):
                        row = csv_record(f, tracking, dt, args.avdd, host_time)
                    if on_sample and typ in (1, 2):
                        on_sample(dict(row, recording=saving and writer is not None))
                    if writer and saving:
                        row.update(crc_errors=decoder.crc_errors, seq_errors=tracking.seq_errors,
                                   imu_dt_errors=tracking.timing_errors, invalid_payloads=invalid)
                        write_started_ns = time.monotonic_ns() if diagnostics else 0
                        writer.writerow(row)
                        if diagnostics:
                            diagnostics.write_ns += time.monotonic_ns() - write_started_ns
                            diagnostics.saved += 1
                    if args.lines and typ in (1, 2):
                        report(format_display(latest))
                if diagnostics:
                    diagnostics.process_ns += time.monotonic_ns() - process_started_ns
                    if gaps:
                        stats = rx.snapshot()
                        stamp = time.monotonic_ns()
                        for gap in gaps:
                            seq = (f" seq={gap['gap_from_seq']}->{gap['seq']} missing={gap['gap_missing_packets']}"
                                   if 'seq' in gap else '')
                            log(f"# GAP stream={gap['gap_stream']} epoch={gap['epoch']}{seq}"
                                f" mcu_us={gap['gap_from_mcu_us']}->{gap['mcu_read_us_unwrapped']}"
                                f" gap_ms={gap['gap_mcu_us']/1000:.3f}"
                                f" queue={stats['rx_queue_bytes']} B crc_or_header_total={decoder.crc_errors}",
                                **gap, **stats, host_monotonic_ns=stamp, crc_errors=decoder.crc_errors,
                                decoder_discarded_bytes=decoder.discarded)
                for message in decoder.messages:
                    log(message.rstrip('\r'))
                decoder.messages.clear()
                now = time.monotonic()
                if 'ltc_received' in latest and now - latest['ltc_received'] > .2:
                    latest['ltc'] = 'UNLOCKED'
                if now - last_display >= 1:
                    report_rx()
                    if args.diagnostics:
                        rates = tracking.rates()
                        stale = [key for key in rates if now - last_arrival.get(key, started - 2) > 1]
                        for key in stale:
                            rates[key] = 0.0
                        if stale:
                            report('数据超时（超过1秒未收到新样本）：' + ', '.join(stale))
                        if health and now - started > 3 and 'imu_diag' not in latest and not no_diag_warned:
                            report('未收到FIFO诊断包：请确认已烧录最新 build/v3.hex；仅更新Python不会更新MCU。')
                            no_diag_warned = True
                        report('速率(MCU时间) ' + ' '.join(f'{k}={v:.1f}Hz' for k,v in rates.items()) +
                              f' CRC错误={decoder.crc_errors} seq_err={tracking.seq_errors} dt_err={tracking.timing_errors}' +
                              f" TX丢帧={health.get('tx_drops', '?')} ADC错过时隙={health.get('missed_slots', '?')}" +
                              f" 最大发起延迟={health.get('max_late_us', '?')}us 故障={health.get('faults', '?')} 存活={health.get('alive', '?')}" +
                              f' ADC故障帧={adc_faults} 饱和={saturated}')
                        if 'imu_diag' in latest:
                            diag = latest['imu_diag']
                            report(f"IMU诊断 STATUS=0x{diag['imu_status']:02X} FIFO={diag['fifo_count']}帧 "
                                  f"峰值={diag['peak_count']} INT1={diag['int1_pin']} IRQ={diag['irq_count']} "
                                  f"读取次数={diag['poll_count']} 最后读取={diag['observed_us']}us")
                    # Never present a held last sample as fresh sensor data.
                    shown = dict(latest)
                    for key, names in (('I', ('a', 'w')), ('H', ('H',)), ('F', ('F',))):
                        if now - last_arrival.get(key, started - 2) > 1:
                            for name in names:
                                shown.pop(name, None)
                    if not args.lines:
                        report(format_display(shown))
                    if handle:
                        flush_started_ns = time.monotonic_ns() if diagnostics else 0
                        handle.flush()
                        if diagnostics:
                            flush_ns = time.monotonic_ns() - flush_started_ns
                            diagnostics.flush_ns += flush_ns
                            diagnostics.max_flush_ns = max(diagnostics.max_flush_ns, flush_ns)
                    last_display = now
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        raise  # Handled by cli(); do not print another message to a closed pipe.
    except (OSError, serial.SerialException) as exc:
        report(f'错误：{exc}')
        return 1
    if args.ltc_csv:
        from export_ltc import export
        args.ltc_csv.parent.mkdir(parents=True, exist_ok=True)
        count, synced = export(args.csv, args.ltc_csv)
        report(f'LTC精简CSV：{args.ltc_csv.resolve()}；{synced}/{count}行已映射，其余时间留空')
    if args.diagnostics:
        report(f'结束：总帧数={tracking.total}；当前epoch速率={tracking.rates()}；'
              f'CRC错误={decoder.crc_errors} 序号异常={tracking.seq_errors} IMU间隔异常={tracking.timing_errors} '
              f'无效载荷={invalid} ADC故障帧={adc_faults} 饱和={saturated}；最后状态={health}')
    return 0


def cli():
    try:
        result = main()
        sys.stdout.flush()
        return result
    except BrokenPipeError:
        # Ctrl+C may terminate tee before our final summary. Prevent a second
        # exception from Python's shutdown flush; serial/CSV contexts are closed.
        with open(os.devnull, 'w') as sink:
            os.dup2(sink.fileno(), sys.stdout.fileno())
        return 0


if __name__ == '__main__':
    raise SystemExit(cli())
