#!/usr/bin/env python3
"""Capture or replay the bytes returned by a serial port before packet decoding."""
import argparse
from contextlib import closing
import csv
import json
import math
from pathlib import Path
import struct
import sys
import time


DEFAULT_MAX_BYTES = 32 * 1024 * 1024
DEFAULT_MAX_CHUNKS = 1_250_000
CHUNK_INDEX = struct.Struct('<QI')  # Read completion monotonic ns, bytes returned.


class RawCapture:
    """Keep a bounded, indexed prefix of one receiver's nonempty reads in RAM."""

    def __init__(self, max_bytes=DEFAULT_MAX_BYTES, max_chunks=DEFAULT_MAX_CHUNKS):
        if max_bytes <= 0 or max_chunks <= 0:
            raise ValueError('raw byte and chunk limits must be positive')
        self.max_bytes = max_bytes
        self.max_chunks = max_chunks
        self.raw = bytearray(max_bytes)
        self.index = bytearray(max_chunks * CHUNK_INDEX.size)
        self.bytes_seen = self.chunks_seen = 0
        self.bytes_saved = self.chunks_saved = self.chunks_partial = 0
        self.start_byte_offset = self.start_chunk_index = 0
        self.first_read_ns = self.last_read_ns = None
        self.limit = None
        self.attached = False

    def attach(self, byte_offset, chunk_index):
        if self.attached:
            raise ValueError('raw capture already belongs to a receiver')
        self.start_byte_offset = byte_offset
        self.start_chunk_index = chunk_index
        self.attached = True

    def append(self, chunk, read_ns):
        if not chunk:
            return
        if self.chunks_seen == 0:
            self.first_read_ns = read_ns
        self.last_read_ns = read_ns
        self.chunks_seen += 1
        size = len(chunk)
        self.bytes_seen += size
        if self.limit:
            return
        if self.bytes_saved == self.max_bytes:
            self.limit = 'byte_limit'
            return
        if self.chunks_saved == self.max_chunks:
            self.limit = 'chunk_limit'
            return

        saved = min(size, self.max_bytes - self.bytes_saved)
        end = self.bytes_saved + saved
        self.raw[self.bytes_saved:end] = chunk if saved == size else chunk[:saved]
        CHUNK_INDEX.pack_into(self.index, self.chunks_saved * CHUNK_INDEX.size, read_ns, size)
        self.bytes_saved = end
        self.chunks_saved += 1
        if saved < size:
            self.chunks_partial += 1
            self.limit = 'byte_limit'

    def dump(self, directory):
        """Write the saved prefix after the source stops and its receiver joins."""
        directory = Path(directory)
        summary = dict(
            version=1, max_bytes=self.max_bytes, max_chunks=self.max_chunks,
            start_byte_offset=self.start_byte_offset, start_chunk_index=self.start_chunk_index,
            bytes_seen=self.bytes_seen, chunks_seen=self.chunks_seen,
            bytes_saved=self.bytes_saved, chunks_saved=self.chunks_saved,
            bytes_omitted=self.bytes_seen - self.bytes_saved,
            chunks_omitted=self.chunks_seen - self.chunks_saved,
            chunks_partial=self.chunks_partial, truncated=self.limit is not None, limit=self.limit,
            first_read_monotonic_ns=self.first_read_ns, last_read_monotonic_ns=self.last_read_ns,
        )
        created = []
        try:
            raw_path = directory / 'serial.bin'
            with raw_path.open('xb') as handle:
                created.append(raw_path)
                handle.write(memoryview(self.raw)[:self.bytes_saved])

            csv_path = directory / 'rx_chunks.csv'
            with csv_path.open('x', encoding='utf-8', newline='') as handle:
                created.append(csv_path)
                writer = csv.writer(handle)
                writer.writerow(('chunk_index', 'read_monotonic_ns', 'byte_offset', 'bytes_read', 'bytes_saved'))
                byte_offset = 0
                for index in range(self.chunks_saved):
                    stamp, size = CHUNK_INDEX.unpack_from(self.index, index * CHUNK_INDEX.size)
                    saved = min(size, self.bytes_saved - byte_offset)
                    writer.writerow((index, stamp, byte_offset, size, saved))
                    byte_offset += saved

            manifest_path = directory / 'rx_capture.json'
            with manifest_path.open('x', encoding='utf-8') as handle:
                created.append(manifest_path)
                json.dump(summary, handle, indent=2)
                handle.write('\n')
        except BaseException:
            for path in created:
                path.unlink(missing_ok=True)
            raise
        return summary


def capture_serial(port_name, baud, duration, output, max_bytes, max_chunks):
    import serial
    from serial_rx import SerialReceiver

    capture = RawCapture(max_bytes=max_bytes, max_chunks=max_chunks)
    output.mkdir(parents=True, exist_ok=False)
    try:
        with closing(serial.Serial(port=None, baudrate=baud, timeout=.05)) as port:
            port.dtr = port.rts = False
            port.port = port_name
            port.open()
            end = time.monotonic() + duration
            with SerialReceiver(port, raw_capture=capture) as receiver:
                while (remaining := end - time.monotonic()) > 0:
                    receiver.get(min(.05, remaining))
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f'Serial capture ended with an error: {exc}', file=sys.stderr)
        return False
    finally:
        print(json.dumps(capture.dump(output), indent=2))
    return True


def replay(source):
    """Inspect the recorded byte stream with the same decoder as the recorder."""
    from diagnostics import PACKET_NAMES
    from read import Decoder

    source = Path(source)
    raw_path = source / 'serial.bin' if source.is_dir() else source
    manifest_path = raw_path.with_name('rx_capture.json')
    has_manifest = raw_path.name == 'serial.bin' and manifest_path.is_file()
    manifest = json.loads(manifest_path.read_text(encoding='utf-8')) if has_manifest else None
    decoder = Decoder()
    packets = {}
    gaps = []
    messages = []
    message_count = 0
    count = 0
    with raw_path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(4096), b''):
            count += len(chunk)
            for frame in decoder.feed(chunk):
                seq, stamp = frame['seq'], frame['read_us']
                kind = PACKET_NAMES[frame['type']]
                if kind not in packets:
                    packets[kind] = dict(frames=0, first_seq=seq, last_seq=seq,
                                         first_read_us=stamp, last_read_us=stamp,
                                         gap_events=0, missing_seq=0, duplicate_or_backward_seq=0)
                stream = packets[kind]
                if stream['frames']:
                    delta = (seq - stream['last_seq']) & 0xFFFFFFFF
                    if 1 < delta < 0x80000000:
                        stream['gap_events'] += 1
                        stream['missing_seq'] += delta - 1
                        if len(gaps) < 100:
                            gaps.append(dict(stream=kind, from_seq=stream['last_seq'], to_seq=seq,
                                             missing_seq=delta - 1, from_read_us=stream['last_read_us'],
                                             to_read_us=stamp))
                    elif delta != 1:
                        stream['duplicate_or_backward_seq'] += 1
                stream['frames'] += 1
                stream['last_seq'], stream['last_read_us'] = seq, stamp
            message_count += len(decoder.messages)
            messages.extend(decoder.messages[:100 - len(messages)])
            decoder.messages.clear()
    if manifest is not None and manifest['bytes_saved'] != count:
        raise ValueError('serial.bin size differs from the capture manifest')
    return dict(bytes_read=count, crc_errors=decoder.crc_errors, header_errors=decoder.header_errors,
                discarded_bytes=decoder.discarded, partial_tail_bytes=len(decoder.buffer),
                packets=packets, first_sequence_gaps=gaps, messages=messages,
                message_count=message_count, capture=manifest)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('capture', help='capture raw bytes without CSV sensor processing')
    capture.add_argument('--port', required=True)
    capture.add_argument('--baud', type=int, default=3_000_000)
    capture.add_argument('--duration', type=float, default=60)
    capture.add_argument('--output', type=Path, required=True)
    capture.add_argument('--max-mib', type=int, default=DEFAULT_MAX_BYTES // (1024 * 1024))
    capture.add_argument('--max-chunks', type=int, default=DEFAULT_MAX_CHUNKS)
    inspect = commands.add_parser('replay', help='decode an existing run directory or raw binary')
    inspect.add_argument('source', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == 'replay':
            print(json.dumps(replay(args.source), indent=2))
            return 0
        if not math.isfinite(args.duration) or args.duration <= 0 or args.baud <= 0:
            parser.error('duration and baud must be positive')
        success = capture_serial(args.port, args.baud, args.duration, args.output,
                                 args.max_mib * 1024 * 1024, args.max_chunks)
        return 0 if success else 1
    except (OSError, ValueError) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    sys.exit(main())
