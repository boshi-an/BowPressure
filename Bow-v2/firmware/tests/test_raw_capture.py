from contextlib import redirect_stdout
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from raw_capture import RawCapture, main, replay
from serial_rx import SerialReceiver
from test_read_fast import packet
from test_serial_rx import Port


class RawCaptureTests(unittest.TestCase):
    def test_attach_after_preview_taps_dropped_chunks(self):
        receiver = SerialReceiver(Port(), capacity=4)
        receiver._enqueue(b'old')
        self.assertEqual(receiver.get(), (0, b'old'))
        capture = RawCapture(max_bytes=32, max_chunks=8)
        receiver.set_raw_capture(capture)
        with patch('serial_rx.monotonic_ns', side_effect=[1000, 2000]):
            receiver._enqueue(b'\x00a')
            receiver._enqueue(b'b\xffc')
        self.assertEqual(receiver.overflow_events(), 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            manifest = capture.dump(path)
            self.assertEqual((path / 'serial.bin').read_bytes(), b'\x00ab\xffc')
            self.assertEqual(manifest['start_byte_offset'], 3)
            self.assertEqual(manifest['start_chunk_index'], 1)
            self.assertEqual(manifest['bytes_seen'], 5)
            self.assertFalse(manifest['truncated'])
            with (path / 'rx_chunks.csv').open(newline='') as handle:
                self.assertEqual(list(csv.reader(handle)), [
                    ['chunk_index', 'read_monotonic_ns', 'byte_offset', 'bytes_read', 'bytes_saved'],
                    ['0', '1000', '0', '2', '2'],
                    ['1', '2000', '2', '3', '3'],
                ])
            self.assertEqual(json.loads((path / 'rx_capture.json').read_text()), manifest)

    def test_byte_limit_partial_chunk_and_later_reads(self):
        capture = RawCapture(max_bytes=4, max_chunks=8)
        capture.append(b'a', 20)
        capture.append(b'bcde', 21)
        capture.append(b'fg', 30)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            manifest = capture.dump(path)
            self.assertEqual((path / 'serial.bin').read_bytes(), b'abcd')
            self.assertEqual(manifest['limit'], 'byte_limit')
            self.assertEqual(manifest['bytes_omitted'], 3)
            self.assertEqual(manifest['chunks_partial'], 1)
            self.assertEqual(manifest['chunks_omitted'], 1)
            self.assertEqual(manifest['last_read_monotonic_ns'], 30)
            with (path / 'rx_chunks.csv').open(newline='') as handle:
                self.assertEqual(list(csv.reader(handle))[-1], ['1', '21', '1', '4', '3'])

    def test_chunk_limit_and_collision(self):
        capture = RawCapture(max_bytes=32, max_chunks=1)
        capture.append(b'ab', 20)
        capture.append(b'c', 30)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            sentinel = path / 'rx_chunks.csv'
            sentinel.write_bytes(b'existing')
            with self.assertRaises(FileExistsError):
                capture.dump(path)
            self.assertFalse((path / 'serial.bin').exists())
            self.assertEqual(sentinel.read_bytes(), b'existing')
            sentinel.unlink()
            manifest = capture.dump(path)
            self.assertEqual(manifest['limit'], 'chunk_limit')
            self.assertEqual(manifest['chunks_omitted'], 1)
            self.assertEqual((path / 'serial.bin').read_bytes(), b'ab')

    def test_replay_crc_and_diagnostic_sequence(self):
        status = packet(3, 54, 100, b'')
        corrupt = bytearray(status)
        corrupt[-1] ^= 1
        raw = status + corrupt + packet(3, 56, 900, b'') + packet(7, 0, 950, b'') + status[:12]
        capture = RawCapture(max_bytes=len(raw), max_chunks=1)
        capture.append(raw, 1000)
        with tempfile.TemporaryDirectory() as directory:
            manifest = capture.dump(directory)
            result = replay(directory)
            self.assertFalse(manifest['truncated'])
            self.assertEqual(result['crc_errors'], 1)
            self.assertEqual(result['partial_tail_bytes'], 12)
            self.assertEqual(result['packets']['STATUS']['missing_seq'], 1)
            self.assertEqual(result['packets']['DIAG']['frames'], 1)
            self.assertEqual(result['first_sequence_gaps'], [
                dict(stream='STATUS', from_seq=54, to_seq=56, missing_seq=1, from_read_us=100, to_read_us=900)
            ])
            self.assertEqual(result['capture'], manifest)

    def test_standalone_capture_and_replay_ignore_sensor_csv(self):
        raw = packet(3, 1, 0, b'')
        port = Port([raw])
        port.open = lambda: None
        port.close = lambda: None
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'run'
            printed = io.StringIO()
            with patch('serial.Serial', return_value=port), redirect_stdout(printed):
                code = main(['capture', '--port', 'fake', '--duration', '.1',
                             '--output', str(output), '--max-mib', '1', '--max-chunks', '4'])
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(printed.getvalue())['bytes_saved'], len(raw))
            self.assertEqual(replay(output)['packets']['STATUS']['frames'], 1)
            self.assertFalse((output / 'adc.csv').exists())
