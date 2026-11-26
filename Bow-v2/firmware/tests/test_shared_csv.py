import csv
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import struct
import read
from test_read_fast import packet


class SharedCsvTests(unittest.TestCase):
    def test_diagnostics_record_corruption_and_shared_gap(self):
        payload = struct.pack('<iHBBII', 123, 0xff00, 0, 32, 100, 400)
        corrupt = bytearray(packet(2, 1, 1000, payload))
        corrupt[12] ^= 1
        diagnostics = packet(7, 0, 23100, struct.pack('<IIIHI', 0x08000000, 22100, 1500, 96, 105))
        stream = packet(2, 0, 500, payload) + corrupt + packet(2, 45, 23000, payload) + diagnostics

        class Port:
            in_waiting = len(stream)
            sent = False

            def open(self): pass
            def close(self): pass

            def read(self, size):
                if self.sent:
                    raise KeyboardInterrupt
                self.sent = True
                return stream

        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / 'capture'
            with patch.object(read.serial, 'Serial', return_value=Port()):
                self.assertEqual(read.main(['--port', 'fake', '--output', str(run),
                                            '--diagnostics', '--raw-capture'],
                                           report=lambda message: None), 0)
            with (run / 'adc.csv').open() as handle:
                self.assertEqual([row['seq'] for row in csv.DictReader(handle)], ['0', '45'])
            with (run / 'mcu.csv').open() as handle:
                logs = list(csv.DictReader(handle))
            shared, adc = [row for row in logs if row['gap_stream']]
            debug = next(row for row in logs if row['record_type'] == 'DIAG')
            final = next(row for row in logs if row['message'].startswith('# RX FINAL'))
            self.assertEqual((shared['gap_stream'], shared['gap_mcu_us']), ('ALL', '22500'))
            self.assertEqual((adc['gap_stream'], adc['gap_from_seq'], adc['seq'], adc['gap_missing_packets']),
                             ('ADC', '0', '45', '44'))
            self.assertEqual([debug[key] for key in read.MCU_DEBUG_FIELDS],
                             [str(0x08000000), '22100', '1500', '96', '105'])
            self.assertEqual(int(final['rx_received_bytes']), len(stream))
            self.assertEqual(final['rx_dequeued_bytes'], final['rx_received_bytes'])
            self.assertEqual((final['decoded_packets'], final['saved_packets']), ('3', '3'))
            self.assertEqual((final['crc_errors'], final['decoder_header_errors']), ('1', '0'))
            self.assertEqual((final['decoder_discarded_bytes'], final['decoder_pending_bytes']), ('32', '0'))
            self.assertEqual((run / 'serial.bin').read_bytes(), stream)

    def test_mixed_stream_is_saved_once_without_filling_other_channels(self):
        stream = b'# Bow hardware V3\r\n'
        stream += packet(1, 0, 100, struct.pack('<6hH', 1024, -2, 3, 4, 5, 6, 65500))
        stream += packet(2, 0, 200, struct.pack('<iHBBII', -100, 0xff00, 0, 32, 150, 50))
        stream += packet(2, 1, 300, struct.pack('<iHBBII', 200, 0xff00, 1, 32, 250, 50))
        stream += packet(3, 0, 400, struct.pack('<IIIIH', 2, 3, 10, 0, 3))
        stream += packet(4, 0, 400, struct.pack('<6BIII', 1, 2, 3, 4, 25, 1, 120, 2, 0))
        stream += packet(5, 0, 400, struct.pack('<BBHIIIH', 2, 1, 2, 300, 290, 395, 8))
        # MCU reset gives a new epoch and resets sequence tracking.
        stream += packet(3, 0, 10, struct.pack('<IIIIH', 0, 0, 0, 0, 3))

        class Port:
            in_waiting = len(stream)
            closed = False
            sent = False
            def open(self): pass
            def close(self): self.closed = True
            def read(self, size):
                if self.sent: raise KeyboardInterrupt
                self.sent = True
                return stream

        port = Port()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'nested' / 'all.csv'
            with patch('sys.argv', ['read.py', '--port', 'fake', '--csv', str(path)]), \
                    patch.object(read.serial, 'Serial', return_value=port), patch('sys.stdout', new_callable=io.StringIO):
                self.assertEqual(read.main(), 0)
            self.assertTrue(port.closed)
            with path.open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r['record_type'] for r in rows],
                             ['IMU', 'ADC', 'ADC', 'STATUS', 'LTC', 'IMU_STATUS', 'STATUS', 'LOG', 'LOG'])
            self.assertEqual(rows[0]['imu_sample_mcu_us'], '100.0')
            self.assertEqual(rows[0]['imu_time_quality'], 'warmup')
            self.assertEqual(rows[1]['imu_sample_mcu_us'], '')
            self.assertEqual(rows[0]['ax_raw'], '16384')
            self.assertEqual(float(rows[0]['ax_g']), 1.0)
            self.assertEqual(rows[0]['raw24'], '')
            self.assertEqual(rows[1]['channel'], 'H')
            self.assertEqual(rows[2]['channel'], 'F')
            self.assertEqual(rows[1]['ax_g'], '')
            self.assertEqual(rows[1]['raw24'], '-100')
            self.assertAlmostEqual(float(rows[1]['diff_mV']), -100 * 3300 / (8388608 * 32))
            self.assertEqual(rows[3]['missed_slots'], '3')
            self.assertEqual(rows[4]['timecode'], '01:02:03:04')
            self.assertEqual(rows[5]['fifo_count'], '2')
            self.assertEqual(rows[5]['peak_count'], '8')
            self.assertEqual(rows[6]['epoch'], '1')
            self.assertEqual(rows[7]['message'], '# Bow hardware V3')
            self.assertEqual(rows[-1]['rx_queue_capacity'], '1048576')
            self.assertEqual(rows[-1]['rx_overflow_events'], '0')
            original = path.read_bytes()
            with patch('sys.argv', ['read.py', '--port', 'fake', '--csv', str(path)]), \
                    patch('sys.stderr', new_callable=io.StringIO):
                with self.assertRaises(SystemExit): read.main()
            self.assertEqual(path.read_bytes(), original)
