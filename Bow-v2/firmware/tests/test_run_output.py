import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import io
import struct
from read import main
from run_output import RunOutput
from export_ltc import merge_directory
from test_read_fast import packet
from plot import load_recording


class OutputTests(unittest.TestCase):
    def test_four_files_and_bracketing(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'run'
            with RunOutput(root) as w:
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, seq=0,
                                ltc_anchor_mcu_us=100000, ltc_anchor_day_us=1000000))
                w.writerow(dict(record_type='IMU', epoch=0, seq=0, sample_mcu_us=120000,
                                mcu_read_us_unwrapped=120100, ax_g=1, ay_g=2, az_g=3,
                                gx_dps=4, gy_dps=5, gz_dps=6))
                w.writerow(dict(record_type='ADC', epoch=0, seq=0, sample_mcu_us=125000,
                                mcu_read_us_unwrapped=125200, channel='H', diff_mV=3))
                w.writerow(dict(record_type='STATUS', epoch=0, seq=0, mcu_read_us_unwrapped=130000, faults=0))
                w.writerow(dict(record_type='LOG', message='boot', host_unix_s=123))
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, seq=1,
                                ltc_anchor_mcu_us=140000, ltc_anchor_day_us=1040000))
            self.assertEqual(sorted(p.name for p in root.iterdir()), ['adc.csv','imu.csv','ltc.csv','mcu.csv','merged.csv'])
            def rows(name):
                with (root/name).open() as f: return list(csv.DictReader(f))
            self.assertNotIn('ltc_us', rows('imu.csv')[0])
            self.assertNotIn('ltc_time', rows('adc.csv')[0])
            self.assertNotIn('sync', rows('adc.csv')[0])
            merged = rows('merged.csv')
            by_type = {r['record_type']: r for r in merged}
            self.assertEqual(by_type['IMU']['ltc_us'], '1020000')
            self.assertEqual(by_type['ADC']['ltc_us'], '1025000')
            self.assertEqual(by_type['STATUS']['ltc_us'], '1030000')
            self.assertEqual(by_type['LOG']['sync'], 'unlocked')
            self.assertEqual([r['record_type'] for r in merged],
                             ['LTC_ANCHOR', 'IMU', 'ADC', 'STATUS', 'LTC_ANCHOR', 'LOG'])
            original = {p.name: p.read_bytes() for p in root.glob('*.csv')}
            merge_directory(root)
            self.assertEqual(original, {p.name: p.read_bytes() for p in root.glob('*.csv')})
            ep, data, skipped = load_recording(root)
            self.assertEqual(ep, 0)
            self.assertEqual(skipped, 0)
            self.assertEqual(data['ACC'][0], (0,1,2,3))
            self.assertAlmostEqual(data['H'][0][0], .005)
            with self.assertRaises(FileExistsError):
                with RunOutput(root): pass

    def test_cli_ctrl_c_closes_and_finalizes(self):
        stream = packet(3, 0, 100, struct.pack('<IIIIH', 0,0,0,0,3))
        class Port:
            in_waiting = len(stream)
            sent = False
            closed = False
            def open(self): pass
            def close(self): self.closed = True
            def read(self, n):
                if self.sent: raise KeyboardInterrupt
                self.sent = True
                return stream
        port = Port()
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)/'run'
            with patch('sys.argv', ['read.py','--port','fake','--output',str(root)]), \
                    patch('read.serial.Serial', return_value=port), patch('sys.stdout',new_callable=io.StringIO):
                self.assertEqual(main(), 0)
            self.assertTrue(port.closed)
            with (root/'merged.csv').open() as f: rows = list(csv.DictReader(f))
            self.assertEqual(rows[0]['sync'], 'unlocked')
            self.assertEqual(len(list(root.iterdir())), 5)
