import csv
from pathlib import Path
import tempfile
import unittest
from plot import load_recording, padded_limits


class PlotTests(unittest.TestCase):
    def test_limits(self):
        self.assertEqual(padded_limits([-2, 8, float('nan')]), (-3, 9))
        self.assertEqual(padded_limits([5, 5]), (4.5, 5.5))
        self.assertEqual(padded_limits([0]), (-1e-6, 1e-6))
        self.assertIsNone(padded_limits([]))

    def test_mixed_rows_and_reset_selection(self):
        fields = ['record_type', 'epoch', 'mcu_read_us_unwrapped', 'channel',
                  'imu_sample_mcu_us', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps', 'diff_mV']
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'input.csv'
            with path.open('w', newline='') as handle:
                w = csv.DictWriter(handle, fieldnames=fields)
                w.writeheader()
                w.writerow(dict(record_type='ADC', epoch=0, mcu_read_us_unwrapped=1000000, channel='H', diff_mV=-2))
                w.writerow(dict(record_type='IMU', epoch=0, mcu_read_us_unwrapped=1500000, imu_sample_mcu_us=1400000,
                                ax_g=1, ay_g=2, az_g=3, gx_dps=4, gy_dps=5, gz_dps=6))
                w.writerow(dict(record_type='STATUS', epoch=0, mcu_read_us_unwrapped=1600000))
                w.writerow(dict(record_type='ADC', epoch=1, mcu_read_us_unwrapped=10, channel='F', diff_mV=8))
            with self.assertRaisesRegex(ValueError, '多个MCU运行段'):
                load_recording(path)
            epoch, data, skipped = load_recording(path, 0)
            self.assertEqual(epoch, 0)
            self.assertEqual(data['H'], [(0, -2)])
            self.assertAlmostEqual(data['ACC'][0][0], .4)
            self.assertEqual(data['ACC'][0][1:], (1, 2, 3))
            self.assertAlmostEqual(data['GYRO'][0][0], .4)
            self.assertEqual(data['GYRO'][0][1:], (4, 5, 6))
            self.assertEqual(data['F'], [])
            self.assertEqual(skipped, 0)
