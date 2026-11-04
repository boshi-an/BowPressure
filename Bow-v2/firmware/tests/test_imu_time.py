import unittest
from imu_time import ImuClock


class ClockTests(unittest.TestCase):
    def test_batching_drift_and_wrap(self):
        clock = ImuClock()
        last = -1
        errors = []
        for i in range(40000):
            sensor = 65000 + i*625
            actual = 100000 + i*630  # Different sensor and MCU clock rates.
            delay = 680 if i % 2 == 0 else 100
            row = clock.update(sensor & 65535, actual+delay, i, 0)
            self.assertEqual(row['sensor_us_unwrapped'], sensor)
            self.assertGreater(row['imu_sample_mcu_us'], last)
            self.assertLessEqual(row['imu_sample_mcu_us'], actual+delay)
            last = row['imu_sample_mcu_us']
            if i > 32000:
                errors.append(last-actual)
        self.assertEqual(row['imu_time_quality'], 'tracking')
        self.assertAlmostEqual(row['imu_clock_scale'], 1.008, places=5)
        self.assertLess(max(errors)-min(errors), 5)
        self.assertAlmostEqual(errors[-1], 100, delta=5)  # Unknown fixed delay remains.

    def test_gap_reset_and_duplicate(self):
        c = ImuClock()
        a = c.update(65500, 1000, 0xffffffff, 0)
        b = c.update(589, 1625, 0, 0)
        self.assertEqual(a['imu_time_segment'], b['imu_time_segment'])
        self.assertEqual(b['sensor_us_unwrapped'], 66125)
        d = c.update(1214, 100000, 99, 0)
        self.assertEqual(d['imu_time_segment'], 1)
        e = c.update(10, 100, 0, 1)
        self.assertEqual(e['imu_time_segment'], 2)
        self.assertEqual(e['imu_sample_mcu_us'], 100)
        self.assertEqual(c.update(10, 101, 0, 1)['imu_time_segment'], 3)

