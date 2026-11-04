import unittest
from sample_time import adc_time, ADC_FILTER_DELAY_US
from read import Tracking, csv_record


class AdcTimeTests(unittest.TestCase):
    def test_ready_not_read_and_wrap(self):
        r = adc_time(1500, 1500, 1000, 383)
        self.assertEqual(r['adc_ready_mcu_us'], 1383)
        self.assertAlmostEqual(r['sample_mcu_us'], 1213.078125)
        self.assertAlmostEqual(ADC_FILTER_DELAY_US, 169.921875)
        r = adc_time(400, 2**32+400, 2**32-100, 383)
        self.assertEqual(r['adc_start_mcu_us'], 2**32-100)
        self.assertEqual(r['adc_ready_mcu_us'], 2**32+283)

    def test_invalid(self):
        for start, age in [(1000, 600), (2000, 383), (1000, 50)]:
            r = adc_time(1500, 1500, start, age)
            self.assertEqual(r['sample_time_basis'], 'invalid_adc_timing')
            self.assertNotIn('sample_mcu_us', r)

    def test_csv_integration(self):
        t = Tracking()
        f = dict(type=2, seq=0, read_us=1500, start_us=1000, age_us=383,
                 channel=0, raw=123, gain=32, status=0xff00)
        dt = t.update(f)
        r = csv_record(f, t, dt, 3.3, 1)
        self.assertEqual(r['mcu_read_us'], 1500)
        self.assertEqual(r['adc_ready_mcu_us'], 1383)
        self.assertAlmostEqual(r['sample_mcu_us'], 1213.078125)
        self.assertEqual(r['raw24'], 123)
