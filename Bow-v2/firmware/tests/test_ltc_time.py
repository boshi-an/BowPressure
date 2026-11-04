import csv
from pathlib import Path
import struct
import tempfile
import unittest
from read import Decoder, Tracking, csv_record
from test_read_fast import packet
from export_ltc import export, DAY, Timeline


class LtcTimeTests(unittest.TestCase):
    def test_anchor_packet_wrap_and_frame_end(self):
        payload = struct.pack('<6BI', 23, 59, 59, 24, 25, 1, 0xfffffff0)
        f = Decoder().feed(packet(6, 0, 20, payload))[0]
        t = Tracking()
        t.extended_us = 2**32+20
        row = csv_record(f, t, None, 3.3, 0)
        self.assertEqual(row['ltc_anchor_mcu_us'], 2**32-16)
        self.assertEqual(row['ltc_anchor_day_us'], 0)  # Next frame, not last frame label.

    def test_interpolation_midnight_gap_and_missing_anchor(self):
        fields = ['record_type', 'epoch', 'mcu_read_us_unwrapped', 'sample_mcu_us',
                  'ltc_anchor_mcu_us', 'ltc_anchor_day_us', 'channel', 'diff_mV', 'ax_g']
        with tempfile.TemporaryDirectory() as d:
            src, dst = Path(d)/'raw.csv', Path(d)/'compact.csv'
            with src.open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=fields);w.writeheader()
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, ltc_anchor_mcu_us=100000, ltc_anchor_day_us=DAY-40000))
                w.writerow(dict(record_type='IMU', epoch=0, sample_mcu_us=120000, ax_g='1.2'))
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, ltc_anchor_mcu_us=140000, ltc_anchor_day_us=0))
                w.writerow(dict(record_type='ADC', epoch=0, sample_mcu_us=160000, channel='H', diff_mV='3'))
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, ltc_anchor_mcu_us=180000, ltc_anchor_day_us=40000))
                w.writerow(dict(record_type='IMU', epoch=0, sample_mcu_us=200000))
                w.writerow(dict(record_type='LTC_ANCHOR', epoch=0, ltc_anchor_mcu_us=300000, ltc_anchor_day_us=160000))
                w.writerow(dict(record_type='IMU', epoch=1, sample_mcu_us=120000))
            n, synced = export(src, dst)
            self.assertEqual(n, 8)
            self.assertEqual(synced, 7)
            with dst.open() as f: rows = list(csv.DictReader(f))
            self.assertEqual(int(rows[1]['ltc_us']), DAY-20000)
            self.assertEqual(rows[1]['ax_g'], '1.2')
            self.assertEqual(int(rows[3]['ltc_us']), DAY+20000)
            self.assertEqual(rows[3]['ltc_time'], '1+00:00:00.020000')
            self.assertEqual(rows[3]['H_mV'], '3')
            self.assertEqual(rows[5]['sync'], 'interpolated_gap')
            self.assertEqual(int(rows[5]['ltc_us']), DAY+60000)
            self.assertEqual(rows[7]['ltc_us'], '')
            with self.assertRaises(FileExistsError): export(src, dst)

    def test_extrapolation_and_jump(self):
        timeline = Timeline.__new__(Timeline)
        timeline.groups = {0: [(100000, 1000000, 0), (140000, 1040000, 0),
                               (200000, 9000000, 1)], 1: [(5000, 10000, 0)]}
        timeline.indices = {ep: [p[0] for p in ps] for ep, ps in timeline.groups.items()}
        def mapped(t, ep=0):
            return timeline.map(dict(record_type='IMU', sample_mcu_us=str(t), epoch=str(ep)))
        self.assertEqual(mapped(80000)['ltc_us'], 980000)
        self.assertEqual(mapped(80000)['sync'], 'extrapolated')
        self.assertEqual(mapped(150000)['ltc_us'], 1050000)
        self.assertEqual(mapped(190000)['ltc_us'], 8990000)
        self.assertEqual(mapped(220000)['sync'], 'extrapolated_nominal')
        self.assertEqual(mapped(6000, 1)['ltc_us'], 11000)
        self.assertEqual(mapped(6000, 2)['ltc_us'], '')
        self.assertEqual(timeline.map(dict(record_type='LOG', epoch='0'))['ltc_us'], '')
