import binascii
import struct
import unittest
from read import Decoder, Tracking, imu_units


def packet(kind, seq, us, payload):
    b = struct.pack('<2sBBII', b'\xa5\x5a', 2, kind, seq, us) + payload.ljust(18, b'\0')
    return b + struct.pack('<H', binascii.crc_hqx(b, 0xFFFF))


class FastTests(unittest.TestCase):
    def test_boot_log_after_binary_and_partial_frame(self):
        d = Decoder()
        f = packet(3, 0, 100, struct.pack('<IIIIH', 0, 0, 0, 4, 1))
        self.assertEqual(len(d.feed(f)), 1)
        d.feed(f[:9] + b'\r\n# Bow hardware V3 fast timer-DMA v8 FIFOdiag\r\n')
        self.assertTrue(any('Bow hardware V3' in message for message in d.messages))
        self.assertEqual(len(d.feed(f)), 1)

    def test_imu_diagnostic_packet(self):
        data = packet(5, 0, 100, struct.pack('<BBHIIIH', 0xff, 1, 65535, 3, 1, 80, 65535))
        f = Decoder().feed(data)[0]
        self.assertEqual(f['fifo_count'], 65535)
        self.assertEqual(f['imu_status'], 255)
        self.assertEqual(f['observed_us'], 80)
        tracking = Tracking()
        tracking.update(f)
        self.assertEqual(tracking.counts, {'I': 0, 'H': 0, 'F': 0})

    def test_fragmentation_and_negative_axes(self):
        frame = packet(1, 7, 1234, struct.pack('<6hH', 8192,-8192,0,16384,-16384,0,65530))
        d = Decoder(); out=[]
        for byte in b'# boot\r\n' + frame:
            out += d.feed(bytes([byte]))
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0]['raw'], (131072,-131072,0,262144,-262144,0))
        self.assertEqual(out[0]['sensor_us16'], 65530)
        self.assertEqual(d.messages, ['# boot\r'])

    def test_corruption_and_resynchronization(self):
        a = packet(2, 0, 500, struct.pack('<iHBBII', -123456, 0xFF60, 0,32,100,400))
        b = bytearray(a); b[15] ^= 4
        d = Decoder()
        out = d.feed(b'garbage' + b + a[:13] + a)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0]['raw'], -123456)
        self.assertEqual(out[0]['age_us'], 400)
        self.assertGreaterEqual(d.crc_errors, 2)

    def test_wraps_gaps_and_reset(self):
        t = Tracking(1600)
        f = {'type':1, 'seq':0xFFFFFFFF, 'read_us':0xFFFFFF00, 'sensor_us16':65000}
        t.update(f)
        self.assertEqual(t.update(dict(f, seq=0, read_us=369, sensor_us16=89)), 625)
        self.assertEqual(t.epoch, 0)
        self.assertEqual(t.seq_errors, 0)
        self.assertAlmostEqual(t.rates()['I'], 1600)
        t.update(dict(f,seq=2,read_us=3494,sensor_us16=3214))
        self.assertEqual(t.seq_errors, 1)
        self.assertEqual(t.timing_errors, 1)
        t.update(dict(f,seq=0,read_us=100,sensor_us16=100))
        self.assertEqual(t.epoch, 1)
        self.assertEqual(t.counts['I'], 1)

    def test_mixed_rates(self):
        t = Tracking(1600); d=Decoder(); seq=0
        events=[]
        for i in range(160):
            events.append((i*625, packet(1,i,i*625,struct.pack('<6hH',1,2,3,4,5,6,(i*625)%65536))))
        for i in range(200):
            events.append((i*500+400, packet(2,i,i*500+400,struct.pack('<iHBBII',100,0xFF00,i%2,32,i*500,400))))
        for _,p in sorted(events):
            t.update(d.feed(p)[0])
        self.assertEqual(t.total, {'I':160,'H':100,'F':100})
        self.assertEqual(t.rates(), {'I':1600,'H':1000,'F':1000})
        self.assertEqual(t.seq_errors, 0)

    def test_3200hz_clock_through_wrap(self):
        tracker = Tracking()
        for i in range(3201):
            us = (625*i)//2
            tracker.update(dict(type=1, seq=i, read_us=us, sensor_us16=us & 0xffff))
        self.assertEqual(tracker.rates()['I'], 3200)
        self.assertEqual(tracker.timing_errors, 0)
        self.assertEqual(tracker.imu_time['imu_time_segment'], 0)
        self.assertEqual(tracker.imu_time['sensor_us_unwrapped'], 1000000)

    def test_units(self):
        a, g = imu_units((16384, -16384, 524286, 131072, -131072, -524286))
        self.assertEqual(a[:2], (1, -1))
        self.assertAlmostEqual(a[2], 32 - 2 / 16384)
        self.assertEqual(g[:2], (1000, -1000))
        self.assertAlmostEqual(g[2], -4000 + 2 * 4000 / 524288)

    def test_hires_low_bits_and_sign(self):
        payload = struct.pack('<6hH', -1, 0, 32767, -32768, -1, 0, 123) + bytes([0xE2, 0x6F, 0xE9])
        f = Decoder().feed(packet(1, 0, 1, payload))[0]
        self.assertEqual(f['raw'], (-2, 6, 524286, -524286, -1, 9))
        self.assertEqual(f['sensor_us16'], 123)

    def test_reject_old_protocol(self):
        b = bytearray(packet(1, 0, 1, b'')); b[2] = 1
        b[30:] = struct.pack('<H', binascii.crc_hqx(b[:30], 0xFFFF))
        self.assertEqual(Decoder().feed(b), [])

    def test_status(self):
        f=Decoder().feed(packet(3,0,1,struct.pack('<IIIIH',4,5,6,8,3)))[0]
        self.assertEqual((f['tx_drops'],f['missed_slots'],f['max_late_us'],f['faults'],f['alive']),(4,5,6,8,3))


if __name__ == '__main__':
    unittest.main()
