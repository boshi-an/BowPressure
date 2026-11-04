"""Decode generated audio through the exact MCU C decoder, without hardware."""
import shutil
import struct
import subprocess
import tempfile
import unittest
import wave
from array import array
from pathlib import Path
from generate_ltc import samples, frame_bytes
from read import Decoder, Tracking
from test_read_fast import packet


def intervals(pcm):
    edges = [n for n in range(1, len(pcm)) if (pcm[n] > 0) != (pcm[n-1] > 0)]
    return [round((b-a)*1e6/48000) for a, b in zip(edges, edges[1:])]


@unittest.skipUnless(shutil.which('cc'), 'native compiler needed')
class LtcTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.exe = str(Path(cls.temp.name)/'ltc')
        root = Path(__file__).resolve().parents[1]
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        str(root/'tests/ltc_decode_harness.c'), '-o', cls.exe], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def decode(self, ds):
        return subprocess.run([self.exe], input='\n'.join(map(str, ds)), text=True,
                              capture_output=True, check=True, timeout=10).stdout.splitlines()

    def test_delivered_wav(self):
        path = Path(__file__).resolve().parents[1]/'test_signals/ltc_25fps_60s.wav'
        with wave.open(str(path)) as w:
            self.assertEqual((w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()),
                             (1, 2, 48000, 60*48000))
            pcm = array('h', w.readframes(w.getnframes()))
        decoded = self.decode(intervals(pcm))
        expected = [f'01:00:{s:02}:{f:02}' for s in range(60) for f in range(25)]
        # Decoder closes a bit at the following boundary: final frame awaits
        # the next edge beyond the WAV. First two frames acquire sync/continuity.
        self.assertEqual(decoded, expected[2:-1])

    def test_midnight_jitter_and_polarity(self):
        pcm = samples(3, start=24*3600*25-25)
        ds = intervals(pcm)
        self.assertEqual(ds, intervals([-v for v in pcm]))
        # +/-20us ISR/audio jitter, also exercises decoder uint elapsed arithmetic.
        out = self.decode([dt + (20 if j%2 else -20) for j, dt in enumerate(ds)])
        self.assertIn('23:59:59:24', out)
        self.assertIn('00:00:00:00', out)
        self.assertEqual(out[-1], '00:00:01:23')

    def test_dropout_recovery_and_reject_noise(self):
        self.assertEqual(self.decode([50, 80, 100, 1000]*100), [])
        ds = intervals(samples(4))
        ds[2500:2510] = [5000]
        out = self.decode(ds)
        self.assertLess(len(out), 98)
        self.assertEqual(out[-1], '01:00:03:23')

    def test_layout_and_parity(self):
        # Independent known BCD bytes for 12:34:56:24; user bits zero.
        b = frame_bytes(((12*60+34)*60+56)*25+24)
        self.assertEqual(b[:7], bytes([4, 2, 6, 5, 4, 3, 2]))
        self.assertEqual(b[7]&3, 1)
        self.assertEqual(b[8:], b'\xfc\xbf')
        self.assertEqual(sum(x.bit_count() for x in b)%2, 0)

    def test_uart_ltc_frame(self):
        payload = struct.pack('<6BIII', 1, 2, 3, 24, 25, 1, 1234, 2, 0)
        f = Decoder().feed(packet(4, 42, 100000, payload))[0]
        self.assertEqual(f['timecode'], '01:02:03:24')
        self.assertTrue(f['locked'])
        self.assertEqual(f['edges'], 1234)
        t=Tracking(); t.update(f)
        self.assertEqual(t.epoch, 0)
        self.assertEqual(t.total, {'I':0, 'H':0, 'F':0})
