#!/usr/bin/env python3
"""Generate forward 25fps non-drop LTC: 48kHz mono PCM16, start 01:00:00:00.
Bit layout: SMPTE LTC / libltc documentation. No third-party dependencies.
Digital amplitude is NOT a promise of analog output voltage.
"""
import argparse
from array import array
from pathlib import Path
import sys
import wave


def frame_bytes(index):
    index %= 24 * 3600 * 25
    seconds, f = divmod(index, 25)
    minutes, s = divmod(seconds, 60)
    h, m = divmod(minutes, 60)
    b = bytearray(10)
    for j, v in zip((0, 2, 4, 6), (f, s, m, h)):
        b[j] = v % 10
        b[j+1] = v // 10
    b[8:10] = b'\xfc\xbf'
    # 25fps phase correction is bit59. Even zero count = even one count.
    if sum(x.bit_count() for x in b) % 2:
        b[7] |= 8
    return b


def samples(seconds=60, amplitude=.25, start=90000):
    level = -1
    result = array('h')
    peak = round(32767 * amplitude)
    for index in range(start, start + seconds * 25):
        for byte in frame_bytes(index):
            for shift in range(8):
                level = -level  # Every bit boundary, 24 samples per bit.
                result.extend([level * peak] * 12)
                if (byte >> shift) & 1:
                    level = -level
                result.extend([level * peak] * 12)
    if sys.byteorder != 'little':
        result.byteswap()
    return result


def write_wav(path, seconds=60, amplitude=.25):
    if seconds <= 0 or not 0 < amplitude <= 1:
        raise ValueError('seconds > 0 and 0 < amplitude <= 1 required')
    pcm = samples(seconds, amplitude)
    with Path(path).open('xb') as file:
        with wave.open(file, 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(48000)
            w.writeframes(pcm.tobytes())


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('--seconds', type=int, default=60)
    p.add_argument('--amplitude', type=float, default=.25)
    a = p.parse_args()
    write_wav(a.output, a.seconds, a.amplitude)
    print(f'{a.output}: 25fps, 01:00:00:00, {a.seconds}s, 48kHz mono PCM16')
