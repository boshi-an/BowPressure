"""Read the shared Bow-v2/v3 version-2 binary UART protocol."""

from __future__ import annotations

import binascii
from collections import deque
from dataclasses import dataclass, replace
import struct
import threading
import time


@dataclass(frozen=True)
class BowSample:
    seq: int
    mcu_read_us: int
    raw24: int
    status: int
    gain: int
    conversion_start_us: int
    ready_after_us: int
    packet_hex: str
    received_monotonic_s: float = 0.0
    received_unix_s: float = 0.0
    epoch: int = 0

    def millivolts(self, avdd_v: float) -> float:
        return self.raw24 * avdd_v * 1000 / (8388608 * self.gain)


class BowDecoder:
    """Resynchronize on valid 32-byte frames; retain only healthy full bridges.

    ADC H/F packets share their sequence counter. Check sequence continuity
    before filtering H so the normal H/F alternation is not counted as loss.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.invalid_adc = 0
        self.faulted_fullbridge = 0
        self.saturated_fullbridge = 0
        self.adc_sequence_gaps = 0
        self.epoch = 0
        self._last_adc_seq = None
        self._last_adc_us = None

    def diagnostics(self) -> dict:
        return {key: getattr(self, key) for key in (
            "crc_errors", "discarded_bytes", "invalid_adc",
            "faulted_fullbridge", "saturated_fullbridge",
            "adc_sequence_gaps", "epoch",
        )}

    def feed(self, data: bytes) -> list[BowSample]:
        self.buffer.extend(data)
        samples = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(b"\xa5\x5a")
            if start < 0:
                count = len(self.buffer) - int(self.buffer[-1] == 0xA5)
                self.discarded_bytes += count
                del self.buffer[:count]
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 32:
                break
            frame = bytes(self.buffer[:32])
            if (frame[2] != 2 or frame[3] not in (1, 2, 3, 4, 5)
                    or binascii.crc_hqx(frame[:30], 0xFFFF)
                    != int.from_bytes(frame[30:], "little")):
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            del self.buffer[:32]
            if frame[3] != 2:
                continue
            seq, read_us = struct.unpack_from("<II", frame, 4)
            raw, status, channel, gain, start_us, age_us = struct.unpack_from(
                "<iHBBII", frame, 12
            )
            if self._last_adc_us is not None:
                delta = (read_us - self._last_adc_us) & 0xFFFFFFFF
                if delta > 0x80000000:
                    self.epoch += 1
                    self._last_adc_seq = None
            if (self._last_adc_seq is not None
                    and (seq - self._last_adc_seq) & 0xFFFFFFFF != 1):
                self.adc_sequence_gaps += 1
            self._last_adc_seq, self._last_adc_us = seq, read_us
            if channel not in (0, 1) or gain != 32 or not -8388608 <= raw < 8388608:
                self.invalid_adc += 1
                continue
            if channel != 1:
                continue
            # Firmware's active-low ADC fault flags; bit 0 is data-ready.
            if ((~(status >> 8)) & 0xFE) or not (status & 0x0100):
                self.faulted_fullbridge += 1
                continue
            if raw in (-8388608, 8388607):
                self.saturated_fullbridge += 1
                continue
            samples.append(BowSample(
                seq, read_us, raw, status, gain, start_us, age_us,
                frame.hex(), epoch=self.epoch,
            ))
        return samples


class BowStream:
    """Continuously drain the serial port, including during operator prompts.

    Timestamps are host receipt times, not synchronized device sample times.
    A bounded history permits matching around a force request's midpoint.
    """

    def __init__(self, port, history_size: int = 10000):
        self.port = port
        self.decoder = BowDecoder()
        self.samples = deque(maxlen=history_size)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._error = None
        self._thread = threading.Thread(target=self._read, name="bow-reader", daemon=True)

    def __enter__(self):
        self.port.reset_input_buffer()
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2)

    def _read(self):
        try:
            while not self._stop.is_set():
                chunk = self.port.read(min(4096, max(1, self.port.in_waiting)))
                monotonic, unix = time.monotonic(), time.time()
                with self._condition:
                    decoded = self.decoder.feed(chunk)
                    # Never match a pre-reset sample with a post-reset query.
                    if self.samples and self.samples[-1].epoch != self.decoder.epoch:
                        self.samples.clear()
                    self.samples.extend(
                        replace(s, received_monotonic_s=monotonic, received_unix_s=unix)
                        for s in decoded if s.epoch == self.decoder.epoch
                    )
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def nearest(self, target: float, *, since: float, max_skew: float,
                timeout: float) -> BowSample:
        """Wait for receipt time to cross target, then select the nearest sample."""
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                if self._error:
                    raise RuntimeError(f"Bow serial reader failed: {self._error}") from self._error
                candidates = [s for s in self.samples if s.received_monotonic_s >= since]
                if candidates and candidates[-1].received_monotonic_s >= target:
                    sample = min(candidates, key=lambda s: abs(s.received_monotonic_s - target))
                    skew = abs(sample.received_monotonic_s - target)
                    if skew > max_skew:
                        raise TimeoutError(
                            f"Bow/force host timing skew {skew:.4f}s exceeds {max_skew:.4f}s"
                        )
                    return sample
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No fresh, healthy full-bridge data; check bow port/baud/wiring")
                self._condition.wait(min(remaining, 0.05))

    def diagnostics(self) -> dict:
        with self._condition:
            return self.decoder.diagnostics()
