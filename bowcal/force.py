"""Read-only SH force gauge protocol from manuals/Modbus_RTU规约_SH_通信协议.pdf."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time


# PDF page 7: N, kN, gf, kgf, tf, lbf, klbf. Use standard gravity.
UNITS = {
    0: ("N", 1.0),
    1: ("kN", 1000.0),
    2: ("gf", 0.00980665),
    3: ("kgf", 9.80665),
    4: ("tf", 9806.65),
    5: ("lbf", 4.4482216152605),
    6: ("klbf", 4448.2216152605),
}


class ProtocolError(ValueError):
    """A response is corrupt, unexpected, or unsuitable for calibration."""


def crc16(data: bytes) -> int:
    """Modbus CRC: init FFFF, reflected polynomial A001, low byte first."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def with_crc(data: bytes) -> bytes:
    return data + struct.pack("<H", crc16(data))


def read_request(address: int, start: int = 0, count: int = 12) -> bytes:
    if not 1 <= address <= 255:
        raise ValueError("Force address must be 1..255 (as specified by the SH manual)")
    if not 1 <= count <= 125 or not 0 <= start <= 65536 - count:
        raise ValueError("Invalid Modbus register range")
    return with_crc(struct.pack(">BBHH", address, 3, start, count))


def response_payload(frame: bytes, address: int, count: int) -> bytes:
    if len(frame) < 5 or crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
        raise ProtocolError("Force response has an invalid CRC or length")
    if frame[0] != address:
        raise ProtocolError(f"Force response address {frame[0]} != requested {address}")
    if frame[1] == 0x83:
        if len(frame) != 5:
            raise ProtocolError("Invalid Modbus exception length")
        raise ProtocolError(f"Force gauge returned Modbus exception 0x{frame[2]:02X}")
    if frame[1] != 3 or frame[2] != 2 * count or len(frame) != 5 + 2 * count:
        raise ProtocolError("Unexpected force response function, byte count, or length")
    return frame[3:-2]


@dataclass(frozen=True)
class ForceSample:
    value: float
    unit_code: int
    unit: str
    newtons: float
    request_monotonic_s: float
    response_monotonic_s: float
    response_unix_s: float
    request_hex: str
    response_hex: str

    @property
    def midpoint(self) -> float:
        return (self.request_monotonic_s + self.response_monotonic_s) / 2


class ForceGauge:
    def __init__(self, port, address: int = 1, timeout: float = 1.0):
        self.port = port
        self.address = address
        self.timeout = timeout
        # 0..11 includes the current reading and its unit in one transaction.
        self.request = read_request(address)
        self._last_end = time.monotonic()
        self._silent_interval = max(0.00175, 3.5 * 10 / port.baudrate)

    def _read_exact(self, count: int, deadline: float) -> bytes:
        result = bytearray()
        old_timeout = self.port.timeout
        try:
            while len(result) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        "Force gauge response timed out; check port, address, baud, and wiring"
                    )
                self.port.timeout = remaining
                data = self.port.read(count - len(result))
                result.extend(data)
        finally:
            self.port.timeout = old_timeout
        return bytes(result)

    def read(self) -> ForceSample:
        delay = self._silent_interval - (time.monotonic() - self._last_end)
        if delay > 0:
            time.sleep(delay)
        self.port.reset_input_buffer()
        started = time.monotonic()
        try:
            if self.port.write(self.request) != len(self.request):
                raise ProtocolError("Incomplete force request write")
            deadline = started + self.timeout
            head = self._read_exact(3, deadline)
            if head[0] != self.address or head[1] not in (3, 0x83):
                raise ProtocolError("Unexpected force response address/function (or adapter echo)")
            if head[1] == 0x83:
                rest = self._read_exact(2, deadline)
            elif head[2] == 24:
                rest = self._read_exact(26, deadline)
            else:
                raise ProtocolError(f"Expected 24 force payload bytes, received count {head[2]}")
            ended, unix = time.monotonic(), time.time()
            frame = head + rest
            payload = response_payload(frame, self.address, 12)
            value = struct.unpack_from(">f", payload, 0)[0]
            unit_code = struct.unpack_from(">H", payload, 22)[0]
            if not math.isfinite(value):
                raise ProtocolError("Force gauge returned NaN or infinity")
            if unit_code not in UNITS:
                raise ProtocolError(f"Unknown force unit code {unit_code}")
            unit, scale = UNITS[unit_code]
            return ForceSample(value, unit_code, unit, value * scale, started, ended,
                               unix, self.request.hex(), frame.hex())
        finally:
            self._last_end = time.monotonic()
