#!/usr/bin/env python3
"""Shared realtime serial reader/parsers for ADS1263(+BNO055) stream."""

from __future__ import annotations

import os
import platform
import time
from dataclasses import dataclass
from typing import Any, Iterator, Optional

import serial

DATA_PART_COUNTS = (4, 5, 10, 11)


@dataclass
class ArduinoDataSample:
    """Parsed DATA row from ads1263_bno055_reader sketch."""

    arduino_us: int
    raw: int
    mv_raw: float
    stream_format: str
    ltc: str = ""
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    gyr_x: float = 0.0
    gyr_y: float = 0.0
    gyr_z: float = 0.0


def classify_data_parts(parts: list[str]) -> Optional[str]:
    """Infer stream format from split DATA fields."""
    if len(parts) in (4, 5):
        return "ads"
    if len(parts) in (10, 11):
        return "ads_imu"
    return None


def parse_data_line(line: str) -> Optional[tuple[str, list[str]]]:
    """Return (stream_format, comma-split parts) for a valid DATA line."""
    if not line.startswith("DATA,"):
        return None
    parts = line.split(",")
    stream_format = classify_data_parts(parts)
    if stream_format is None:
        return None
    return stream_format, parts


def parse_data_parts(parts: list[str], stream_format: str) -> Optional[ArduinoDataSample]:
    """Parse DATA fields into typed values."""
    try:
        arduino_us = int(parts[1])
        raw = int(parts[2])
        mv_raw = float(parts[3])
    except (ValueError, IndexError):
        return None

    ltc = ""
    if stream_format == "ads":
        if len(parts) >= 5:
            ltc = parts[4].strip()
        return ArduinoDataSample(
            arduino_us=arduino_us,
            raw=raw,
            mv_raw=mv_raw,
            stream_format=stream_format,
            ltc=ltc,
        )

    try:
        acc_x = float(parts[4])
        acc_y = float(parts[5])
        acc_z = float(parts[6])
        gyr_x = float(parts[7])
        gyr_y = float(parts[8])
        gyr_z = float(parts[9])
        if len(parts) >= 11:
            ltc = parts[10].strip()
    except (ValueError, IndexError):
        return None

    return ArduinoDataSample(
        arduino_us=arduino_us,
        raw=raw,
        mv_raw=mv_raw,
        stream_format=stream_format,
        ltc=ltc,
        acc_x=acc_x,
        acc_y=acc_y,
        acc_z=acc_z,
        gyr_x=gyr_x,
        gyr_y=gyr_y,
        gyr_z=gyr_z,
    )


class SerialLineGenerator:
    """Blocking generator over decoded, stripped lines from a serial port."""

    def __init__(self, port: str, baud: int, *, post_open_delay_s: float = 0.05, timeout_s: float = 0.1) -> None:
        self._port = port
        self._baud = baud
        self._post_open_delay_s = post_open_delay_s
        self._timeout_s = timeout_s
        self._ser: Optional[serial.Serial] = None

    def __enter__(self) -> "SerialLineGenerator":
        self._ser = serial.Serial(
            self._port,
            self._baud,
            timeout=self._timeout_s,
            write_timeout=self._timeout_s,
        )
        # Optimize USB serial latency on Linux
        if platform.system() == "Linux" and hasattr(self._ser, 'port'):
            try:
                # Set USB latency timer to 1ms (from default 16ms) for FTDI/USB adapters
                os.system(f"echo 1 | sudo tee /sys/bus/usb-serial/devices/{os.path.basename(self._port)}/latency_timer > /dev/null 2>&1")
            except Exception:
                pass  # Silently fail if permission denied or not USB
        time.sleep(self._post_open_delay_s)
        self.reset_input_buffer()
        return self

    def __exit__(self, *exc: Any) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def reset_input_buffer(self) -> None:
        if self._ser is not None:
            self._ser.reset_input_buffer()

    def lines(self, stop_event: Optional[Any] = None) -> Iterator[str]:
        assert self._ser is not None
        while True:
            if stop_event is not None and bool(stop_event.is_set()):
                break
            raw = self._ser.readline()
            if not raw:
                continue
            yield raw.decode("utf-8", errors="replace").strip()
