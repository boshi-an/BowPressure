#!/usr/bin/env python3
# This script calibrates the bow pressure sensor.
# Actual force = f(raw_value, bowing_position)
# By changing the bowing position and applying known force, we can fit the function.
#
# The calibration process is done as a double loop:
# 1. Outer loop: change the bowing position
# 2. Inner loop: change the force

import json
import serial
import sys
import time
from datetime import datetime
from typing import Dict, Any

MAX_FORCE = 2.0
NUM_SAMPLES_PER_POSITION = 20
START_LENGTH = 0.01
END_LENGTH = 0.64
BOW_LENGTH = 0.65
STEP_SIZE = 0.1

_UNIT_SCALE_TO_NEWTON = {
    "55": 9.80665,              # kg -> N
    "56": 4.4482216152605,      # lb -> N
    "57": 0.00980665,           # g -> N
    "58": 0.27801385095378125,  # oz -> N
    "59": 1.0,                  # N -> N
}


def _parse_force_packet(packet: str) -> float:
    if len(packet) != 16:
        raise ValueError(f"Expected 16 characters, got {len(packet)}: {packet!r}")
    if packet[1] != "4" or packet[2] != "1":
        raise ValueError(f"Unexpected packet header: {packet!r}")

    unit_code = packet[3:5]
    polarity = packet[5]
    decimal_pos = packet[6]
    digits = packet[7:15]

    if unit_code not in _UNIT_SCALE_TO_NEWTON:
        raise ValueError(f"Unknown unit code: {unit_code!r}")
    if polarity not in {"0", "1"}:
        raise ValueError(f"Invalid polarity: {polarity!r}")
    if decimal_pos not in {"0", "1", "2", "3"}:
        raise ValueError(f"Invalid decimal position: {decimal_pos!r}")
    if not digits.isdigit():
        raise ValueError(f"Invalid digits: {digits!r}")

    value = int(digits) / (10 ** int(decimal_pos))
    if polarity == "1":
        value = -value

    return value * _UNIT_SCALE_TO_NEWTON[unit_code]


class ForceReader:
    """Reads force from the scale, discarding stale buffer data on each call."""

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 1.0,
        bytesize: int = serial.EIGHTBITS,
        parity: str = serial.PARITY_NONE,
        stopbits: int = serial.STOPBITS_ONE,
    ):
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
        )

    def read_fresh(self) -> Dict[str, Any]:
        """Flush stale data, then return the next valid force reading."""
        self._ser.reset_input_buffer()
        while True:
            raw = self._ser.read(16)
            if len(raw) != 16:
                continue
            try:
                packet = raw.decode("ascii")
                force_N = _parse_force_packet(packet)
            except (UnicodeDecodeError, ValueError):
                continue
            return {"raw": packet, "force_N": force_N}

    def close(self):
        self._ser.close()


class BowReader:
    """Reads mV from realtime-arduino ads1263_bno055_reader serial DATA lines."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 230400, timeout: float = 1.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._sample_count = 0
        self._error_count = 0
        time.sleep(0.2)
        self._ser.reset_input_buffer()

    def read_fresh(self) -> Dict[str, Any]:
        """Flush stale data, then return the next mV reading from a valid DATA line."""
        # Match ForceReader semantics: drop any buffered/stale serial lines and
        # wait for the next line produced after this call.
        self._ser.reset_input_buffer()
        values = []
        last_line = ""
        while True:
            raw_line = self._ser.readline()
            if not raw_line:
                self._error_count += 1
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#") or line.startswith("STATS,"):
                continue
            if not line.startswith("DATA,"):
                continue

            parts = line.split(",")
            if len(parts) not in (10, 11):
                self._error_count += 1
                continue

            try:
                value = float(parts[3])  # mV
            except ValueError:
                self._error_count += 1
                continue
            if abs(value) > 200:
                self._error_count += 1
                continue

            values.append(value)
            last_line = line
            if len(values) < 10:
                continue

            mean_value = sum(values) / len(values)
            self._sample_count += len(values)
            return {
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "raw_line": last_line,
                "value": mean_value,
                "samples": self._sample_count,
                "errors": self._error_count,
            }

    def close(self):
        self._ser.close()


force_reader = ForceReader("/dev/ttyUSB0", 9600)
bow_reader = BowReader("/dev/ttyACM1", 230400)

current_bowing_position = START_LENGTH


def initialize_bowing_position():
    # Need to be implemented.
    global current_bowing_position
    print(f"Initializing bowing position to {START_LENGTH}")
    input()
    current_bowing_position = START_LENGTH


def change_bowing_position():
    # Need to be implemented.
    global current_bowing_position
    current_bowing_position += STEP_SIZE
    if current_bowing_position > END_LENGTH:
        return False
    print(f"Changing bowing position to {current_bowing_position}")
    input()
    return True


def query_force():
    global force_reader
    return force_reader.read_fresh()


def query_bow_readings():
    global bow_reader
    return bow_reader.read_fresh()


def save_datapoints(datapoints: list, filename: str):
    """Append calibration datapoints to the given file as JSON lines."""
    if not datapoints:
        return

    with open(filename, "a", encoding="utf-8") as f:
        for dp in datapoints:
            f.write(json.dumps(dp, ensure_ascii=False))
            f.write("\n")


def run_calibration(save_file: str):
    current_force_min = float("inf")
    current_force_max = float("-inf")
    current_datapoints = []

    while True:
        real_data = query_force()
        real_applied_force = real_data["force_N"]
        bow_readings_data = query_bow_readings()
        bow_readings = bow_readings_data["value"]

        print(f"real_applied_force: {real_applied_force}, bow_readings(mV): {bow_readings}")

        if real_applied_force is None or bow_readings is None:
            continue

        datapoint = {
            "bowing_position": current_bowing_position,
            "bowing_fraction": current_bowing_position / BOW_LENGTH,
            "real_applied_force": real_applied_force,
            "bow_readings": bow_readings,
            "bow_raw_line": bow_readings_data["raw_line"],
            "force_raw_packet": real_data["raw"],
        }

        current_force_min = min(current_force_min, real_applied_force)
        current_force_max = max(current_force_max, real_applied_force)
        current_datapoints.append(datapoint)

        if current_force_max > MAX_FORCE and len(current_datapoints) > NUM_SAMPLES_PER_POSITION:
            if not change_bowing_position():
                save_datapoints(current_datapoints, save_file)
                break
            else:
                save_datapoints(current_datapoints, save_file)
                current_force_min = float("inf")
                current_force_max = float("-inf")
                current_datapoints = []


def main():
    save_file = sys.argv[1] if len(sys.argv) > 1 else "calibration_data.json"
    initialize_bowing_position()
    try:
        run_calibration(save_file)
    finally:
        force_reader.close()
        bow_reader.close()


if __name__ == "__main__":
    main()
