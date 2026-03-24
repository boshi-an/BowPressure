# This script calibrates the bow pressure sensor.
# Actual force = f(raw_value, bowing_position)
# By changing the bowing position and applying known force, we can fit the function.

# The calibration process is done as a double loop:
# 1. Outer loop: change the bowing position
# 2. Inner loop: change the force

import json
import serial
import sys
import time
from datetime import datetime
from typing import Iterator, Dict, Any

MAX_FORCE = 2.0
NUM_SAMPLES_PER_POSITION = 10
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


def _modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _build_write_request(addr: int, reg: int, value: int) -> bytes:
    buf = bytearray(8)
    buf[0] = addr & 0xFF
    buf[1] = 0x06
    buf[2] = (reg >> 8) & 0xFF
    buf[3] = reg & 0xFF
    buf[4] = (value >> 8) & 0xFF
    buf[5] = value & 0xFF
    crc = _modbus_crc16(buf[:6])
    buf[6] = crc & 0xFF
    buf[7] = (crc >> 8) & 0xFF
    return bytes(buf)


def _write_register(ser: serial.Serial, addr: int, reg: int, value: int) -> bool:
    req = _build_write_request(addr, reg, value)
    ser.reset_input_buffer()
    ser.write(req)
    resp = ser.read(8)
    if len(resp) < 8:
        return False
    crc_recv = resp[6] | (resp[7] << 8)
    return crc_recv == _modbus_crc16(resp[:6])


def _build_read_request(addr: int, start_reg: int, count: int) -> bytes:
    buf = bytearray(8)
    buf[0] = addr & 0xFF
    buf[1] = 0x03
    buf[2] = (start_reg >> 8) & 0xFF
    buf[3] = start_reg & 0xFF
    buf[4] = (count >> 8) & 0xFF
    buf[5] = count & 0xFF
    crc = _modbus_crc16(buf[:6])
    buf[6] = crc & 0xFF
    buf[7] = (crc >> 8) & 0xFF
    return bytes(buf)


def _parse_bow_response(resp: bytes) -> int | None:
    # Expected Modbus FC03 response for 2 registers:
    # addr(1) + fc(1) + bytecount(1=0x04) + data(4) + crc(2) = 9 bytes
    if len(resp) < 9:
        return None
    if resp[1] != 0x03 or resp[2] != 0x04:
        return None
    crc_recv = resp[-2] | (resp[-1] << 8)
    crc_calc = _modbus_crc16(resp[:-2])
    if crc_recv != crc_calc:
        return None
    reg0 = (resp[3] << 8) | resp[4]
    reg1 = (resp[5] << 8) | resp[6]
    value = (reg1 << 16) | reg0
    # Interpret as signed 32-bit
    if value & 0x80000000:
        value -= 0x100000000
    return value


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
    """Reads bow pressure from the CMCU-06A over Modbus RTU."""

    def __init__(self, port: str = "/dev/ttyACM0", timeout: float = 0.3):
        self._req = _build_read_request(addr=0x01, start_reg=0x0000, count=0x0002)
        self._ser = serial.Serial(
            port=port,
            baudrate=19200,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._sample_count = 0
        self._error_count = 0

        addr = 0x01
        time.sleep(0.2)
        if not _write_register(self._ser, addr, 23, 1):
            print("[warn] failed to disable write protection")
        time.sleep(0.1)
        if _write_register(self._ser, addr, 17, 10):
            print("  Average filter -> 10")
        else:
            print("[warn] failed to set average filter")
        time.sleep(0.1)
        _write_register(self._ser, addr, 23, 0)
        time.sleep(1.1)

    def read_fresh(self) -> Dict[str, Any]:
        """Flush stale data, then return the next bow pressure reading."""
        while True:
            self._ser.reset_input_buffer()

            written = self._ser.write(self._req)
            if written != len(self._req):
                self._error_count += 1
                continue

            resp = self._ser.read(9)
            value = _parse_bow_response(resp)
            ts = datetime.now().isoformat(timespec="milliseconds")

            if value is not None:
                self._sample_count += 1
            else:
                self._error_count += 1

            return {
                "timestamp": ts,
                "raw_bytes": resp,
                "value": value,
                "samples": self._sample_count,
                "errors": self._error_count,
            }

    def close(self):
        self._ser.close()

force_reader = ForceReader("/dev/ttyUSB0", 9600)
bow_reader = BowReader("/dev/ttyACM0")

# Example:
# for reading in stream_force_readings("COM3", 9600):
#     print(f'{reading["value"]} {reading["unit"]}   raw={reading["raw"]}')

current_bowing_position = START_LENGTH

def initialize_bowing_position() :

    # Need to be implemented.

    global current_bowing_position

    print(f"Initializing bowing position to {START_LENGTH}")
    input()

    current_bowing_position = START_LENGTH

def change_bowing_position() :

    # Need to be implemented.

    global current_bowing_position

    current_bowing_position += STEP_SIZE
    if current_bowing_position > END_LENGTH :
        return False

    print(f"Changing bowing position to {current_bowing_position}")
    input()

    return True

def query_force() :

    global force_reader
    return force_reader.read_fresh()

def query_bow_readings() :

    global bow_reader
    return bow_reader.read_fresh()

def save_datapoints(datapoints: list, filename: str) :
    """Append calibration datapoints to the given file as JSON lines."""
    if not datapoints:
        return

    with open(filename, "a", encoding="utf-8") as f:
        for dp in datapoints:
            f.write(json.dumps(dp, ensure_ascii=False))
            f.write("\n")

def run_calibration(save_file: str) :

    current_force_min = float('inf')
    current_force_max = float('-inf')
    current_datapoints = []

    while True :

        real_data = query_force()
        real_applied_force = real_data["force_N"]
        bow_readings = query_bow_readings()["value"]

        print(f"real_applied_force: {real_applied_force}, bow_readings: {bow_readings}")

        if real_applied_force is None or bow_readings is None :
            continue

        datapoint = {
            "bowing_position": current_bowing_position,
            "bowing_fraction": current_bowing_position / BOW_LENGTH,
            "real_applied_force": real_applied_force,
            "bow_readings": bow_readings,
        }
        
        current_force_min = min(current_force_min, real_applied_force)
        current_force_max = max(current_force_max, real_applied_force)
        current_datapoints.append(datapoint)

        if current_force_max > MAX_FORCE and len(current_datapoints) > NUM_SAMPLES_PER_POSITION:
            if not change_bowing_position() :
                save_datapoints(current_datapoints, save_file)
                break
            else :
                save_datapoints(current_datapoints, save_file)
                current_force_min = float('inf')
                current_force_max = float('-inf')
                current_datapoints = []

def main() :

    save_file = sys.argv[1] if len(sys.argv) > 1 else "calibration_data.json"

    initialize_bowing_position()
    run_calibration(save_file)


if __name__ == "__main__":

    main()
