#!/usr/bin/env python3
"""Print live external force readings without connecting a bow."""

import argparse
from datetime import datetime
import sys
import time

from bowcal.calibration import open_serial, positive_float, positive_int
from bowcal.force import ForceGauge


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", "--force-port", dest="port", help="force gauge serial port")
    parser.add_argument("--baud", "--force-baud", dest="baud", type=positive_int,
                        default=9600, help="serial baud rate (8N1)")
    parser.add_argument("--address", "--force-address", dest="address", type=int,
                        default=1, help="Modbus device address")
    parser.add_argument("--poll-hz", type=positive_float, default=10.0,
                        help="maximum polling rate")
    parser.add_argument("--timeout", type=positive_float, default=1.0,
                        help="force response timeout in seconds")
    args = parser.parse_args(argv)
    if not args.list_ports:
        if not args.port:
            parser.error("--port is required")
        if not 1 <= args.address <= 255:
            parser.error("--address must be 1..255")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Install dependencies: python -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    try:
        if args.list_ports:
            for port in list_ports.comports():
                print(f"{port.device}\t{port.description}")
            return 0
        with open_serial(serial, args.port, args.baud, args.timeout) as port:
            gauge = ForceGauge(port, args.address, args.timeout)
            print(f"Monitoring {args.port}, {args.baud} baud, address {args.address}. "
                  "Press Ctrl+C to stop.", flush=True)
            while True:
                started = time.monotonic()
                sample = gauge.read()
                timestamp = datetime.fromtimestamp(sample.response_unix_s).astimezone().isoformat(
                    timespec="milliseconds"
                )
                roundtrip_ms = (sample.response_monotonic_s - sample.request_monotonic_s) * 1000
                print(f"{timestamp}  {sample.newtons:+.6f} N  "
                      f"(gauge: {sample.value:+.6f} {sample.unit})  "
                      f"response: {roundtrip_ms:.1f} ms", flush=True)
                time.sleep(max(0, 1 / args.poll_hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("\nForce monitor stopped.")
        return 130
    except (OSError, ValueError, serial.SerialException) as exc:
        print(f"Force monitor failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
