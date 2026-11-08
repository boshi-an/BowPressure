"""Operator-guided ten-position calibration and incremental CSV recording."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from .bow import BowStream
from .force import ForceGauge


POSITIONS = tuple((5 + 10 * i) / 100 for i in range(10))
FIELDS = (
    "position_index", "sample_index", "distance_fraction", "distance_m",
    "fullbridge_raw24", "fullbridge_mv", "force_N", "force_value", "force_unit",
    "force_unit_code", "bow_seq", "bow_epoch", "bow_mcu_read_us",
    "bow_conversion_start_us", "bow_ready_after_us", "bow_status_hex", "bow_gain",
    "avdd_v", "bow_received_unix_s", "bow_received_monotonic_s",
    "force_response_unix_s", "force_request_monotonic_s", "force_response_monotonic_s",
    "pair_skew_ms", "force_roundtrip_ms", "bow_packet_hex",
    "force_request_hex", "force_response_hex",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class Recording:
    """Create new files exclusively; preserve accepted rows on abort or error."""

    def __init__(self, output: Path, settings: dict):
        self.output = output
        self.metadata_path = output.with_suffix(".json")
        self.metadata = {
            "schema_version": 1,
            "started_utc": utc_now(),
            "status": "running",
            "settings": settings,
            "distance_fractions": POSITIONS,
            "positions_completed": [],
            "pairs_recorded": 0,
            "pairing": "Nearest bow host receipt to force request/response midpoint",
        }
        self._stack = ExitStack()

    def __enter__(self):
        if self.output.suffix.lower() != ".csv":
            raise ValueError("Output must have a .csv extension")
        for path in (self.output, self.metadata_path):
            if path.exists():
                raise FileExistsError(f"Refusing to overwrite {path}")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.csv_file = self._stack.enter_context(
                self.output.open("x", newline="", encoding="utf-8")
            )
            self.json_file = self._stack.enter_context(
                self.metadata_path.open("x", encoding="utf-8")
            )
            self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
            self.writer.writeheader()
            self.csv_file.flush()
            self.checkpoint()
        except BaseException:
            self._stack.close()
            raise
        return self

    def append(self, row: dict):
        self.writer.writerow(row)
        self.csv_file.flush()
        self.metadata["pairs_recorded"] += 1

    def checkpoint(self):
        self.json_file.seek(0)
        json.dump(self.metadata, self.json_file, indent=2, allow_nan=False)
        self.json_file.write("\n")
        self.json_file.truncate()
        self.json_file.flush()

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type:
                self.metadata["status"] = (
                    "aborted" if issubclass(exc_type, (KeyboardInterrupt, EOFError)) else "failed"
                )
                self.metadata["error"] = str(exc) or exc_type.__name__
            elif len(self.metadata["positions_completed"]) == len(POSITIONS):
                self.metadata["status"] = "complete"
            else:
                self.metadata["status"] = "incomplete"
            self.metadata["finished_utc"] = utc_now()
            self.checkpoint()
        finally:
            self._stack.close()


def run_sequence(bow, gauge, recording: Recording, args, *, prompt=input, report=print):
    """At each position, prompt, settle, then record N freshly queried pairs."""
    previous_bow = None
    try:
        for position_index, fraction in enumerate(POSITIONS, start=1):
            distance_m = fraction * args.bow_length_m if args.bow_length_m else ""
            location = f"{fraction:.2f} of bow length ({fraction:.0%}) from the frog"
            if distance_m != "":
                location += f", {distance_m * 1000:.1f} mm"
            report(f"\nPosition {position_index}/{len(POSITIONS)}: {location}.")
            while True:
                answer = prompt("Place the contact point, then press Enter to record (q to stop): ")
                if answer.strip().lower() == "q":
                    raise KeyboardInterrupt("Stopped by operator")
                if not answer.strip():
                    break
            report(f"Settling for {args.settle_s:g} s...")
            time.sleep(args.settle_s)
            since = time.monotonic()
            force_values = []
            report("Recording: slowly increase and decrease force at this fixed position.")
            for sample_index in range(1, args.samples_per_position + 1):
                cycle_started = time.monotonic()
                force = gauge.read()
                roundtrip_ms = (force.response_monotonic_s - force.request_monotonic_s) * 1000
                if roundtrip_ms > args.max_roundtrip_ms:
                    raise TimeoutError(
                        f"Force roundtrip {roundtrip_ms:.1f} ms exceeds {args.max_roundtrip_ms:g} ms"
                    )
                sample = bow.nearest(force.midpoint, since=since,
                                     max_skew=args.max_skew_ms / 1000, timeout=args.timeout)
                identity = (sample.epoch, sample.seq)
                if identity == previous_bow:
                    raise TimeoutError("Bow sample would be reused; check stream or lower --poll-hz")
                previous_bow = identity
                skew_ms = (sample.received_monotonic_s - force.midpoint) * 1000
                recording.append(dict(
                    position_index=position_index, sample_index=sample_index,
                    distance_fraction=f"{fraction:.2f}", distance_m=distance_m,
                    fullbridge_raw24=sample.raw24, fullbridge_mv=sample.millivolts(args.avdd),
                    force_N=force.newtons, force_value=force.value,
                    force_unit=force.unit, force_unit_code=force.unit_code,
                    bow_seq=sample.seq, bow_epoch=sample.epoch,
                    bow_mcu_read_us=sample.mcu_read_us,
                    bow_conversion_start_us=sample.conversion_start_us,
                    bow_ready_after_us=sample.ready_after_us,
                    bow_status_hex=f"0x{sample.status:04X}", bow_gain=sample.gain,
                    avdd_v=args.avdd, bow_received_unix_s=sample.received_unix_s,
                    bow_received_monotonic_s=sample.received_monotonic_s,
                    force_response_unix_s=force.response_unix_s,
                    force_request_monotonic_s=force.request_monotonic_s,
                    force_response_monotonic_s=force.response_monotonic_s,
                    pair_skew_ms=skew_ms, force_roundtrip_ms=roundtrip_ms,
                    bow_packet_hex=sample.packet_hex, force_request_hex=force.request_hex,
                    force_response_hex=force.response_hex,
                ))
                force_values.append(force.newtons)
                if sample_index == 1 or sample_index % 10 == 0 or sample_index == args.samples_per_position:
                    report(f"  {sample_index}/{args.samples_per_position}: "
                           f"F={force.newtons:+.5f} N, bridge={sample.raw24:+d} counts "
                           f"({sample.millivolts(args.avdd):+.6f} mV), skew={skew_ms:+.1f} ms")
                if sample_index < args.samples_per_position:
                    time.sleep(max(0, 1 / args.poll_hz - (time.monotonic() - cycle_started)))
            recording.metadata["positions_completed"].append(fraction)
            recording.metadata["bow_diagnostics"] = bow.diagnostics()
            recording.checkpoint()
            report(f"Saved {len(force_values)} pairs; force range "
                   f"{min(force_values):+.5f} to {max(force_values):+.5f} N.")
    finally:
        recording.metadata["bow_diagnostics"] = bow.diagnostics()


def positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return value


def nonnegative_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("must be a finite nonnegative number")
    return value


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--bow-port", help="Bow-v2/v3 USB-TTL serial port")
    parser.add_argument("--force-port", help="SH Modbus force gauge serial port")
    parser.add_argument("--bow-baud", type=positive_int, default=3000000)
    parser.add_argument("--force-baud", type=positive_int, default=9600)
    parser.add_argument("--force-address", type=int, default=1)
    parser.add_argument("--samples-per-position", type=positive_int, default=100)
    parser.add_argument("--poll-hz", type=positive_float, default=10.0,
                        help="maximum force polling rate (default: 10)")
    parser.add_argument("--settle-s", type=nonnegative_float, default=1.0)
    parser.add_argument("--timeout", type=positive_float, default=1.0)
    parser.add_argument("--max-skew-ms", type=positive_float, default=50.0)
    parser.add_argument("--max-roundtrip-ms", type=positive_float, default=250.0)
    parser.add_argument("--bow-length-m", type=positive_float,
                        help="usable frog-to-tip length; adds physical distances to prompts/CSV")
    parser.add_argument("--avdd", type=positive_float, default=3.3,
                        help="assumed/measured ADC AVDD for mV conversion (default: 3.3)")
    parser.add_argument("--output", type=Path, default=Path("recordings") / (
        "calibration-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".csv"
    ))
    args = parser.parse_args(argv)
    if args.list_ports:
        return args
    if not args.bow_port or not args.force_port:
        parser.error("--bow-port and --force-port are required")
    if Path(args.bow_port).resolve() == Path(args.force_port).resolve():
        parser.error("bow and force sensor must use different serial ports")
    if not 1 <= args.force_address <= 255:
        parser.error("--force-address must be 1..255")
    if not 2.7 <= args.avdd <= 3.6:
        parser.error("--avdd must be 2.7..3.6 V, matching Bow-v2/v3")
    if args.output.suffix.lower() != ".csv":
        parser.error("--output must have a .csv extension")
    return args


def open_serial(serial_module, name, baud, timeout):
    # Set control lines before opening to avoid toggling connected hardware.
    port = serial_module.Serial(port=None, baudrate=baud, bytesize=8, parity="N",
                                stopbits=1, timeout=0.05, write_timeout=timeout,
                                xonxoff=False, rtscts=False, dsrdtr=False)
    port.dtr = port.rts = False
    port.port = name
    port.open()
    return port


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Install dependencies: python -m pip install -r requirements.txt", file=sys.stderr)
        return 1
    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return 0
    settings = {key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()}
    try:
        with Recording(args.output, settings) as recording:
            with ExitStack() as stack:
                bow_port = stack.enter_context(open_serial(serial, args.bow_port, args.bow_baud, args.timeout))
                force_port = stack.enter_context(open_serial(serial, args.force_port, args.force_baud, args.timeout))
                bow = stack.enter_context(BowStream(bow_port))
                gauge = ForceGauge(force_port, args.force_address, args.timeout)
                print(f"Saving pairs to {args.output.resolve()}")
                print("Use live/track mode on the force gauge, zero it unloaded, and keep bow tension fixed.")
                print("Positions are fractions of usable bow length: 0 = frog, 1 = tip.")
                now = time.monotonic()
                bow.nearest(now, since=now, max_skew=args.timeout, timeout=args.timeout)
                preview = gauge.read()
                print(f"Sensors connected. External force: {preview.newtons:+.5f} N ({preview.unit}).")
                run_sequence(bow, gauge, recording, args)
    except (KeyboardInterrupt, EOFError):
        print(f"\nCalibration stopped; accepted pairs remain in {args.output}", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        print("Any accepted pairs remain saved; use a new output filename to restart.", file=sys.stderr)
        return 1
    print(f"Completed all {len(POSITIONS)} positions; saved "
          f"{recording.metadata['pairs_recorded']} pairs to {args.output}.")
    return 0
