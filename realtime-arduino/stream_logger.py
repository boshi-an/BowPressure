#!/usr/bin/env python3
"""Log ADS1263 serial stream to CSV with optional live plots."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterator, Optional

import serial


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Log ADS1263 stream from Arduino (ADS-only or ADS+BNO055 combined)."
    )
    p.add_argument("--port", default="/dev/ttyACM1", help="Serial device path")
    p.add_argument("--baud", type=int, default=230400, help="Serial baud rate")
    p.add_argument("--output", default="ads1263_stream.csv", help="CSV output path")
    p.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional duration in seconds (0 means run until Ctrl+C)",
    )
    p.add_argument(
        "--plot",
        action="store_true",
        help="Real-time plot: mV; with IMU stream also acc (mg) and d(gyro)/dt (deg/s²)",
    )
    p.add_argument(
        "--plot-window-s",
        type=float,
        default=10.0,
        help="Time window in seconds for real-time plot",
    )
    return p.parse_args()


@dataclass
class ProcessedSample:
    """One decoded DATA row after processing (host timestamps applied)."""

    arduino_us: int
    raw: int
    mv: float
    csv_row: list
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    aang_x: float = 0.0
    aang_y: float = 0.0
    aang_z: float = 0.0


class StreamDataProcessor:
    """
    Parse DATA comma fields into numeric values and CSV rows.
    Angular acceleration for plotting is d(gyro)/dt (deg/s²). Pass-through otherwise.
    """

    def __init__(self) -> None:
        self._prev_gyr: Optional[tuple[float, float, float]] = None
        self._prev_us_for_aang: Optional[int] = None

    def process(self, parts: list[str], stream_format: str) -> Optional[ProcessedSample]:
        try:
            arduino_us = int(parts[1])
            raw = int(parts[2])
            mv = float(parts[3])
        except (ValueError, IndexError):
            return None

        now_epoch = time.time()
        now_iso = dt.datetime.now().isoformat(timespec="milliseconds")

        acc_x = acc_y = acc_z = 0.0
        aang_x = aang_y = aang_z = float("nan")

        if stream_format == "ads":
            row = [now_iso, f"{now_epoch:.6f}", arduino_us, raw, f"{mv:.3f}"]
            return ProcessedSample(
                arduino_us=arduino_us,
                raw=raw,
                mv=mv,
                csv_row=row,
            )

        try:
            acc_x = float(parts[4])
            acc_y = float(parts[5])
            acc_z = float(parts[6])
            gyr_x = float(parts[7])
            gyr_y = float(parts[8])
            gyr_z = float(parts[9])
        except (ValueError, IndexError):
            return None

        if (
            self._prev_gyr is not None
            and self._prev_us_for_aang is not None
            and arduino_us > self._prev_us_for_aang
        ):
            dt_s = (arduino_us - self._prev_us_for_aang) / 1_000_000.0
            if dt_s > 1e-9:
                aang_x = (gyr_x - self._prev_gyr[0]) / dt_s
                aang_y = (gyr_y - self._prev_gyr[1]) / dt_s
                aang_z = (gyr_z - self._prev_gyr[2]) / dt_s
        self._prev_gyr = (gyr_x, gyr_y, gyr_z)
        self._prev_us_for_aang = arduino_us

        row = [
            now_iso,
            f"{now_epoch:.6f}",
            arduino_us,
            raw,
            f"{mv:.3f}",
            f"{acc_x:.6f}",
            f"{acc_y:.6f}",
            f"{acc_z:.6f}",
            f"{gyr_x:.6f}",
            f"{gyr_y:.6f}",
            f"{gyr_z:.6f}",
        ]
        return ProcessedSample(
            arduino_us=arduino_us,
            raw=raw,
            mv=mv,
            csv_row=row,
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
            aang_x=aang_x,
            aang_y=aang_y,
            aang_z=aang_z,
        )


class CsvStreamSaver:
    """Write stream_logger CSV with header chosen from first DATA format."""

    _HEADER_ADS = [
        "host_iso_time",
        "host_epoch_s",
        "arduino_timestamp_us",
        "raw",
        "mV",
    ]
    _HEADER_ADS_IMU = [
        "host_iso_time",
        "host_epoch_s",
        "arduino_timestamp_us",
        "raw",
        "mV",
        "acc_x_mg",
        "acc_y_mg",
        "acc_z_mg",
        "gyr_x_dps",
        "gyr_y_dps",
        "gyr_z_dps",
    ]

    def __init__(self, path: str) -> None:
        self._path = path
        self._file: Any = None
        self._writer: Optional[Any] = None

    def __enter__(self) -> CsvStreamSaver:
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        return self

    def __exit__(self, *exc: Any) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()

    def write_header(self, stream_format: str) -> None:
        assert self._writer is not None
        if stream_format == "ads":
            self._writer.writerow(self._HEADER_ADS)
        else:
            self._writer.writerow(self._HEADER_ADS_IMU)

    def write_sample(self, sample: ProcessedSample) -> None:
        assert self._writer is not None
        self._writer.writerow(sample.csv_row)

    def flush(self) -> None:
        if self._file is not None:
            self._file.flush()


class SerialLineGenerator:
    """Blocking generator over decoded, stripped lines from a serial port."""

    def __init__(self, port: str, baud: int, *, post_open_delay_s: float = 0.2) -> None:
        self._port = port
        self._baud = baud
        self._post_open_delay_s = post_open_delay_s
        self._ser: Optional[serial.Serial] = None

    def __enter__(self) -> SerialLineGenerator:
        self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
        time.sleep(self._post_open_delay_s)
        self._ser.reset_input_buffer()
        return self

    def __exit__(self, *exc: Any) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def lines(self) -> Iterator[str]:
        assert self._ser is not None
        while True:
            raw = self._ser.readline()
            if not raw:
                continue
            yield raw.decode("utf-8", errors="replace").strip()


class StreamPlotter:
    """Optional matplotlib real-time plots for mV (and IMU-derived traces)."""

    def __init__(self, plot_window_s: float) -> None:
        self._plot_window_s = plot_window_s
        self._plt: Any = None
        self._fig: Any = None
        self._axes: Any = None
        self._line_mv: Any = None
        self._lines_acc: list[Any] = []
        self._lines_aang: list[Any] = []
        self._plot_layout: Optional[str] = None
        self._plot_elapsed: deque[float] = deque()
        self._plot_mv: deque[float] = deque()
        self._plot_acc_x: deque[float] = deque()
        self._plot_acc_y: deque[float] = deque()
        self._plot_acc_z: deque[float] = deque()
        self._plot_aang_x: deque[float] = deque()
        self._plot_aang_y: deque[float] = deque()
        self._plot_aang_z: deque[float] = deque()
        self._update_counter = 0
        self._enabled = True

        try:
            import matplotlib.pyplot as plt_module  # pylint: disable=import-error

            self._plt = plt_module
        except ImportError as exc:
            print(f"[warn] Failed to import matplotlib ({exc}). Plot disabled.")
            self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled and self._plt is not None

    def ensure_layout(self, layout: str) -> None:
        if not self.enabled:
            return
        plt = self._plt
        if self._plot_layout == layout:
            return
        if self._fig is not None:
            plt.close(self._fig)
        self._plot_layout = layout
        plt.ion()
        if layout == "ads":
            self._fig, ax0 = plt.subplots(figsize=(10, 4))
            self._axes = ax0
            (self._line_mv,) = ax0.plot([], [], linewidth=1.0, color="tab:blue")
            ax0.set_xlabel("Elapsed Time (s)")
            ax0.set_ylabel("mV")
            ax0.set_title("ADS1263 (real-time)")
            ax0.grid(True, alpha=0.3)
            self._lines_acc = []
            self._lines_aang = []
            self._fig.tight_layout()
        else:
            self._fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
            self._axes = axs
            (self._line_mv,) = axs[0].plot([], [], linewidth=1.0, color="tab:blue")
            axs[0].set_ylabel("mV")
            axs[0].set_title("ADS1263 + BNO055 (real-time)")
            axs[0].grid(True, alpha=0.3)
            colors = ("tab:red", "tab:green", "tab:purple")
            self._lines_acc = []
            for i, lab in enumerate(("acc_x", "acc_y", "acc_z")):
                (ln,) = axs[1].plot(
                    [], [], linewidth=1.0, color=colors[i], label=f"{lab} (mg)"
                )
                self._lines_acc.append(ln)
            axs[1].set_ylabel("Acceleration (mg)")
            axs[1].legend(loc="upper right", fontsize=8)
            axs[1].grid(True, alpha=0.3)
            self._lines_aang = []
            for i, lab in enumerate(("dω_x/dt", "dω_y/dt", "dω_z/dt")):
                (ln,) = axs[2].plot(
                    [],
                    [],
                    linewidth=1.0,
                    color=colors[i],
                    linestyle="--",
                    label=f"{lab} (deg/s²)",
                )
                self._lines_aang.append(ln)
            axs[2].set_ylabel("Angular accel. (deg/s²)")
            axs[2].set_xlabel("Elapsed Time (s)")
            axs[2].legend(loc="upper right", fontsize=8)
            axs[2].grid(True, alpha=0.3)
            self._fig.tight_layout()

    def on_sample(
        self,
        *,
        stream_format: str,
        first_arduino_us: Optional[int],
        sample: ProcessedSample,
    ) -> None:
        if not self.enabled or first_arduino_us is None:
            return
        plt = self._plt
        assert plt is not None
        arduino_us = sample.arduino_us
        mv = sample.mv
        elapsed_s = (arduino_us - first_arduino_us) / 1_000_000.0
        self._plot_elapsed.append(elapsed_s)
        self._plot_mv.append(mv)
        min_time = max(0.0, elapsed_s - self._plot_window_s)
        while self._plot_elapsed and self._plot_elapsed[0] < min_time:
            self._plot_elapsed.popleft()
            self._plot_mv.popleft()
            if stream_format == "ads_imu":
                self._plot_acc_x.popleft()
                self._plot_acc_y.popleft()
                self._plot_acc_z.popleft()
                self._plot_aang_x.popleft()
                self._plot_aang_y.popleft()
                self._plot_aang_z.popleft()

        if stream_format == "ads_imu":
            self._plot_acc_x.append(sample.acc_x)
            self._plot_acc_y.append(sample.acc_y)
            self._plot_acc_z.append(sample.acc_z)
            self._plot_aang_x.append(
                sample.aang_x if not math.isnan(sample.aang_x) else 0.0
            )
            self._plot_aang_y.append(
                sample.aang_y if not math.isnan(sample.aang_y) else 0.0
            )
            self._plot_aang_z.append(
                sample.aang_z if not math.isnan(sample.aang_z) else 0.0
            )

        self._update_counter += 1
        if self._update_counter < 10:
            return
        self._update_counter = 0

        if stream_format == "ads" and self._line_mv is not None and self._axes is not None:
            self._line_mv.set_data(list(self._plot_elapsed), list(self._plot_mv))
            if self._plot_elapsed:
                self._axes.set_xlim(self._plot_elapsed[0], self._plot_elapsed[-1] + 1e-9)
                y_min = min(self._plot_mv)
                y_max = max(self._plot_mv)
                if y_min == y_max:
                    y_min -= 1.0
                    y_max += 1.0
                pad = 0.05 * (y_max - y_min)
                self._axes.set_ylim(y_min - pad, y_max + pad)
            self._fig.canvas.draw_idle()
            plt.pause(0.001)
        elif (
            stream_format == "ads_imu"
            and self._fig is not None
            and self._axes is not None
            and self._line_mv is not None
            and len(self._lines_acc) == 3
            and len(self._lines_aang) == 3
        ):
            t = list(self._plot_elapsed)
            self._line_mv.set_data(t, list(self._plot_mv))
            self._lines_acc[0].set_data(t, list(self._plot_acc_x))
            self._lines_acc[1].set_data(t, list(self._plot_acc_y))
            self._lines_acc[2].set_data(t, list(self._plot_acc_z))
            self._lines_aang[0].set_data(t, list(self._plot_aang_x))
            self._lines_aang[1].set_data(t, list(self._plot_aang_y))
            self._lines_aang[2].set_data(t, list(self._plot_aang_z))
            if self._plot_elapsed:
                axs = self._axes
                axs[0].set_xlim(self._plot_elapsed[0], self._plot_elapsed[-1] + 1e-9)
                for axi, series in (
                    (axs[0], [self._plot_mv]),
                    (axs[1], [self._plot_acc_x, self._plot_acc_y, self._plot_acc_z]),
                    (axs[2], [self._plot_aang_x, self._plot_aang_y, self._plot_aang_z]),
                ):
                    flat = [x for dq in series for x in dq]
                    if not flat:
                        continue
                    y_min = min(flat)
                    y_max = max(flat)
                    if y_min == y_max:
                        y_min -= 1.0
                        y_max += 1.0
                    pad = 0.05 * (y_max - y_min)
                    axi.set_ylim(y_min - pad, y_max + pad)
            self._fig.canvas.draw_idle()
            plt.pause(0.001)

    def show_blocking(self) -> None:
        if self._plt is not None and self.enabled:
            self._plt.ioff()
            self._plt.show()


def main() -> int:
    args = parse_args()
    start_wall = time.time()
    sample_count = 0
    bad_lines = 0
    first_arduino_us: Optional[int] = None
    last_arduino_us: Optional[int] = None
    prev_arduino_us: Optional[int] = None
    mv_sum = 0.0
    mv_sumsq = 0.0
    dt_sum = 0.0
    dt_sumsq = 0.0
    dt_count = 0

    stream_format: Optional[str] = None
    processor = StreamDataProcessor()
    plotter = StreamPlotter(args.plot_window_s) if args.plot else None

    print(f"Opening {args.port} @ {args.baud}...")
    print(f"Logging to {args.output}")
    print("Press Ctrl+C to stop.")

    with SerialLineGenerator(args.port, args.baud) as reader, CsvStreamSaver(
        args.output
    ) as saver:
        try:
            for line in reader.lines():
                if args.duration > 0 and (time.time() - start_wall) >= args.duration:
                    print("Duration reached.")
                    break

                if line.startswith("STATS,"):
                    print(line)
                    continue

                if not line.startswith("DATA,"):
                    print(f"[info] {line}")
                    continue

                parts = line.split(",")
                if len(parts) not in (4, 10):
                    bad_lines += 1
                    continue

                if stream_format is None:
                    stream_format = "ads_imu" if len(parts) == 10 else "ads"
                    saver.write_header(stream_format)
                    print(f"[info] stream format: {stream_format}")
                    if plotter is not None and plotter.enabled:
                        plotter.ensure_layout(stream_format)

                if len(parts) == 4 and stream_format != "ads":
                    bad_lines += 1
                    continue
                if len(parts) == 10 and stream_format != "ads_imu":
                    bad_lines += 1
                    continue

                assert stream_format is not None
                sample = processor.process(parts, stream_format)
                if sample is None:
                    bad_lines += 1
                    continue

                saver.write_sample(sample)
                sample_count += 1
                arduino_us = sample.arduino_us
                mv = sample.mv

                first_arduino_us = (
                    arduino_us if first_arduino_us is None else first_arduino_us
                )
                last_arduino_us = arduino_us

                mv_sum += mv
                mv_sumsq += mv * mv
                if prev_arduino_us is not None:
                    d_us = arduino_us - prev_arduino_us
                    dt_sum += float(d_us)
                    dt_sumsq += float(d_us) * float(d_us)
                    dt_count += 1
                prev_arduino_us = arduino_us

                if plotter is not None:
                    plotter.on_sample(
                        stream_format=stream_format,
                        first_arduino_us=first_arduino_us,
                        sample=sample,
                    )

                if sample_count % 200 == 0:
                    print(f"logged={sample_count}")
                    saver.flush()

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            saver.flush()
            if plotter is not None:
                plotter.show_blocking()

    elapsed_wall = time.time() - start_wall
    print(f"samples={sample_count}, bad_lines={bad_lines}, wall_s={elapsed_wall:.3f}")
    if first_arduino_us is not None and last_arduino_us is not None and sample_count > 1:
        dt_us = last_arduino_us - first_arduino_us
        if dt_us > 0:
            fps = (sample_count - 1) * 1_000_000.0 / dt_us
            print(f"arduino_fps={fps:.3f}")

    if sample_count > 0:
        mv_mean = mv_sum / float(sample_count)
        mv_var = (mv_sumsq / float(sample_count)) - (mv_mean * mv_mean)
        mv_var = mv_var if mv_var > 0.0 else 0.0
        mv_std = mv_var ** 0.5
        print(f"mv_mean={mv_mean:.6f}, mv_std={mv_std:.6f}")

    if dt_count > 0:
        dt_mean = dt_sum / float(dt_count)
        dt_var = (dt_sumsq / float(dt_count)) - (dt_mean * dt_mean)
        dt_var = dt_var if dt_var > 0.0 else 0.0
        dt_std = dt_var ** 0.5
        print(f"dt_us_mean={dt_mean:.3f}, dt_us_std={dt_std:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
