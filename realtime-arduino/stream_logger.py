#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import sys
import time
from collections import deque

import serial


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Log ADS1263 stream from Arduino.")
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
        help="Enable real-time plotting of mV vs elapsed time",
    )
    p.add_argument(
        "--plot-window-s",
        type=float,
        default=10.0,
        help="Time window in seconds for real-time plot",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    start_wall = time.time()
    sample_count = 0
    bad_lines = 0
    first_arduino_us = None
    last_arduino_us = None
    prev_arduino_us = None
    mv_sum = 0.0
    mv_sumsq = 0.0
    dt_sum = 0.0
    dt_sumsq = 0.0
    dt_count = 0
    update_counter = 0

    plot_enabled = args.plot
    plot_elapsed = deque()
    plot_mv = deque()
    plt = None
    fig = None
    ax = None
    line_handle = None
    if plot_enabled:
        try:
            import matplotlib.pyplot as plt_module  # pylint: disable=import-error
            plt = plt_module
        except ImportError as exc:
            print(f"[warn] Failed to import matplotlib ({exc}). Plot disabled.")
            plot_enabled = False

    if plot_enabled and plt is not None:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 4))
        (line_handle,) = ax.plot([], [], linewidth=1.0, color="tab:blue")
        ax.set_xlabel("Elapsed Time (s)")
        ax.set_ylabel("mV")
        ax.set_title("ADS1263 Stream (Real-Time)")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(f"Logging to {args.output}")
    print("Press Ctrl+C to stop.")

    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "host_iso_time",
                "host_epoch_s",
                "arduino_timestamp_us",
                "raw",
                "mV",
            ]
        )

        try:
            while True:
                if args.duration > 0 and (time.time() - start_wall) >= args.duration:
                    print("Duration reached.")
                    break

                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("STATS,"):
                    print(line)
                    continue

                if not line.startswith("DATA,"):
                    print(f"[info] {line}")
                    continue

                parts = line.split(",")
                if len(parts) != 4:
                    bad_lines += 1
                    continue

                try:
                    arduino_us = int(parts[1])
                    raw = int(parts[2])
                    mv = float(parts[3])
                except ValueError:
                    bad_lines += 1
                    continue

                now_epoch = time.time()
                now_iso = dt.datetime.now().isoformat(timespec="milliseconds")
                writer.writerow([now_iso, f"{now_epoch:.6f}", arduino_us, raw, f"{mv:.3f}"])
                sample_count += 1
                first_arduino_us = arduino_us if first_arduino_us is None else first_arduino_us
                last_arduino_us = arduino_us

                mv_sum += mv
                mv_sumsq += mv * mv
                if prev_arduino_us is not None:
                    d_us = arduino_us - prev_arduino_us
                    dt_sum += float(d_us)
                    dt_sumsq += float(d_us) * float(d_us)
                    dt_count += 1
                prev_arduino_us = arduino_us

                if plot_enabled and first_arduino_us is not None:
                    elapsed_s = (arduino_us - first_arduino_us) / 1_000_000.0
                    plot_elapsed.append(elapsed_s)
                    plot_mv.append(mv)
                    min_time = max(0.0, elapsed_s - args.plot_window_s)
                    while plot_elapsed and plot_elapsed[0] < min_time:
                        plot_elapsed.popleft()
                        plot_mv.popleft()

                    update_counter += 1
                    if update_counter >= 10 and line_handle is not None and ax is not None and fig is not None and plt is not None:
                        update_counter = 0
                        line_handle.set_data(list(plot_elapsed), list(plot_mv))
                        if plot_elapsed:
                            ax.set_xlim(plot_elapsed[0], plot_elapsed[-1] + 1e-9)
                            y_min = min(plot_mv)
                            y_max = max(plot_mv)
                            if y_min == y_max:
                                y_min -= 1.0
                                y_max += 1.0
                            pad = 0.05 * (y_max - y_min)
                            ax.set_ylim(y_min - pad, y_max + pad)
                        fig.canvas.draw_idle()
                        plt.pause(0.001)

                if sample_count % 200 == 0:
                    print(f"logged={sample_count}")
                    f.flush()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            f.flush()
            ser.close()
            if plot_enabled and plt is not None:
                plt.ioff()
                plt.show()

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
