#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot saved ADS1263 CSV files.")
    p.add_argument(
        "files",
        nargs="+",
        help="CSV file paths (supports one or multiple files)",
    )
    p.add_argument(
        "--column",
        default="mV",
        choices=["mV", "raw"],
        help="Column to plot for stream-format CSVs (default: mV)",
    )
    return p.parse_args()


def load_elapsed_and_value(path: Path, column: str) -> tuple[pd.Series, pd.Series, str]:
    df = pd.read_csv(path)

    # New stream_logger format
    if {"arduino_timestamp_us", "raw", "mV"}.issubset(df.columns):
        elapsed = (df["arduino_timestamp_us"] - df["arduino_timestamp_us"].iloc[0]) / 1_000_000.0
        value = df[column]
        label = f"{path.name} ({column})"
        return elapsed, value, label

    # Legacy format
    if {"timestamp", "raw_value"}.issubset(df.columns):
        ts = pd.to_datetime(df["timestamp"])
        elapsed = (ts - ts.iloc[0]).dt.total_seconds()
        value = df["raw_value"]
        label = f"{path.name} (raw_value)"
        return elapsed, value, label

    raise ValueError(
        f"Unsupported format in {path}. Expected either "
        "['arduino_timestamp_us','raw','mV'] or ['timestamp','raw_value']."
    )


def main() -> int:
    args = parse_args()
    paths = [Path(p) for p in args.files]

    fig, ax = plt.subplots(figsize=(12, 5))
    stats_lines = []

    for path in paths:
        elapsed, value, label = load_elapsed_and_value(path, args.column)
        ax.plot(elapsed, value, linewidth=0.9, label=label)

        mean = value.mean()
        std = value.std(ddof=0)
        stats_lines.append(f"{path.name}: mean={mean:.4f}, std={std:.4f}, n={len(value)}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(args.column if len(paths) == 1 else "Value")
    ax.set_title("Saved CSV Plot")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    print("\n".join(stats_lines))
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

