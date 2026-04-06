import sys

import matplotlib.pyplot as plt
import pandas as pd

filename = sys.argv[1] if len(sys.argv) > 1 else "bowpressure_log.csv"

df = pd.read_csv(filename)


def delta_seconds_series(ts_us: pd.Series) -> pd.Series:
    """Δt between adjacent rows in seconds (first row NaN)."""
    return ts_us.diff() / 1_000_000.0


if {"arduino_timestamp_us", "raw", "mV"}.issubset(df.columns):
    # New stream_logger format.
    df["elapsed_s"] = (
        df["arduino_timestamp_us"] - df["arduino_timestamp_us"].iloc[0]
    ) / 1_000_000.0
    df["delta_t_s"] = delta_seconds_series(df["arduino_timestamp_us"])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1.plot(df["elapsed_s"], df["mV"], linewidth=0.9, color="tab:blue")
    ax1.set_ylabel("mV")
    ax1.grid(True, alpha=0.3)

    ax2.plot(df["elapsed_s"], df["delta_t_s"], linewidth=0.8, color="tab:green")
    ax2.set_ylabel("Δt (s)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

elif "timestamp" in df.columns:
    # Legacy logger format (timestamp column; optional raw_value not plotted).
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    df["elapsed_s"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()
    df["delta_t_s"] = df["timestamp"].diff().dt.total_seconds()

    fig, ax = plt.subplots(figsize=(12, 5))
    dt = df["delta_t_s"].dropna()
    ax.plot(df.loc[dt.index, "elapsed_s"], dt, linewidth=0.8, color="tab:green")
    ax.set_ylabel("Δt (s)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

else:
    raise ValueError(
        "Unsupported CSV format. Expected either a 'timestamp' column (legacy) or "
        "['arduino_timestamp_us', 'raw', 'mV'] (stream_logger)."
    )

fig.suptitle(f"Bow Pressure — {filename}")
fig.tight_layout()
plt.show()
