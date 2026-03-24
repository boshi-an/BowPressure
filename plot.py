import sys
import pandas as pd
import matplotlib.pyplot as plt

filename = sys.argv[1] if len(sys.argv) > 1 else "bowpressure_log.csv"

df = pd.read_csv(filename)

if {"timestamp", "raw_value"}.issubset(df.columns):
    # Legacy logger format.
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    df["elapsed_s"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(df["elapsed_s"], df["raw_value"], linewidth=0.8, label="raw")
    ax.set_ylabel("Raw Value")

elif {"arduino_timestamp_us", "raw", "mV"}.issubset(df.columns):
    # New stream_logger format.
    df["elapsed_s"] = (df["arduino_timestamp_us"] - df["arduino_timestamp_us"].iloc[0]) / 1_000_000.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1.plot(df["elapsed_s"], df["mV"], linewidth=0.9, color="tab:blue")
    ax1.set_ylabel("mV")
    ax1.grid(True, alpha=0.3)

    ax2.plot(df["elapsed_s"], df["raw"], linewidth=0.8, color="tab:orange")
    ax2.set_ylabel("Raw")
    ax2.grid(True, alpha=0.3)
    ax = ax2
else:
    raise ValueError(
        "Unsupported CSV format. Expected either "
        "['timestamp', 'raw_value'] or ['arduino_timestamp_us', 'raw', 'mV']."
    )

ax.set_xlabel("Time (s)")
fig.suptitle(f"Bow Pressure — {filename}")
fig.tight_layout()
plt.show()
