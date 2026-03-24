import sys
import pandas as pd
import matplotlib.pyplot as plt

filename = sys.argv[1] if len(sys.argv) > 1 else "bowpressure_log.csv"

df = pd.read_csv(filename, parse_dates=["timestamp"])
df["elapsed_s"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()

fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(df["elapsed_s"], df["raw_value"], linewidth=0.8)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Raw Value")
ax.set_title(f"Bow Pressure — {filename}")
ax.grid(True, alpha=0.3)
fig.tight_layout()
plt.show()
