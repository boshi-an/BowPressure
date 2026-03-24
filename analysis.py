import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

filename = sys.argv[1] if len(sys.argv) > 1 else "calibration_data.json"

data = []
with open(filename, encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if line:
            data.append(json.loads(line))

x = np.array([d["bowing_fraction"] for d in data])
y = np.array([d["real_applied_force"] for d in data])
z = np.array([d["bow_readings"] for d in data])

# --- Interpolated surface grid ---
grid_x = np.linspace(x.min(), x.max(), 100)
grid_y = np.linspace(y.min(), y.max(), 100)
gx, gy = np.meshgrid(grid_x, grid_y)

grid_z_interp = griddata((x, y), z, (gx, gy), method="cubic")

# --- Fit z = a*x*y + b*x + c*y + d via least squares ---
A = np.column_stack([x * y, x, y, np.ones_like(x)])
coeffs, residuals, _, _ = np.linalg.lstsq(A, z, rcond=None)
a, b, c, d = coeffs

z_fit = A @ coeffs
errors = z - z_fit
rmse = np.sqrt(np.mean(errors ** 2))
max_err = np.max(np.abs(errors))

print(f"Fitted: z = {a:.4f}*x*y + {b:.4f}*x + {c:.4f}*y + {d:.4f}")
print(f"  RMSE:      {rmse:.4f}")
print(f"  Max error: {max_err:.4f}")
print(f"  a = {a:.6f}")
print(f"  b = {b:.6f}")
print(f"  c = {c:.6f}")
print(f"  d = {d:.6f}")

base = os.path.splitext(filename)[0]
params_file = f"{base}_params.json"
with open(params_file, "w", encoding="utf-8") as pf:
    json.dump({"a": a, "b": b, "c": c, "d": d, "rmse": rmse, "max_error": max_err}, pf, indent=2)
print(f"  Parameters saved to {params_file}")

grid_z_fit = a * gx * gy + b * gx + c * gy + d

# --- Plot ---
fig = plt.figure(figsize=(18, 7))

# Left: interpolated surface + data
ax1 = fig.add_subplot(121, projection="3d")
ax1.plot_surface(gx, gy, grid_z_interp, cmap="viridis", alpha=0.7, edgecolor="none")
ax1.scatter(x, y, z, c="red", s=8, alpha=0.5, label="data points")
ax1.set_xlabel("Bowing Fraction")
ax1.set_ylabel("Real Applied Force (N)")
ax1.set_zlabel("Bow Readings (raw)")
ax1.set_title("Interpolated Surface")
ax1.legend()

# Right: fitted surface + data
ax2 = fig.add_subplot(122, projection="3d")
ax2.plot_surface(gx, gy, grid_z_fit, cmap="plasma", alpha=0.7, edgecolor="none")
ax2.scatter(x, y, z, c="red", s=8, alpha=0.5, label="data points")
ax2.set_xlabel("Bowing Fraction")
ax2.set_ylabel("Real Applied Force (N)")
ax2.set_zlabel("Bow Readings (raw)")
ax2.set_title(f"Fitted: z=axy+bx+cy+d  (RMSE={rmse:.2f})")
ax2.legend()

plt.tight_layout()
plt.show()

