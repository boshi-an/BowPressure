#!/usr/bin/env python3
"""
Fit a linear model: ADS1263 mV ~ stacked IMU window.

Each frame uses acceleration (mg), angular velocity / gyro (dps), and angular
acceleration d(gyro)/dt (deg/s²).

Window: N_PAST + 1 + N_FUTURE frames × 9 values + bias.

  y[i] ≈ w·stack([acc,gyr,aang] per frame) + b

Usage:
  python3 acceleration_compensation/fit.py recording.csv
  python3 acceleration_compensation/fit.py part1.csv part2.csv
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

# Allow `python3 acceleration_compensation/fit.py` from repo root.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from acceleration_compensation.recordings import load_recorded_csv

ACC_COLS = ("acc_x_mg", "acc_y_mg", "acc_z_mg")
GYR_COLS = ("gyr_x_dps", "gyr_y_dps", "gyr_z_dps")
TARGET_COL = "mV"

N_PAST = 2
N_FUTURE = 2
N_FRAMES = N_PAST + 1 + N_FUTURE  # center at index i
# Per frame: acc (mg) + gyro (dps) + angular accel (deg/s²)
VALUES_PER_FRAME = 9
N_FEATURES = N_FRAMES * VALUES_PER_FRAME


@dataclass
class StackedDataset:
    """Design matrix and targets for centered windows."""

    X: np.ndarray  # (n_samples, N_FEATURES), no bias column
    y: np.ndarray  # mV at center index
    ts_us: np.ndarray  # arduino_timestamp_us at center


@dataclass
class FitResult:
    coef: np.ndarray
    metrics: dict[str, float]
    y: np.ndarray
    y_hat: np.ndarray
    residual: np.ndarray
    elapsed_s: np.ndarray


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Linear regression: predict ADS1263 mV from "
            f"{N_PAST} past + {N_FUTURE} future frames "
            f"({N_FRAMES} × acc + gyro + angular accel)."
        )
    )
    p.add_argument(
        "csv_files",
        nargs="+",
        type=Path,
        help="Recorded CSV path(s) from stream_logger (combined ADS+IMU)",
    )
    return p.parse_args()


def load_data(csv_files: list[Path]) -> pd.DataFrame:
    """1. Load and concatenate recordings; validate required columns."""
    frames = [load_recorded_csv(p, require_imu=True) for p in csv_files]
    df = pd.concat(frames, axis=0, ignore_index=True)

    need = set(ACC_COLS) | set(GYR_COLS) | {"arduino_timestamp_us", TARGET_COL}
    missing = [c for c in sorted(need) if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}")
    return df


def angular_acceleration_dps2(
    gyr: np.ndarray,
    ts_us: np.ndarray,
) -> np.ndarray:
    """d(gyro)/dt in deg/s²; first row is zero (no previous sample)."""
    n = gyr.shape[0]
    out = np.zeros_like(gyr, dtype=np.float64)
    for i in range(1, n):
        dt_s = (ts_us[i] - ts_us[i - 1]) / 1_000_000.0
        if dt_s > 1e-9 and np.isfinite(dt_s):
            out[i] = (gyr[i] - gyr[i - 1]) / dt_s
    return out


def frame_vector(
    acc_row: np.ndarray,
    gyr_row: np.ndarray,
    aang_row: np.ndarray,
) -> np.ndarray:
    """Single frame: acc xyz (mg) + gyro xyz (dps) + angular accel xyz (deg/s²)."""
    return np.hstack([acc_row, gyr_row, aang_row])


def create_dataset(df: pd.DataFrame) -> StackedDataset:
    """
    2. Build stacked windows: for center index i, use frames
    i - N_PAST … i + N_FUTURE (inclusive). Target y = mV[i].
    """
    ts_us = df["arduino_timestamp_us"].to_numpy(dtype=np.float64)
    mV = df[TARGET_COL].to_numpy(dtype=np.float64)
    acc = df[list(ACC_COLS)].to_numpy(dtype=np.float64)
    gyr = df[list(GYR_COLS)].to_numpy(dtype=np.float64)
    aang = angular_acceleration_dps2(gyr, ts_us)

    n = len(mV)
    rows_x: list[np.ndarray] = []
    rows_y: list[float] = []
    rows_ts: list[float] = []

    for i in range(N_PAST, n - N_FUTURE):
        parts: list[np.ndarray] = []
        bad = False
        for off in range(-N_PAST, N_FUTURE + 1):
            j = i + off
            f = frame_vector(acc[j], gyr[j], aang[j])
            if not np.isfinite(f).all():
                bad = True
                break
            parts.append(f)
        if bad:
            continue
        if not np.isfinite(mV[i]) or not np.isfinite(ts_us[i]):
            continue
        rows_x.append(np.concatenate(parts))
        rows_y.append(mV[i])
        rows_ts.append(ts_us[i])

    if len(rows_y) < N_FEATURES + 1:
        raise ValueError(
            f"Too few valid stacked rows: {len(rows_y)} (need at least {N_FEATURES + 1})"
        )

    X = np.asarray(rows_x, dtype=np.float64)
    y = np.asarray(rows_y, dtype=np.float64)
    ts_center = np.asarray(rows_ts, dtype=np.float64)
    return StackedDataset(X=X, y=y, ts_us=ts_center)


def fit_model(dataset: StackedDataset) -> FitResult:
    """3. Ordinary least squares with intercept."""
    X_body = dataset.X
    y = dataset.y
    ts_us = dataset.ts_us

    ones = np.ones((X_body.shape[0], 1), dtype=np.float64)
    X = np.hstack([X_body, ones])

    coef, res_lstsq, _, _ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ coef
    residual = y - y_hat

    rmse = float(np.sqrt(np.mean(residual**2)))
    mae = float(np.mean(np.abs(residual)))
    ss_res = float(np.sum(residual**2))
    y_mean = float(np.mean(y))
    ss_tot = float(np.sum((y - y_mean) ** 2))
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan")

    metrics: dict[str, float] = {
        "n": float(X.shape[0]),
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
        "ss_res": ss_res,
    }
    if res_lstsq.size:
        metrics["lstsq_residual_sum_sq"] = float(res_lstsq[0])

    elapsed_s = (ts_us - ts_us[0]) / 1_000_000.0
    return FitResult(
        coef=coef,
        metrics=metrics,
        y=y,
        y_hat=y_hat,
        residual=residual,
        elapsed_s=elapsed_s,
    )


def print_fit_results(result: FitResult) -> None:
    """4. Print metrics and coefficients."""
    metrics = result.metrics
    coef = result.coef
    print(f"Target: {TARGET_COL}  (samples: {int(metrics['n'])})")
    print(
        f"Window: {N_PAST} past + center + {N_FUTURE} future = "
        f"{N_FRAMES} frames × {VALUES_PER_FRAME} = {N_FEATURES} + bias"
    )
    w = coef[:-1]
    b = coef[-1]
    idx = 0
    for off in range(-N_PAST, N_FUTURE + 1):
        chunk = w[idx : idx + VALUES_PER_FRAME]
        idx += VALUES_PER_FRAME
        print(
            f"  t{off:+d}  "
            f"acc_x {chunk[0]: .6g}  acc_y {chunk[1]: .6g}  acc_z {chunk[2]: .6g}  "
            f"gyr_x {chunk[3]: .6g}  gyr_y {chunk[4]: .6g}  gyr_z {chunk[5]: .6g}  "
            f"aang_x {chunk[6]: .6g}  aang_y {chunk[7]: .6g}  aang_z {chunk[8]: .6g}"
        )
    print(f"  {'bias':12s}  {b: .8g}")
    print(
        f"RMSE: {metrics['rmse']:.6g}  MAE: {metrics['mae']:.6g}  R²: {metrics['r2']:.6f}"
    )


def plot_fit(result: FitResult) -> None:
    """Measured mV and residual vs elapsed time."""
    elapsed_s = result.elapsed_s
    y_mV = result.y
    residual = result.residual

    fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
    ax0.plot(elapsed_s, y_mV, color="tab:blue", linewidth=0.8, label="mV (measured)")
    ax0.set_ylabel("mV")
    ax0.set_title(
        f"ADS1263 + linear stack ({N_PAST}+{N_FUTURE}+1 frames × acc+gyro+aang)"
    )
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="best")

    ax1.plot(elapsed_s, residual, color="tab:orange", linewidth=0.8, label="mV − fit")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Residual (mV)")
    ax1.axhline(0.0, color="gray", linewidth=0.8, linestyle="--")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    fig.tight_layout()
    plt.show()


def main() -> int:
    args = parse_args()
    try:
        df = load_data(list(args.csv_files))
        dataset = create_dataset(df)
        result = fit_model(dataset)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    print_fit_results(result)
    plot_fit(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
