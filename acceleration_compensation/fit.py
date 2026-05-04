#!/usr/bin/env python3
"""
Fit ADS1263 mV from stacked IMU windows.

Supports multiple model types via abstract Model/Trainer layers:
- Linear regression (closed-form least squares)
- MLP (PyTorch trainer)

Usage:
  python3 acceleration_compensation/fit.py recording.csv
  python3 acceleration_compensation/fit.py part1.csv part2.csv --model linear
  python3 acceleration_compensation/fit.py recording.csv --model mlp --epochs 300
"""

from __future__ import annotations

import argparse
import json
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

# Allow `python3 acceleration_compensation/fit.py` from repo root.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from acceleration_compensation.recordings import load_recorded_csv

ACC_COLS = ("acc_x_ema_mg", "acc_y_ema_mg", "acc_z_ema_mg")
GYR_COLS = ("gyr_x_ema_dps", "gyr_y_ema_dps", "gyr_z_ema_dps")
AANG_COLS = ("aang_x_ema_dps2", "aang_y_ema_dps2", "aang_z_ema_dps2")
TARGET_COL = "mV_ema"

DEFAULT_N_PAST = 4
DEFAULT_N_FUTURE = 0
# Per frame: acc (mg) + gyro (dps) + angular accel (deg/s²)
VALUES_PER_FRAME = 9


@dataclass
class StackedDataset:
    """Design matrix and targets for centered windows."""

    X: np.ndarray  # (n_samples, N_FEATURES), no bias column
    y: np.ndarray  # mV at center index
    ts_us: np.ndarray  # arduino_timestamp_us at center


@dataclass
class FitResult:
    model_name: str
    coef: Optional[np.ndarray]
    metrics: dict[str, float]
    y: np.ndarray
    y_hat: np.ndarray
    residual: np.ndarray
    elapsed_s: np.ndarray
    extra: dict[str, Any]


class BaseModel(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def predict(self, x: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def export_json_dict(self) -> dict[str, Any]:
        raise NotImplementedError

    def save_artifacts(self, base_path: Path) -> dict[str, str]:
        return {}


class BaseTrainer(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def fit(self, dataset: StackedDataset) -> BaseModel:
        raise NotImplementedError


@dataclass
class LinearRegressionModel(BaseModel):
    coef: np.ndarray

    @property
    def name(self) -> str:
        return "linear"

    def predict(self, x: np.ndarray) -> np.ndarray:
        ones = np.ones((x.shape[0], 1), dtype=np.float64)
        x_with_bias = np.hstack([x, ones])
        return x_with_bias @ self.coef

    def export_json_dict(self) -> dict[str, Any]:
        return {
            "weights": self.coef[:-1].tolist(),
            "bias": float(self.coef[-1]),
        }


class LinearLeastSquaresTrainer(BaseTrainer):
    @property
    def name(self) -> str:
        return "lstsq"

    def fit(self, dataset: StackedDataset) -> BaseModel:
        ones = np.ones((dataset.X.shape[0], 1), dtype=np.float64)
        x_with_bias = np.hstack([dataset.X, ones])
        coef, _, _, _ = np.linalg.lstsq(x_with_bias, dataset.y, rcond=None)
        return LinearRegressionModel(coef=coef)


class TorchMLPModel(BaseModel):
    def __init__(
        self,
        network: Any,
        x_mean: np.ndarray,
        x_std: np.ndarray,
        y_mean: float,
        y_std: float,
        hidden_sizes: list[int],
    ) -> None:
        self._network = network
        self._x_mean = x_mean.astype(np.float32)
        self._x_std = x_std.astype(np.float32)
        self._y_mean = float(y_mean)
        self._y_std = float(y_std)
        self._hidden_sizes = hidden_sizes

    @property
    def name(self) -> str:
        return "mlp"

    def predict(self, x: np.ndarray) -> np.ndarray:
        try:
            import torch
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("PyTorch is required for MLP prediction.") from exc
        x_np = x.astype(np.float32)
        x_norm = (x_np - self._x_mean) / self._x_std
        x_tensor = torch.from_numpy(x_norm)
        with torch.no_grad():
            y_norm = self._network(x_tensor).squeeze(-1).cpu().numpy()
        y = y_norm.astype(np.float64) * self._y_std + self._y_mean
        return y

    def export_json_dict(self) -> dict[str, Any]:
        return {
            "hidden_sizes": self._hidden_sizes,
            "x_mean": self._x_mean.astype(np.float64).tolist(),
            "x_std": self._x_std.astype(np.float64).tolist(),
            "y_mean": self._y_mean,
            "y_std": self._y_std,
        }

    def save_artifacts(self, base_path: Path) -> dict[str, str]:
        try:
            import torch
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("PyTorch is required to save MLP artifacts.") from exc
        model_path = base_path.with_suffix(".pt")
        payload = {
            "state_dict": self._network.state_dict(),
            "hidden_sizes": self._hidden_sizes,
            "x_mean": self._x_mean,
            "x_std": self._x_std,
            "y_mean": self._y_mean,
            "y_std": self._y_std,
        }
        torch.save(payload, model_path)
        return {"weights_file": str(model_path)}


class TorchMLPTrainer(BaseTrainer):
    def __init__(
        self,
        hidden_sizes: list[int],
        epochs: int,
        learning_rate: float,
        weight_decay: float,
    ) -> None:
        self._hidden_sizes = hidden_sizes
        self._epochs = epochs
        self._learning_rate = learning_rate
        self._weight_decay = weight_decay

    @property
    def name(self) -> str:
        return "torch_adam"

    def fit(self, dataset: StackedDataset) -> BaseModel:
        try:
            import torch
            import torch.nn as nn
        except ImportError as exc:
            raise RuntimeError(
                "PyTorch is required for --model mlp. Install with: pip install torch"
            ) from exc

        x = dataset.X.astype(np.float32)
        y = dataset.y.astype(np.float32)

        x_mean = x.mean(axis=0)
        x_std = x.std(axis=0)
        x_std = np.where(x_std < 1e-8, 1.0, x_std)
        x_norm = (x - x_mean) / x_std

        y_mean = float(y.mean())
        y_std = float(y.std())
        if y_std < 1e-8:
            y_std = 1.0
        y_norm = (y - y_mean) / y_std

        x_tensor = torch.from_numpy(x_norm)
        y_tensor = torch.from_numpy(y_norm).unsqueeze(-1)

        layers: list[nn.Module] = []
        in_dim = x_tensor.shape[1]
        for h in self._hidden_sizes:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.ReLU())
            in_dim = h
        layers.append(nn.Linear(in_dim, 1))
        network = nn.Sequential(*layers)

        optimizer = torch.optim.Adam(
            network.parameters(),
            lr=self._learning_rate,
            weight_decay=self._weight_decay,
        )
        loss_fn = nn.MSELoss()

        network.train()
        for epoch in range(self._epochs):
            optimizer.zero_grad()
            pred = network(x_tensor)
            loss = loss_fn(pred, y_tensor)
            loss.backward()
            optimizer.step()
            if (epoch + 1) % max(1, self._epochs // 10) == 0:
                print(f"[mlp] epoch {epoch + 1:4d}/{self._epochs} loss={loss.item():.6f}")

        network.eval()
        return TorchMLPModel(
            network=network,
            x_mean=x_mean,
            x_std=x_std,
            y_mean=y_mean,
            y_std=y_std,
            hidden_sizes=self._hidden_sizes,
        )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Linear regression: predict ADS1263 mV from "
            "stacked past/future IMU frames."
        )
    )
    p.add_argument(
        "csv_files",
        nargs="+",
        type=Path,
        help="Recorded CSV path(s) from stream_logger (combined ADS+IMU)",
    )
    p.add_argument(
        "--n-past",
        type=int,
        default=DEFAULT_N_PAST,
        help="Number of past frames in each stacked feature window.",
    )
    p.add_argument(
        "--n-future",
        type=int,
        default=DEFAULT_N_FUTURE,
        help="Number of future frames in each stacked feature window.",
    )
    p.add_argument(
        "--model",
        choices=("linear", "mlp"),
        default="linear",
        help="Model type to fit.",
    )
    p.add_argument(
        "--epochs",
        type=int,
        default=200,
        help="MLP: number of training epochs.",
    )
    p.add_argument(
        "--learning-rate",
        type=float,
        default=1e-3,
        help="MLP: optimizer learning rate.",
    )
    p.add_argument(
        "--weight-decay",
        type=float,
        default=0.0,
        help="MLP: Adam weight decay.",
    )
    p.add_argument(
        "--hidden-sizes",
        type=int,
        nargs="+",
        default=[64],
        help="MLP: hidden layer sizes, e.g. --hidden-sizes 128 64",
    )
    p.add_argument(
        "--mv-ema-alpha",
        type=float,
        default=0.20,
        help="EMA alpha used to generate mV_ema in the training CSV.",
    )
    p.add_argument(
        "--aang-ema-alpha",
        type=float,
        default=0.20,
        help="EMA alpha used to generate aang_*_ema_dps2 in the training CSV.",
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


def frame_vector(
    acc_row: np.ndarray,
    gyr_row: np.ndarray,
    aang_row: np.ndarray,
) -> np.ndarray:
    """Single frame: acc xyz (mg) + gyro xyz (dps) + angular accel xyz (deg/s²)."""
    return np.hstack([acc_row, gyr_row, aang_row])


def create_dataset(df: pd.DataFrame, n_past: int, n_future: int) -> StackedDataset:
    """
    2. Build stacked windows: for center index i, use frames
    i - n_past … i + n_future (inclusive). Target y = mV[i].
    """
    ts_us = df["arduino_timestamp_us"].to_numpy(dtype=np.float64)
    mV = df[TARGET_COL].to_numpy(dtype=np.float64)
    acc = df[list(ACC_COLS)].to_numpy(dtype=np.float64)
    gyr = df[list(GYR_COLS)].to_numpy(dtype=np.float64)
    if set(AANG_COLS).issubset(df.columns):
        aang = df[list(AANG_COLS)].to_numpy(dtype=np.float64)
    else:
        # Backward compatibility for older CSVs without EMA angular acceleration.
        aang = np.zeros_like(gyr, dtype=np.float64)

    n = len(mV)
    rows_x: list[np.ndarray] = []
    rows_y: list[float] = []
    rows_ts: list[float] = []

    for i in range(n_past, n - n_future):
        parts: list[np.ndarray] = []
        bad = False
        for off in range(-n_past, n_future + 1):
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

    n_frames = n_past + 1 + n_future
    n_features = n_frames * VALUES_PER_FRAME
    if len(rows_y) < n_features + 1:
        raise ValueError(
            f"Too few valid stacked rows: {len(rows_y)} (need at least {n_features + 1})"
        )

    X = np.asarray(rows_x, dtype=np.float64)
    y = np.asarray(rows_y, dtype=np.float64)
    ts_center = np.asarray(rows_ts, dtype=np.float64)
    return StackedDataset(X=X, y=y, ts_us=ts_center)


def _compute_metrics(y: np.ndarray, y_hat: np.ndarray) -> dict[str, float]:
    residual = y - y_hat

    rmse = float(np.sqrt(np.mean(residual**2)))
    mae = float(np.mean(np.abs(residual)))
    ss_res = float(np.sum(residual**2))
    y_mean = float(np.mean(y))
    ss_tot = float(np.sum((y - y_mean) ** 2))
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan")

    return {
        "n": float(y.shape[0]),
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
        "ss_res": ss_res,
    }


def fit_model(dataset: StackedDataset, trainer: BaseTrainer) -> FitResult:
    """Train via trainer abstraction and evaluate fit metrics."""
    model = trainer.fit(dataset)
    y = dataset.y
    y_hat = model.predict(dataset.X)
    residual = y - y_hat
    metrics = _compute_metrics(y=y, y_hat=y_hat)

    elapsed_s = (dataset.ts_us - dataset.ts_us[0]) / 1_000_000.0
    coef: Optional[np.ndarray] = None
    if isinstance(model, LinearRegressionModel):
        coef = model.coef
    return FitResult(
        model_name=model.name,
        coef=coef,
        metrics=metrics,
        y=y,
        y_hat=y_hat,
        residual=residual,
        elapsed_s=elapsed_s,
        extra={"trainer": trainer.name, "model": model},
    )


def print_fit_results(result: FitResult, n_past: int, n_future: int) -> None:
    """4. Print metrics and coefficients."""
    metrics = result.metrics
    n_frames = n_past + 1 + n_future
    n_features = n_frames * VALUES_PER_FRAME
    print(
        f"Target: {TARGET_COL}  (samples: {int(metrics['n'])})  "
        f"model={result.model_name} trainer={result.extra['trainer']}"
    )
    print(
        f"Window: {n_past} past + center + {n_future} future = "
        f"{n_frames} frames × {VALUES_PER_FRAME} = {n_features}"
    )
    coef = result.coef
    if coef is not None:
        w = coef[:-1]
        b = coef[-1]
        idx = 0
        for off in range(-n_past, n_future + 1):
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


def plot_fit(result: FitResult, n_past: int, n_future: int) -> None:
    """Measured mV and residual vs elapsed time."""
    elapsed_s = result.elapsed_s
    y_mV = result.y
    residual = result.residual

    fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
    ax0.plot(elapsed_s, y_mV, color="tab:blue", linewidth=0.8, label="mV (measured)")
    ax0.set_ylabel("mV")
    ax0.set_title(
        f"ADS1263 + linear stack ({n_past}+{n_future}+1 frames × acc+gyro+aang)"
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


def save_coefficients(
    result: FitResult,
    csv_files: list[Path],
    n_past: int,
    n_future: int,
    mv_ema_alpha: float,
    aang_ema_alpha: float,
) -> None:
    """Save fitted model payload for each input CSV."""
    metrics = {k: float(v) for k, v in result.metrics.items()}
    model: BaseModel = result.extra["model"]
    n_frames = n_past + 1 + n_future
    n_features = n_frames * VALUES_PER_FRAME
    for csv_path in csv_files:
        resolved = csv_path.expanduser().resolve()
        out_path = resolved.with_name(f"{resolved.stem}_{result.model_name}_model.json")
        artifact_base = resolved.with_name(f"{resolved.stem}_{result.model_name}_weights")
        artifacts = model.save_artifacts(artifact_base)
        payload = {
            "source_csv": str(resolved),
            "target": TARGET_COL,
            "model": result.model_name,
            "trainer": result.extra["trainer"],
            "n_past": n_past,
            "n_future": n_future,
            "n_frames": n_frames,
            "values_per_frame": VALUES_PER_FRAME,
            "n_features": n_features,
            "mv_ema_alpha": float(mv_ema_alpha),
            "aang_ema_alpha": float(aang_ema_alpha),
            "model_params": model.export_json_dict(),
            "metrics": metrics,
            "artifacts": artifacts,
        }
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
            f.write("\n")
        print(f"Saved model: {out_path}")


def main() -> int:
    args = parse_args()
    try:
        if args.n_past < 0 or args.n_future < 0:
            raise ValueError("--n-past and --n-future must be non-negative.")
        df = load_data(list(args.csv_files))
        dataset = create_dataset(df, n_past=args.n_past, n_future=args.n_future)
        if args.model == "linear":
            trainer: BaseTrainer = LinearLeastSquaresTrainer()
        else:
            trainer = TorchMLPTrainer(
                hidden_sizes=list(args.hidden_sizes),
                epochs=args.epochs,
                learning_rate=args.learning_rate,
                weight_decay=args.weight_decay,
            )
        result = fit_model(dataset, trainer=trainer)
    except (ValueError, RuntimeError) as e:
        print(e, file=sys.stderr)
        return 1

    print_fit_results(result, n_past=args.n_past, n_future=args.n_future)
    save_coefficients(
        result,
        list(args.csv_files),
        n_past=args.n_past,
        n_future=args.n_future,
        mv_ema_alpha=args.mv_ema_alpha,
        aang_ema_alpha=args.aang_ema_alpha,
    )
    plot_fit(result, n_past=args.n_past, n_future=args.n_future)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
