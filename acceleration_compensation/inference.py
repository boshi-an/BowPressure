#!/usr/bin/env python3
"""Realtime inference using fitted linear/MLP models on serial ADS+IMU stream."""

from __future__ import annotations

import argparse
import json
import math
import sys
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Allow imports from repo root and realtime-arduino folder.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_RT_DIR = _REPO_ROOT / "realtime-arduino"
for _p in (_REPO_ROOT, _RT_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from streamer import SerialLineGenerator, parse_data_line, parse_data_parts


@dataclass
class RuntimeSample:
    arduino_us: int
    mv_ema: float
    frame_vec: np.ndarray


class FeatureProcessor:
    """Compute EMA mV and EMA angular acceleration from raw stream samples."""

    def __init__(self, mv_ema_alpha: float, aang_ema_alpha: float) -> None:
        self._mv_alpha = float(max(0.0, min(1.0, mv_ema_alpha)))
        self._aang_alpha = float(max(0.0, min(1.0, aang_ema_alpha)))
        self._mv_ema: Optional[float] = None
        self._prev_us: Optional[int] = None
        self._prev_gyr: Optional[tuple[float, float, float]] = None
        self._aang_ema: Optional[tuple[float, float, float]] = None

    def process(self, sample: object) -> Optional[RuntimeSample]:
        # Typed at runtime from streamer.ArduinoDataSample.
        if getattr(sample, "stream_format", "") != "ads_imu":
            return None
        mv_raw = float(getattr(sample, "mv_raw"))
        if self._mv_ema is None:
            self._mv_ema = mv_raw
        else:
            self._mv_ema = self._mv_alpha * mv_raw + (1.0 - self._mv_alpha) * self._mv_ema

        arduino_us = int(getattr(sample, "arduino_us"))
        gyr = (
            float(getattr(sample, "gyr_x")),
            float(getattr(sample, "gyr_y")),
            float(getattr(sample, "gyr_z")),
        )

        aang_x = aang_y = aang_z = 0.0
        if self._prev_us is not None and self._prev_gyr is not None and arduino_us > self._prev_us:
            dt_s = (arduino_us - self._prev_us) / 1_000_000.0
            if dt_s > 1e-9:
                raw_aang = (
                    (gyr[0] - self._prev_gyr[0]) / dt_s,
                    (gyr[1] - self._prev_gyr[1]) / dt_s,
                    (gyr[2] - self._prev_gyr[2]) / dt_s,
                )
                if self._aang_ema is None:
                    self._aang_ema = raw_aang
                else:
                    self._aang_ema = (
                        self._aang_alpha * raw_aang[0] + (1.0 - self._aang_alpha) * self._aang_ema[0],
                        self._aang_alpha * raw_aang[1] + (1.0 - self._aang_alpha) * self._aang_ema[1],
                        self._aang_alpha * raw_aang[2] + (1.0 - self._aang_alpha) * self._aang_ema[2],
                    )
                aang_x, aang_y, aang_z = self._aang_ema

        self._prev_us = arduino_us
        self._prev_gyr = gyr

        frame_vec = np.asarray(
            [
                float(getattr(sample, "acc_x")),
                float(getattr(sample, "acc_y")),
                float(getattr(sample, "acc_z")),
                gyr[0],
                gyr[1],
                gyr[2],
                aang_x,
                aang_y,
                aang_z,
            ],
            dtype=np.float64,
        )
        if not np.isfinite(frame_vec).all():
            return None
        assert self._mv_ema is not None
        return RuntimeSample(arduino_us=arduino_us, mv_ema=float(self._mv_ema), frame_vec=frame_vec)


class InferenceModel(ABC):
    @abstractmethod
    def predict(self, x: np.ndarray) -> np.ndarray:
        raise NotImplementedError


@dataclass
class LinearInferenceModel(InferenceModel):
    weights: np.ndarray
    bias: float

    def predict(self, x: np.ndarray) -> np.ndarray:
        return x @ self.weights + self.bias


class MlpInferenceModel(InferenceModel):
    def __init__(self, weights_path: Path) -> None:
        try:
            import torch
            import torch.nn as nn
        except ImportError as exc:
            raise RuntimeError(
                "PyTorch is required for mlp inference. Install with: pip install torch"
            ) from exc

        # PyTorch 2.6 defaults to weights_only=True, which rejects our checkpoint
        # because it intentionally stores numpy normalization arrays alongside state_dict.
        try:
            ckpt = torch.load(weights_path, map_location="cpu", weights_only=False)
        except TypeError:
            # Older torch versions do not support weights_only kwarg.
            ckpt = torch.load(weights_path, map_location="cpu")
        hidden_sizes = list(ckpt["hidden_sizes"])
        x_mean = np.asarray(ckpt["x_mean"], dtype=np.float32)
        x_std = np.asarray(ckpt["x_std"], dtype=np.float32)
        self._x_mean = x_mean
        self._x_std = np.where(x_std < 1e-8, 1.0, x_std)
        self._y_mean = float(ckpt["y_mean"])
        self._y_std = float(ckpt["y_std"]) if float(ckpt["y_std"]) > 1e-8 else 1.0

        layers: list[nn.Module] = []
        in_dim = int(self._x_mean.shape[0])
        for h in hidden_sizes:
            layers.append(nn.Linear(in_dim, int(h)))
            layers.append(nn.ReLU())
            in_dim = int(h)
        layers.append(nn.Linear(in_dim, 1))
        self._network = nn.Sequential(*layers)
        self._network.load_state_dict(ckpt["state_dict"])
        self._network.eval()
        self._torch = torch

    def predict(self, x: np.ndarray) -> np.ndarray:
        x32 = x.astype(np.float32)
        x_norm = (x32 - self._x_mean) / self._x_std
        xt = self._torch.from_numpy(x_norm)
        with self._torch.no_grad():
            y_norm = self._network(xt).squeeze(-1).cpu().numpy()
        return y_norm.astype(np.float64) * self._y_std + self._y_mean


@dataclass
class LoadedModel:
    model: InferenceModel
    n_past: int
    n_future: int
    values_per_frame: int
    n_features: int
    model_type: str
    mv_ema_alpha: float
    aang_ema_alpha: float


class RealtimeCompensator:
    """Windowed realtime compensator over decoded ADS+IMU samples."""

    def __init__(self, loaded: LoadedModel, mv_ema_alpha: float, aang_ema_alpha: float) -> None:
        self._loaded = loaded
        self._n_frames = loaded.n_past + 1 + loaded.n_future
        if loaded.values_per_frame != 9:
            raise ValueError("RealtimeCompensator expects values_per_frame=9.")
        self._processor = FeatureProcessor(
            mv_ema_alpha=mv_ema_alpha,
            aang_ema_alpha=aang_ema_alpha,
        )
        self._frame_window: deque[np.ndarray] = deque(maxlen=self._n_frames)
        self._sample_window: deque[RuntimeSample] = deque(maxlen=self._n_frames)

    def observe(self, sample: object) -> Optional[tuple[RuntimeSample, float]]:
        runtime = self._processor.process(sample)
        if runtime is None:
            return None
        self._frame_window.append(runtime.frame_vec)
        self._sample_window.append(runtime)
        if len(self._frame_window) < self._n_frames:
            return None
        stacked = np.concatenate(list(self._frame_window), axis=0).reshape(1, -1)
        if stacked.shape[1] != self._loaded.n_features:
            return None
        pred = float(self._loaded.model.predict(stacked)[0])
        center = self._sample_window[self._loaded.n_past]
        return center, pred


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Realtime inference from ADS1263+BNO055 stream.")
    p.add_argument("--model-json", type=Path, required=True, help="Path to fitted model json.")
    p.add_argument("--port", default="/dev/ttyACM1", help="Serial port.")
    p.add_argument("--baud", type=int, default=230400, help="Serial baud.")
    p.add_argument(
        "--print-every",
        type=int,
        default=20,
        help="Print one status line every N predictions.",
    )
    return p.parse_args()


def _as_float_list(v: object) -> list[float]:
    if not isinstance(v, list):
        raise ValueError("Expected list in model json.")
    return [float(x) for x in v]


def load_model(model_json: Path) -> LoadedModel:
    with open(model_json, "r", encoding="utf-8") as f:
        payload = json.load(f)

    n_past = int(payload["n_past"])
    n_future = int(payload["n_future"])
    values_per_frame = int(payload["values_per_frame"])
    n_features = int(payload["n_features"])
    # New model payloads persist EMA settings from training; use defaults for legacy files.
    mv_ema_alpha = float(payload.get("mv_ema_alpha", 0.20))
    aang_ema_alpha = float(payload.get("aang_ema_alpha", 0.20))

    # Backward compatible: old *_coef.json has direct weights/bias.
    if "weights" in payload and "bias" in payload:
        w = np.asarray(_as_float_list(payload["weights"]), dtype=np.float64)
        b = float(payload["bias"])
        return LoadedModel(
            model=LinearInferenceModel(weights=w, bias=b),
            n_past=n_past,
            n_future=n_future,
            values_per_frame=values_per_frame,
            n_features=n_features,
            model_type="linear",
            mv_ema_alpha=mv_ema_alpha,
            aang_ema_alpha=aang_ema_alpha,
        )

    model_type = str(payload.get("model", "")).strip().lower()
    params = payload.get("model_params", {})
    if model_type == "linear":
        if not isinstance(params, dict):
            raise ValueError("linear model_params must be a dict.")
        w = np.asarray(_as_float_list(params["weights"]), dtype=np.float64)
        b = float(params["bias"])
        model: InferenceModel = LinearInferenceModel(weights=w, bias=b)
    elif model_type == "mlp":
        artifacts = payload.get("artifacts", {})
        if not isinstance(artifacts, dict) or "weights_file" not in artifacts:
            raise ValueError("mlp model json missing artifacts.weights_file.")
        weights_file = Path(str(artifacts["weights_file"])).expanduser()
        if not weights_file.is_absolute():
            weights_file = (model_json.parent / weights_file).resolve()
        model = MlpInferenceModel(weights_file)
    else:
        raise ValueError(f"Unsupported model type: {model_type!r}")

    return LoadedModel(
        model=model,
        n_past=n_past,
        n_future=n_future,
        values_per_frame=values_per_frame,
        n_features=n_features,
        model_type=model_type,
        mv_ema_alpha=mv_ema_alpha,
        aang_ema_alpha=aang_ema_alpha,
    )


def main() -> int:
    args = parse_args()
    loaded = load_model(args.model_json)

    compensator = RealtimeCompensator(
        loaded=loaded,
        mv_ema_alpha=loaded.mv_ema_alpha,
        aang_ema_alpha=loaded.aang_ema_alpha,
    )

    print(
        f"Opening {args.port} @ {args.baud}, model={loaded.model_type}, "
        f"window={loaded.n_past}+1+{loaded.n_future}, "
        f"mv_ema_alpha={loaded.mv_ema_alpha:.3f}, aang_ema_alpha={loaded.aang_ema_alpha:.3f}"
    )
    print("Press Ctrl+C to stop.")

    count = 0
    mae_acc = 0.0
    mse_acc = 0.0
    lines_seen = 0
    bad = 0

    with SerialLineGenerator(args.port, args.baud) as reader:
        try:
            for line in reader.lines():
                lines_seen += 1
                if not line or line.startswith("#"):
                    continue
                if line.startswith("STATS,"):
                    continue
                parsed = parse_data_line(line)
                if parsed is None:
                    continue
                stream_format, parts = parsed
                sample = parse_data_parts(parts, stream_format)
                if sample is None:
                    bad += 1
                    continue

                observed = compensator.observe(sample)
                if observed is None:
                    if stream_format != "ads_imu":
                        bad += 1
                    continue
                center, pred = observed
                residual = center.mv_ema - pred
                if math.isfinite(residual):
                    count += 1
                    mae_acc += abs(residual)
                    mse_acc += residual * residual
                    if count % max(1, args.print_every) == 0:
                        rmse = math.sqrt(max(0.0, mse_acc / count))
                        mae = mae_acc / count
                        print(
                            f"n={count} ts_us={center.arduino_us} "
                            f"mV_meas={center.mv_ema:.6f} mV_pred={pred:.6f} "
                            f"res={residual:.6f} mae={mae:.6f} rmse={rmse:.6f}"
                        )
        except KeyboardInterrupt:
            print("\nStopping...")

    if count > 0:
        rmse = math.sqrt(max(0.0, mse_acc / count))
        mae = mae_acc / count
        print(f"Done. predictions={count}, mae={mae:.6f}, rmse={rmse:.6f}, bad={bad}, lines={lines_seen}")
    else:
        print(f"Done. no predictions generated, bad={bad}, lines={lines_seen}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
