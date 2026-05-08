"""Plug-in bow pressure stream: load compensation model, serial ADS+IMU, rolling buffer."""

from __future__ import annotations

import argparse
import importlib.util
import json
import multiprocessing as mp
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional
import numpy as np

import matplotlib.pyplot as plt

_REPO_ROOT = Path(__file__).resolve().parent.parent
_RT_DIR = _REPO_ROOT / "realtime-arduino"
for _p in (_REPO_ROOT, _RT_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

# Load inference without importing acceleration_compensation/__init__.py (pandas, etc.).
_inf_path = _REPO_ROOT / "acceleration_compensation" / "inference.py"
_inf_spec = importlib.util.spec_from_file_location("bow_pressure._inference", _inf_path)
if _inf_spec is None or _inf_spec.loader is None:
    raise RuntimeError(f"Cannot load inference module from {_inf_path}")
_inference = importlib.util.module_from_spec(_inf_spec)
# Required so @dataclass in inference can resolve sys.modules[cls.__module__].
sys.modules[_inf_spec.name] = _inference
_inf_spec.loader.exec_module(_inference)
load_model = _inference.load_model
RealtimeCompensator = _inference.RealtimeCompensator
LoadedModel = _inference.LoadedModel

from streamer import SerialLineGenerator, parse_data_line, parse_data_parts

__all__ = ["BowPressure", "BowPressureReading", "CalibrationParams"]


@dataclass(frozen=True)
class CalibrationParams:
    """
    Coefficients from ``analysis_calibration_data.py``: mV as a bilinear form in
    bowing (contact) fraction and applied force (N):

        mV = a * contact_fraction * F + b * contact_fraction + c * F + d
        where b and c are ignored for they are not physically meaningful
    """

    a: float
    d: float
    rmse: float = 0.0
    max_error: float = 0.0

    @classmethod
    def from_json(cls, path: Path | str) -> CalibrationParams:
        raw_path = Path(path).expanduser().resolve()
        with open(raw_path, "r", encoding="utf-8") as f:
            payload = json.load(f)

        # Ignore c and b for they are not physically meaningful
        # d will be intepreted as the mv at rest
        return cls(
            a=float(payload["a"]),
            d=float(payload["d"]),
            rmse=float(payload.get("rmse", 0.0)),
            max_error=float(payload.get("max_error", 0.0)),
        )

    def newtons_from_mv(self, mv: float, contact_fraction: float, mv_at_rest: float = 0) -> float:
        """Solve for F (N) given mV and contact fraction (same as bowing_fraction in calibration)."""
        x = float(contact_fraction)
        z = float(mv)

        denom = self.a * x
        if abs(denom) < 1e-12:
            raise ValueError(
                f"Calibration denominator a*contact_fraction+c is ~0 "
                f"(a={self.a}, c={self.c}, contact_fraction={x})."
            )
        return (z - mv_at_rest) / denom


@dataclass(frozen=True)
class BowPressureReading:
    """One compensated sample: EMA mV, model prediction, and Arduino timestamp."""

    arduino_us: int
    mv_ema: float
    mv_predicted: float
    force_N: Optional[float] = None

def _serial_worker(
    model_path: str,
    serial_port: str,
    baud: int,
    buffer_size: int,
    shared_buffer: "mp.managers.ListProxy[list[float | int]]",
    stop_event: "mp.synchronize.Event",
    serial_timeout_s: float = 0.1,
) -> None:
    # Increase worker process priority for lower latency
    try:
        import os
        os.nice(-5)  # Requires appropriate permissions; silently fails if denied
    except (OSError, ImportError, PermissionError):
        pass
    
    loaded = load_model(Path(model_path))
    compensator = RealtimeCompensator(
        loaded,
        mv_ema_alpha=loaded.mv_ema_alpha,
        aang_ema_alpha=loaded.aang_ema_alpha,
    )
    with SerialLineGenerator(serial_port, baud, timeout_s=serial_timeout_s) as reader:
        for line in reader.lines():
            if stop_event.is_set():
                break
            if not line or line.startswith("#") or line.startswith("STATS,"):
                continue
            parsed = parse_data_line(line)
            if parsed is None:
                continue
            stream_format, parts = parsed
            sample = parse_data_parts(parts, stream_format)
            if sample is None:
                continue
            observed = compensator.observe(sample)
            if observed is None:
                continue
            center, pred_comp = observed
            mv_pred = float(center.mv_ema) - float(pred_comp)
            shared_buffer.append([int(center.arduino_us), float(center.mv_ema), float(mv_pred)])
            while len(shared_buffer) > buffer_size:
                shared_buffer.pop(0)


class BowPressure:
    """
    Load an acceleration-compensation model, read the serial stream, and retain
    the newest ``buffer_size`` readings with predictions.
    """

    def __init__(
        self,
        model_path: Path | str,
        serial_port: str,
        buffer_size: int,
        *,
        baud: int = 230400,
        calibration: CalibrationParams | Path | str | None = None,
        default_contact_fraction: float | None = None,
        serial_timeout_s: float = 0.1,
    ) -> None:
        if buffer_size < 1:
            raise ValueError("buffer_size must be at least 1.")
        self._model_path = Path(model_path).expanduser().resolve()
        self._serial_port = serial_port
        self._baud = int(baud)
        self._buffer_size = int(buffer_size)
        self._buffer: deque[BowPressureReading] = deque(maxlen=buffer_size)
        if calibration is None:
            self._calibration: Optional[CalibrationParams] = None
        elif isinstance(calibration, CalibrationParams):
            self._calibration = calibration
        else:
            self._calibration = CalibrationParams.from_json(calibration)
        self._default_contact_fraction = default_contact_fraction

        self._loaded = load_model(self._model_path)
        self._serial_timeout_s = float(serial_timeout_s)

        self._manager: Optional[mp.managers.SyncManager] = None
        self._shared_buffer = None
        self._stop_event = None
        self._worker: Optional[mp.Process] = None

    @property
    def model_path(self) -> Path:
        return self._model_path

    @property
    def loaded_model(self) -> LoadedModel:
        return self._loaded

    @property
    def calibration(self) -> Optional[CalibrationParams]:
        return self._calibration

    @property
    def buffer(self) -> tuple[BowPressureReading, ...]:
        """Newest readings in order (oldest … newest)."""
        if self._shared_buffer is not None:
            return tuple(
                BowPressureReading(
                    arduino_us=int(item[0]),
                    mv_ema=float(item[1]),
                    mv_predicted=float(item[2]),
                    force_N=None,
                )
                for item in self._shared_buffer
            )
        return tuple(self._buffer)

    @property
    def last_reading(self) -> Optional[BowPressureReading]:
        current = self.buffer
        return current[-1] if current else None

    def connect(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        if self._manager is None:
            self._manager = mp.Manager()
        if self._shared_buffer is None:
            self._shared_buffer = self._manager.list()
        if self._stop_event is None:
            self._stop_event = mp.Event()
        self._stop_event.clear()

        self._worker = mp.Process(
            target=_serial_worker,
            args=(
                str(self._model_path),
                self._serial_port,
                self._baud,
                self._buffer_size,
                self._shared_buffer,
                self._stop_event,
                self._serial_timeout_s,
            ),
            daemon=True,
        )
        self._worker.start()

        # Ensure the stream is producing data before returning.
        # Use shorter polling interval (2ms) for lower latency
        while True:
            if self._shared_buffer is not None and len(self._shared_buffer) > 0:
                break
            if self._worker is None or not self._worker.is_alive():
                raise RuntimeError("Serial worker exited before first sample arrived.")
            time.sleep(0.002)
        
        print("Bow sensor connected.")
        print(f"Buffer size: {self._buffer_size}")

    def close(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
            if self._worker.is_alive():
                self._worker.terminate()
                self._worker.join(timeout=1.0)
            self._worker = None
        if self._manager is not None:
            self._manager.shutdown()
            self._manager = None
        self._shared_buffer = None
        self._stop_event = None

    def flush(self) -> None:
        if self._worker is None or not self._worker.is_alive():
            raise RuntimeError("Serial worker not running; call connect() or use 'with BowPressure(...)'.")
        if self._shared_buffer is None:
            return
        # multiprocessing.Manager().list() proxy has no clear(), so drain manually.
        while len(self._shared_buffer) > 0:
            self._shared_buffer.pop()

    def __enter__(self) -> BowPressure:
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def get_raw(self) -> list[BowPressureReading] :
        if self._worker is None or not self._worker.is_alive():
            raise RuntimeError("Serial worker not running; call connect() or use 'with BowPressure(...)'.")
        return [
            BowPressureReading(
                arduino_us=int(item[0]),
                mv_ema=float(item[1]),
                mv_predicted=float(item[2]),
                force_N=None,
            )
            for item in self._shared_buffer
        ]

    def get_sample(self, contact_fraction: float | None = None, mv_at_rest: float = 0) -> list[float]:
        """
        Compute and return force values for each buffered element (oldest … newest).
        """
        if self._worker is None or not self._worker.is_alive():
            raise RuntimeError("Serial worker not running; call connect() or use 'with BowPressure(...)'.")
        frac = self._default_contact_fraction if contact_fraction is None else contact_fraction
        if self._calibration is None:
            raise RuntimeError("No calibration loaded; pass calibration= to BowPressure().")
        if frac is None:
            raise ValueError("contact_fraction required (or set default_contact_fraction=).")
        return np.mean([self._calibration.newtons_from_mv(reading.mv_predicted, frac, mv_at_rest) for reading in self.buffer])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot bow force readings from the serial stream.")
    parser.add_argument(
        "--model-path",
        default="recordings/calibration1/ads1263_stream_mlp_model.json",
        help="Path to the compensation model JSON.",
    )
    parser.add_argument(
        "--serial-port",
        default="/dev/ttyACM0",
        help="Serial port for the bow pressure sensor.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=230400,
        help="Serial baud rate.",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=200,
        help="Number of recent samples to keep and plot.",
    )
    parser.add_argument(
        "--calibration",
        default="calibrations/bow2_crap_params.json",
        help="Path to the calibration JSON used to convert mV into force.",
    )
    parser.add_argument(
        "--contact-fraction",
        type=float,
        default=0.5,
        help="Contact fraction used for force conversion (default: 0.5).",
    )
    parser.add_argument(
        "--mv-at-rest",
        type=float,
        default=0.0,
        help="Resting mV offset used during force conversion.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with BowPressure(
        model_path=args.model_path,
        serial_port=args.serial_port,
        buffer_size=args.buffer_size,
        baud=args.baud,
        calibration=args.calibration,
        default_contact_fraction=args.contact_fraction,
    ) as bow_pressure:
        plt.ion()
        fig, ax = plt.subplots(figsize=(12, 5))
        (line,) = ax.plot([], [], linewidth=1.0, color="tab:red")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Force (N)")
        ax.set_title(f"Bow force readings (contact_fraction={args.contact_fraction})")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

        try:
            while True:
                buffer = bow_pressure.buffer
                if not buffer:
                    plt.pause(0.01)
                    continue

                elapsed_s = [
                    (reading.arduino_us - buffer[0].arduino_us) / 1_000_000.0
                    for reading in buffer
                ]
                calibration = bow_pressure.calibration
                if calibration is None:
                    raise RuntimeError("No calibration loaded; pass calibration= to BowPressure().")
                force_readings = [
                    calibration.newtons_from_mv(
                        reading.mv_predicted,
                        args.contact_fraction,
                        args.mv_at_rest,
                    )
                    for reading in buffer
                ]

                line.set_data(elapsed_s, force_readings)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw_idle()
                plt.pause(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            plt.ioff()
            plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
