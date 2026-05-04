"""Plug-in bow pressure stream: load compensation model, serial ADS+IMU, rolling buffer."""

from __future__ import annotations

import importlib.util
import json
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Literal, Optional

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
    """

    a: float
    b: float
    c: float
    d: float
    rmse: float = 0.0
    max_error: float = 0.0

    @classmethod
    def from_json(cls, path: Path | str) -> CalibrationParams:
        raw_path = Path(path).expanduser().resolve()
        with open(raw_path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        return cls(
            a=float(payload["a"]),
            b=float(payload["b"]),
            c=float(payload["c"]),
            d=float(payload["d"]),
            rmse=float(payload.get("rmse", 0.0)),
            max_error=float(payload.get("max_error", 0.0)),
        )

    def newtons_from_mv(self, mv: float, contact_fraction: float) -> float:
        """Solve for F (N) given mV and contact fraction (same as bowing_fraction in calibration)."""
        x = float(contact_fraction)
        z = float(mv)
        denom = self.a * x + self.c
        if abs(denom) < 1e-12:
            raise ValueError(
                f"Calibration denominator a*contact_fraction+c is ~0 "
                f"(a={self.a}, c={self.c}, contact_fraction={x})."
            )
        return (z - self.b * x - self.d) / denom


@dataclass(frozen=True)
class BowPressureReading:
    """One compensated sample: EMA mV, model prediction, and Arduino timestamp."""

    arduino_us: int
    mv_ema: float
    mv_predicted: float
    force_N: Optional[float] = None

    @property
    def residual(self) -> float:
        return self.mv_ema - self.mv_predicted


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
        force_mv_source: Literal["predicted", "ema"] = "predicted",
    ) -> None:
        if buffer_size < 1:
            raise ValueError("buffer_size must be at least 1.")
        self._model_path = Path(model_path).expanduser().resolve()
        self._serial_port = serial_port
        self._baud = int(baud)
        self._buffer: deque[BowPressureReading] = deque(maxlen=buffer_size)
        if calibration is None:
            self._calibration: Optional[CalibrationParams] = None
        elif isinstance(calibration, CalibrationParams):
            self._calibration = calibration
        else:
            self._calibration = CalibrationParams.from_json(calibration)
        self._default_contact_fraction = default_contact_fraction
        self._force_mv_source = force_mv_source

        self._loaded = load_model(self._model_path)
        self._compensator = RealtimeCompensator(
            self._loaded,
            mv_ema_alpha=self._loaded.mv_ema_alpha,
            aang_ema_alpha=self._loaded.aang_ema_alpha,
        )

        self._reader: Optional[SerialLineGenerator] = None
        self._line_iter: Optional[Iterator[str]] = None

    @property
    def model_path(self) -> Path:
        return self._model_path

    @property
    def loaded_model(self) -> LoadedModel:
        return self._loaded

    @property
    def calibration(self) -> Optional[CalibrationParams]:
        return self._calibration

    def newtons(
        self,
        mv: float,
        contact_fraction: float | None = None,
    ) -> float:
        """Convert mV to newtons using loaded calibration (same inversion as ``CalibrationParams.newtons_from_mv``)."""
        if self._calibration is None:
            raise RuntimeError("No calibration loaded; pass calibration= to BowPressure().")
        frac = self._default_contact_fraction if contact_fraction is None else contact_fraction
        if frac is None:
            raise ValueError("contact_fraction required (or set default_contact_fraction=).")
        return self._calibration.newtons_from_mv(mv, frac)

    @property
    def buffer(self) -> tuple[BowPressureReading, ...]:
        """Newest readings in order (oldest … newest)."""
        return tuple(self._buffer)

    @property
    def last_reading(self) -> Optional[BowPressureReading]:
        return self._buffer[-1] if self._buffer else None

    def connect(self) -> None:
        if self._reader is not None:
            return
        self._reader = SerialLineGenerator(self._serial_port, self._baud)
        self._reader.__enter__()
        self._line_iter = self._reader.lines()

    def close(self) -> None:
        if self._reader is not None:
            self._reader.__exit__(None, None, None)
            self._reader = None
            self._line_iter = None

    def __enter__(self) -> BowPressure:
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _next_line(self) -> str:
        if self._line_iter is None:
            raise RuntimeError("Serial not connected; call connect() or use 'with BowPressure(...)'.")
        return next(self._line_iter)

    def poll(self, contact_fraction: float | None = None) -> BowPressureReading:
        """
        Block until one full window produces a prediction, append it to the buffer, and return it.

        If calibration is loaded and ``contact_fraction`` (or ``default_contact_fraction``) is set,
        ``force_N`` is filled using the bilinear fit from ``analysis_calibration_data.py``.
        mV used for that mapping defaults to ``mv_predicted`` (see ``force_mv_source``).
        """
        while True:
            line = self._next_line()
            if not line or line.startswith("#") or line.startswith("STATS,"):
                continue
            parsed = parse_data_line(line)
            if parsed is None:
                continue
            stream_format, parts = parsed
            sample = parse_data_parts(parts, stream_format)
            if sample is None:
                continue
            observed = self._compensator.observe(sample)
            if observed is None:
                continue
            center, pred_comp = observed
            mv_pred = float(sample.mv_raw) - float(pred_comp)
            frac = contact_fraction if contact_fraction is not None else self._default_contact_fraction
            force_n: Optional[float] = None
            if self._calibration is not None and frac is not None:
                mv_for_f = mv_pred if self._force_mv_source == "predicted" else center.mv_ema
                force_n = self._calibration.newtons_from_mv(mv_for_f, frac)
            reading = BowPressureReading(
                arduino_us=center.arduino_us,
                mv_ema=center.mv_ema,
                mv_predicted=mv_pred,
                force_N=force_n,
            )
            self._buffer.append(reading)
            return reading
