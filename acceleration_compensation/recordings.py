"""Load CSV files produced by realtime-arduino/stream_logger.py."""

from __future__ import annotations

from pathlib import Path
from typing import FrozenSet

import pandas as pd

# stream_logger.py column names
ADS_COLUMNS: FrozenSet[str] = frozenset(
    {
        "host_iso_time",
        "host_epoch_s",
        "arduino_timestamp_us",
        "raw",
        "mV",
    }
)
IMU_COLUMNS: FrozenSet[str] = frozenset(
    {
        "acc_x_mg",
        "acc_y_mg",
        "acc_z_mg",
        "gyr_x_dps",
        "gyr_y_dps",
        "gyr_z_dps",
    }
)

NUMERIC_ADS = ("host_epoch_s", "arduino_timestamp_us", "raw", "mV")
NUMERIC_IMU = (
    "acc_x_mg",
    "acc_y_mg",
    "acc_z_mg",
    "gyr_x_dps",
    "gyr_y_dps",
    "gyr_z_dps",
)


def load_recorded_csv(
    path: str | Path,
    *,
    require_imu: bool = False,
) -> pd.DataFrame:
    """
    Read a recorded session CSV.

    Parameters
    ----------
    path
        Path to CSV (e.g. from ``stream_logger.py``).
    require_imu
        If True, require IMU columns (combined ADS+BNO055 stream). Use this when
        fitting models that predict ADC from acc/gyro.

    Returns
    -------
    pandas.DataFrame
        A copy with expected numeric columns converted to float/int as appropriate.
    """
    p = Path(path).expanduser().resolve()
    if not p.is_file():
        raise FileNotFoundError(f"Not a file: {p}")

    df = pd.read_csv(p, encoding="utf-8")
    cols = set(df.columns.astype(str))

    missing_ads = ADS_COLUMNS - cols
    if missing_ads:
        raise ValueError(
            f"{p.name}: missing ADS columns {sorted(missing_ads)}. "
            f"Expected at least {sorted(ADS_COLUMNS)}."
        )

    if require_imu:
        missing_imu = IMU_COLUMNS - cols
        if missing_imu:
            raise ValueError(
                f"{p.name}: missing IMU columns {sorted(missing_imu)}. "
                "Record with ads1263_bno055_reader + stream_logger, or pass require_imu=False."
            )

    for name in NUMERIC_ADS:
        if name in df.columns:
            df[name] = pd.to_numeric(df[name], errors="coerce")

    if not require_imu and not IMU_COLUMNS.issubset(cols):
        return df

    for name in NUMERIC_IMU:
        if name in df.columns:
            df[name] = pd.to_numeric(df[name], errors="coerce")

    return df


def has_imu_columns(df: pd.DataFrame) -> bool:
    """True if ``df`` includes acc/gyro columns from a combined stream."""
    return IMU_COLUMNS.issubset(df.columns)
