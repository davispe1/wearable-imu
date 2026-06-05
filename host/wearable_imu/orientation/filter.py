"""
filter.py — orientation filter dispatcher.

Selects and runs the filter specified by config.ORIENTATION_FILTER:
    "complementary" — simple accel + gyro complementary filter
    "madgwick"      — Madgwick AHRS (beta tuning parameter)
    "vqf"           — Versatile Quaternion-based Filter (Laidig & Seel 2022)
    "sflp"          — pass-through: use quaternion already computed on the node

Each filter exposes the same interface:
    update(ax, ay, az, gx, gy, gz, [mx, my, mz], dt) -> np.ndarray [qw, qx, qy, qz]

TODO:
    - Implement ComplementaryFilter.
    - Wrap ahrs.filters.Madgwick.
    - Wrap ahrs.filters.VQF (or the standalone vqf package).
    - Implement SFLPPassthrough (just return the quaternion from the node).
    - Factory function: get_filter() → returns the configured filter instance.
"""

from __future__ import annotations
import numpy as np
from .. import config


class OrientationFilter:
    """Base class / interface for orientation filters."""

    def update(
        self,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
        dt: float,
        mx: float = 0.0, my: float = 0.0, mz: float = 0.0,
    ) -> np.ndarray:
        """Return quaternion [qw, qx, qy, qz]."""
        raise NotImplementedError


def get_filter() -> OrientationFilter:
    """Return the filter instance selected by config.ORIENTATION_FILTER."""
    # TODO: instantiate and return the right filter
    raise NotImplementedError(
        f"Orientation filter {config.ORIENTATION_FILTER!r} not yet implemented."
    )
