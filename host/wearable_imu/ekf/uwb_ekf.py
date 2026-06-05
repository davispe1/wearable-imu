"""
uwb_ekf.py — Extended Kalman Filter fusing UWB inter-node distances. (Phase 2)

The EKF takes:
    - Orientation-derived joint positions (from forward_kinematics.py) as the prediction.
    - UWB pairwise inter-node distances as measurements.

Output: drift-corrected joint positions.

This module is a PHASE-2 component. v1 runs without it.

TODO (phase 2):
    - Define state vector (joint positions + velocities).
    - Implement predict() — integrate IMU-derived velocities.
    - Implement update(ranges) — correct state with UWB distance measurements.
    - Reference: UIP (SIGGRAPH 2024) uses a per-pair EKF for the same purpose.
"""

from __future__ import annotations
import numpy as np


class UwbEkf:
    """EKF that fuses UWB inter-node ranges to correct IMU drift. (Phase 2)"""

    def __init__(self) -> None:
        # TODO: initialise state, covariance, process/measurement noise
        raise NotImplementedError("UwbEkf is a phase-2 component — not yet implemented.")

    def predict(self, dt: float) -> None:
        """Propagate state using IMU-derived kinematics."""
        raise NotImplementedError

    def update(self, ranges: dict[tuple[int, int], float]) -> np.ndarray:
        """
        Correct state with UWB distance measurements.

        Args:
            ranges: {(node_i, node_j): distance_m, ...}

        Returns:
            Corrected joint positions array.
        """
        raise NotImplementedError
