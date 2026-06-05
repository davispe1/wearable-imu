"""
forward_kinematics.py — Denavit-Hartenberg forward kinematics for the upper limb.

Converts joint angles + segment lengths (from config.py) into 3D joint positions
for rendering.

DH parameter table (to be defined — see docs/05-host-pipeline.md):
    shoulder (3 DOF) → elbow (1 DOF) → wrist (2 DOF)

TODO:
    - Define DH parameter table.
    - Implement dh_matrix(a, alpha, d, theta) -> 4×4 homogeneous transform.
    - Implement solve(joint_angles) -> dict[str, np.ndarray] of 3D joint positions.
"""

from __future__ import annotations
import numpy as np
from .. import config


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Standard DH transformation matrix."""
    # TODO: implement
    raise NotImplementedError


def solve(joint_angles: dict) -> dict[str, np.ndarray]:
    """
    Compute 3D joint positions from joint angles and segment lengths.

    Args:
        joint_angles: dict mapping joint name → angle(s) in radians.

    Returns:
        dict mapping joint name → 3D position [x, y, z] in metres.
    """
    # TODO: implement DH chain: shoulder → elbow → wrist → hand tip
    raise NotImplementedError
