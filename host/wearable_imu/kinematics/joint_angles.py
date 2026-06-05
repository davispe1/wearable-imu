"""
joint_angles.py — compute joint angles from adjacent-segment quaternions.

A joint angle is the relative rotation between two adjacent body segments.
Given q_parent and q_child (world-frame quaternions), the joint rotation is:
    q_rel = q_parent.conjugate() * q_child

This is then decomposed into anatomical angles (flexion/extension, ab/adduction, rotation).

TODO:
    - Implement relative_rotation(q_parent, q_child) -> np.ndarray [qw, qx, qy, qz].
    - Implement decompose_euler(q_rel) -> (flex, abd, rot) in degrees.
    - Define the segment adjacency graph for the upper-limb model
      (torso → upper-arm → forearm → hand).
"""

from __future__ import annotations
import numpy as np


def relative_rotation(q_parent: np.ndarray, q_child: np.ndarray) -> np.ndarray:
    """Return q_rel = q_parent^-1 * q_child."""
    # TODO: implement quaternion product with conjugate
    raise NotImplementedError


def decompose_euler(q_rel: np.ndarray) -> tuple[float, float, float]:
    """Decompose q_rel into (flexion, abduction, rotation) in degrees."""
    # TODO: implement anatomical Euler decomposition
    raise NotImplementedError
