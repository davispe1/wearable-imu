"""
viewer.py — live 3D skeleton viewer.

Renders joint positions as a stick-figure skeleton, updated in real time
from the forward-kinematics output. Uses pyqtgraph (GLViewWidget) by default;
swap in open3d if preferred.

TODO:
    - Implement SkeletonViewer: set up GLViewWidget, define bone connectivity.
    - Implement update(joint_positions): redraw skeleton each frame.
    - Run at config.VIZ_FPS in a Qt timer or dedicated render thread.
    - Add overlay: joint angles, node battery, FPS counter.
"""

from __future__ import annotations
from .. import config


class SkeletonViewer:
    """Live 3D skeleton viewer (pyqtgraph GLViewWidget)."""

    # Bone connectivity: list of (joint_a, joint_b) name pairs
    BONES: list[tuple[str, str]] = [
        ("shoulder", "elbow"),
        ("elbow", "wrist"),
        ("wrist", "hand_tip"),
    ]

    def __init__(self) -> None:
        # TODO: initialise Qt app and GLViewWidget
        raise NotImplementedError("Viewer not yet implemented.")

    def update(self, joint_positions: dict[str, object]) -> None:
        """Redraw the skeleton with new joint positions."""
        raise NotImplementedError
