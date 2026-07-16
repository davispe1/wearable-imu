"""
model.py — shared data types for the host pipeline.

These are transport-agnostic: ingest (hardware or sim) produces them, and every
downstream stage (sync, orientation, kinematics) consumes them. Keeping them here
(rather than in protocol.py) avoids a circular import between protocol and sim.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class NodeSample:
    """One IMU sample from one node, in raw counts (host applies scale).

    For RAW_9DOF the quaternion fields are None; for SFLP_QUAT the accel/gyro
    fields are None and (qw..qz) hold the Q15-decoded unit quaternion.
    """

    node_id: int
    node_seq: int
    timestamp_us: int

    # RAW_9DOF payload (raw int16 counts)
    accel: tuple[int, int, int] | None = None      # (ax, ay, az)
    gyro: tuple[int, int, int] | None = None        # (gx, gy, gz)

    # SFLP_QUAT payload (unit quaternion, already decoded from Q15)
    quat: tuple[float, float, float, float] | None = None   # (qw, qx, qy, qz)

    # Magnetometer raw counts (present in both formats; zeros if MAG disabled)
    mag: tuple[int, int, int] = (0, 0, 0)


@dataclass(slots=True)
class ImuFrame:
    """A decoded MSG_IMU frame: one timestamp epoch's worth of node samples."""

    frame_seq: int
    fmt: int                                   # 0 = RAW_9DOF, 1 = SFLP_QUAT
    samples: list[NodeSample] = field(default_factory=list)


@dataclass(slots=True)
class RangePair:
    node_a: int
    node_b: int
    dist_mm: int


@dataclass(slots=True)
class RangeFrame:
    """A decoded MSG_RANGE frame: UWB inter-node distances for one ranging round."""

    frame_seq: int
    timestamp_us: int
    pairs: list[RangePair] = field(default_factory=list)
