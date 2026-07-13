"""
opensim_export/segment_map.py — segment kind + side -> OpenSim IMU column name.

OpenSense expects each orientation column to be named after the model body's IMU frame. The
Rajagopal model names its lower-limb IMU frames ``<body>_imu``; the relevant bodies are::

    pelvis -> pelvis_imu          (no side suffix)
    thigh  -> femur_<r|l>_imu
    shank  -> tibia_<r|l>_imu
    foot   -> calcn_<r|l>_imu

The side char comes from the session's measured leg (right -> ``r``, left -> ``l``). Only the
measured leg is emitted — the other leg is never fabricated, because this is a single-leg rig
and OpenSense simply will not place IMUs on bodies that have no column.

Columns are ordered proximal -> distal (pelvis, femur, tibia, calcn), matching anatomy and
the OpenSense convention of listing the base/root segment first.
"""
from __future__ import annotations

# Segment kind -> OpenSim body stem. ``pelvis`` carries no side suffix.
OPENSIM_STEM = {"pelvis": "pelvis", "thigh": "femur", "shank": "tibia", "foot": "calcn"}

# Proximal -> distal ordering for the emitted columns.
KIND_ORDER = {"pelvis": 0, "thigh": 1, "shank": 2, "foot": 3}


def side_char(side: str) -> str:
    """'r' or 'l' from a side label ('right'/'left', or anything starting r/l)."""
    return "l" if str(side).lower().startswith("l") else "r"


def imu_column(kind: str, side: str) -> str:
    """OpenSim IMU column name for a segment kind on a given side.

    >>> imu_column("pelvis", "right")
    'pelvis_imu'
    >>> imu_column("thigh", "left")
    'femur_l_imu'
    """
    kind = str(kind).lower()
    if kind not in OPENSIM_STEM:
        raise ValueError(f"unknown segment kind {kind!r}; valid kinds are {tuple(OPENSIM_STEM)}")
    stem = OPENSIM_STEM[kind]
    if kind == "pelvis":
        return f"{stem}_imu"
    return f"{stem}_{side_char(side)}_imu"


def ordered_columns(config) -> list[tuple[str, str, str]]:
    """Ordered ``(node_id, kind, column_name)`` for every measured segment in a config.

    Sorted proximal -> distal. Only the segments present in ``config.sensors`` are returned —
    the unmeasured leg is never fabricated.
    """
    cols = [(node, kind, imu_column(kind, config.side))
            for node, kind in config.sensors.items()]
    cols.sort(key=lambda c: KIND_ORDER.get(c[1], 99))
    return cols
