"""
gaitlib/rawdata.py — Raw 9-DOF input contract, parsing, and per-channel rate alignment.

INPUT DATA CONTRACT
-------------------
The library consumes raw 9-DOF inertial/magnetic data only — no serial ports, sockets,
pins, or registers. Each **sample** carries:

    timestamp   seconds, monotonic per node, any 0-origin           (float)
    node_id     sensor id, must match a key in the mounting config  (str)
    ax, ay, az  linear acceleration, sensor frame                   (m/s^2)
    gx, gy, gz  angular velocity, sensor frame                      (rad/s)
    mx, my, mz  magnetometer, sensor frame                          (any consistent units;
                                                                     calibration normalises
                                                                     scale + bias)

``compute`` accepts the raw data in any of these equivalent forms:

  1. **Per-node dict** (the canonical internal form)::

        {node_id: {"t": (N,), "acc": (N,3), "gyr": (N,3), "mag": (N,3)
                   [, "t_mag": (M,)]}}

     If the magnetometer is sampled at a different rate, give it on its own ``t_mag``
     timeline (or simply a different length); it is aligned to the IMU timeline internally.

  2. **Long-format rows** — an iterable of per-sample mappings or sequences with the
     columns above (``node_id, timestamp, ax..mz``). Rows are grouped by ``node_id`` and
     sorted by time.

  3. **Long-format array** — a 2-D ``(rows, 11)`` array plus the matching ``node`` column,
     or a structured/record array whose field names match the schema (aliases accepted).

SAMPLE-RATE HANDLING
--------------------
The ideal case is all 9 DOF synchronous at the IMU rate (e.g. 100 Hz) — then alignment is
a no-op. When the magnetometer arrives at a different rate, :func:`align_node` resamples it
(linear interpolation) onto the accel/gyro sample instants so fusion sees one synchronous
9-DOF stream. The expected rates are declared in the mounting config (``rates.imu_hz`` /
``rates.mag_hz``); actual timing is read from the timestamps when available.

FIRMWARE NOTE: configure the magnetometer at the same output data rate (ODR) as the IMU
(e.g. 100 Hz) so the 9 DOF are already synchronous and no resampling is needed.
"""
from __future__ import annotations

import warnings
import numpy as np


# Accepted column aliases when parsing arbitrary long-format input.
_T_ALIASES = ("timestamp", "t", "t_s", "time", "time_s", "t_opt_s", "t_native_s")
_NODE_ALIASES = ("node_id", "node", "sensor", "id")
_AXES = {
    "ax": ("ax", "acc_x", "a_x"), "ay": ("ay", "acc_y", "a_y"), "az": ("az", "acc_z", "a_z"),
    "gx": ("gx", "gyr_x", "g_x"), "gy": ("gy", "gyr_y", "g_y"), "gz": ("gz", "gyr_z", "g_z"),
    "mx": ("mx", "mag_x", "m_x"), "my": ("my", "mag_y", "m_y"), "mz": ("mz", "mag_z", "m_z"),
}


# --------------------------------------------------------------------------- #
def _interp_cols(t_grid, t_src, sig):
    sig = np.asarray(sig, float)
    if sig.ndim == 1:
        return np.interp(t_grid, t_src, sig)
    return np.column_stack([np.interp(t_grid, t_src, sig[:, k]) for k in range(sig.shape[1])])


def align_node(node: dict, *, imu_hz: float | None = None,
               mag_hz: float | None = None) -> dict:
    """Return a node stream with the magnetometer aligned to the accel/gyro timeline.

    ``node`` is ``{"t","acc","gyr","mag"[,"t_mag"]}``. If the mag has its own ``t_mag`` or a
    differing length, it is interpolated onto ``t``. When the mag already matches the IMU
    samples this is a no-op (returns the same arrays).
    """
    t = np.asarray(node["t"], float)
    acc = np.asarray(node["acc"], float)
    gyr = np.asarray(node["gyr"], float)
    mag = np.asarray(node.get("mag"), float) if node.get("mag") is not None else None
    out = {"t": t, "acc": acc, "gyr": gyr}

    if mag is None or len(mag) == 0:
        out["mag"] = np.zeros((len(t), 3))
        out["mag_present"] = False
        return out

    t_mag = node.get("t_mag")
    if t_mag is not None:
        t_mag = np.asarray(t_mag, float)
    elif len(mag) != len(t):
        # No explicit mag timeline but a different length -> assume mag spans the same
        # interval at its own uniform rate (mag_hz if given, else inferred from counts).
        if mag_hz and imu_hz:
            t_mag = t[0] + np.arange(len(mag)) / float(mag_hz)
        else:
            t_mag = np.linspace(t[0], t[-1], len(mag))
    else:
        # Same length as IMU: already synchronous -> no resampling.
        out["mag"] = mag
        out["mag_present"] = True
        return out

    out["mag"] = _interp_cols(t, t_mag, mag)   # align mag -> IMU instants
    out["mag_present"] = True
    return out


# --------------------------------------------------------------------------- #
def _resolve_long_columns(names):
    low = [str(n).strip().lower() for n in names]
    def find(aliases):
        return next((low.index(a) for a in aliases if a in low), None)
    t_idx = find(_T_ALIASES)
    node_idx = find(_NODE_ALIASES)
    cols = {k: find(al) for k, al in _AXES.items()}
    return t_idx, node_idx, cols


def _rows_to_streams(rows):
    """Group an iterable of per-sample mappings/sequences into per-node arrays."""
    rows = list(rows)
    if not rows:
        return {}
    first = rows[0]
    if isinstance(first, dict):
        names = list(first.keys())
        t_idx, node_idx, cols = _resolve_long_columns(names)
        if t_idx is None or node_idx is None:
            raise ValueError("row dicts need a timestamp and a node_id column "
                             f"(got keys {names})")
        keys = list(names)
        get = lambda r, j: r.get(keys[j]) if j is not None else None
    else:
        raise ValueError("sequence rows must be paired with a column spec; pass a dict per "
                         "row, a per-node dict, or a structured array instead")

    buckets = {}
    for r in rows:
        node = str(get(r, node_idx))
        buckets.setdefault(node, []).append(r)
    streams = {}
    for node, rr in buckets.items():
        t = np.array([float(get(x, t_idx)) for x in rr])
        def col(name):
            j = cols[name]
            return (np.array([float(get(x, j)) for x in rr]) if j is not None
                    else np.zeros(len(rr)))
        acc = np.column_stack([col("ax"), col("ay"), col("az")])
        gyr = np.column_stack([col("gx"), col("gy"), col("gz")])
        mag = np.column_stack([col("mx"), col("my"), col("mz")])
        order = np.argsort(t)
        streams[node] = {"t": t[order], "acc": acc[order], "gyr": gyr[order], "mag": mag[order]}
    return streams


def _structured_to_streams(arr):
    names = arr.dtype.names
    t_idx, node_idx, cols = _resolve_long_columns(names)
    if t_idx is None or node_idx is None:
        raise ValueError("structured array needs timestamp + node_id fields")
    name = lambda j: names[j]
    nodes = np.asarray(arr[name(node_idx)]).astype(str)
    t = np.asarray(arr[name(t_idx)], float)
    def col(c):
        j = cols[c]
        return np.asarray(arr[name(j)], float) if j is not None else np.zeros(len(arr))
    acc = np.column_stack([col("ax"), col("ay"), col("az")])
    gyr = np.column_stack([col("gx"), col("gy"), col("gz")])
    mag = np.column_stack([col("mx"), col("my"), col("mz")])
    streams = {}
    for nd in np.unique(nodes):
        sel = nodes == nd
        order = np.argsort(t[sel])
        streams[str(nd)] = {"t": t[sel][order], "acc": acc[sel][order],
                            "gyr": gyr[sel][order], "mag": mag[sel][order]}
    return streams


# --------------------------------------------------------------------------- #
def load_raw(raw_data, config) -> dict:
    """Normalise any accepted input form to aligned per-node streams.

    Returns ``{node: {"t","acc","gyr","mag","mag_present"}}`` with the magnetometer aligned
    to each node's accel/gyro timeline (a no-op when already synchronous). Only nodes
    declared in the mounting ``config.sensors`` are kept; extras are dropped with a warning.
    """
    # 1) per-node dict?
    if isinstance(raw_data, dict) and raw_data and all(
            isinstance(v, dict) and "acc" in v for v in raw_data.values()):
        streams = {k: dict(v) for k, v in raw_data.items()}
    # 2) structured / record numpy array?
    elif isinstance(raw_data, np.ndarray) and raw_data.dtype.names:
        streams = _structured_to_streams(raw_data)
    # 3) iterable of rows
    else:
        streams = _rows_to_streams(raw_data)

    declared = set(config.sensors.keys())
    out = {}
    for node, nd in streams.items():
        if node not in declared:
            warnings.warn(f"gaitlib: node {node!r} not in mounting config; ignoring it")
            continue
        out[node] = align_node(nd, imu_hz=config.imu_hz, mag_hz=config.mag_hz)
    if not out:
        raise ValueError("no data for any node declared in the mounting config "
                         f"(declared {sorted(declared)}, got {sorted(streams)})")
    return out


def infer_rate(t) -> float:
    """Sample rate (Hz) from a timestamp vector (median dt)."""
    t = np.asarray(t, float)
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.0
    return float(1.0 / dt) if dt > 0 else 0.0
