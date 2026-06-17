"""
pipeline/export_open3d.py — Export a processed session for the external Open3D viewer.

This is a **separate export step**, not part of gaitlib or the app's runtime: it only
*reads* results that gaitlib already produced and writes two small files. It does NOT import
``open3d`` and adds no dependency to gaitlib or the app — the Open3D viewer lives in its own
project and consumes these files.

Output (written into the session's output folder, i.e. ``<session>/results/``):

  gait_frames.csv : t_s, hip_deg, knee_deg, ankle_deg   (one row per sample)
                    The clean **sagittal flexion** angles from gaitlib
                    (``results.joints[<joint>]["flexion"]`` — NOT 3D orientation), for a
                    SINGLE leg (the leg declared in the mounting config).

  meta.json       : {"leg": "<right|left>", "fs": <Hz>,
                     "segment_lengths_m": {"pelvis":.., "thigh":.., "shank":.., "foot":..}}

The segment lengths are estimated from the subject's stature using Winter's body-segment
proportions; when no stature is available a default height is used and noted in the file
(``segment_lengths_source``).

LIVE-HARDWARE PATH (future)
---------------------------
The ``gait_frames`` schema (t_s, hip_deg, knee_deg, ankle_deg) is the viewer's frame
contract. The exact same per-frame record can be **streamed over a socket** from live
hardware instead of written to a CSV — the per-frame schema is identical, so the Open3D
viewer needs no change to switch from file playback to a live feed.

Usage
-----
    python -m pipeline.export_open3d <session_dir>

``<session_dir>`` is a session folder (app layout: ``raw/data.csv`` + ``session.json``,
optionally a computed ``results/``). If results are already cached they are reused; otherwise
gaitlib is run to produce them. The two files are written under ``<session_dir>/results/``.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

# Winter's body-segment lengths as a fraction of stature (H). These match the
# `visualization.anthropometry` ratios already used elsewhere in this repo.
WINTER_SEGMENT_FRAC = {"pelvis": 0.100, "thigh": 0.245, "shank": 0.246, "foot": 0.152}
DEFAULT_HEIGHT_M = 1.70

# The single-leg joint chain the viewer expects, proximal -> distal.
VIEWER_JOINTS = ("hip", "knee", "ankle")


def segment_lengths_m(height_m: float) -> dict:
    """Per-segment lengths (m) from stature via Winter's proportions."""
    return {seg: round(height_m * frac, 4) for seg, frac in WINTER_SEGMENT_FRAC.items()}


def infer_leg(*, side=None, foot_node=None, sensors=None) -> str:
    """Best-effort 'right'/'left' from session metadata or mounting config."""
    if side:
        s = str(side).lower()
        if s.startswith("r"):
            return "right"
        if s.startswith("l"):
            return "left"
    if foot_node:
        f = str(foot_node).upper()
        if f.startswith("R"):
            return "right"
        if f.startswith("L"):
            return "left"
    for seg in (sensors or {}).values():
        s = str(seg).lower()
        if "right" in s:
            return "right"
        if "left" in s:
            return "left"
    return "right"


# --------------------------------------------------------------------------- #
def write_open3d_inputs(out_dir, t_s, angles_deg, *, leg, fs, height_m=None,
                        height_source=None):
    """Write ``gait_frames.csv`` + ``meta.json`` for the Open3D viewer.

    Parameters
    ----------
    out_dir : str               destination folder (created if missing)
    t_s : (N,) array            sample times (s)
    angles_deg : dict           {"hip","knee","ankle"} -> (N,) sagittal flexion (deg)
    leg : str                   "right" or "left"
    fs : float                  sample rate (Hz)
    height_m : float or None    subject stature; default used (and noted) if None
    height_source : str or None human-readable provenance for the stature

    Returns ``(frames_path, meta_path, meta)``.
    """
    os.makedirs(out_dir, exist_ok=True)
    t_s = np.asarray(t_s, float)
    n = len(t_s)

    notes = []
    cols = {"t_s": t_s}
    for j in VIEWER_JOINTS:
        a = angles_deg.get(j)
        if a is None:
            notes.append(f"{j}_deg missing in results — written as zeros")
            cols[f"{j}_deg"] = np.zeros(n)
        else:
            cols[f"{j}_deg"] = np.asarray(a, float)

    frames_path = os.path.join(out_dir, "gait_frames.csv")
    arr = np.column_stack([cols[k] for k in ("t_s", "hip_deg", "knee_deg", "ankle_deg")])
    np.savetxt(frames_path, arr, delimiter=",",
               header="t_s,hip_deg,knee_deg,ankle_deg", comments="", fmt="%.6g")

    if height_m is None:
        height_m = DEFAULT_HEIGHT_M
        height_source = height_source or f"default ({DEFAULT_HEIGHT_M} m — no subject stature available)"
    else:
        height_source = height_source or "subject stature"

    meta = {
        "leg": leg,
        "fs": round(float(fs), 4),
        "segment_lengths_m": segment_lengths_m(height_m),
        "segment_lengths_source": f"Winter body-segment proportions × stature ({height_source})",
        "stature_m": round(float(height_m), 4),
        "n_frames": n,
        "frame_schema": ["t_s", "hip_deg", "knee_deg", "ankle_deg"],
    }
    if notes:
        meta["notes"] = notes
    meta_path = os.path.join(out_dir, "meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    return frames_path, meta_path, meta


# --------------------------------------------------------------------------- #
def _select_leg_joints(joint_names, leg):
    """Map the viewer joints (hip/knee/ankle) onto whatever names the results use.

    Handles single-leg configs ('ankle'/'knee'/'hip') and leg-suffixed names
    ('hip_r'/'hip_l', etc.). Returns {viewer_joint: results_joint_name}.
    """
    names = set(joint_names)
    suffix = "_r" if leg == "right" else "_l"
    mapping = {}
    for j in VIEWER_JOINTS:
        if j in names:
            mapping[j] = j
        elif j + suffix in names:
            mapping[j] = j + suffix
    return mapping


def export_from_results(results, out_dir, *, leg=None, height_m=None, height_source=None):
    """Export straight from an in-memory gaitlib ``GaitResults`` object.

    Pulls ``results.joints[<joint>]["flexion"]`` for the single leg. Used by the app's
    "Export for Open3D" button (results already in memory) and as the compute path of the
    CLI.
    """
    cfg = results.config or {}
    if leg is None:
        leg = infer_leg(foot_node=cfg.get("foot_node"), sensors=cfg.get("sensors"))
    mapping = _select_leg_joints(results.joints.keys(), leg)
    angles = {vj: results.joints[name]["flexion"] for vj, name in mapping.items()}
    return write_open3d_inputs(out_dir, results.t, angles, leg=leg, fs=results.fs,
                               height_m=height_m, height_source=height_source)


# --------------------------------------------------------------------------- #
def _read_session_meta(session_dir):
    p = os.path.join(session_dir, "session.json")
    if os.path.exists(p):
        return json.load(open(p))
    return {}


def _export_from_cached_timeseries(session_dir, ts_path, meta):
    """Read a processed session's cached results/timeseries.csv (which stores the gaitlib
    flexion as the <joint>_deg columns) and write the viewer inputs — no recompute."""
    d = np.genfromtxt(ts_path, delimiter=",", names=True)
    names = d.dtype.names
    leg = infer_leg(side=meta.get("side"), foot_node=meta.get("foot_node"))
    mapping = _select_leg_joints(
        [n[:-4] for n in names if n.endswith("_deg") and not n.endswith("_9dof")], leg)
    angles = {vj: np.asarray(d[f"{name}_deg"], float) for vj, name in mapping.items()}
    t = np.asarray(d["t_s"], float)
    fs = meta.get("fs_hz") or (1.0 / np.median(np.diff(t)) if len(t) > 1 else 0.0)
    height_m = meta.get("height_m")
    out_dir = os.path.join(session_dir, "results")
    return write_open3d_inputs(out_dir, t, angles, leg=leg, fs=fs, height_m=height_m)


def export_session(session_dir):
    """Export Open3D inputs for a session folder. Reuses cached results when present,
    otherwise runs gaitlib (via the app's pipeline runner) to produce them first."""
    ts_path = os.path.join(session_dir, "results", "timeseries.csv")
    meta = _read_session_meta(session_dir)
    if os.path.exists(ts_path):
        return _export_from_cached_timeseries(session_dir, ts_path, meta)

    # Not computed yet → run gaitlib on the session's raw streams (lazy app import so this
    # module never pulls the app/Qt stack unless it actually has to compute).
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from app.session_store import load_session_streams, default_joint_topology
    from app.pipeline_runner import run_pipeline

    nd, meta = load_session_streams(session_dir)
    if meta is None:
        meta = {}
    topo = default_joint_topology(list(nd.keys()))
    res = run_pipeline(nd, joints=topo["joints"], foot_node=topo["foot"],
                       pelvis_node=topo["pelvis"])
    leg = infer_leg(side=meta.get("side") or topo.get("side"), foot_node=topo.get("foot"))
    return export_from_results(res, os.path.join(session_dir, "results"),
                               leg=leg, height_m=meta.get("height_m"))


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Export a processed session for the external Open3D viewer "
                    "(gait_frames.csv + meta.json). Open3D itself is not a dependency.")
    ap.add_argument("session", help="session folder (raw/data.csv + session.json, "
                                    "optionally a computed results/)")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.session):
        ap.error(f"not a directory: {args.session}")
    frames_path, meta_path, meta = export_session(args.session)
    print(f"wrote {frames_path}  ({meta['n_frames']} frames)")
    print(f"wrote {meta_path}    leg={meta['leg']} fs={meta['fs']}Hz "
          f"segments_m={meta['segment_lengths_m']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
