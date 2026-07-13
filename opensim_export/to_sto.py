"""
opensim_export/to_sto.py — write OpenSense orientation .sto files for a session.

Runs the full chain — :func:`core.rawdata.load_session` -> :func:`core.fusion_vqf.fuse_session`
-> ``.sto`` — and writes two files into ``<session>/results/``:

  ``<id>_orientations.sto`` : the FULL trial, one quaternion per segment per frame.
  ``<id>_calibration.sto``  : a SINGLE data row, the mean orientation over the first ~1 s —
                              the static pose OpenSense uses to place the model on the subject.

Both use the EXACT OpenSense quaternion-table header::

    DataRate=<fs>
    DataType=Quaternion
    version=3
    OpenSimVersion=4.5
    endheader

followed by a ``time`` column and one TAB-separated column per measured segment; each
quaternion cell is ``w,x,y,z`` (comma-separated, scalar-first). Column names come from
:mod:`opensim_export.segment_map` (pelvis_imu, femur_<r|l>_imu, ...). Only the measured leg
gets columns.

This step adds **no OpenSim dependency** — it just writes text files. OpenSim/OpenSense runs
separately (see ``opensim/`` and ``docs/opensim_steps.md``) and consumes these files.

CLI
---
    python -m opensim_export.to_sto <session_dir> [--mode 6D|9D|auto]

``<session_dir>`` holds per-node CSVs (RF.csv/RS.csv/RT.csv/SA.csv) or a combined
``raw/data.csv``. Run it from the ``gait-opensim/`` project root so ``core`` and
``opensim_export`` import as top-level packages.
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from core.rawdata import load_session
from core.config import config_for_nodes
from core.fusion_vqf import fuse_session
from opensim_export.segment_map import ordered_columns

# OpenSense .sto header constants (quaternion orientation table).
STO_DATATYPE = "Quaternion"
STO_VERSION = "3"
STO_OPENSIM_VERSION = "4.5"

CALIBRATION_WINDOW_S = 1.0   # static pose = mean orientation over the first ~1 s


# --------------------------------------------------------------------------- #
def _mean_quat(q: np.ndarray) -> np.ndarray:
    """Mean of a quaternion block (M,4): hemisphere-align to the first sample, renormalise."""
    q = np.asarray(q, float)
    aligned = np.where((q @ q[0])[:, None] < 0, -q, q)
    m = aligned.mean(axis=0)
    return m / (np.linalg.norm(m) + 1e-12)


def _write_sto(path, fs, times, columns) -> str:
    """Write one OpenSense ``.sto``. ``columns`` = ``[(name, quat_array (N,4)), ...]``."""
    header = (f"DataRate={float(fs):.6f}\n"
              f"DataType={STO_DATATYPE}\n"
              f"version={STO_VERSION}\n"
              f"OpenSimVersion={STO_OPENSIM_VERSION}\n"
              f"endheader\n")
    col_line = "\t".join(["time"] + [name for name, _ in columns]) + "\n"
    with open(path, "w", newline="\n") as f:
        f.write(header)
        f.write(col_line)
        for i in range(len(times)):
            cells = [f"{times[i]:.6f}"]
            for _, q in columns:
                w, x, y, z = q[i]
                cells.append(f"{w:.8f},{x:.8f},{y:.8f},{z:.8f}")
            f.write("\t".join(cells) + "\n")
    return path


# --------------------------------------------------------------------------- #
def export_session(session_dir, *, mode: str = "6D",
                   side: str | None = None) -> tuple[str, str, dict]:
    """Write both ``.sto`` files for a session. Returns ``(orient_path, calib_path, info)``.

    ``side`` ("right"/"left") forces the measured leg; by default it is inferred from the node
    ids (this is a single-leg rig — only the measured leg is exported).
    """
    streams, meta = load_session(session_dir)
    config = config_for_nodes(meta["nodes"], side=side, mode=mode)
    fused = fuse_session(streams, config)

    t = np.asarray(fused["t"], float)
    fs = float(fused["fs"])
    orientations = fused["orientations"]
    n = len(t)

    cols = ordered_columns(config)                       # [(node, kind, column_name), ...]
    named = [(name, orientations[node]) for node, _kind, name in cols if node in orientations]
    if not named:
        raise ValueError("no measured segment maps to an OpenSim IMU column")

    out_dir = os.path.join(session_dir, "results")
    os.makedirs(out_dir, exist_ok=True)
    session_id = meta["session_id"]

    # Full trial.
    orient_path = os.path.join(out_dir, f"{session_id}_orientations.sto")
    _write_sto(orient_path, fs, t, named)

    # Static calibration: mean over the first ~1 s, written as a SINGLE data row.
    n_cal = max(1, min(n, int(round(CALIBRATION_WINDOW_S * fs))))
    calib_cols = [(name, _mean_quat(q[:n_cal])[None, :]) for name, q in named]
    calib_path = os.path.join(out_dir, f"{session_id}_calibration.sto")
    _write_sto(calib_path, fs, t[:1], calib_cols)

    info = {
        "session_id": session_id, "side": config.side, "fs": fs, "n_rows": n,
        "columns": [name for name, _ in named],
        "modes": {node: fused["modes"][node] for node, _k, _nm in cols if node in orientations},
        "calibration_window_s": (n_cal / fs) if fs else 0.0, "n_calibration_rows": 1,
    }
    return orient_path, calib_path, info


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Write OpenSense orientation .sto files (<id>_orientations.sto + "
                    "<id>_calibration.sto) for a session. OpenSim is NOT a dependency.")
    ap.add_argument("session", help="session folder (per-node CSVs RF.csv/RS.csv/... or "
                                    "raw/data.csv)")
    ap.add_argument("--mode", choices=("6D", "9D", "auto"), default="6D",
                    help="VQF fusion mode (default: 6D, magnetometer-free)")
    ap.add_argument("--side", choices=("right", "left"), default=None,
                    help="force the measured leg (default: infer from node ids)")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.session):
        ap.error(f"not a directory: {args.session}")

    orient_path, calib_path, info = export_session(args.session, mode=args.mode, side=args.side)
    print(f"wrote {orient_path}")
    print(f"      {info['n_rows']} rows, fs={info['fs']:.4f} Hz, side={info['side']}, "
          f"modes={info['modes']}")
    print(f"wrote {calib_path}")
    print(f"      {info['n_calibration_rows']} row (mean of first "
          f"{info['calibration_window_s']:.2f} s)")
    print(f"columns: time, {', '.join(info['columns'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
