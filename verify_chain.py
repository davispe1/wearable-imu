"""
verify_chain.py — end-to-end self-check for the gait-opensim pipeline.

Runs two checks and prints a short report:

  1. CONVENTION — feed VQF a known static sensor tilt and confirm its quaternion is
     sensor->earth, scalar-first (rotating measured gravity yields earth +Z). Uses SciPy.
  2. FULL CHAIN — loader -> VQF -> to_sto on one session, into a TEMP copy (the real session
     folder is never touched). Prints the .sto header, column names and row count, and asserts
     the calibration .sto has exactly one data row.

It also asserts that importing this package does NOT pull in `opensim` (OpenSim is a separate
GUI app, not a Python dependency here).

    python verify_chain.py [--session <dir>]

Run from the gait-opensim/ project root. With no --session it searches the bundled example
slices (../data, ./data).
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.rawdata import load_session                 # noqa: E402
from core.config import config_for_nodes              # noqa: E402
from core.fusion_vqf import fuse_segment              # noqa: E402
from opensim_export.to_sto import export_session       # noqa: E402

_CANDIDATE_SESSIONS = [
    "data/P04_S01_2minWalk", "data/P02_S01_2minWalk", "data/P01_S01_2minWalk",
    "../data/P04_S01_2minWalk",  # fallback: old layout with data one level up
]
_PER_NODE = ("RF.csv", "RS.csv", "RT.csv", "SA.csv", "LF.csv", "LS.csv", "LT.csv")


# --------------------------------------------------------------------------- #
def check_convention() -> bool:
    """Confirm VQF's quat6D is sensor->earth (scalar-first) on a known 30° tilt."""
    R_se = R.from_euler("x", 30, degrees=True)         # sensor->earth
    g_E = np.array([0.0, 0.0, 9.81])
    a_S = R_se.inv().apply(g_E)                         # accel reads earth-up in sensor frame
    seg = {"t": np.arange(2000) / 100.0,
           "acc": np.tile(a_S, (2000, 1)), "gyr": np.zeros((2000, 3)),
           "mag": None, "mag_present": False}
    q, _ = fuse_segment(seg, mode="6D", imu_hz=100.0)
    qr = R.from_quat(np.r_[q[-1, 1:], q[-1, 0]])        # [w,x,y,z] -> scipy [x,y,z,w]
    reconstructed = qr.apply(a_S)                       # sensor->earth: should be [0,0,9.81]
    recovered_tilt = qr.as_euler("xyz", degrees=True)[0]
    ok = np.allclose(reconstructed, g_E, atol=1e-2) and abs(recovered_tilt - 30.0) < 0.5
    print("[1] CONVENTION  sensor->earth, scalar-first")
    print(f"    q*acc_S = {np.round(reconstructed,3)} (expect [0 0 9.81]); "
          f"recovered tilt = {recovered_tilt:.2f}° (expect 30°)  -> {'PASS' if ok else 'FAIL'}")
    return ok


def _resolve_session(arg) -> str:
    if arg:
        return arg
    for c in _CANDIDATE_SESSIONS:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), c)
        if os.path.isdir(p) and any(os.path.exists(os.path.join(p, f)) for f in _PER_NODE):
            return p
    raise SystemExit("no session found; pass --session <dir> with per-node CSVs")


def check_full_chain(session_dir, *, mode="6D") -> bool:
    """Run loader -> VQF -> to_sto on a TEMP copy and validate the .sto outputs."""
    tmp = tempfile.mkdtemp(prefix="gait_opensim_verify_")
    try:
        sess_copy = os.path.join(tmp, os.path.basename(os.path.normpath(session_dir)))
        os.makedirs(sess_copy)
        for f in os.listdir(session_dir):
            if f.lower().endswith(".csv") and os.path.isfile(os.path.join(session_dir, f)):
                shutil.copy(os.path.join(session_dir, f), sess_copy)

        _streams, meta = load_session(sess_copy)
        cfg = config_for_nodes(meta["nodes"], mode=mode)
        orient_path, calib_path, info = export_session(sess_copy, mode=mode)

        with open(orient_path) as fh:
            head = [next(fh).rstrip("\n") for _ in range(6)]
            data_rows = sum(1 for _ in fh)
        with open(calib_path) as fh:
            lines = fh.read().splitlines()
        endh = lines.index("endheader")
        calib_data_rows = len(lines) - (endh + 2)      # +1 endheader, +1 column line

        print(f"\n[2] FULL CHAIN  session={meta['session_id']} side={cfg.side} "
              f"nodes={meta['nodes']} mode={mode}")
        print("    --- orientations.sto header ---")
        for ln in head[:5]:
            print(f"      {ln}")
        print(f"    columns : {head[5]}")
        print(f"    data rows: {data_rows}  (fs={info['fs']:.4f} Hz, modes={info['modes']})")
        print(f"    calibration.sto data rows: {calib_data_rows}")

        ok = (head[1] == "DataType=Quaternion" and head[3] == "OpenSimVersion=4.5"
              and head[4] == "endheader" and data_rows == info["n_rows"] and data_rows > 0
              and calib_data_rows == 1)
        print(f"    -> {'PASS' if ok else 'FAIL'} "
              f"(header exact, full trial exported, calibration has exactly one row)")
        return ok
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_no_opensim() -> bool:
    """Confirm the package did not import `opensim`."""
    imported = "opensim" in sys.modules
    print(f"\n[3] NO OPENSIM DEP  'opensim' in sys.modules = {imported} "
          f"-> {'PASS' if not imported else 'FAIL'}")
    return not imported


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="End-to-end self-check for gait-opensim.")
    ap.add_argument("--session", help="session dir with per-node CSVs (default: bundled example)")
    ap.add_argument("--mode", choices=("6D", "9D", "auto"), default="6D")
    args = ap.parse_args(argv)

    session = _resolve_session(args.session)
    results = [check_convention(), check_full_chain(session, mode=args.mode), check_no_opensim()]
    ok = all(results)
    print(f"\n{'='*60}\nVERIFY: {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
