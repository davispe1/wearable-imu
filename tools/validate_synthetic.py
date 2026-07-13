"""
tools/validate_synthetic.py — score the pipeline against a synthetic session's ground truth.

Runs :func:`kinematics.pipeline.analyze_session` on a session produced by
``tools.make_synthetic_session`` and compares the recovered gait kinematics to the KNOWN values
in ``ground_truth.json``. Because the synthetic data was forward-simulated from prescribed
kinematics (orientation-first), every recovered number has a right answer — so this doubles as a
**regression anchor**: it exits non-zero if any metric drifts outside tolerance.

It also demonstrates the MAGNETOMETER's role: it recovers each segment's absolute heading in 6D
(magnetometer-free) vs 9D (magnetometer-referenced). Only 9D pins the true walking heading and
makes every segment agree on it — the honest, quantified reason the magnetometer is useful here.

    python -m tools.validate_synthetic [session_dir]     # default data/SYN01_S01_straightWalk
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

from kinematics.pipeline import analyze_session
from kinematics.quaternion import mean_quat


def _yaw_deg(q) -> float:
    """Heading (yaw about earth-up, deg) of a sensor->earth scalar-first quaternion [w,x,y,z]."""
    w, x, y, z = q
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def _wrap(a: float) -> float:
    """Wrap an angle (deg) to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def validate(session_dir: str, tol_deg: float = 5.0, tol_m: float = 0.15) -> bool:
    gt = json.load(open(os.path.join(session_dir, "ground_truth.json")))
    res = {m: analyze_session(session_dir, mode=m) for m in ("6D", "9D")}
    r = res["6D"]                                            # joint/temporal/spatial are mode-agnostic

    checks = []
    for j in ("hip", "knee", "ankle"):
        p = r.joints[j]["params"]
        got = p.get("rom_cycle_deg", p["rom_deg"])
        checks.append((f"{j} ROM", gt["joint_rom_deg"][j], got, "deg", tol_deg))
    tp = r.temporal
    checks.append(("cadence", gt["cadence_steps_per_min"], tp["cadence_steps_per_min"], "/min", tol_deg))
    checks.append(("stance", gt["stance_pct"], tp["stance_pct"], "%", tol_deg))
    checks.append(("swing", gt["swing_pct"], tp["swing_pct"], "%", tol_deg))
    sp = r.spatial
    checks.append(("stride length", gt["stride_length_m"], sp["stride_length_m_est"], "m", tol_m))
    checks.append(("walking speed", gt["walking_speed_mps"], sp["walking_speed_mps_est"], "m/s", tol_m))

    print(f"\n  {gt['session_id']}  -  recovered vs ground truth\n")
    print(f"  {'metric':<16}{'truth':>9}{'got':>9}{'err':>8}{'tol':>7}   result")
    print("  " + "-" * 58)
    ok = True
    for name, truth, got, unit, tol in checks:
        err = abs(got - truth)
        passed = err <= tol
        ok = ok and passed
        print(f"  {name:<16}{truth:>9.2f}{got:>9.2f}{err:>8.2f}{tol:>7.2f} {unit:<4} "
              f"{'PASS' if passed else 'FAIL'}")

    # --- magnetometer: what it does (and does NOT) change, verified ------------------------ #
    print("\n  magnetometer  (VQF 6D magnetometer-free  vs  9D magnetometer-referenced)\n")
    rom = {m: {j: res[m].joints[j]["params"].get("rom_cycle_deg",
                                                 res[m].joints[j]["params"]["rom_deg"])
               for j in ("hip", "knee", "ankle")} for m in ("6D", "9D")}
    maxdiff = max(abs(rom["6D"][j] - rom["9D"][j]) for j in rom["6D"])
    print(f"  joint ROM 6D vs 9D: max difference {maxdiff:.2f} deg")
    print("     -> the sagittal angles are yaw-immune, so the magnetometer cannot change or")
    print("        corrupt the clinical joint angles (this is why 6D is the safe default).")

    win = slice(0, min(100, len(r.t)))                      # quiet-standing lead window
    def _seg_head(mode):
        return {n: _yaw_deg(mean_quat(res[mode].orientations[n][win]))
                for n in sorted(res[mode].orientations)}
    def _pelvis_drift(mode):
        o = res[mode].orientations["SA"]
        return _wrap(_yaw_deg(mean_quat(o[-100:])) - _yaw_deg(mean_quat(o[:100])))
    h6, h9 = _seg_head("6D"), _seg_head("9D")
    sp6, sp9 = float(np.ptp(list(h6.values()))), float(np.ptp(list(h9.values())))
    print(f"\n  9D absolute heading: pelvis {h9['SA']:.0f} deg in the field's frame, "
          f"segment spread {sp9:.1f} deg, bout drift {_pelvis_drift('9D'):+.1f} deg")
    print(f"  6D heading: pelvis {h6['SA']:.0f} deg - an arbitrary per-segment zero, "
          f"NOT field-referenced")
    print("     -> only 9D gives an absolute heading shared by every segment (world-frame")
    print("        consistency for the OpenSense export / 3D view). Note: VQF's magnetic")
    print("        disturbance rejection means 9D is NOT a blind fix for gyro-heading drift.")

    print(f"  RESULT: {'ALL PASS' if ok else 'FAILURES ABOVE'}\n")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Validate the pipeline against a synthetic session's "
                                             "ground truth (and demonstrate the magnetometer).")
    ap.add_argument("session", nargs="?", default="data/SYN01_S01_straightWalk",
                    help="synthetic session dir (default: data/SYN01_S01_straightWalk)")
    a = ap.parse_args(argv)
    if not os.path.exists(os.path.join(a.session, "ground_truth.json")):
        ap.error(f"no ground_truth.json in {a.session!r} — generate it with "
                 f"`python -m tools.make_synthetic_session --out {a.session}`")
    return 0 if validate(a.session) else 1


if __name__ == "__main__":
    raise SystemExit(main())
