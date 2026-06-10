"""
selftest.py — RAW-DATA CONTRACT proof.

Claim: every IMU-computed quantity (joint angle, angular velocity, ROM, step events,
cadence) is a pure function of the IMU data alone. The marker-derived reference angles
and the gait-event labels are read ONLY by the validation stage.

Proof:
  1. Run the kinematic core (run.compute_core) -> snapshot computed quantities.
  2. SCRAMBLE the reference (time-shuffle marker frames) and DROP the labels by monkey-
     patching the marker/event readers, then run the core again.
  3. Assert every computed quantity is BIT-FOR-BIT identical (the core never saw markers).
  4. Assert the validation RMSE and step-event error CHANGE (they do depend on the
     reference / labels) — confirming the validation actually uses them.
"""
from __future__ import annotations
import numpy as np
import yaml

import run
from validation import reference as R


def _snapshot(ctx):
    s = {}
    for mode in ctx["results"]:
        for j, d in ctx["results"][mode]["joints"].items():
            s[f"{mode}/{j}/flexion"] = np.asarray(d["flexion"]).copy()
            s[f"{mode}/{j}/ang_vel"] = np.asarray(d["ang_vel"]).copy()
            s[f"{mode}/{j}/rom"] = float(d["rom"])
    s["foot_strike"] = np.asarray(ctx["ev"]["foot_strike"]).copy()
    s["cadence"] = float(ctx["cad"]["cadence_steps_per_min"])
    s["n_strides"] = int(ctx["cad"]["n_strides"])
    return s


def _identical(a, b):
    keys = set(a) | set(b)
    bad = []
    for k in keys:
        va, vb = a.get(k), b.get(k)
        if isinstance(va, np.ndarray):
            if va.shape != vb.shape or not np.array_equal(va, vb):
                bad.append(k)
        else:
            if va != vb:
                bad.append(k)
    return bad


def main(cfg_path="config/default.yaml"):
    cfg = yaml.safe_load(open(cfg_path))

    print("1) Running kinematic core (clean) ...")
    ctx_clean = run.compute_core(cfg)
    snap_clean = _snapshot(ctx_clean)
    val_clean = run.validate(cfg, ctx_clean)

    # --- corrupt the reference/labels by monkeypatching the marker/event readers ---
    orig_read = R.read_markers
    orig_events = R.c3d_events
    rng = np.random.default_rng(1)

    def scrambled_read(path):
        M, rate, c = orig_read(path)
        n = next(iter(M.values())).shape[0]
        perm = rng.permutation(n)
        M = {k: v[perm] for k, v in M.items()}   # destroy kinematics (shuffle frames)
        return M, rate, c

    def dropped_events(c):
        return {}                                  # drop all labels

    print("2) Scrambling reference + dropping labels; re-running the core ...")
    R.read_markers = scrambled_read
    R.c3d_events = dropped_events
    try:
        ctx_corrupt = run.compute_core(cfg)
        snap_corrupt = _snapshot(ctx_corrupt)
        val_corrupt = run.validate(cfg, ctx_corrupt)
    finally:
        R.read_markers = orig_read
        R.c3d_events = orig_events

    # --- assertions ---
    print("\n3) CONTRACT CHECK — computed quantities must be bit-for-bit identical:")
    bad = _identical(snap_clean, snap_corrupt)
    if bad:
        print(f"   FAIL: these computed quantities changed: {bad}")
        ok_core = False
    else:
        print(f"   PASS: all {len(snap_clean)} computed quantities identical "
              "(angles, velocities, ROM, step events, cadence).")
        ok_core = True

    print("\n4) VALIDATION must respond to the reference/labels:")
    changed = []
    for j in cfg["selection"]["joints"]:
        a = val_clean["per_joint_rmse_deg"]["6dof"][j]
        b = val_corrupt["per_joint_rmse_deg"]["6dof"][j]
        print(f"   {j:5s} RMSE: clean={a:5.1f}  scrambled={b:5.1f}  {'CHANGED' if abs(a-b)>1e-6 else 'same'}")
        if abs(a-b) > 1e-6:
            changed.append(j)
    se_a, se_b = val_clean["step_event_timing_error_s"], val_corrupt["step_event_timing_error_s"]
    print(f"   step-event error: clean={se_a:.3f}s  labels-dropped={se_b}  "
          f"(matched steps {val_clean['n_matched_steps']} -> {val_corrupt['n_matched_steps']})")
    ok_val = len(changed) == len(list(cfg["selection"]["joints"])) and val_corrupt["n_matched_steps"] == 0

    print("\n=== SELFTEST RESULT ===")
    if ok_core and ok_val:
        print("PASS — core is a pure function of IMU; validation depends on reference/labels.")
        return 0
    print("FAIL — contract violated (see above).")
    return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
