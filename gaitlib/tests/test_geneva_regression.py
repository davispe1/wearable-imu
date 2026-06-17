"""
gaitlib/tests/test_geneva_regression.py — EQUIVALENCE / regression check.

This is NOT an accuracy validation. It asserts that the refactored ``gaitlib.compute``
reproduces the previously validated pipeline's known parameters on the bundled Geneva
slice (P01, 2-min walk, right leg, nodes RF/RS/RT/SA):

    cadence  ~= 109 steps/min
    steady   ROM  ankle ~53.3 deg, knee ~86.4 deg, hip ~53.8 deg
    99 steady strides

The real-hardware ACCURACY validation is separate and manual (goniometer / known reference
angles); optical markers were only a past validation aid and gaitlib never requires them.

Run:  python -m pytest gaitlib/tests/  -q
  or: python gaitlib/tests/test_geneva_regression.py
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import gaitlib

SLICE = os.path.join(ROOT, "data", "P01_S01_2minWalk")
NODES = ["RF", "RS", "RT", "SA"]

# Known validated parameters (from the previously validated pipeline).
EXPECTED_CADENCE = 109.0
EXPECTED_STRIDES = 99
EXPECTED_ROM = {"ankle": 53.3, "knee": 86.4, "hip": 53.8}
ROM_TOL = 1.0          # deg
CADENCE_TOL = 1.0      # steps/min


def _load_slice():
    """Load the bundled Geneva per-node slices as a gaitlib per-node raw_data dict."""
    raw = {}
    for n in NODES:
        p = os.path.join(SLICE, f"{n}.csv")
        with open(p, newline="") as f:
            rdr = csv.reader(f)
            header = next(rdr)
            rows = np.array([[float(x) for x in r] for r in rdr if r])
        cols = {h: i for i, h in enumerate(header)}
        t = rows[:, cols["t_opt_s"]]
        acc = rows[:, [cols["ax"], cols["ay"], cols["az"]]]
        gyr = rows[:, [cols["gx"], cols["gy"], cols["gz"]]]
        mag = rows[:, [cols["mx"], cols["my"], cols["mz"]]]
        raw[n] = {"t": t, "acc": acc, "gyr": gyr, "mag": mag}
    return raw


def run_regression():
    if not os.path.isdir(SLICE):
        raise SystemExit(f"SKIP: Geneva slice not found at {SLICE} (datasets are not bundled)")
    raw = _load_slice()
    cfg = gaitlib.default_config(side="right", imu_hz=256.0)
    res = gaitlib.compute(raw, cfg)

    cad = res.gait["cadence_steps_per_min"]
    strides = res.gait["n_steady_strides"]
    roms = {j: res.joints[j]["params"]["rom_deg"] for j in EXPECTED_ROM}

    print(f"cadence = {cad:.2f} steps/min   (expected ~{EXPECTED_CADENCE})")
    print(f"steady strides = {strides}      (expected {EXPECTED_STRIDES})")
    for j, want in EXPECTED_ROM.items():
        print(f"{j:6s} steady ROM = {roms[j]:.2f} deg   (expected ~{want})")

    ok = True
    if abs(cad - EXPECTED_CADENCE) > CADENCE_TOL:
        ok = False; print(f"  FAIL cadence off by {cad - EXPECTED_CADENCE:+.2f}")
    if strides != EXPECTED_STRIDES:
        ok = False; print(f"  FAIL strides {strides} != {EXPECTED_STRIDES}")
    for j, want in EXPECTED_ROM.items():
        if abs(roms[j] - want) > ROM_TOL:
            ok = False; print(f"  FAIL {j} ROM off by {roms[j] - want:+.2f}")
    return ok


# -- pytest entry points ----------------------------------------------------- #
def test_geneva_equivalence():
    import pytest
    if not os.path.isdir(SLICE):
        pytest.skip("Geneva slice not bundled")
    raw = _load_slice()
    res = gaitlib.compute(raw, gaitlib.default_config(side="right", imu_hz=256.0))
    assert abs(res.gait["cadence_steps_per_min"] - EXPECTED_CADENCE) <= CADENCE_TOL
    assert res.gait["n_steady_strides"] == EXPECTED_STRIDES
    for j, want in EXPECTED_ROM.items():
        assert abs(res.joints[j]["params"]["rom_deg"] - want) <= ROM_TOL


if __name__ == "__main__":
    import sys
    ok = run_regression()
    print("\n=== REGRESSION", "PASS ===" if ok else "FAIL ===")
    sys.exit(0 if ok else 1)
