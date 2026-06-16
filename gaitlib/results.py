"""
gaitlib/results.py — The object returned by :func:`gaitlib.compute`.

``GaitResults`` bundles everything the pipeline produced: the common time grid, per-joint
angle/velocity/acceleration traces, per-joint and gait parameters, the detected gait
events, the steady-state mask, and any warnings (e.g. joints skipped for missing sensors).

It is plain data plus a few convenience exporters. In particular, the **computed joint
angle is easy to export for manual accuracy validation** against a goniometer or known
reference angles (:meth:`joint_angle_table` / :meth:`save_joint_angles_csv`) — the library
itself never needs optical/marker ground truth to run.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import numpy as np


@dataclass
class GaitResults:
    fs: float                              # common sample rate (Hz)
    t: np.ndarray                          # (N,) time grid (s), 0 at bout start
    joints: dict                           # name -> {flexion, ang_vel, ang_acc, params}
    gait: dict                             # bout-level gait parameters
    events: dict                           # foot_strike / mid_swing / toe_off (sample idx)
    steady_state: np.ndarray               # (N,) bool mask (turns excluded)
    turnarounds: list = field(default_factory=list)   # (start_idx, end_idx, deg)
    primary_mode: str = "6dof"
    config: dict = field(default_factory=dict)         # mounting config used (dict form)
    warnings: list = field(default_factory=list)       # human-readable notes
    meta: dict = field(default_factory=dict)

    # -- joint helpers ------------------------------------------------------- #
    @property
    def joint_names(self):
        return list(self.joints.keys())

    def angle(self, joint, mode=None):
        """Flexion trace (deg) for a joint (primary mode unless a mode is given)."""
        j = self.joints[joint]
        if mode and mode != self.primary_mode and "modes" in j:
            return j["modes"][mode]["flexion"]
        return j["flexion"]

    # -- parameter summary --------------------------------------------------- #
    def summary(self) -> dict:
        """A JSON-serialisable dict of all parameters (no time-series arrays)."""
        return {
            "fs_hz": self.fs,
            "duration_s": float(self.t[-1]) if len(self.t) else 0.0,
            "primary_fusion": self.primary_mode,
            "joints": list(self.joints.keys()),
            "per_joint": {j: d["params"] for j, d in self.joints.items()},
            "gait": self.gait,
            "n_turnarounds": len(self.turnarounds),
            "turnarounds": [{"t_start_s": float(self.t[s]), "t_end_s": float(self.t[e]),
                             "deg": float(a)} for s, e, a in self.turnarounds],
            "warnings": self.warnings,
            "config": self.config,
            "meta": self.meta,
        }

    def save_summary_json(self, path):
        with open(path, "w") as f:
            json.dump(self.summary(), f, indent=2)
        return path

    # -- time-series export -------------------------------------------------- #
    def timeseries_columns(self) -> dict:
        """Ordered column dict (name -> array) of the full time-series output."""
        cols = {"t_s": self.t}
        for j, d in self.joints.items():
            cols[f"{j}_deg"] = d["flexion"]
            cols[f"{j}_vel_dps"] = d["ang_vel"]
            cols[f"{j}_acc_dps2"] = d["ang_acc"]
            if "modes" in d:
                for m, md in d["modes"].items():
                    if m != self.primary_mode:
                        cols[f"{j}_deg_{m}"] = md["flexion"]
        n = len(self.t)
        for ev_name in ("foot_strike", "mid_swing", "toe_off"):
            idx = np.asarray(self.events.get(ev_name, []), int)
            idx = idx[idx < n]
            col = np.zeros(n, int)
            col[idx] = 1
            cols[ev_name] = col
        cols["steady_state"] = self.steady_state.astype(int)
        return cols

    def save_timeseries_csv(self, path):
        cols = self.timeseries_columns()
        header = ",".join(cols.keys())
        arr = np.column_stack([np.asarray(v, float) for v in cols.values()])
        np.savetxt(path, arr, delimiter=",", header=header, comments="", fmt="%.6g")
        return path

    # -- manual accuracy validation ----------------------------------------- #
    def joint_angle_table(self) -> dict:
        """Time + each joint's computed flexion (deg) — the trace to compare, by hand,
        against a goniometer or known reference angles.  ``{"t_s": .., "<joint>_deg": ..}``"""
        out = {"t_s": self.t}
        for j, d in self.joints.items():
            out[f"{j}_deg"] = d["flexion"]
        return out

    def save_joint_angles_csv(self, path):
        """Write the computed joint angles for manual (goniometer) accuracy comparison."""
        cols = self.joint_angle_table()
        header = ",".join(cols.keys())
        arr = np.column_stack([np.asarray(v, float) for v in cols.values()])
        np.savetxt(path, arr, delimiter=",", header=header, comments="", fmt="%.6g")
        return path

    # -- convenience --------------------------------------------------------- #
    def __repr__(self):
        return (f"GaitResults(fs={self.fs:.1f}Hz, dur={self.t[-1]:.1f}s, "
                f"joints={self.joint_names}, "
                f"cadence={self.gait.get('cadence_steps_per_min', float('nan')):.1f})")
