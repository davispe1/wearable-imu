"""
adapters/geneva.py — Geneva (Grouvel 2023) DatasetAdapter -> IMUTrial.

Maps the dataset's files into the three-field contract:
  IMUTrial.imu        — BIN-native accel/gyro/magnetometer per sensor (the ONLY thing
                        the kinematic core may read). Loaded from the data/ slices.
  IMUTrial.reference  — marker-derived joint angles per mocap window (validation only).
  IMUTrial.labels     — c3d gait events (Zeni) per window (validation only).

The wall: the core consumes ``imu``; ``reference`` and ``labels`` are touched only by the
validation stage. ``scramble_reference`` / ``drop_labels`` produce a corrupted copy used by
the selftest to prove the core output is independent of reference/labels.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import copy
import os
import numpy as np

import align
from validation import reference as R


@dataclass
class IMUTrial:
    imu: dict                     # node -> {t_native, t_opt, acc, gyr, mag} (BIN-native)
    reference: dict = field(default_factory=dict)   # trial -> {joint -> ref flexion (deg)}
    labels: dict = field(default_factory=dict)      # trial -> {event -> [times]}
    meta: dict = field(default_factory=dict)


class GenevaAdapter:
    def __init__(self, cfg):
        self.cfg = cfg
        ds = cfg["dataset"]
        self.root, self.subj, self.sess = ds["root"], ds["subject"], ds["session"]
        self.task = cfg["selection"]["task"]

    def slice_dir(self):
        return os.path.join(self.cfg["output"]["data_dir"],
                            f"{self.subj}_{self.sess}_{self.task}")

    def load(self) -> IMUTrial:
        # --- imu view: the BIN-native per-sensor slices written by extract.py ---
        imu = {}
        sd = self.slice_dir()
        for node in self.cfg["selection"]["nodes"]:
            p = os.path.join(sd, f"{node}.csv")
            a = np.loadtxt(p, delimiter=",", skiprows=1)
            imu[node] = {"t_native": a[:, 0], "t_opt": a[:, 1],
                         "acc": a[:, 2:5], "gyr": a[:, 5:8], "mag": a[:, 8:11]}

        # --- reference (markers) + labels (events): validation-only ---
        reference, labels = {}, {}
        try:
            neutral = R.neutral_reference(align.c3d_path(self.root, self.subj, self.sess, "Static", "01"))
        except Exception:
            neutral = {j: 0.0 for j in self.cfg["selection"]["joints"]}
        for tr in self.cfg["selection"]["trials"]:
            c3dp = align.c3d_path(self.root, self.subj, self.sess, self.task, tr)
            try:
                ang, rate, c = R.window_reference(c3dp, neutral)
                reference[tr] = {"angles": ang, "rate": rate}
                labels[tr] = R.c3d_events(c)
            except Exception:
                pass
        return IMUTrial(imu=imu, reference=reference, labels=labels,
                        meta={"neutral_ref": neutral})


def scramble_reference(trial: IMUTrial, seed=0) -> IMUTrial:
    """Return a copy with the reference joint angles time-shuffled (kinematics destroyed)."""
    rng = np.random.default_rng(seed)
    t = copy.deepcopy(trial)
    for tr, rec in t.reference.items():
        for j, arr in rec["angles"].items():
            perm = rng.permutation(len(arr))
            rec["angles"][j] = np.asarray(arr)[perm]
    return t


def drop_labels(trial: IMUTrial) -> IMUTrial:
    t = copy.deepcopy(trial)
    t.labels = {tr: {} for tr in t.labels}
    return t
