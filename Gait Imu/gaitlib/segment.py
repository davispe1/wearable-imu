"""
gaitlib/segment.py — Turnaround detection and steady-state straight-walking masking.

The 2-min walk is a continuous back-and-forth: straight passes separated by ~180-deg
turnarounds. Turns are detected from the pelvis vertical-axis angular rate and are
EXCLUDED from steady-state gait statistics (cadence, stride, ROM) but KEPT (flagged) for
the heading / 6-vs-9-DOF comparison.
"""
from __future__ import annotations
import numpy as np


def _lowpass_grav(acc, fs, fc=0.4):
    w = max(1, int(fs / fc))
    k = np.ones(w) / w
    return np.column_stack([np.convolve(acc[:, i], k, "same") for i in range(3)])


def vertical_yaw_rate(acc, gyr, fs):
    """Angular rate about the gravity (vertical) axis in the sensor frame (rad/s)."""
    g = _lowpass_grav(acc, fs)
    gu = g / (np.linalg.norm(g, axis=1, keepdims=True) + 1e-12)
    return np.einsum("ij,ij->i", gyr, gu)


def detect_turnarounds(acc, gyr, fs, min_turn_deg=120.0, rate_thresh_dps=50.0, smooth_s=0.5):
    """Detect ~180-deg turns from pelvis vertical-axis angular rate.

    Returns (turns, yaw_rate_smoothed) with turns = list of (start_idx, end_idx, deg).
    """
    yr = vertical_yaw_rate(acc, gyr, fs)
    w = max(1, int(smooth_s * fs))
    yr_s = np.convolve(yr, np.ones(w)/w, "same")
    active = np.abs(yr_s) > np.radians(rate_thresh_dps)
    turns = []
    i, n = 0, len(yr_s)
    while i < n:
        if active[i]:
            s = i; sign = np.sign(yr_s[i])
            while i < n and active[i] and np.sign(yr_s[i]) == sign:
                i += 1
            ang = np.degrees(np.trapezoid(yr_s[s:i], dx=1.0/fs))
            if abs(ang) >= min_turn_deg:
                turns.append((s, i, float(ang)))
        else:
            i += 1
    return turns, yr_s


def steady_state_mask(n, turns, fs, pad_s=0.7):
    """Boolean mask of steady straight-walking samples (turns + pad removed)."""
    m = np.ones(n, bool)
    pad = int(pad_s * fs)
    for s, e, _ in turns:
        m[max(0, s-pad):min(n, e+pad)] = False
    return m


def contiguous_runs(mask):
    runs = []
    j = 0
    while j < len(mask):
        if mask[j]:
            s = j
            while j < len(mask) and mask[j]:
                j += 1
            runs.append((s, j))
        else:
            j += 1
    return runs
