"""
kincore/gait.py — Foot-IMU gait event detection, cadence and step count.

Uses the foot sagittal angular velocity (gyro projected on the foot's principal/
mediolateral walking axis). Each stride has a large mid-swing peak; initial contact
(foot strike) is the sharp negative swing just after mid-swing. Steady-state segments
(turnarounds excluded) are used for cadence/stride statistics.
"""
from __future__ import annotations
import numpy as np
from scipy.signal import find_peaks


def principal_axis(gyr):
    """Mediolateral (flexion) axis of the foot = largest-variance gyro direction."""
    C = gyr.T @ gyr
    w, V = np.linalg.eigh(C)
    return V[:, -1]


def sagittal_rate(gyr):
    ax = principal_axis(gyr)
    s = gyr @ ax
    if np.mean(s**3) < 0:        # orient so swing peaks are positive
        s = -s
    return s, ax


def detect_events(foot_gyr, fs, min_stride_s=0.6):
    """Detect mid-swing peaks and initial-contact (foot-strike) events.

    Returns dict with mid_swing (idx), foot_strike (idx), toe_off (idx).
    """
    s, _ = sagittal_rate(foot_gyr)
    # smooth lightly
    k = max(1, int(0.03*fs))
    s_s = np.convolve(s, np.ones(k)/k, "same")
    dist = int(min_stride_s * fs)
    thr = 0.5 * np.percentile(np.abs(s_s), 95)
    mids, _ = find_peaks(s_s, distance=dist, height=thr)
    strikes, toeoffs = [], []
    for m in mids:
        # foot strike: minimum within ~0.5 s AFTER mid-swing
        a, b = m, min(len(s_s), m + int(0.5*fs))
        if b > a + 1:
            strikes.append(a + int(np.argmin(s_s[a:b])))
        # toe-off: minimum within ~0.4 s BEFORE mid-swing
        a2, b2 = max(0, m - int(0.4*fs)), m
        if b2 > a2 + 1:
            toeoffs.append(a2 + int(np.argmin(s_s[a2:b2])))
    return {"mid_swing": np.array(mids),
            "foot_strike": np.array(sorted(set(strikes))),
            "toe_off": np.array(sorted(set(toeoffs))),
            "sagittal_rate": s_s}


def cadence_stats(strike_idx, fs, mask=None):
    """Stride/step stats from foot-strike indices (optionally restricted to a mask).

    Returns dict: n_strides, stride_times, stride_time_mean/std, cadence_steps_per_min.
    Cadence (both feet) ~ 2 x single-foot stride rate.
    """
    idx = np.array(strike_idx)
    if mask is not None and len(idx):
        idx = idx[mask[idx]]
    if len(idx) < 2:
        return {"n_strides": int(len(idx)), "stride_times": [], "stride_time_mean": float("nan"),
                "stride_time_std": float("nan"), "cadence_steps_per_min": float("nan")}
    st = np.diff(idx) / fs
    # drop implausible (turn gaps) > 2.5 s
    st = st[st < 2.5]
    stride_rate = 1.0 / np.mean(st)            # strides/s (one foot)
    return {
        "n_strides": int(len(idx)),
        "stride_times": st.tolist(),
        "stride_time_mean": float(np.mean(st)),
        "stride_time_std": float(np.std(st)),
        "cadence_steps_per_min": float(stride_rate * 60.0 * 2.0),
    }
