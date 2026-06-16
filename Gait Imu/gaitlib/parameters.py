"""
gaitlib/parameters.py — Commonly measured gait parameters from joint angles + events.

This is the final pipeline stage: it turns the per-joint flexion time-series and the foot
gait events into the parameters clinicians and researchers report.

Per joint:
  * range of motion (ROM)            full bout and steady-state
  * peak angles                      min / max flexion
  * angular velocity + peak          peak |angular velocity|
  * repetition count                 number of flexion cycles (steady state)
  * active duration                  time the joint is actively moving (steady state)

Gait (whole bout, steady state):
  * cadence (steps/min)
  * step time / stride time (mean, std)
  * stance / swing (% of gait cycle)

All quantities are pure functions of the kinematic-core outputs; no markers, no optical
reference, no hardware.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks

from . import angles as A


# --------------------------------------------------------------------------- #
def joint_parameters(flexion, ang_vel, mask, fs, *, min_cycle_s=0.6,
                     active_vel_dps=10.0):
    """Per-joint parameters from a flexion trace (deg) and its angular velocity (deg/s).

    ``mask`` is the steady-state boolean mask (turns excluded). Steady-state quantities use
    it; full-bout quantities use the whole trace.
    """
    flexion = np.asarray(flexion, float)
    ang_vel = np.asarray(ang_vel, float)
    steady = flexion[mask] if mask.any() else flexion
    vel_steady = ang_vel[mask] if mask.any() else ang_vel

    # repetition count: flexion cycles over the steady-state trace
    dist = max(1, int(min_cycle_s * fs))
    height = np.nanmedian(steady) if len(steady) else 0.0
    peaks, _ = find_peaks(steady, distance=dist) if len(steady) > dist else (np.array([]), {})
    reps = int(len(peaks))

    # active duration: steady samples whose |angular velocity| exceeds a small threshold
    active = np.count_nonzero(np.abs(vel_steady) > active_vel_dps)
    active_s = float(active / fs) if fs else 0.0

    return {
        "rom_deg": float(A.rom(steady)) if len(steady) else float("nan"),
        "rom_deg_full": float(A.rom(flexion)),
        "peak_min_deg": float(np.nanmin(flexion)),
        "peak_max_deg": float(np.nanmax(flexion)),
        "peak_abs_vel_dps": float(np.nanmax(np.abs(ang_vel))) if len(ang_vel) else float("nan"),
        "mean_abs_vel_dps": float(np.nanmean(np.abs(vel_steady))) if len(vel_steady) else float("nan"),
        "repetition_count": reps,
        "active_duration_s": active_s,
    }


# --------------------------------------------------------------------------- #
def gait_parameters(cad, ev, mask, fs):
    """Bout-level gait parameters from cadence stats + foot events + steady mask.

    ``cad`` is the dict from :func:`gaitlib.gait.cadence_stats`; ``ev`` is the dict from
    :func:`gaitlib.gait.detect_events`.
    """
    stride_mean = cad.get("stride_time_mean", float("nan"))
    stance, swing = _stance_swing(ev, mask, fs)
    return {
        "cadence_steps_per_min": cad.get("cadence_steps_per_min", float("nan")),
        "stride_time_mean_s": stride_mean,
        "stride_time_std_s": cad.get("stride_time_std", float("nan")),
        "step_time_s": (stride_mean / 2.0) if stride_mean == stride_mean else float("nan"),
        "stance_pct": stance,
        "swing_pct": swing,
        "n_steady_strides": int(cad.get("n_strides", 0)),
        "n_foot_strikes": int(len(ev.get("foot_strike", []))),
    }


def _stance_swing(ev, mask, fs):
    """Stance/swing % per stride: stance = strike->toe-off, swing = toe-off->next strike.

    Uses detected toe-off events; restricted to steady strides under 2.5 s.
    """
    strikes = np.asarray(ev.get("foot_strike", []), int)
    toe = np.asarray(ev.get("toe_off", []), int)
    if len(strikes) < 2 or len(toe) < 1:
        return float("nan"), float("nan")
    swing_fracs = []
    n = len(mask)
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a > int(2.5 * fs) or b >= n or not mask[a:b].all():
            continue
        t = toe[(toe > a) & (toe < b)]
        if len(t) == 0:
            continue
        swing = (b - t[-1]) / (b - a)         # last toe-off before the next strike
        if 0.2 < swing < 0.6:
            swing_fracs.append(swing)
    if not swing_fracs:
        return float("nan"), float("nan")
    sw = 100.0 * float(np.mean(swing_fracs))
    return 100.0 - sw, sw


# --------------------------------------------------------------------------- #
def overlay_cycles(flexion, strikes, steady, fs, n_points=101, max_cycles=60):
    """Resample each steady stride of a flexion trace to 0-100 % gait cycle.

    Returns ``(grid, cycles, mean)`` — useful for the app's overlaid-cycle plot and for
    cycle-averaged ROM. Pure helper, no side effects.
    """
    flexion = np.asarray(flexion, float)
    strikes = np.asarray(strikes, int)
    grid = np.linspace(0, 100, n_points)
    cycles = []
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a < 5 or b - a > int(2.5 * fs) or b > len(flexion):
            continue
        if not steady[a:b].all():
            continue
        seg = flexion[a:b]
        x = np.linspace(0, 100, len(seg))
        cycles.append(np.interp(grid, x, seg))
        if len(cycles) >= max_cycles:
            break
    mean = np.mean(cycles, axis=0) if cycles else None
    return grid, cycles, mean
