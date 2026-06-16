"""
app/analysis.py — Load a computed session and derive view-ready quantities.

Wraps the cached gaitlib output (results/timeseries.csv + summary.json) and adds the
display layer the UI needs: neutral-referenced flexion for the joint curves, gait
event indices, per-frame "live parameter" snapshots, gait-cycle statistics, and
overlaid-cycle curves. Nothing here recomputes kinematics — it only reshapes the
library's output for presentation.
"""
from __future__ import annotations
import numpy as np

from app import session_store as store

JOINT_ORDER = ["ankle", "knee", "hip"]
JCOLOR = {"ankle": (196, 78, 82), "knee": (85, 168, 104), "hip": (76, 114, 176)}


class LoadedSession:
    """A computed session ready to drive the Visualize and Results views."""

    def __init__(self, sdir):
        self.sdir = sdir
        self.meta = store.read_metadata(sdir)
        self.d, self.summary = store.load_results(sdir)
        if self.d is None:
            raise FileNotFoundError("session has no computed results yet")
        names = self.d.dtype.names
        self.joints = [j for j in JOINT_ORDER if f"{j}_deg" in names]
        self.t = np.asarray(self.d["t_s"], float)
        self.n = len(self.t)
        self.dt = float(np.median(np.diff(self.t))) if self.n > 1 else 1 / 256
        self.fs = 1.0 / self.dt
        self.steady = np.asarray(self.d["steady_state"], float) > 0
        self.foot_strike = np.where(np.asarray(self.d["foot_strike"], float) > 0)[0]
        self.mid_swing = (np.where(np.asarray(self.d["mid_swing"], float) > 0)[0]
                          if "mid_swing" in names else np.array([], int))

        # neutral-referenced display angles (curves) + matching velocity
        raw = {j: np.asarray(self.d[f"{j}_deg"], float) for j in self.joints}
        self.ref, self.disp, self.vel = {}, {}, {}
        self._compute_display(raw)

    def _compute_display(self, raw):
        for j in self.joints:
            a = raw[j]
            if j == "knee":
                off = float(np.nanpercentile(a[self.steady] if self.steady.any() else a, 8))
                sign = 1.0
                disp = np.clip((a - off) * sign, 0.0, None)
            else:
                off = float(np.nanmedian(a[self.steady] if self.steady.any() else a))
                disp = a - off
            self.ref[j] = off
            self.disp[j] = disp
            self.vel[j] = np.gradient(disp, self.dt)

    # -- per-frame -------------------------------------------------------- #
    def index_at(self, t):
        return int(np.clip(np.searchsorted(self.t, t), 0, self.n - 1))

    def param_snapshot(self, i):
        """Live in-capture parameters accumulated up to sample i."""
        out = {"t": float(self.t[i]), "joints": {}}
        for j in self.joints:
            disp = self.disp[j][: i + 1]
            vel = self.vel[j][: i + 1]
            out["joints"][j] = {
                "angle": float(self.disp[j][i]),
                "vel": float(self.vel[j][i]),
                "rom": float(np.nanmax(disp) - np.nanmin(disp)) if len(disp) else 0.0,
                "peak_max": float(np.nanmax(disp)) if len(disp) else 0.0,
                "peak_min": float(np.nanmin(disp)) if len(disp) else 0.0,
                "peak_vel": float(np.nanmax(np.abs(vel))) if len(vel) else 0.0,
            }
        steps = int(np.sum(self.foot_strike <= i))
        out["steps"] = steps
        out["cadence"] = self._cadence_upto(i)
        return out

    def _cadence_upto(self, i, k=6):
        fs = self.foot_strike[self.foot_strike <= i]
        if len(fs) < 2:
            return float("nan")
        st = np.diff(self.t[fs[-(k + 1):]])
        st = st[st < 2.5]
        if not len(st):
            return float("nan")
        return float((1.0 / np.mean(st)) * 60.0 * 2.0)


# --------------------------------------------------------------------------- #
def gait_cycle_params(ls: LoadedSession):
    """Stance/swing %, stride variability from foot events (steady stretches only)."""
    names = ls.d.dtype.names
    toe = np.where(np.asarray(ls.d["mid_swing"], float) > 0)[0] if "mid_swing" in names else []
    strikes = ls.foot_strike
    strikes = strikes[ls.steady[strikes]] if len(strikes) else strikes
    stride_t = np.diff(ls.t[strikes]) if len(strikes) > 1 else np.array([])
    stride_t = stride_t[stride_t < 2.5]
    out = {
        "n_strides": int(max(0, len(strikes) - 1)),
        "stride_time_mean_s": float(np.mean(stride_t)) if len(stride_t) else float("nan"),
        "stride_time_std_s": float(np.std(stride_t)) if len(stride_t) else float("nan"),
        "stride_cv_pct": float(100 * np.std(stride_t) / np.mean(stride_t)) if len(stride_t) else float("nan"),
        "step_time_s": float(np.mean(stride_t) / 2) if len(stride_t) else float("nan"),
    }
    # stance/swing from toe-off (~mid-swing proxy is not toe-off; use detected toe_off if present)
    out["stance_pct"], out["swing_pct"] = _stance_swing(ls)
    return out


def _stance_swing(ls):
    """Stance/swing % per stride: stance = strike→toe-off, swing = toe-off→next strike.

    Uses the pipeline's detected toe-off events when present (column `toe_off`).
    """
    names = ls.d.dtype.names
    if "toe_off" not in names:
        return float("nan"), float("nan")
    toe = np.where(np.asarray(ls.d["toe_off"], float) > 0)[0]
    fs = ls.foot_strike
    if len(fs) < 2 or len(toe) < 1:
        return float("nan"), float("nan")
    swing_fracs = []
    for a, b in zip(fs[:-1], fs[1:]):
        if b - a > int(2.5 * ls.fs) or not ls.steady[a:b].all():
            continue
        t = toe[(toe > a) & (toe < b)]
        if len(t) == 0:
            continue
        swing = (b - t[-1]) / (b - a)         # last toe-off before next strike
        if 0.2 < swing < 0.6:
            swing_fracs.append(swing)
    if not swing_fracs:
        return float("nan"), float("nan")
    sw = 100 * float(np.mean(swing_fracs))
    return 100 - sw, sw


def overlay_cycles(ls: LoadedSession, joint, n_points=101, max_cycles=40):
    """Resample each steady stride of `joint` to 0–100 % cycle; return (grid, cycles, mean)."""
    if joint not in ls.disp:
        return np.linspace(0, 100, n_points), [], None
    disp = ls.disp[joint]
    strikes = ls.foot_strike
    grid = np.linspace(0, 100, n_points)
    cycles = []
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a < 5 or b - a > int(2.5 * ls.fs):
            continue
        if not ls.steady[a:b].all():
            continue
        seg = disp[a:b]
        x = np.linspace(0, 100, len(seg))
        cycles.append(np.interp(grid, x, seg))
        if len(cycles) >= max_cycles:
            break
    mean = np.mean(cycles, axis=0) if cycles else None
    return grid, cycles, mean
