"""
align.py — Locate a trial's time window inside the continuous BIN recording.

The BIN is a single ~69-minute file per sensor containing every task. Each trial's
optical capture has an absolute start timestamp in the .c3d (TRIAL/TIMECAPTURESTART),
and the BIN header carries the device RTC start time. These two clocks differ by a
small, near-constant **session skew** (≈ −12 s for P01) that we calibrate once from
long, content-rich trials via gyro cross-correlation against ``sync_data``.

Localization of a trial is then:
    bin_idx = round((c3d_capture_time - bin_rtc_start + skew) * fs_high)
optionally refined by a tiny local cross-correlation.

Why not pure cross-correlation? A 3 s walking template is not unique across 69 min
that contain many gait trials (it produces ambiguous ~0.4 peaks). Absolute timestamps
remove the ambiguity; sync_data only calibrates the clock offset and does a sub-second
refine.

CONTRACT: ``sync_data`` appears ONLY in this module, as local variables. It is never
returned as IMU data and never enters fusion. What crosses back out is an integer BIN
sample offset (+ correlation score) on the BIN's own clock.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta
import os
import glob
import numpy as np
import pandas as pd
import ezc3d
from scipy.signal import resample_poly, fftconvolve


# --------------------------------------------------------------------------- #
# Paths & timestamps
# --------------------------------------------------------------------------- #
def _subj_dir(root, subject, session):
    return os.path.join(root, f"{subject}_{session}")


def sync_csv_path(root, subject, session, task, trial):
    return os.path.join(_subj_dir(root, subject, session), "SYNC_DATA",
                        f"{subject}_{session}_{task}_{trial}.csv")


def c3d_path(root, subject, session, task, trial):
    return os.path.join(_subj_dir(root, subject, session), "RAW_DATA",
                        f"{subject}_{session}_{task}_{trial}.c3d")


def c3d_capture_datetime(path: str) -> datetime:
    """Absolute optical capture start from TRIAL/DATECAPTURESTART + TIMECAPTURESTART."""
    tr = ezc3d.c3d(path)["parameters"]["TRIAL"]
    d = tr["DATECAPTURESTART"]["value"][0]
    tc = tr["TIMECAPTURESTART"]["value"][0]
    base = datetime.strptime(d + " " + tc[:8], "%Y-%m-%d %H:%M:%S")
    frac = float("0" + tc[8:]) if len(tc) > 8 else 0.0
    return base + timedelta(seconds=frac)


def bin_rtc_datetime(start_datetime: str) -> datetime:
    return datetime.strptime(start_datetime, "%Y-%m-%d %H:%M:%S")


# --------------------------------------------------------------------------- #
# Normalized cross-correlation (sliding Pearson r), O(N)
# --------------------------------------------------------------------------- #
def ncc_valid(signal: np.ndarray, template: np.ndarray) -> np.ndarray:
    n = len(template)
    t = template - template.mean()
    t_ss = float((t * t).sum())
    if t_ss <= 0 or len(signal) < n:
        return np.zeros(max(0, len(signal) - n + 1))
    num = fftconvolve(signal, t[::-1], mode="valid")
    cs = np.concatenate([[0.0], np.cumsum(signal)])
    css = np.concatenate([[0.0], np.cumsum(signal * signal)])
    win_sum = cs[n:] - cs[:-n]
    win_ss = css[n:] - css[:-n]
    win_var = np.clip(win_ss - win_sum * win_sum / n, 1e-12, None)
    return num / np.sqrt(win_var * t_ss)


def _sync_gyro_mag(path, node):
    cols = [f"P6_{node}_gyro_{a}" for a in ("x", "y", "z")]
    return np.linalg.norm(pd.read_csv(path, usecols=cols).to_numpy(float), axis=1)


def _sync_acc_mag(path, node):
    cols = [f"P6_{node}_acc_{a}" for a in ("x", "y", "z")]
    return np.linalg.norm(pd.read_csv(path, usecols=cols).to_numpy(float), axis=1)


# --------------------------------------------------------------------------- #
# Session skew calibration
# --------------------------------------------------------------------------- #
@dataclass
class SkewModel:
    skew_s: float           # median (loc - timestamp_pred) over calibration trials
    n_used: int
    detail: list            # [(name, pred_s, loc_s, corr, skew_s)]


def estimate_session_skew(bin_gyr_mag_256, fs_high, bin_rtc, root, subject, session,
                          node, min_len_s=20.0, min_corr=0.6) -> SkewModel:
    """Calibrate the constant IMU<->optical clock skew from long, content-rich trials.

    Aligns every long ``sync_data`` trial's gyro magnitude against the BIN gyro
    magnitude (band-matched to 100 Hz) and takes the median skew of confident locks.
    """
    gyrb = resample_poly(bin_gyr_mag_256, 25, 64)  # 256 -> 100 Hz, anti-aliased
    fs = 100.0
    detail = []
    skews = []
    pattern = os.path.join(_subj_dir(root, subject, session), "SYNC_DATA",
                           f"{subject}_{session}_*.csv")
    for p in sorted(glob.glob(pattern)):
        name = os.path.basename(p)[len(f"{subject}_{session}_"):-4]
        if "_" not in name:
            continue
        task, trial = name.rsplit("_", 1)
        try:
            sg = _sync_gyro_mag(p, node)
        except Exception:
            continue
        if len(sg) < min_len_s * fs:
            continue
        try:
            cap = c3d_capture_datetime(c3d_path(root, subject, session, task, trial))
        except Exception:
            continue
        pred = (cap - bin_rtc).total_seconds()
        r = ncc_valid(gyrb, sg)
        if len(r) == 0:
            continue
        k = int(np.argmax(r))
        loc = k / fs
        corr = float(r[k])
        skew = loc - pred
        detail.append((name, pred, loc, corr, skew))
        if corr >= min_corr:
            skews.append(skew)
    if not skews:
        # fall back to all locks if none cleared the threshold
        skews = [d[4] for d in detail]
    skew = float(np.median(skews)) if skews else 0.0
    return SkewModel(skew_s=skew, n_used=len(skews), detail=detail)


# --------------------------------------------------------------------------- #
# Per-trial localization
# --------------------------------------------------------------------------- #
@dataclass
class AlignResult:
    bin_start_idx: int
    bin_end_idx: int
    n_sync: int
    fs_sync: float
    corr: float
    skew_s: float
    pred_idx: int
    refine_shift_s: float
    t_start_s: float
    t_end_s: float


def align_trial(bin_acc_mag_256, bin_gyr_mag_256, fs_high, bin_rtc,
                root, subject, session, task, trial, node,
                skew_s, refine_window_s=4.0, refine_min_corr=0.5) -> AlignResult:
    """Localize one trial: timestamp anchor + skew, then small local xcorr refine.

    The refine is applied ONLY when its correlation is confident; short (~3 s)
    walking templates are not, so for those we keep the timestamp+skew anchor
    (uncertainty ~ the skew spread, < 0.5 s). Sub-sample validation alignment is
    done marker-based at the validation stage.
    """
    sync_path = sync_csv_path(root, subject, session, task, trial)
    n_sync = sum(1 for _ in open(sync_path)) - 1
    fs_sync = 100.0
    cap = c3d_capture_datetime(c3d_path(root, subject, session, task, trial))
    pred_s = (cap - bin_rtc).total_seconds() + skew_s
    pred_idx = int(round(pred_s * fs_high))

    # Refine with a tiny local window (band-match BIN to 100 Hz).
    W = int(refine_window_s * fs_high)
    lo = max(0, pred_idx - W)
    hi = min(len(bin_gyr_mag_256), pred_idx + W + int(n_sync * fs_high / fs_sync))
    seg = resample_poly(bin_gyr_mag_256[lo:hi], 25, 64)
    sg = _sync_gyro_mag(sync_path, node)
    r = ncc_valid(seg, sg)
    if len(r) and float(np.max(r)) >= refine_min_corr:
        k = int(np.argmax(r))
        corr = float(r[k])
        start_100 = (lo / fs_high) * fs_sync + k       # 100 Hz samples from file start
        bin_start_idx = int(round(start_100 / fs_sync * fs_high))
    else:
        corr = float(np.max(r)) if len(r) else float("nan")
        bin_start_idx = pred_idx                       # keep timestamp+skew anchor

    n_sync_256 = int(round(n_sync * fs_high / fs_sync))
    return AlignResult(
        bin_start_idx=bin_start_idx,
        bin_end_idx=bin_start_idx + n_sync_256,
        n_sync=n_sync,
        fs_sync=fs_sync,
        corr=corr,
        skew_s=skew_s,
        pred_idx=pred_idx,
        refine_shift_s=(bin_start_idx - pred_idx) / fs_high,
        t_start_s=bin_start_idx / fs_high,
        t_end_s=(bin_start_idx + n_sync_256) / fs_high,
    )


if __name__ == "__main__":
    import yaml
    from bin_reader import read_bin, ACC_TO_MS2, GYR_TO_RADS
    cfg = yaml.safe_load(open("config/default.yaml"))
    root = cfg["dataset"]["root"]; subj = cfg["dataset"]["subject"]; sess = cfg["dataset"]["session"]
    node = cfg["selection"]["ref_align_node"]; sensor = cfg["sensor_map"][node]
    binpath = os.path.join(root, f"{subj}_{sess}", "RAW_DATA",
                           f"{subj}_{sess}_{sensor}_Inertial_sensor.BIN")
    bd = read_bin(binpath)
    acc_mag = np.linalg.norm(bd.acc_raw.astype(float) * ACC_TO_MS2, axis=1)
    gyr_mag = np.linalg.norm(bd.gyr_raw.astype(float) * GYR_TO_RADS, axis=1)
    rtc = bin_rtc_datetime(bd.start_datetime)
    sk = estimate_session_skew(gyr_mag, bd.fs_high, rtc, root, subj, sess, node)
    print(f"Session skew = {sk.skew_s:+.2f} s (from {sk.n_used} confident long trials)")
    for name, pred, loc, corr, skew in sk.detail:
        flag = "*" if corr >= 0.6 else " "
        print(f"  {flag} {name:20s} pred={pred:7.1f} loc={loc:7.1f} corr={corr:.3f} skew={skew:+.1f}")
    print("\n2minWalk localization (timestamp+skew, refined):")
    for trial in cfg["selection"]["trials"]:
        res = align_trial(acc_mag, gyr_mag, bd.fs_high, rtc, root, subj, sess,
                          cfg["selection"]["task"], trial, node, sk.skew_s)
        print(f"  2minWalk_{trial}: idx={res.bin_start_idx:8d} t=[{res.t_start_s:7.2f},{res.t_end_s:7.2f}]s "
              f"corr={res.corr:.3f} refine={res.refine_shift_s:+.2f}s")
