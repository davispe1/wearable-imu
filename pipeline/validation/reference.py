"""
validation/reference.py — Marker-derived reference joint angles + IMU comparison.

Reference sagittal joint flexion is built from the c3d joint centres (RHJC/RKJC/RAJC)
and pelvis markers, projected onto the sagittal plane (perpendicular to the pelvis
mediolateral axis) and zeroed at the Static neutral pose. The IMU-computed angles are
resampled onto the 100 Hz marker timeline (using the optical-time alignment), sub-sample
aligned, sign-matched, and scored by RMSE.

MARKERS ARE READ ONLY HERE. Gait events (Zeni) are read here as reference step events.
"""
from __future__ import annotations
import numpy as np
import ezc3d


def read_markers(path):
    c = ezc3d.c3d(path)
    labels = c["parameters"]["POINT"]["LABELS"]["value"]
    pts = c["data"]["points"]            # (4, nmarkers, nframes)
    rate = float(c["parameters"]["POINT"]["RATE"]["value"][0])
    M = {lab: pts[:3, i, :].T for i, lab in enumerate(labels)}   # label -> (nframes,3)
    return M, rate, c


def _unit(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.clip(n, 1e-9, None)


def _signed_angle(u, v, ml):
    """Signed angle (rad) from u to v in the plane perpendicular to ml (per frame)."""
    ml = _unit(ml)
    up = u - (np.einsum("ij,ij->i", u, ml))[:, None] * ml
    vp = v - (np.einsum("ij,ij->i", v, ml))[:, None] * ml
    up = _unit(up); vp = _unit(vp)
    cross = np.cross(up, vp)
    sin = np.einsum("ij,ij->i", cross, ml)
    cos = np.einsum("ij,ij->i", up, vp)
    return np.arctan2(sin, cos)


def sagittal_reference(M, side="R"):
    """Return dict of signed sagittal flexion (rad) per joint over frames.

    `side` = "R" or "L". Uses joint centres for segment long axes and the pelvis ML axis
    for the sagittal plane. Angles are raw (not yet neutral-subtracted).
    """
    other = "L" if side == "R" else "R"
    def g(name): return M[name]
    HJC, KJC, AJC = g(f"{side}HJC"), g(f"{side}KJC"), g(f"{side}AJC")
    # pelvis mediolateral axis (contralateral - ipsilateral hip joint centre)
    if f"{other}HJC" in M:
        ml = _unit(g(f"{other}HJC") - HJC)
    else:
        ml = _unit(g(f"{other}ASI") - g(f"{side}ASI"))
    # pelvis anterior axis and down axis
    ap = _unit(g("midASIS") - g("SACR")) if ("midASIS" in M and "SACR" in M) else None
    thigh = KJC - HJC
    shank = AJC - KJC
    foot = (g(f"{side}TOE") - AJC) if f"{side}TOE" in M else (g(f"{side}FMH") - AJC)
    out = {}
    out["knee"] = _signed_angle(thigh, shank, ml)        # straight ~0
    out["ankle"] = _signed_angle(shank, foot, ml)
    if ap is not None:
        pelvis_down = _unit(np.cross(ml, ap))            # ml x ap -> vertical-ish
        # force it to point downward (global -z) so the sign convention is leg-independent
        if np.nanmean(pelvis_down[:, 2]) > 0:
            pelvis_down = -pelvis_down
    else:
        pelvis_down = np.tile([0, 0, -1.0], (len(thigh), 1))
    out["hip"] = _signed_angle(pelvis_down, thigh, ml)
    return out


def neutral_reference(static_path, side="R"):
    M, _, _ = read_markers(static_path)
    ref = sagittal_reference(M, side)
    return {k: float(np.nanmedian(v)) for k, v in ref.items()}


def window_reference(c3d_path, neutral, side="R"):
    """Reference flexion (deg) per joint over a trial window, neutral-subtracted."""
    M, rate, c = read_markers(c3d_path)
    raw = sagittal_reference(M, side)
    ang = {k: np.degrees(raw[k] - neutral[k]) for k in raw}
    return ang, rate, c


def c3d_events(c):
    """Reference gait events (Zeni) -> dict context-label -> times (s)."""
    p = c["parameters"]
    if "EVENT" not in p:
        return {}
    ev = p["EVENT"]
    out = {}
    try:
        ctx = ev["CONTEXTS"]["value"]; lab = ev["LABELS"]["value"]
        times = np.array(ev["TIMES"]["value"])[1]      # row 1 = seconds
        for cc, ll, tt in zip(ctx, lab, times):
            out.setdefault(f"{cc} {ll}", []).append(float(tt))
    except Exception:
        pass
    return out


def best_lag_rmse(imu_ang, ref_ang, fs_ref, max_lag_s=0.3):
    """Sub-sample sign-matched RMSE between IMU and reference angle (same length, fs_ref).

    Returns (rmse_deg, lag_s, sign, corr). IMU is shifted by integer lag (<= max_lag)
    to remove residual timing; sign is matched to the reference.
    """
    n = min(len(imu_ang), len(ref_ang))
    a = imu_ang[:n].copy(); b = ref_ang[:n].copy()
    a = a - np.nanmean(a); b = b - np.nanmean(b)
    sign = 1.0 if np.nansum(a*b) >= 0 else -1.0
    a = a * sign
    maxl = int(max_lag_s * fs_ref)
    best = (np.inf, 0, 0.0)
    for lag in range(-maxl, maxl+1):
        if lag >= 0:
            aa = a[lag:]; bb = b[:len(aa)]
        else:
            bb = b[-lag:]; aa = a[:len(bb)]
        m = min(len(aa), len(bb))
        if m < 10:
            continue
        rmse = float(np.sqrt(np.nanmean((aa[:m]-bb[:m])**2)))
        if rmse < best[0]:
            corr = float(np.corrcoef(aa[:m], bb[:m])[0, 1])
            best = (rmse, lag, corr)
    return best[0], best[1]/fs_ref, sign, best[2]
