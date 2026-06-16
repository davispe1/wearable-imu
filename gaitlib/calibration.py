"""
gaitlib/calibration.py — Magnetometer hard/soft-iron calibration.

Two steps, both fitted from VARIED-ORIENTATION task windows (CalibrationTask hip/
pelvis rotations + TUG turns) — NOT walking and NOT sitting (gait/sitting don't excite
enough orientations to constrain the ellipsoid):

  1. Ellipsoid fit  -> hard-iron offset ``b`` and soft-iron matrix ``A`` such that
     ``A @ (m - b)`` lies on the unit sphere.
  2. Frame alignment -> a signed axis permutation ``P`` (the magnetometer chip axes
     may be permuted/flipped vs the accel/gyro axes). We pick the ``P`` that makes the
     magnetic dip angle (angle between field and gravity) most constant across the
     varied orientations — that is the geometric signature of mag and accel sharing a
     frame. Without this, 9-DOF fusion would be unfairly broken.

Apply BEFORE fusion. Calibrated, frame-aligned mag = ``P @ A @ (m - b)`` (unit-ish).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import permutations, product
import numpy as np


@dataclass
class MagCalibration:
    b: np.ndarray            # (3,) hard-iron offset (raw mag units)
    A: np.ndarray            # (3,3) soft-iron transform
    P: np.ndarray            # (3,3) signed permutation (mag->accel frame)
    dip_mean_deg: float      # mean angle between calibrated mag and gravity
    dip_std_deg: float       # spread (low => mag/accel share a frame & field clean)
    sphere_residual: float   # std of |A(m-b)| around 1.0 after ellipsoid fit
    n_samples: int = 0
    source_windows: list = field(default_factory=list)

    def apply(self, mag_raw: np.ndarray) -> np.ndarray:
        """Calibrate and frame-align raw magnetometer samples (N,3)."""
        m = (mag_raw - self.b) @ self.A.T
        return m @ self.P.T

    def to_dict(self):
        return {
            "hard_iron_offset_b": self.b.tolist(),
            "soft_iron_A": self.A.tolist(),
            "frame_permutation_P": self.P.tolist(),
            "dip_mean_deg": self.dip_mean_deg,
            "dip_std_deg": self.dip_std_deg,
            "sphere_residual": self.sphere_residual,
            "n_samples": int(self.n_samples),
            "source_windows": self.source_windows,
        }


# --------------------------------------------------------------------------- #
def fit_ellipsoid(M: np.ndarray):
    """Algebraic ellipsoid fit. Returns (b, A) with A(m-b) ~ unit sphere."""
    x, y, z = M[:, 0], M[:, 1], M[:, 2]
    D = np.column_stack([x*x, y*y, z*z, 2*y*z, 2*x*z, 2*x*y, 2*x, 2*y, 2*z])
    rhs = np.ones(len(M))
    v, *_ = np.linalg.lstsq(D, rhs, rcond=None)
    a, b_, c, f, g, h, p, q, r = v
    Q = np.array([[a, h, g], [h, b_, f], [g, f, c]])
    n = np.array([p, q, r])
    center = -np.linalg.solve(Q, n)
    # value at center: k = 1 + n^T Q^-1 n  (so x'^T (Q/k) x' = 1)
    k = 1.0 + n @ np.linalg.solve(Q, n)
    Qn = Q / k
    # symmetric PD square root -> A
    w, V = np.linalg.eigh(Qn)
    w = np.clip(w, 1e-12, None)
    A = V @ np.diag(np.sqrt(w)) @ V.T
    return center, A


def _signed_perms():
    """All 48 signed axis permutations as 3x3 matrices."""
    mats = []
    for perm in permutations(range(3)):
        for signs in product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, pidx in enumerate(perm):
                R[i, pidx] = signs[i]
            mats.append(R)
    return mats


def align_frame(mag_cal: np.ndarray, acc_at_mag: np.ndarray):
    """Find signed permutation P minimizing dip-angle variance (mag vs gravity)."""
    mu = mag_cal / (np.linalg.norm(mag_cal, axis=1, keepdims=True) + 1e-12)
    au = acc_at_mag / (np.linalg.norm(acc_at_mag, axis=1, keepdims=True) + 1e-12)
    best = None
    for P in _signed_perms():
        mp = mu @ P.T
        d = np.einsum("ij,ij->i", mp, au)        # cos(dip) per sample
        d = np.clip(d, -1, 1)
        ang = np.degrees(np.arccos(d))
        std = float(np.std(ang))
        if best is None or std < best[0]:
            best = (std, P, float(np.mean(ang)))
    return best[1], best[2], best[0]


def fit_mag_calibration(mag_samples: np.ndarray, acc_at_mag: np.ndarray,
                        source_windows=None) -> MagCalibration:
    b, A = fit_ellipsoid(mag_samples)
    mc = (mag_samples - b) @ A.T
    sphere_res = float(np.std(np.linalg.norm(mc, axis=1) - 1.0))
    P, dip_mean, dip_std = align_frame(mc, acc_at_mag)
    return MagCalibration(
        b=b, A=A, P=P,
        dip_mean_deg=dip_mean, dip_std_deg=dip_std,
        sphere_residual=sphere_res,
        n_samples=len(mag_samples),
        source_windows=source_windows or [],
    )


def identity_calibration(mag_dim_units=1.0) -> MagCalibration:
    return MagCalibration(b=np.zeros(3), A=np.eye(3)/mag_dim_units, P=np.eye(3),
                          dip_mean_deg=float("nan"), dip_std_deg=float("nan"),
                          sphere_residual=float("nan"))


# --------------------------------------------------------------------------- #
def gather_orientation_samples(mag_raw_64, acc_si_256, fs_high, fs_mag, windows_256):
    """Collect raw mag + accel-at-mag-instants over given 256 Hz windows.

    windows_256: list of (start_idx, end_idx) in 256 Hz samples.
    Accel is sampled at the magnetometer instants via the shared clock.
    """
    t_mag_all = np.arange(len(mag_raw_64)) / fs_mag
    t_acc_all = np.arange(len(acc_si_256)) / fs_high
    mags, accs, used = [], [], []
    for (s, e) in windows_256:
        t0, t1 = s / fs_high, e / fs_high
        jm = np.where((t_mag_all >= t0) & (t_mag_all < t1))[0]
        if len(jm) < 10:
            continue
        m = mag_raw_64[jm]
        tj = t_mag_all[jm]
        a = np.column_stack([np.interp(tj, t_acc_all, acc_si_256[:, k]) for k in range(3)])
        mags.append(m); accs.append(a)
        used.append({"t_start_s": t0, "t_end_s": t1, "n": int(len(jm))})
    if not mags:
        return np.empty((0, 3)), np.empty((0, 3)), used
    return np.vstack(mags), np.vstack(accs), used
