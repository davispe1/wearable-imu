"""
gaitlib/angles.py — Relative joint angles from per-segment orientations.

Joint angle = orientation of the distal segment relative to the proximal segment,
referenced to the neutral (static) pose so the standing posture reads ~0. The sagittal
**flexion** component is extracted about the joint's principal (mediolateral) axis,
which is gravity-anchored and therefore yaw-immune — so 6-DOF orientations suffice and
the (here unreliable) magnetometer heading does not contaminate flexion.

All inputs are IMU-derived quaternions; nothing here sees markers.
"""
from __future__ import annotations
import numpy as np
from .fusion import q_mult, q_conj, q_norm


def slerp_resample(q_src, t_src, t_grid):
    """Resample a quaternion sequence onto t_grid (linear-interp + renormalize).

    dt is small (1/256 s) so nlerp ~ slerp; we also guard sign continuity.
    """
    q = q_src.copy()
    # enforce hemisphere continuity
    for i in range(1, len(q)):
        if np.dot(q[i], q[i-1]) < 0:
            q[i] = -q[i]
    out = np.empty((len(t_grid), 4))
    for k in range(4):
        out[:, k] = np.interp(t_grid, t_src, q[:, k])
    n = np.linalg.norm(out, axis=1, keepdims=True)
    return out / np.clip(n, 1e-12, None)


def rotvec_from_quat(q):
    """Rotation vector (axis*angle, rad) for a quaternion array (N,4) scalar-first."""
    w = np.clip(q[:, 0], -1, 1)
    ang = 2*np.arccos(w)
    s = np.sqrt(np.clip(1 - w*w, 0, 1))
    axis = np.where(s[:, None] > 1e-8, q[:, 1:]/np.where(s[:, None] > 1e-8, s[:, None], 1), 0.0)
    # wrap angle to [-pi,pi]
    ang = np.where(ang > np.pi, ang - 2*np.pi, ang)
    return axis * ang[:, None]


def gravity_in_sensor(q):
    """Earth 'up' expressed in the sensor frame for each quaternion (N,4) -> (N,3).

    grav_sensor = R(q)^T @ [0,0,1]; equals the third ROW of R(q). Robust, fused tilt.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.column_stack([2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)])


def _joint_axis(gyr):
    """Mediolateral (flexion) axis in a sensor frame = largest-variance gyro direction."""
    C = gyr.T @ gyr
    w, V = np.linalg.eigh(C)
    return V[:, -1]


def _segment_sagittal_rotation(grav, grav0, axis):
    """Signed angle (rad) that gravity has rotated about `axis` from neutral grav0.

    Yaw-immune: uses only the gravity direction projected onto the plane perpendicular
    to the joint axis. Drift-free (no integration).
    """
    j = axis / (np.linalg.norm(axis) + 1e-12)
    g0 = grav0 - (grav0 @ j) * j
    g0 = g0 / (np.linalg.norm(g0) + 1e-12)
    gp = grav - (grav @ j)[:, None] * j
    gp = gp / (np.linalg.norm(gp, axis=1, keepdims=True) + 1e-12)
    cross = np.cross(np.broadcast_to(g0, gp.shape), gp)
    sin = cross @ j
    cos = gp @ g0
    return np.arctan2(sin, cos)


def joint_angles(q_dist, q_prox, gyr_dist, gyr_prox, grav0_dist, grav0_prox,
                 axis_mask=None, fs=256.0, tau=1.0):
    """Sagittal joint flexion (deg) over time, yaw-immune and drift-free.

    flexion(t) = dTheta_distal(t) - dTheta_proximal(t), where each dTheta is the
    rotation of that segment's gravity about its joint (mediolateral) axis, relative
    to the neutral pose. The joint axes are sign-aligned so flexion grows consistently.

    `axis_mask` restricts the joint-axis estimation to steady-state samples (turnarounds
    excluded) so the pelvis axis is the mediolateral one, not the turn (yaw) axis.
    """
    gd_axis = gyr_dist if axis_mask is None else gyr_dist[axis_mask]
    gp_axis = gyr_prox if axis_mask is None else gyr_prox[axis_mask]
    jd = _joint_axis(gd_axis)
    jp = _joint_axis(gp_axis)
    # sign-align joint axes: distal & proximal sagittal rates should be positively related
    rd = gyr_dist @ jd
    rp = gyr_prox @ jp
    if np.corrcoef(rd, rp)[0, 1] < 0:
        jp = -jp
    gd = gravity_in_sensor(q_dist)
    gp = gravity_in_sensor(q_prox)
    th_d = _segment_sagittal_rotation(gd, grav0_dist/np.linalg.norm(grav0_dist), jd)
    th_p = _segment_sagittal_rotation(gp, grav0_prox/np.linalg.norm(grav0_prox), jp)
    # gravity-projection joint angle (drift-free, but lags during fast motion):
    flex_grav = np.unwrap(th_d - th_p)
    # joint angular velocity about the joint axes (good for fast motion, drifts):
    joint_rate = gyr_dist @ jd - gyr_prox @ jp        # rad/s
    # complementary fusion: gyro for high frequency, gravity for low frequency / drift
    flex = _complementary(flex_grav, joint_rate, fs, tau)
    return {
        "flexion": np.degrees(flex),
        "flexion_gravity_only": np.degrees(flex_grav),
        "joint_rate_dps": np.degrees(joint_rate),
        "joint_axis_distal": jd,
        "joint_axis_proximal": jp,
    }


def _complementary(theta_lf, rate, fs, tau):
    """Complementary filter: high-pass gyro integral + low-pass gravity-based angle."""
    dt = 1.0 / fs
    a = tau / (tau + dt)
    out = np.empty_like(theta_lf)
    out[0] = theta_lf[0]
    for i in range(1, len(theta_lf)):
        out[i] = a * (out[i-1] + rate[i] * dt) + (1 - a) * theta_lf[i]
    return out


def derivative(x, fs):
    """Central finite difference (per-sample), same length."""
    d = np.gradient(x, 1.0/fs)
    return d


def rom(x):
    """Range of motion = max - min."""
    return float(np.nanmax(x) - np.nanmin(x))
