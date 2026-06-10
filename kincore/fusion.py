"""
kincore/fusion.py — Per-segment orientation from BIN-native IMU.

Implements the canonical Madgwick filter in two modes from the **same** native input:
  * 6-DOF (IMU): accel + gyro only. Yaw/heading is unobservable -> drifts with gyro bias.
  * 9-DOF (MARG): accel + gyro + magnetometer. Heading is corrected by the magnetometer.

Both run on the 256 Hz BIN clock. Quaternions are sensor->earth, scalar-first [w,x,y,z],
earth frame = ENU-like with gravity along +z (NWU mag convention as in Madgwick's paper).

References: S. Madgwick, "An efficient orientation filter for inertial and inertial/
magnetic sensor arrays" (2010).
"""
from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------- #
# Quaternion helpers (scalar-first)
# --------------------------------------------------------------------------- #
def q_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def q_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def q_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else q


def q_rotate(q, v):
    """Rotate vector v from sensor frame to earth frame by q (sensor->earth)."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    return q_mult(q_mult(q, qv), q_conj(q))[1:]


def quat_to_euler(q):
    """ZYX intrinsic (yaw, pitch, roll) in radians from sensor->earth quaternion."""
    w, x, y, z = q
    # roll (x)
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    # pitch (y)
    sp = 2*(w*y - z*x)
    sp = np.clip(sp, -1.0, 1.0)
    pitch = np.arcsin(sp)
    # yaw (z)
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([yaw, pitch, roll])


def quats_to_euler(qs):
    return np.array([quat_to_euler(q) for q in qs])


# --------------------------------------------------------------------------- #
# Initialisation
# --------------------------------------------------------------------------- #
def init_quat_from_acc_mag(acc, mag=None):
    """Initial orientation from a (near-static) accel (gravity) and optional mag.

    Builds an earth frame: z = up (from -gravity/+specific-force), and if mag given,
    resolves heading; otherwise yaw = 0.
    """
    a = acc / (np.linalg.norm(acc) + 1e-12)
    # accel measures specific force; at rest it points "up" (opposes gravity).
    # Build rotation whose body z-axis maps to measured up.
    # roll/pitch from accel:
    roll = np.arctan2(a[1], a[2])
    pitch = np.arctan2(-a[0], np.sqrt(a[1]*a[1] + a[2]*a[2]))
    yaw = 0.0
    if mag is not None and np.linalg.norm(mag) > 0:
        m = mag / np.linalg.norm(mag)
        # tilt-compensated heading
        mx = m[0]*np.cos(pitch) + m[2]*np.sin(pitch)
        my = (m[0]*np.sin(roll)*np.sin(pitch) + m[1]*np.cos(roll)
              - m[2]*np.sin(roll)*np.cos(pitch))
        yaw = np.arctan2(-my, mx)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    q = np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])
    return q_norm(q)


# --------------------------------------------------------------------------- #
# Madgwick updates
# --------------------------------------------------------------------------- #
def _madgwick_6dof(q, gyr, acc, dt, beta):
    q0, q1, q2, q3 = q
    if np.linalg.norm(acc) == 0:
        qdot = 0.5 * q_mult(q, np.array([0.0, *gyr]))
        return q_norm(q + qdot * dt)
    a = acc / np.linalg.norm(acc)
    ax, ay, az = a
    # gradient (objective: gravity)
    f = np.array([
        2*(q1*q3 - q0*q2) - ax,
        2*(q0*q1 + q2*q3) - ay,
        2*(0.5 - q1*q1 - q2*q2) - az,
    ])
    J = np.array([
        [-2*q2,  2*q3, -2*q0, 2*q1],
        [ 2*q1,  2*q0,  2*q3, 2*q2],
        [ 0.0,  -4*q1, -4*q2, 0.0],
    ])
    grad = J.T @ f
    grad = grad / (np.linalg.norm(grad) + 1e-12)
    qdot = 0.5 * q_mult(q, np.array([0.0, *gyr])) - beta * grad
    return q_norm(q + qdot * dt)


def _madgwick_9dof(q, gyr, acc, mag, dt, beta):
    if mag is None or np.linalg.norm(mag) == 0:
        return _madgwick_6dof(q, gyr, acc, dt, beta)
    q0, q1, q2, q3 = q
    if np.linalg.norm(acc) == 0:
        qdot = 0.5 * q_mult(q, np.array([0.0, *gyr]))
        return q_norm(q + qdot * dt)
    a = acc / np.linalg.norm(acc)
    ax, ay, az = a
    m = mag / np.linalg.norm(mag)
    mx, my, mz = m

    # reference direction of earth's magnetic field
    h = q_mult(q_mult(q, np.array([0.0, mx, my, mz])), q_conj(q))
    bx = np.sqrt(h[1]*h[1] + h[2]*h[2])
    bz = h[3]

    f = np.array([
        2*(q1*q3 - q0*q2) - ax,
        2*(q0*q1 + q2*q3) - ay,
        2*(0.5 - q1*q1 - q2*q2) - az,
        2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2) - mx,
        2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3) - my,
        2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2) - mz,
    ])
    J = np.array([
        [-2*q2,             2*q3,            -2*q0,             2*q1],
        [ 2*q1,             2*q0,             2*q3,             2*q2],
        [ 0.0,             -4*q1,            -4*q2,             0.0],
        [-2*bz*q2,          2*bz*q3,         -4*bx*q2-2*bz*q0, -4*bx*q3+2*bz*q1],
        [-2*bx*q3+2*bz*q1,  2*bx*q2+2*bz*q0,  2*bx*q1+2*bz*q3, -2*bx*q0+2*bz*q2],
        [ 2*bx*q2,          2*bx*q3-4*bz*q1,  2*bx*q0-4*bz*q2,  2*bx*q1],
    ])
    grad = J.T @ f
    grad = grad / (np.linalg.norm(grad) + 1e-12)
    qdot = 0.5 * q_mult(q, np.array([0.0, *gyr])) - beta * grad
    return q_norm(q + qdot * dt)


def run_madgwick(gyr, acc, mag=None, fs=256.0, beta=0.05, q0=None, init_n=64):
    """Run Madgwick over a sequence.

    gyr (N,3) rad/s, acc (N,3) m/s^2, mag (N,3) or None (same units, frame as acc).
    Returns quaternions (N,4) sensor->earth.
    """
    n = len(gyr)
    dt = 1.0 / fs
    if q0 is None:
        a0 = acc[:init_n].mean(axis=0)
        m0 = mag[:init_n].mean(axis=0) if mag is not None else None
        q0 = init_quat_from_acc_mag(a0, m0)
    q = q0.copy()
    out = np.empty((n, 4))
    use_mag = mag is not None
    for i in range(n):
        if use_mag:
            q = _madgwick_9dof(q, gyr[i], acc[i], mag[i], dt, beta)
        else:
            q = _madgwick_6dof(q, gyr[i], acc[i], dt, beta)
        out[i] = q
    return out


def rot_matrix(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])


def pick_horizontal_axis(qs):
    """Choose the body axis whose earth-frame projection is most horizontal on average.

    Gives a gimbal-lock-free heading reference (azimuth of that axis), robust to the
    sensor's mounting orientation (e.g. pelvis sensor with a near-vertical body axis).
    """
    Rs = np.array([rot_matrix(q) for q in qs])      # (N,3,3) sensor->earth
    # earth-frame image of each body axis = columns of R; horizontal magnitude = sqrt(x^2+y^2)
    horiz = np.sqrt(Rs[:, 0, :]**2 + Rs[:, 1, :]**2).mean(axis=0)   # per body axis
    return int(np.argmax(horiz)), Rs


def heading_deg(qs, body_axis=None, Rs=None):
    """Gimbal-lock-free unwrapped heading (deg): azimuth of a horizontal body axis.

    Heading = atan2(earth_y, earth_x) of the chosen body axis, i.e. rotation about the
    earth vertical. Avoids the ZYX-Euler yaw singularity at pitch ~ +/-90 deg.
    """
    if Rs is None:
        body_axis, Rs = pick_horizontal_axis(qs)
    elif body_axis is None:
        horiz = np.sqrt(Rs[:, 0, :]**2 + Rs[:, 1, :]**2).mean(axis=0)
        body_axis = int(np.argmax(horiz))
    fwd = Rs[:, :, body_axis]                        # earth-frame image of body axis
    return np.degrees(np.unwrap(np.arctan2(fwd[:, 1], fwd[:, 0]))), body_axis, Rs


def tilt_deg(qs, Rs=None):
    """Tilt = angle between the sensor's body-z mapped to earth and earth vertical (deg).

    Mag does not (much) affect tilt, so 6-DOF and 9-DOF should agree here — a good
    validity check that the only intended difference is heading.
    """
    if Rs is None:
        Rs = np.array([rot_matrix(q) for q in qs])
    bz_earth = Rs[:, :, 2]                            # earth image of body z
    cosang = np.clip(bz_earth[:, 2], -1, 1)
    return np.degrees(np.arccos(cosang))
