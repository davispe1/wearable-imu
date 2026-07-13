"""
tools/make_synthetic_session.py — a physically-consistent SYNTHETIC gait session.

WHY THIS EXISTS
---------------
The pipeline (``core`` + ``kinematics``) has a strict raw-data contract: per-node 9-DOF CSVs
(accel m/s^2, gyro rad/s, magnetometer) on a shared hub clock. Real Physilog sessions satisfy
it but carry NO ground truth — you cannot know the true hip/knee/ankle angle a subject walked.
This tool generates a session that (a) lands exactly in that CSV contract and (b) ships the
KNOWN ground truth, so the whole chain can be *validated*, not just run.

WHY IT IS RELIABLE (forward simulation, orientation-first)
----------------------------------------------------------
We never hand-draw sensor signals. We prescribe the true kinematics — per-segment orientation
R(t) (sensor->earth) and the foot's world trajectory — from a normative sagittal gait model,
then DERIVE every sensor channel by the correct physics, matching the exact conventions the
pipeline speaks (see ``kinematics/quaternion.py``):

    gyro (sensor frame)  = vee( R^T @ dR/dt )              body angular velocity  [rad/s]
    accel (sensor frame) = R^T @ (a_lin_world - g_world)   specific force         [m/s^2]
    mag  (sensor frame)  = R^T @ B_world                   constant earth field   [uT]

with earth Z up and g_world = [0,0,-9.81]. Because the accel is a genuine specific force and
the gyro is the true derivative of R, VQF re-fuses them back into R (up to the heading it can
observe): 6D recovers tilt (drift-free), 9D additionally recovers absolute heading from the
clean magnetometer. That round-trip is the reliability guarantee — and the reason the
magnetometer is *useful* here rather than decorative.

THE MODEL (idealised, planar-sagittal, clean by design)
-------------------------------------------------------
* Normative sagittal curves: hip ~ single cosine (ROM 40 deg), knee ~ classic double bump
  (ROM ~57 deg), foot pitch ~ flat stance + push-off plantarflexion + swing clearance.
* A short QUIET-STANDING lead/tail so the pipeline's neutral (its quietest window) is a true
  standing pose -> reported flexion reads ~0 at stance, matching the ground truth absolutely.
* One instrumented leg (right): nodes SA (pelvis), RT (thigh), RS (shank), RF (foot), the
  shipped default rig.
* Straight walk on a constant heading ``psi`` (default 40 deg) so 9D has a real absolute
  heading to recover and 6D does not — the honest demonstration of the magnetometer's value.

This is a fixture / regression anchor / demo, NOT real data: it has no soft-tissue artefact and
(by default) no sensor noise or mounting misalignment. Those are available as flags for realism.

USAGE
-----
    python -m tools.make_synthetic_session                 # writes data/SYN01_S01_straightWalk
    python -m tools.make_synthetic_session --out data/MY --strides 12 --heading-deg 40
    python -m tools.make_synthetic_session --gyro-drift 1.5 # inject a known gyro-heading drift
                                                            # (the 9D-vs-6D correction demo)
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np
from scipy.ndimage import gaussian_filter1d

G_WORLD = np.array([0.0, 0.0, -9.81])       # gravity acceleration (earth frame, Z up)

# --- normative sagittal curves, sampled at 0..100 % of the gait cycle (deg) --------------- #
# Knee: classic loading-response bump (~18 deg) + swing peak (~60 deg). Ankle (+ = dorsiflexion):
# brief controlled plantarflexion at loading, dorsiflexion through stance, push-off
# plantarflexion (~-15 deg) at toe-off, swing back to neutral. Physiological joint ROM ~25 deg
# (the FOOT SEGMENT still pitches ~65 deg, because it rides the shank -> foot pitch = shank+ankle).
_PCT = np.linspace(0.0, 1.0, 21)
_KNEE = np.array([3, 8, 14, 18, 17, 13, 8, 5, 4, 5, 8, 15, 28, 42, 54, 60, 59, 50, 34, 15, 3.0])
_ANKLE = np.array([0, -3, -5, -4, -1, 2, 5, 8, 10, 9, 5, -3, -13, -15, -8, -2, 1, 2, 1, 0, 0.0])


def _hip_deg(phase):
    """Hip flexion (deg) over the cycle: single cosine, flexed at contact, extended mid-stance."""
    return 10.0 + 20.0 * np.cos(2.0 * np.pi * phase)          # ROM 40: +30 at IC, -10 at 50 %


def _cyc(curve, phase):
    """Sample a 0..100 % normative curve (deg) at fractional gait phases (wraps at 1.0)."""
    return np.interp(phase % 1.0, _PCT, curve, period=1.0)


# --- small rotation helpers ---------------------------------------------------------------- #
def _Ry(a):
    """Rotation about +Y by angle a (rad), stacked over a (N,) vector -> (N,3,3)."""
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, z, s], -1), np.stack([z, o, z], -1),
                     np.stack([-s, z, c], -1)], -2)


def _Rz(a):
    """Rotation about +Z by angle a (rad) -> (N,3,3)."""
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, -s, z], -1), np.stack([s, c, z], -1),
                     np.stack([z, z, o], -1)], -2)


def _seg_R(alpha_deg, psi):
    """Sensor->earth rotation for a segment tilted ``alpha`` (deg, + = flexed forward) on
    constant heading ``psi`` (rad). R = Rz(psi) @ Ry(-alpha); distal end moves +forward for
    positive alpha, and the sensor Z is earth-up at the neutral (alpha=0) pose."""
    a = np.radians(np.asarray(alpha_deg, float))
    return _Rz(np.full_like(a, psi)) @ _Ry(-a)


def _gyro_from_R(R, dt):
    """Body-frame angular velocity (rad/s) from an (N,3,3) rotation sequence: vee(R^T @ dR)."""
    dR = np.gradient(R, dt, axis=0)
    W = np.einsum("nji,njk->nik", R, dR)                     # R^T @ dR (skew-symmetric)
    return np.stack([W[:, 2, 1], W[:, 0, 2], W[:, 1, 0]], -1)


def _accel_from(R, pos, dt):
    """Specific force in the sensor frame (m/s^2): R^T @ (d2pos/dt2 - g_world)."""
    a_lin = np.gradient(np.gradient(pos, dt, axis=0), dt, axis=0)
    return np.einsum("nji,nj->ni", R, a_lin - G_WORLD)


def _mag_from(R, B_world):
    """Constant earth magnetic field expressed in the sensor frame (uT): R^T @ B_world."""
    return np.einsum("nji,j->ni", R, B_world)


def _heelstrike_notch(angle, ic_idx, fs, amp_deg, sigma_s=0.022):
    """Add a brief 'loading-response' transient to the shank tilt at each initial contact (deg).

    A purely smooth rigid-body model has the shank still near peak forward velocity at heel
    strike, so the event detector's 'initial contact = angular-rate minimum after mid-swing'
    lands ~0.1 s late and inflates the detected swing fraction. Real heel strike arrests the
    limb sharply (loading response / impact), giving a distinct rate reversal right at contact.

    The transient is an ODD wavelet (derivative-of-Gaussian): its steepest descent — hence the
    angular-rate MINIMUM the detector keys on — sits exactly at IC, and it integrates to zero so
    it adds NO net angle (joint ROMs unchanged). Injecting it in the ANGLE (not the gyro) makes
    the derived gyroscope carry it with the physically-correct sign automatically, because the
    detector's oriented sagittal rate is proportional to +d(shank tilt)/dt.
    """
    sig = sigma_s * fs
    x = np.arange(len(angle))
    for i in ic_idx:
        u = (x - i) / sig
        angle = angle - amp_deg * u * np.exp(-0.5 * u * u)
    return angle


# --- foot world trajectory (drives ZUPT stride length) ------------------------------------- #
def _foot_path(t, t0, stride_time, n_strides, stride_len, walk_dir, clearance):
    """Foot sensor world position (N,3): planted through stance, min-jerk swing each stride.

    Genuinely stationary during stance (velocity 0) so the ZUPT anchor is a true foot-flat,
    and it advances exactly ``stride_len`` per stride so the recovered stride length is known.
    """
    stance_frac = 0.62
    pos = np.zeros((len(t), 3))
    for i, ti in enumerate(t):
        k = (ti - t0) / stride_time
        if k < 0:                                            # quiet-standing lead: planted at 0
            plant = 0.0
        elif k >= n_strides:                                 # quiet-standing tail: final plant
            plant = n_strides * stride_len
        else:
            ki, ph = int(k), k - int(k)
            if ph <= stance_frac:                            # stance: planted at plant_k
                plant = ki * stride_len
            else:                                            # swing: plant_k -> plant_{k+1}
                tau = (ph - stance_frac) / (1.0 - stance_frac)
                mj = 10 * tau**3 - 15 * tau**4 + 6 * tau**5  # min-jerk 0->1
                plant = (ki + mj) * stride_len
                pos[i, 2] = clearance * np.sin(np.pi * tau)  # swing arc (vertical clearance)
        pos[i, 0] += plant * walk_dir[0]
        pos[i, 1] += plant * walk_dir[1]
    return pos


def _taper(t, t0, t1, ramp):
    """1.0 inside [t0,t1] with cosine 0->1 / 1->0 ramps of width ``ramp`` at each end.

    Eases the walking angles out of / into the quiet-standing pose so there is no velocity
    step (hence no spurious gyro spike) at the static<->walk junctions.
    """
    e = np.ones_like(t)
    up, dn = (t - t0) / ramp, (t1 - t) / ramp
    e = np.where(t < t0, 0.0, e)
    e = np.where(t > t1, 0.0, e)
    m = (t >= t0) & (t < t0 + ramp)
    e[m] = 0.5 - 0.5 * np.cos(np.pi * up[m])
    m = (t <= t1) & (t > t1 - ramp)
    e[m] = 0.5 - 0.5 * np.cos(np.pi * dn[m])
    return e


# --- main build ---------------------------------------------------------------------------- #
def build(fs=100.0, n_strides=10, stride_time=1.1, stride_len=1.35, heading_deg=40.0,
          lead_s=1.5, tail_s=1.0, clearance=0.12, impact=9.0, gyro_drift_dps=0.0,
          noise=0.0, seed=0):
    """Return ``(t, nodes, ground_truth)``; ``nodes`` maps node id -> dict of channel arrays."""
    psi = np.radians(heading_deg)
    walk_dir = np.array([np.cos(psi), np.sin(psi), 0.0])
    t0 = lead_s                                              # first foot strike
    t1 = lead_s + n_strides * stride_time                   # last foot plant
    total = t1 + tail_s
    t = np.arange(0.0, total, 1.0 / fs)
    dt = 1.0 / fs
    n = len(t)

    # Gait phase per sample (only defined during the walk; static parts hold the neutral pose).
    walking = (t >= t0) & (t < t1)
    phase = np.zeros(n)
    phase[walking] = ((t[walking] - t0) / stride_time) % 1.0
    env = _taper(t, t0, t1, ramp=0.35)                      # ease angles in/out of standing

    # Prescribed sagittal angles (deg). Pelvis tilts twice per stride; segment tilts chain up.
    hip = _hip_deg(phase) * env * walking
    knee = _cyc(_KNEE, phase) * env * walking
    ankle = _cyc(_ANKLE, phase) * env * walking
    pelvis = 3.0 * np.sin(2.0 * np.pi * 2.0 * phase) * env * walking
    # Smooth the base curves (remove any stride-boundary discretisation step), then chain into
    # absolute segment tilts. The loading-response notch rides the shank (hence the foot) and is
    # added AFTER smoothing so it stays sharp; hip and ankle ground truth are left untouched.
    hip, knee, ankle, pelvis = (gaussian_filter1d(x, 2.0) for x in (hip, knee, ankle, pelvis))
    ic_idx = [int(round((t0 + k * stride_time) * fs)) for k in range(n_strides + 1)]
    a_pelvis = pelvis
    a_thigh = a_pelvis + hip
    a_shank = a_thigh - knee
    if impact:
        a_shank = _heelstrike_notch(a_shank, ic_idx, fs, impact)
    a_foot = a_shank + ankle                               # foot pitch = shank tilt + ankle

    # Segment orientations (sensor->earth).
    R = {"SA": _seg_R(a_pelvis, psi), "RT": _seg_R(a_thigh, psi),
         "RS": _seg_R(a_shank, psi), "RF": _seg_R(a_foot, psi)}

    # Sensor world positions (for realistic linear acceleration). Hip advances at walk speed
    # with a small bounce/sway; thigh & shank sensors hang off the kinematic chain; the foot
    # follows its own planted-stance trajectory (so ZUPT sees a true foot-flat).
    v = stride_len / stride_time
    s = np.clip(t - t0, 0, n_strides * stride_time) * v
    ph_c = np.where(walking, (t - t0) / stride_time, 0.0)
    hip_pos = (np.outer(s, walk_dir)
               + np.outer(0.02 * np.sin(2 * np.pi * ph_c) * walking, [0, 1, 0])   # lateral sway
               + np.outer(0.01 * np.cos(2 * np.pi * 2 * ph_c) * walking, [0, 0, 1]))  # bounce
    hip_pos[:, 2] += 0.92                                    # hip height
    L_th, L_sh = 0.42, 0.42

    def _u(alpha_deg):                                       # distal unit direction of a segment
        a = np.radians(alpha_deg)
        return np.stack([np.sin(a) * walk_dir[0], np.sin(a) * walk_dir[1], -np.cos(a)], -1)

    knee_pos = hip_pos + L_th * _u(a_thigh)
    pos = {
        "SA": hip_pos + np.array([0, 0, 0.08]),
        "RT": hip_pos + 0.5 * L_th * _u(a_thigh),
        "RS": knee_pos + 0.5 * L_sh * _u(a_shank),
        "RF": _foot_path(t, t0, stride_time, n_strides, stride_len, walk_dir, clearance),
    }
    for k in pos:
        pos[k] = gaussian_filter1d(pos[k], sigma=2.0, axis=0)

    # Earth magnetic field: |B|=48 uT, inclination 60 deg, +X = magnetic north.
    incl = np.radians(60.0)
    B_world = 48.0 * np.array([np.cos(incl), 0.0, -np.sin(incl)])

    rng = np.random.default_rng(seed)
    nodes = {}
    for k in R:
        gyr = _gyro_from_R(R[k], dt)
        acc = _accel_from(R[k], pos[k], dt)
        mag = _mag_from(R[k], B_world)
        if gyro_drift_dps:                                   # known constant heading-axis drift
            gyr = gyr + np.radians(gyro_drift_dps) * np.array([0, 0, 1.0])
        if noise:                                            # optional gaussian sensor noise
            acc = acc + rng.normal(0, noise, acc.shape)
            gyr = gyr + rng.normal(0, np.radians(noise * 5), gyr.shape)
            mag = mag + rng.normal(0, noise, mag.shape)
        nodes[k] = {"acc": acc, "gyr": gyr, "mag": mag}

    # Ground truth the pipeline can be scored against.
    strikes = [round(t0 + k * stride_time, 4) for k in range(n_strides + 1)]
    steady = walking & (env > 0.99)                          # full-amplitude strides only
    def _rom(x):
        return float(np.nanmax(x[steady]) - np.nanmin(x[steady]))
    ankle_truth = a_foot - a_shank                           # pipeline defines ankle = foot-shank
    gt = {
        "session_id": None, "fs": fs, "side": "right", "heading_deg": heading_deg,
        "model": "planar-sagittal normative gait, forward-simulated (orientation-first)",
        "n_strides": n_strides, "stride_time_s": stride_time,
        "cadence_steps_per_min": 120.0 / stride_time, "stance_pct": 62.0, "swing_pct": 38.0,
        "stride_length_m": stride_len, "walking_speed_mps": v,
        "foot_strike_times_s": strikes,
        "magnetometer": {"field_uT": B_world.round(3).tolist(), "inclination_deg": 60.0,
                         "gyro_drift_dps": gyro_drift_dps},
        "joint_rom_deg": {"hip": _rom(hip), "knee": _rom(knee), "ankle": _rom(ankle_truth)},
        "joint_peak_flexion_deg": {"hip": float(np.nanmax(hip[steady])),
                                   "knee": float(np.nanmax(knee[steady])),
                                   "ankle": float(np.nanmax(ankle_truth[steady]))},
        "note": "Idealised fixture: no soft-tissue artefact; noise/misalignment off by default.",
    }
    return t, nodes, gt


# --- writing ------------------------------------------------------------------------------- #
_HEADER = "t_native_s,t_opt_s,ax,ay,az,gx,gy,gz,mx,my,mz"
_OFFSETS = {"SA": 2648.9, "RT": 2642.5, "RS": 2635.2, "RF": 2632.6}   # cosmetic per-node clocks


def write_session(out_dir, t, nodes, gt):
    """Write per-node CSVs (RF/RS/RT/SA.csv) + ground_truth.json into ``out_dir``."""
    os.makedirs(out_dir, exist_ok=True)
    gt = dict(gt, session_id=os.path.basename(os.path.normpath(out_dir)))
    for node, ch in nodes.items():
        rows = np.column_stack([t + _OFFSETS.get(node, 0.0), t,
                                ch["acc"], ch["gyr"], ch["mag"]])
        np.savetxt(os.path.join(out_dir, f"{node}.csv"), rows, delimiter=",",
                   header=_HEADER, comments="", fmt="%.6f")
    with open(os.path.join(out_dir, "ground_truth.json"), "w") as f:
        json.dump(gt, f, indent=2)
    return out_dir


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate a synthetic, ground-truthed gait session "
                                             "in the pipeline's per-node CSV contract.")
    ap.add_argument("--out", default="data/SYN01_S01_straightWalk", help="output session dir")
    ap.add_argument("--fs", type=float, default=100.0, help="sample rate (Hz)")
    ap.add_argument("--strides", type=int, default=10, help="number of strides")
    ap.add_argument("--stride-time", type=float, default=1.1, help="stride time (s)")
    ap.add_argument("--stride-length", type=float, default=1.35, help="stride length (m)")
    ap.add_argument("--heading-deg", type=float, default=40.0, help="constant walking heading")
    ap.add_argument("--impact", type=float, default=9.0,
                    help="heel-strike loading-response notch amplitude (deg); 0 disables")
    ap.add_argument("--gyro-drift", type=float, default=0.0,
                    help="inject a known heading-axis gyro drift (deg/s) for the 9D-vs-6D demo")
    ap.add_argument("--noise", type=float, default=0.0, help="gaussian sensor noise (0 = clean)")
    ap.add_argument("--seed", type=int, default=0, help="noise RNG seed")
    a = ap.parse_args(argv)

    t, nodes, gt = build(fs=a.fs, n_strides=a.strides, stride_time=a.stride_time,
                         stride_len=a.stride_length, heading_deg=a.heading_deg,
                         impact=a.impact, gyro_drift_dps=a.gyro_drift, noise=a.noise, seed=a.seed)
    out = write_session(a.out, t, nodes, gt)
    print(f"wrote {out}/  ({len(t)} samples @ {a.fs:g} Hz, {a.strides} strides, "
          f"heading {a.heading_deg:g} deg)")
    print(f"  nodes: {', '.join(sorted(nodes))}   +ground_truth.json")
    rom = gt["joint_rom_deg"]
    print(f"  true ROM  hip={rom['hip']:.1f}  knee={rom['knee']:.1f}  ankle={rom['ankle']:.1f} deg"
          f"   cadence={gt['cadence_steps_per_min']:.1f}/min  stride_len={gt['stride_length_m']}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
