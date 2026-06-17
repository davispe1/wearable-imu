"""
gaitlib/pipeline.py — The high-level entry point: raw 9-DOF data -> gait parameters.

``compute(raw_data, mounting_config)`` is the single public function. It is **pure**: data
in, parameters out. It knows nothing about serial ports, sockets, pins, or registers — those
are separate layers. Internally it runs the validated kinematic stages, unchanged in
substance, in order:

    calibration  (magnetometer hard/soft-iron; 9-DOF only)        gaitlib.calibration
    filtering    (per-channel rate alignment + grid resampling)   gaitlib.rawdata
    fusion       (6/9-DOF Madgwick orientation per segment)       gaitlib.fusion
    joint angles (yaw-immune sagittal flexion, relative)          gaitlib.angles
    gait events  (foot strike / toe-off / mid-swing, cadence)     gaitlib.gait + segment
    parameters   (per-joint ROM/peaks/reps; gait cadence/stance)  gaitlib.parameters

6-DOF is the primary orientation (sagittal flexion is yaw-immune and drift-free); 9-DOF can
be requested for comparison but does not require — and the library never requires — optical
or marker reference data.
"""
from __future__ import annotations

import numpy as np

from . import fusion as F, calibration as C, angles as A, gait as G, segment as S
from . import parameters as P
from .config import MountingConfig
from .rawdata import load_raw, infer_rate
from .results import GaitResults


# --------------------------------------------------------------------------- #
def _neutral_gravity(acc, gyr, fs, win_s=0.5):
    """Sensor-frame 'up' at neutral = mean accel over the lowest-motion window.

    No dedicated static trial is assumed; the quietest ~win_s stretch (smallest gyro
    energy) is used as the neutral reference that sets where flexion reads zero.
    """
    acc = np.asarray(acc, float)
    gmag = np.linalg.norm(np.asarray(gyr, float), axis=1)
    w = max(1, int(win_s * fs))
    if len(gmag) <= w:
        a0 = acc.mean(0)
    else:
        e = np.convolve(gmag, np.ones(w), "valid")
        i = int(np.argmin(e))
        a0 = acc[i:i + w].mean(0)
    return a0 / (np.linalg.norm(a0) + 1e-12)


def _resample(sig, t_src, t_grid):
    sig = np.asarray(sig, float)
    if sig.ndim == 1:
        return np.interp(t_grid, t_src, sig)
    return np.column_stack([np.interp(t_grid, t_src, sig[:, k]) for k in range(sig.shape[1])])


def _fit_mag(node_stream, fs, min_samples=2000):
    """Fit hard/soft-iron calibration from this recording's own mag+accel (whole stream)."""
    acc = node_stream["acc"]
    mag = np.asarray(node_stream["mag"], float)
    n = len(mag)
    ms, asamp, _ = C.gather_orientation_samples(mag, acc, fs, fs, [(0, n)])
    if len(ms) < min_samples:
        return C.identity_calibration()
    try:
        return C.fit_mag_calibration(ms, asamp, source_windows=["recording"])
    except Exception:
        return C.identity_calibration()


# --------------------------------------------------------------------------- #
def compute(raw_data, mounting_config=None) -> GaitResults:
    """Convert raw 9-DOF sensor data into commonly measured gait parameters.

    Parameters
    ----------
    raw_data
        The recorded inertial/magnetic data in any form accepted by
        :func:`gaitlib.rawdata.load_raw` — a per-node dict, an iterable of per-sample rows,
        or a structured array (see :mod:`gaitlib.rawdata` for the input contract). Per
        sample: ``timestamp, node_id, ax,ay,az (m/s^2), gx,gy,gz (rad/s), mx,my,mz``.
    mounting_config
        A :class:`gaitlib.config.MountingConfig` (or a dict, or ``None`` for the default
        4-sensor single-leg config). Declares which sensors are present, each sensor's body
        segment, the joint topology, the per-channel sample rates, and fusion options.

    Returns
    -------
    GaitResults
        Per-joint angle/velocity traces + parameters, bout-level gait parameters, gait
        events, steady-state mask, and warnings. Joints whose two segments are not both
        present are skipped with a warning (or raise, in ``strict`` mode).
    """
    cfg = MountingConfig.coerce(mounting_config)
    streams = load_raw(raw_data, cfg)
    present = list(streams.keys())
    warnings_out = []

    # which joints can we compute given the sensors actually present?
    joints, skipped = cfg.resolve_joints(present)
    for jname, missing in skipped.items():
        warnings_out.append(f"joint {jname!r} skipped: missing sensor(s) {missing}")
    if not joints:
        warnings_out.append("no joints computable from the present sensors")

    # foot / pelvis nodes (fall back gracefully if the declared ones are absent)
    foot_node = cfg.foot_node if cfg.foot_node in present else (present[0] if present else None)
    if cfg.foot_node and cfg.foot_node not in present:
        warnings_out.append(f"foot node {cfg.foot_node!r} absent; gait events use {foot_node!r}")
    pelvis_node = cfg.pelvis_node if cfg.pelvis_node in present else None

    # sample rate: from the foot node's timestamps, else the declared IMU rate
    fs = infer_rate(streams[foot_node]["t"]) or cfg.imu_hz
    run_modes = tuple(cfg.fusion.run_modes)

    # common grid over the overlap of all present nodes (they share t=0 at start)
    t_end = min(float(streams[n]["t"][-1]) for n in present)
    tg = np.arange(0.0, t_end, 1.0 / fs)

    # ---- per-node neutral reference + grid-resampled accel/gyro ---- #
    grav0, gyr_grid, acc_grid, calib = {}, {}, {}, {}
    quats = {m: {} for m in run_modes}
    for n in present:
        nd = streams[n]
        grav0[n] = _neutral_gravity(nd["acc"], nd["gyr"], fs)
        gyr_grid[n] = _resample(nd["gyr"], nd["t"], tg)
        acc_grid[n] = _resample(nd["acc"], nd["t"], tg)
        if "9dof" in run_modes:
            calib[n] = _fit_mag(nd, fs)

    # ---- orientation fusion per node, per mode (native clock) ---- #
    for n in present:
        nd = streams[n]
        for m in run_modes:
            if m == "9dof":
                magc = calib[n].apply(np.asarray(nd["mag"], float))
                q = F.run_madgwick(nd["gyr"], nd["acc"], magc, fs, cfg.fusion.beta_9dof)
            else:
                q = F.run_madgwick(nd["gyr"], nd["acc"], None, fs, cfg.fusion.beta_6dof)
            quats[m][n] = A.slerp_resample(q, np.asarray(nd["t"], float), tg)

    # ---- turnarounds + steady-state mask (pelvis; fallback: all steady) ---- #
    if pelvis_node:
        turns, _ = S.detect_turnarounds(acc_grid[pelvis_node], gyr_grid[pelvis_node], fs)
    else:
        turns = []
        warnings_out.append("no pelvis node: turnarounds not detected; whole bout treated steady")
    mask = S.steady_state_mask(len(tg), turns, fs)

    # ---- joint angles per mode ---- #
    primary = "6dof" if "6dof" in run_modes else run_modes[0]
    joint_out = {}
    for jname, (dist, prox) in joints.items():
        per_mode = {}
        for m in run_modes:
            ja = A.joint_angles(quats[m][dist], quats[m][prox], gyr_grid[dist], gyr_grid[prox],
                                grav0[dist], grav0[prox], axis_mask=mask, fs=fs,
                                tau=cfg.fusion.joint_tau_s)
            flex = ja["flexion"]
            per_mode[m] = {"flexion": flex,
                           "ang_vel": A.derivative(flex, fs),
                           "ang_acc": A.derivative(A.derivative(flex, fs), fs)}
        prim = per_mode[primary]
        rec = {"flexion": prim["flexion"], "ang_vel": prim["ang_vel"],
               "ang_acc": prim["ang_acc"]}
        if len(run_modes) > 1:
            rec["modes"] = per_mode
        joint_out[jname] = rec

    # ---- gait events + cadence on the foot (raw gyro, fusion-independent) ---- #
    ev = G.detect_events(gyr_grid[foot_node], fs) if foot_node else {
        "foot_strike": np.array([], int), "mid_swing": np.array([], int),
        "toe_off": np.array([], int)}
    cad = G.cadence_stats(ev.get("foot_strike", []), fs, mask=mask)

    # ---- parameters (final stage) ---- #
    for jname, rec in joint_out.items():
        rec["params"] = P.joint_parameters(rec["flexion"], rec["ang_vel"], mask, fs)
    gait_params = P.gait_parameters(cad, ev, mask, fs)

    return GaitResults(
        fs=fs, t=tg, joints=joint_out, gait=gait_params, events=ev,
        steady_state=mask, turnarounds=turns, primary_mode=primary,
        config=cfg.to_dict(), warnings=warnings_out,
        meta={"nodes": present, "foot_node": foot_node, "pelvis_node": pelvis_node,
              "run_modes": list(run_modes)},
    )
