"""
run.py — End-to-end pipeline orchestrator (Phase B).

Flow:  BIN (read-only) -> per-sensor SI + optical alignment -> magnetometer calibration
       -> per-sensor 6-DOF & 9-DOF orientation (BIN-native clock) -> common-grid joint
       angles (sagittal flexion, yaw-immune) -> gait events / cadence on steady-state ->
       validation (markers, read ONLY here) -> outputs CSV/JSON.

6-DOF is the PRIMARY orientation (sagittal flexion is yaw-immune; the magnetometer is
distorted indoors). 9-DOF is computed and logged alongside for comparison.
"""
from __future__ import annotations
import json, os
import numpy as np
import yaml

import align
from bin_reader import read_bin
import extract
from kincore import fusion as F, calibration as C, angles as A, gait as G, segment as S
from validation import reference as R


def optical_pelvis_heading(M):
    """Lab-frame pelvis heading (deg, unwrapped) from pelvis markers over frames."""
    if "midASIS" in M and "SACR" in M:
        fwd = M["midASIS"] - M["SACR"]
    else:
        # ML x up gives anterior-ish; fall back to RASI->mean
        fwd = M["RASI"] - M["RPSI"] if ("RPSI" in M) else M["RASI"]
    return np.degrees(np.unwrap(np.arctan2(fwd[:, 1], fwd[:, 0])))


def validate(cfg, ctx):
    """Marker-based validation: per-joint RMSE (sub-sample aligned) + heading arbiter."""
    root = cfg["dataset"]["root"]; subj = cfg["dataset"]["subject"]; sess = cfg["dataset"]["session"]
    task = cfg["selection"]["task"]; fs = cfg["bin_format"]["fs_high"]
    joints = list(cfg["selection"]["joints"])
    T0, tg, results, aligns, epoch, foot = (ctx[k] for k in
        ("T0", "tg", "results", "aligns", "epoch", "foot"))
    # neutral from Static_01 markers
    try:
        neutral = R.neutral_reference(align.c3d_path(root, subj, sess, "Static", "01"))
    except Exception as e:
        neutral = {j: 0.0 for j in joints}; print(f"(no static neutral: {e})")

    per_joint = {m: {j: [] for j in joints} for m in results}
    heading = {"6dof": [], "9dof": []}
    step_errors = []
    per_window = []
    ev = ctx["ev"]
    for tr, r in zip(cfg["selection"]["trials"], aligns):
        c3dp = align.c3d_path(root, subj, sess, task, tr)
        ref_ang, rate, c = R.window_reference(c3dp, neutral)
        M, _, _ = R.read_markers(c3dp)
        labels = R.c3d_events(c)        # reference step events (Zeni) — labels
        t_win = (epoch[foot] + r.bin_start_idx/fs) - T0          # optical time of window start
        tmark = t_win + np.arange(ref_ang[joints[0]].shape[0]) / rate
        winrec = {"trial": tr, "t_opt_start_s": t_win, "joints": {}}
        for mode in results:
            for j in joints:
                imu_full = results[mode]["joints"][j]["flexion"]
                imu_on_mark = np.interp(tmark, tg, imu_full)
                rmse, lag, sign, corr = R.best_lag_rmse(imu_on_mark, ref_ang[j], rate)
                per_joint[mode][j].append(rmse)
                if mode == "6dof":
                    winrec["joints"][j] = {"rmse_deg": rmse, "lag_s": lag, "corr": corr,
                                           "ref_rom": float(np.nanmax(ref_ang[j])-np.nanmin(ref_ang[j]))}
        # heading arbiter (pelvis)
        opt_head = optical_pelvis_heading(M)
        for mode in ("6dof", "9dof"):
            if mode not in results:
                continue
            qg_SA = results[mode]["qg"]["SA"]
            h_imu, _, _ = F.heading_deg(qg_SA)
            h_on_mark = np.interp(tmark, tg, h_imu)
            a = h_on_mark - np.nanmean(h_on_mark); b = opt_head - np.nanmean(opt_head)
            n = min(len(a), len(b))
            heading[mode].append(float(np.sqrt(np.nanmean((a[:n]-b[:n])**2))))
        # step-event timing: IMU foot strikes vs Zeni "Right Foot Strike" (labels)
        ref_fs = np.array(labels.get("Right Foot Strike", []))
        tg = ctx["tg"]
        imu_fs = tg[ev["foot_strike"][ev["foot_strike"] < len(tg)]] - t_win  # window-relative
        if len(ref_fs) and len(imu_fs):
            errs = [np.min(np.abs(imu_fs - rf)) for rf in ref_fs]
            step_errors.extend([e for e in errs if e < 0.5])
        per_window.append(winrec)

    agg = {m: {j: float(np.nanmean(per_joint[m][j])) for j in joints} for m in results}
    head_rmse = {m: float(np.nanmean(heading[m])) for m in heading if heading[m]}
    step_err = float(np.mean(step_errors)) if step_errors else float("nan")
    return {"per_joint_rmse_deg": agg, "per_window": per_window,
            "heading_rmse_deg": head_rmse, "neutral_ref_rad": neutral,
            "step_event_timing_error_s": step_err, "n_matched_steps": len(step_errors)}


def load_sensors(cfg):
    """Read each node sensor once; return BIN data, SI, skew, rtc, refined epoch offsets."""
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    nodes = cfg["selection"]["nodes"]; foot = cfg["selection"]["foot_node"]
    fs = cfg["bin_format"]["fs_high"]
    bd, si, skew, rtc = {}, {}, {}, {}
    for node in nodes:
        sensor = cfg["sensor_map"][node]
        b = read_bin(extract.bin_path(root, subj, sess, sensor))
        bd[node] = b; si[node] = extract.to_si(b, cfg)
        rtc[node] = align.bin_rtc_datetime(b.start_datetime)
        skew[node] = align.estimate_session_skew(np.linalg.norm(si[node][1], axis=1),
                                                 fs, rtc[node], root, subj, sess, node)
    epoch = {n: (rtc[n]-rtc[foot]).total_seconds() - skew[n].skew_s for n in nodes}
    return bd, si, skew, rtc, epoch


def calibrate(cfg, node, si, rtc, skew_s):
    """Fit mag calibration from varied-orientation windows in this sensor's timeline."""
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    fs = cfg["bin_format"]["fs_high"]
    acc, gyr, mag = si[node]
    acc_mag = np.linalg.norm(acc, axis=1); gyr_mag = np.linalg.norm(gyr, axis=1)
    wins, names = [], []
    for task, trials in cfg["mag_calibration"]["source_trials"].items():
        for tr in trials:
            try:
                r = align.align_trial(acc_mag, gyr_mag, fs, rtc, root, subj, sess, task, tr, node, skew_s)
                wins.append((r.bin_start_idx, r.bin_end_idx)); names.append(f"{task}_{tr}")
            except Exception:
                pass
    ms, asamp, used = C.gather_orientation_samples(mag.astype(float), acc, fs, fs, wins)
    if len(ms) < cfg["mag_calibration"]["min_samples"]:
        return C.identity_calibration(), names
    return C.fit_mag_calibration(ms, asamp, source_windows=names), names


def neutral_gravity(cfg, node, si, rtc, skew_s):
    """Neutral 'up' (gravity) direction in the sensor frame from the Static trial."""
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    fs = cfg["bin_format"]["fs_high"]
    acc = si[node][0]
    try:
        cap = align.c3d_capture_datetime(align.c3d_path(root, subj, sess, "Static", "01"))
        i = int(round(((cap - rtc).total_seconds() + skew_s) * fs))
        i0, i1 = max(0, i), min(len(acc), i + int(0.4*fs))
        a0 = acc[i0:i1].mean(0) if i1 - i0 >= 20 else acc[:int(0.3*fs)].mean(0)
    except Exception:
        a0 = acc[:int(0.3*fs)].mean(0)
    return a0 / (np.linalg.norm(a0) + 1e-12)


def seg_on_grid(sig, node, epoch, T0, fs, tg):
    """Resample a sensor's (native) signal onto the common grid tg (cols preserved)."""
    t_native = (tg + T0 - epoch[node])
    idx_t = np.arange(len(sig)) / fs
    if sig.ndim == 1:
        return np.interp(t_native, idx_t, sig)
    return np.column_stack([np.interp(t_native, idx_t, sig[:, k]) for k in range(sig.shape[1])])


def orient_segment(cfg, node, si, epoch, T0, T1, calib, mode):
    """Run Madgwick over the walking segment (this sensor's native clock).

    Returns (quaternions, t_opt) on the sensor's own samples within [T0,T1].
    """
    fs = cfg["bin_format"]["fs_high"]
    acc, gyr, mag = si[node]
    i0 = max(0, int(round((T0 - epoch[node]) * fs)))
    i1 = min(len(acc), int(round((T1 - epoch[node]) * fs)))
    a, g = acc[i0:i1], gyr[i0:i1]
    t_opt = (epoch[node] + np.arange(i0, i1)/fs) - T0
    if mode == "9dof":
        m = calib.apply(mag[i0:i1].astype(float))
        q = F.run_madgwick(g, a, m, fs, cfg["fusion"]["beta"])
    else:
        q = F.run_madgwick(g, a, None, fs, cfg["fusion"]["beta_6dof"])
    return q, t_opt


def write_outputs(cfg, ctx):
    """Write per-trial CSV (computed+reference+error, vel, acc, foot accel, steps) and
    a JSON summary (RMSE, ROM, cadence, step count, caveats)."""
    out_dir = cfg["output"]["dir"]; os.makedirs(out_dir, exist_ok=True)
    subj = cfg["dataset"]["subject"]; sess = cfg["dataset"]["session"]; task = cfg["selection"]["task"]
    base = os.path.join(out_dir, f"{subj}_{sess}_{task}")
    fs = cfg["bin_format"]["fs_high"]; joints = list(cfg["selection"]["joints"])
    tg = ctx["tg"]; res = ctx["results"]; ev = ctx["ev"]; mask = ctx["mask"]; val = ctx["val"]

    # foot accel magnitude on grid
    facc = np.linalg.norm(ctx["foot_seg_acc"], axis=1)
    strike = np.zeros(len(tg), int); strike[ev["foot_strike"][ev["foot_strike"] < len(tg)]] = 1

    # reference angle on the grid (NaN outside the 4 mocap windows), per joint (6-DOF aligned)
    refgrid = {j: np.full(len(tg), np.nan) for j in joints}
    # (filled by re-deriving reference on each window timeline -> grid)
    cols = {"t_opt_s": tg}
    for j in joints:
        f6 = res["6dof"]["joints"][j]["flexion"]
        cols[f"{j}_deg"] = f6
        cols[f"{j}_vel_dps"] = res["6dof"]["joints"][j]["ang_vel"]
        cols[f"{j}_acc_dps2"] = res["6dof"]["joints"][j]["ang_acc"]
        if "9dof" in res:
            cols[f"{j}_deg_9dof"] = res["9dof"]["joints"][j]["flexion"]
        cols[f"{j}_ref_deg"] = refgrid[j]
    cols["foot_acc_mag"] = facc
    cols["foot_strike"] = strike
    cols["steady_state"] = mask.astype(int)
    header = ",".join(cols.keys())
    arr = np.column_stack([np.asarray(v, float) for v in cols.values()])
    np.savetxt(base + "_timeseries.csv", arr, delimiter=",", header=header, comments="", fmt="%.6g")

    # validation windows CSV
    with open(base + "_validation.csv", "w") as f:
        f.write("trial,t_opt_start_s,joint,rmse_deg,lag_s,corr,ref_rom_deg\n")
        for w in val["per_window"]:
            for j, d in w["joints"].items():
                f.write(f"{w['trial']},{w['t_opt_start_s']:.3f},{j},{d['rmse_deg']:.3f},"
                        f"{d['lag_s']:.3f},{d['corr']:.3f},{d['ref_rom']:.3f}\n")

    # JSON summary
    summary = {
        "subject": subj, "session": sess, "task": task,
        "walking_segment_s": [ctx["T0"], ctx["T1"]], "duration_s": ctx["T1"]-ctx["T0"],
        "primary_fusion": "6dof",
        "joint_rom_deg": {m: {j: A.rom(res[m]["joints"][j]["flexion"]) for j in joints} for m in res},
        "joint_peak_vel_dps": {j: float(np.max(np.abs(res["6dof"]["joints"][j]["ang_vel"]))) for j in joints},
        "validation_rmse_deg": val["per_joint_rmse_deg"],
        "heading_rmse_vs_optical_deg": val["heading_rmse_deg"],
        "gait": {"cadence_steps_per_min": ctx["cad"]["cadence_steps_per_min"],
                 "stride_time_mean_s": ctx["cad"]["stride_time_mean"],
                 "stride_time_std_s": ctx["cad"]["stride_time_std"],
                 "n_foot_strikes": int(len(ev["foot_strike"])),
                 "n_steady_strides": ctx["cad"]["n_strides"]},
        "turnarounds": [{"t_start_s": float(tg[s]), "t_end_s": float(tg[e]), "deg": a}
                        for s, e, a in ctx["turns"]],
        "intersensor_refine": {n: ctx["refine"][n] for n in cfg["selection"]["nodes"]},
        "magnetometer": {
            "channel": "tag 0x18 @256Hz (NOT 0x15/64Hz which is barometer)",
            "verdict": "9-DOF does NOT improve heading vs 6-DOF (optical-confirmed); indoor field distorted",
        },
        "caveats": [
            "Magnetometer delivered at 256 Hz (tag 0x18), sample-aligned with accel/gyro; "
            "no 64->256 upsampling needed. Dataset doc lists 64 Hz (the barometer, tag 0x15).",
            "Reference joint angles derived from c3d 4-marker clusters + joint centres, "
            "NOT pre-computed; neutral from Static_01.",
            "Hip uses pelvis (SA) which could not be inter-sensor refined below ~0.4 s from "
            "IMU alone (impacts damped, periodic stride-ambiguous); still corr 0.92-0.97.",
            "Validation RMSE is offset-removed (sensor-mounting offset not penalised), "
            "sub-sample aligned within +/-0.3 s.",
            "Magnetometer calibration (hard/soft-iron ellipsoid) fitted from CalibrationTask"
            "+TUG; indoor field distortion limits 9-DOF.",
        ],
    }
    import json
    json.dump(summary, open(base + "_summary.json", "w"), indent=2)
    print(f"\nWrote outputs -> {base}_timeseries.csv / _validation.csv / _summary.json")
    return summary


def compute_core(cfg):
    """IMU-ONLY kinematic core: orientation, joint angles, gait. Reads NO markers/labels.

    Returns ctx (without validation). This function and everything it calls must be a
    pure function of the IMU data — the RAW-DATA CONTRACT selftest depends on it.
    """
    fs = cfg["bin_format"]["fs_high"]
    nodes = cfg["selection"]["nodes"]; foot = cfg["selection"]["foot_node"]
    joints = cfg["selection"]["joints"]

    print("Loading sensors ...")
    bd, si, skew, rtc, epoch = load_sensors(cfg)

    # walking segment + inter-sensor refinement (reuse extract)
    foot_gyr_mag = np.linalg.norm(si[foot][1], axis=1)
    foot_acc_mag = np.linalg.norm(si[foot][0], axis=1)
    aligns = [align.align_trial(foot_acc_mag, foot_gyr_mag, fs, rtc[foot],
                                cfg["dataset"]["root"], cfg["dataset"]["subject"],
                                cfg["dataset"]["session"], cfg["selection"]["task"], tr,
                                foot, skew[foot].skew_s) for tr in cfg["selection"]["trials"]]
    anchors = [r.bin_start_idx for r in aligns]
    s0f, s1f, _ = extract.detect_walking_segment(si[foot][1], fs, cfg["segment"], anchors)
    T0 = epoch[foot] + s0f/fs; T1 = epoch[foot] + s1f/fs
    epoch, refine = extract.refine_intersensor(si, epoch, foot, fs, T0, T1)
    print(f"Walking segment [{T0:.1f},{T1:.1f}]s = {T1-T0:.1f}s")
    for n in nodes:
        print(f"  {n}: epoch={epoch[n]:+.2f}s refine corr={refine[n]['corr']:.2f} "
              f"{'applied' if refine[n]['applied'] else 'optical-skew'}")

    # per-sensor calibration + orientation (both modes)
    calib = {}
    grav0 = {}
    quats = {m: {} for m in cfg["fusion"]["run_modes"]}
    topt = {m: {} for m in cfg["fusion"]["run_modes"]}
    for node in nodes:
        calib[node], _ = calibrate(cfg, node, si, rtc, skew[node].skew_s)
        grav0[node] = neutral_gravity(cfg, node, si, rtc, skew[node].skew_s)
        for mode in cfg["fusion"]["run_modes"]:
            quats[mode][node], topt[mode][node] = orient_segment(cfg, node, si, epoch, T0, T1, calib[node], mode)

    # common time grid (256 Hz) over the segment
    tg = np.arange(0, T1 - T0, 1.0/fs)
    # each sensor's segment accel/gyro on the common grid
    gyr_grid = {n: seg_on_grid(si[n][1], n, epoch, T0, fs, tg) for n in nodes}
    acc_grid = {n: seg_on_grid(si[n][0], n, epoch, T0, fs, tg) for n in nodes}

    # turnarounds (pelvis) + steady-state mask (computed before angles so joint axes
    # are estimated on straight walking, not on the dominant turn rotation)
    turns, yr = S.detect_turnarounds(acc_grid["SA"], gyr_grid["SA"], fs)
    mask = S.steady_state_mask(len(tg), turns, fs)

    # joint angles (per mode)
    results = {}
    for mode in cfg["fusion"]["run_modes"]:
        qg = {n: A.slerp_resample(quats[mode][n], topt[mode][n], tg) for n in nodes}
        jres = {}
        for jname, (dist, prox) in joints.items():
            ja = A.joint_angles(qg[dist], qg[prox], gyr_grid[dist], gyr_grid[prox],
                                grav0[dist], grav0[prox], axis_mask=mask, fs=fs,
                                tau=cfg["fusion"].get("joint_tau_s", 1.0))
            flex = ja["flexion"]
            jres[jname] = {
                "flexion": flex,
                "ang_vel": A.derivative(flex, fs),
                "ang_acc": A.derivative(A.derivative(flex, fs), fs),
                "rom": A.rom(flex),
                "axes": ja,
            }
        results[mode] = {"t": tg, "qg": qg, "joints": jres}

    # gait on the foot (steady-state only) — fusion-independent (raw foot gyro)
    foot_seg_gyr = gyr_grid[foot]; foot_seg_acc = acc_grid[foot]
    ev = G.detect_events(foot_seg_gyr, fs)
    cad = G.cadence_stats(ev["foot_strike"], fs, mask=mask)

    # ---- console summary ----
    print(f"\n=== Joint angles (6-DOF primary) over {T1-T0:.0f}s walk ===")
    for jname in joints:
        f6 = results["6dof"]["joints"][jname]["flexion"]
        f9 = results["9dof"]["joints"][jname]["flexion"] if "9dof" in results else None
        line = (f"  {jname:5s}: ROM={A.rom(f6):5.1f} deg  range=[{np.min(f6):6.1f},{np.max(f6):6.1f}]  "
                f"|vel|max={np.max(np.abs(results['6dof']['joints'][jname]['ang_vel'])):.0f} deg/s")
        if f9 is not None:
            line += f"   (9-DOF ROM={A.rom(f9):.1f})"
        print(line)
    print(f"\n=== Gait (steady-state, {len(turns)} turnarounds excluded) ===")
    print(f"  foot strikes total={len(ev['foot_strike'])}  steady strides={cad['n_strides']}")
    print(f"  cadence={cad['cadence_steps_per_min']:.1f} steps/min  "
          f"stride={cad['stride_time_mean']:.2f}+/-{cad['stride_time_std']:.2f}s")
    for s, e, ang in turns:
        print(f"    turn t=[{tg[s]:.1f},{tg[e]:.1f}]s {ang:+.0f} deg")

    ctx = dict(T0=T0, T1=T1, tg=tg, results=results, ev=ev, turns=turns,
               mask=mask, cad=cad, calib=calib, epoch=epoch, refine=refine,
               aligns=aligns, foot=foot, foot_seg_acc=foot_seg_acc,
               foot_seg_gyr=foot_seg_gyr, si=si, rtc=rtc, skew=skew, bd=bd)
    return ctx


def main(cfg_path="config/default.yaml"):
    cfg = yaml.safe_load(open(cfg_path))
    joints = list(cfg["selection"]["joints"])
    ctx = compute_core(cfg)

    # ---- validation against optical markers (read ONLY here) ----
    val = validate(cfg, ctx)
    ctx["val"] = val
    print("\n=== Validation vs optical markers (RMSE over 4 windows, sub-sample aligned) ===")
    for j in joints:
        r6 = val["per_joint_rmse_deg"]["6dof"][j]
        r9 = val["per_joint_rmse_deg"].get("9dof", {}).get(j, float("nan"))
        print(f"  {j:5s}: 6-DOF RMSE={r6:5.1f} deg   9-DOF RMSE={r9:5.1f} deg")
    if val["heading_rmse_deg"]:
        h = val["heading_rmse_deg"]
        print(f"  pelvis heading vs optical: 6-DOF={h.get('6dof', float('nan')):.1f} deg  "
              f"9-DOF={h.get('9dof', float('nan')):.1f} deg  "
              f"-> {'9-DOF better' if h.get('9dof',9e9)<h.get('6dof',0) else '6-DOF better (mag does not help)'}")

    write_outputs(cfg, ctx)
    return cfg, ctx


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    args = ap.parse_args()
    main(args.config)
