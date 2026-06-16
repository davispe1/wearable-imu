"""
fusion_check.py — STOP-point deliverable: 6-DOF vs 9-DOF heading on the pelvis.

Pipeline up to fusion only (no joint angles / gait yet):
  1. Fit magnetometer calibration from varied-orientation windows (CalibrationTask,
     TUG) located in the BIN via timestamp+skew alignment.
  2. Apply calibration to the walking-segment magnetometer.
  3. Run Madgwick 6-DOF (accel+gyro) and 9-DOF (accel+gyro+mag) on the SA pelvis,
     from the SAME native input.
  4. Detect 180-degree turnarounds (pelvis vertical-axis angular rate).
  5. Quantify yaw drift: per-pass heading, drift slope during straight passes, and
     pass-to-pass consistency (alternating passes should be ~180 deg apart in truth).

The question this answers: does the magnetometer actually improve heading here, or does
the indoor force-plate lab distort the field enough that 9-DOF is no better (or worse)?
"""
from __future__ import annotations

import json
import os
import sys
import numpy as np
import yaml

# repo root on path so the moved pipeline can import the gaitlib library
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bin_reader import read_bin
import align
from gaitlib import fusion as F
from gaitlib import calibration as C


def lowpass_grav(acc, fs, fc=0.4):
    """Crude gravity estimate: moving average ~ 1/fc seconds."""
    w = max(1, int(fs / fc))
    k = np.ones(w) / w
    return np.column_stack([np.convolve(acc[:, i], k, mode="same") for i in range(3)])


def vertical_yaw_rate(acc, gyr, fs):
    """Angular rate about the gravity (vertical) axis, in sensor frame (rad/s)."""
    grav = lowpass_grav(acc, fs)
    gu = grav / (np.linalg.norm(grav, axis=1, keepdims=True) + 1e-12)
    return np.einsum("ij,ij->i", gyr, gu)


def detect_turnarounds(acc, gyr, fs, min_turn_deg=120.0, smooth_s=0.5):
    """Detect 180-deg turnarounds from pelvis vertical-axis angular rate.

    Returns list of (start_idx, end_idx, signed_deg) for sustained same-sign rotations
    exceeding min_turn_deg.
    """
    yr = vertical_yaw_rate(acc, gyr, fs)
    w = max(1, int(smooth_s * fs))
    yr_s = np.convolve(yr, np.ones(w) / w, mode="same")
    active = np.abs(yr_s) > np.radians(50.0)   # turning faster than 50 deg/s
    turns = []
    i, n = 0, len(yr_s)
    while i < n:
        if active[i]:
            s = i
            sign = np.sign(yr_s[i])
            while i < n and active[i] and np.sign(yr_s[i]) == sign:
                i += 1
            ang = np.degrees(np.trapezoid(yr_s[s:i], dx=1.0/fs))
            if abs(ang) >= min_turn_deg:
                turns.append((s, i, float(ang)))
        else:
            i += 1
    return turns, yr_s


def locate_windows(cfg, node, acc_mag, gyr_mag, rtc, skew_s):
    """Calibration source windows (256 Hz idx) in THIS node's own timeline."""
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    fs_high = cfg["bin_format"]["fs_high"]
    wins = []
    for task, trials in cfg["mag_calibration"]["source_trials"].items():
        for trial in trials:
            try:
                res = align.align_trial(acc_mag, gyr_mag, fs_high, rtc,
                                        root, subj, sess, task, trial, node, skew_s)
            except Exception:
                continue
            wins.append((task, trial, res.bin_start_idx, res.bin_end_idx, res.corr))
    return wins


def main(cfg_path="config/default.yaml", node="SA"):
    cfg = yaml.safe_load(open(cfg_path))
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    fs_high = cfg["bin_format"]["fs_high"]; fs_mag = cfg["bin_format"]["fs_mag"]
    bf = cfg["bin_format"]
    sensor = cfg["sensor_map"][node]

    # --- read BIN for this node (self-consistent: own clock/skew throughout) ---
    print(f"Reading BIN for {node} (sensor {sensor}) ...")
    bd = read_bin(os.path.join(root, f"{subj}_{sess}", "RAW_DATA",
                               f"{subj}_{sess}_{sensor}_Inertial_sensor.BIN"))
    acc_si = bd.acc_raw.astype(float) * (bf["gravity"] / bf["acc_counts_per_g"])
    rtc = align.bin_rtc_datetime(bd.start_datetime)
    acc_mag = np.linalg.norm(acc_si, axis=1)
    gyr_si_full = bd.gyr_raw.astype(float) * (np.pi/180.0 / bf["gyr_counts_per_dps"])
    gyr_mag = np.linalg.norm(gyr_si_full, axis=1)

    skew = align.estimate_session_skew(gyr_mag, fs_high, rtc, root, subj, sess, node)
    conf = [d for d in skew.detail if d[3] >= 0.6]
    print(f"{node} skew = {skew.skew_s:+.2f}s (median of {skew.n_used} confident); "
          + ", ".join(f"{d[0]}:{d[4]:+.1f}({d[3]:.2f})" for d in conf))

    # --- magnetometer calibration from varied-orientation windows (own timeline) ---
    wins = locate_windows(cfg, node, acc_mag, gyr_mag, rtc, skew.skew_s)
    win_256 = [(w[2], w[3]) for w in wins]
    mag_s, acc_s, used = C.gather_orientation_samples(
        bd.mag_raw.astype(float), acc_si, fs_high, fs_high, win_256)
    print(f"\nCalibration windows ({node}): {len(used)} used, {len(mag_s)} mag samples")
    for w in wins:
        print(f"  {w[0]}_{w[1]}: idx[{w[2]},{w[3]}] refine_corr={w[4]:.3f}")
    calib = C.fit_mag_calibration(mag_s, acc_s,
                                  source_windows=[f"{w[0]}_{w[1]}" for w in wins])
    print("\n--- Magnetometer calibration (%s) ---" % node)
    print(f"  hard-iron offset b (raw): {np.round(calib.b,1)}")
    print(f"  soft-iron A diag: {np.round(np.diag(calib.A),4)} (off-diag max "
          f"{np.max(np.abs(calib.A-np.diag(np.diag(calib.A)))):.4f})")
    print(f"  frame permutation P:\n{calib.P.astype(int)}")
    print(f"  sphere residual: {calib.sphere_residual:.4f} (0=perfect ellipsoid fit)")
    print(f"  magnetic dip: {calib.dip_mean_deg:.1f} +/- {calib.dip_std_deg:.1f} deg "
          f"(low std => mag & accel share a frame and field is clean)")

    # --- load walking-segment slice, apply calibration to mag ---
    seg_dir = os.path.join(cfg["output"]["data_dir"], f"{subj}_{sess}_{cfg['selection']['task']}")
    arr = np.loadtxt(os.path.join(seg_dir, f"{node}.csv"), delimiter=",", skiprows=1)
    t = arr[:, 1]; acc = arr[:, 2:5]; gyr = arr[:, 5:8]; mag_raw_counts = arr[:, 8:11]
    # slice mag is raw counts (tag 0x18, 256 Hz); calibration normalises it.
    mag_cal = calib.apply(mag_raw_counts)

    report = json.load(open(os.path.join(seg_dir, "extract_report.json")))
    passes = report["optical_windows"]
    # map each optical window to slice index range via the common t_opt axis
    for p in passes:
        p["a"] = int(np.searchsorted(t, p["t_opt_start_s"]))
        p["b"] = int(np.searchsorted(t, p["t_opt_end_s"]))

    # --- run both fusions from the SAME native input ---
    print("\nRunning Madgwick 6-DOF and 9-DOF on %s ..." % node)
    q6 = F.run_madgwick(gyr, acc, mag=None, fs=fs_high, beta=cfg["fusion"]["beta_6dof"])
    q9 = F.run_madgwick(gyr, acc, mag=mag_cal, fs=fs_high, beta=cfg["fusion"]["beta"])
    # gimbal-lock-free heading using a common horizontal body axis (chosen from 6-DOF)
    h6, body_axis, Rs6 = F.heading_deg(q6)
    h9, _, Rs9 = F.heading_deg(q9, body_axis=body_axis)
    h6 = h6 - h6[0]; h9 = h9 - h9[0]
    # validity: tilt (gravity tracking) must agree between 6/9-DOF; mag only moves heading
    tilt6 = F.tilt_deg(q6, Rs6); tilt9 = F.tilt_deg(q9, Rs9)
    tilt_rms = float(np.sqrt(np.mean((tilt9 - tilt6) ** 2)))
    print(f"\nValidity check: tilt RMS diff 9-vs-6 = {tilt_rms:.2f} deg "
          f"(should be small — mag must not change gravity tracking); heading body axis={body_axis}")

    # --- turnarounds ---
    turns, yr_s = detect_turnarounds(acc, gyr, fs_high)
    print(f"\nTurnarounds detected (pelvis vertical-axis): {len(turns)}")
    for s, e, ang in turns:
        print(f"  t=[{t[s]:.1f},{t[e]:.1f}]s  rotation={ang:+.0f} deg")

    # --- magnetic field quality over the walking segment (the crux) ---
    Bmag = np.linalg.norm(mag_raw_counts, axis=1)
    print(f"\nField magnitude over the walk: |B| = {Bmag.mean():.0f} +/- {Bmag.std():.0f} counts "
          f"({100*Bmag.std()/Bmag.mean():.1f}%); range [{Bmag.min():.0f},{Bmag.max():.0f}]")
    print("  (a clean undistorted field would be near-constant; large swings = indoor distortion)")

    # --- straight-segment heading drift (true heading constant between turns) ---
    turn_mask = np.zeros(len(t), bool)
    for s, e, _ in turns:
        turn_mask[max(0, s-int(0.5*fs_high)):min(len(t), e+int(0.5*fs_high))] = True
    straights = []
    j = 0
    while j < len(t):
        if not turn_mask[j]:
            s = j
            while j < len(t) and not turn_mask[j]:
                j += 1
            if (j - s) > int(1.5*fs_high):     # >1.5 s straight
                straights.append((s, j))
        else:
            j += 1

    def straight_drift(h):
        rates = []
        for s, e in straights:
            rates.append(abs(np.polyfit(t[s:e], h[s:e], 1)[0]))
        return float(np.mean(rates)) if rates else float("nan"), rates
    d6, r6 = straight_drift(h6)
    d9, r9 = straight_drift(h9)
    print(f"\nStraight-segment heading drift (true heading constant; |slope| deg/s):")
    print(f"  {len(straights)} straight segments")
    print(f"  6-DOF mean |drift| = {d6:.2f} deg/s   per-seg: {[round(float(x),1) for x in r6]}")
    print(f"  9-DOF mean |drift| = {d9:.2f} deg/s   per-seg: {[round(float(x),1) for x in r9]}")
    verdict = ("9-DOF MORE stable (mag helps)" if d9 < d6 else
               "9-DOF LESS stable (mag hurts — likely field distortion)")
    print(f"  => {verdict}")

    # --- per-pass mean heading & drift slope ---
    def pass_stats(h):
        out = []
        for p in passes:
            a, b = p["a"], min(p["b"], len(h)-1)
            if b <= a:
                continue
            seg = h[a:b]; tt = t[a:b]
            slope = np.polyfit(tt, seg, 1)[0]      # deg/s drift within straight pass
            out.append((p["trial"], float(np.mean(seg)), float(slope), a, b))
        return out
    ps6, ps9 = pass_stats(h6), pass_stats(h9)

    print("\nPer-pass mean heading (deg, relative) and in-pass drift slope (deg/s):")
    print("  pass |   6-DOF mean  slope |   9-DOF mean  slope")
    for (t6, m6, s6, *_), (t9, m9, s9, *_) in zip(ps6, ps9):
        print(f"   {t6}  | {m6:9.1f} {s6:7.2f} | {m9:9.1f} {s9:7.2f}")

    # The subject turns the SAME way each lap, so true heading accumulates monotonically
    # (~constant increment between consecutive captured passes). Both fusions should show
    # a similar, steadily increasing per-pass heading; large irregularity = heading error.
    def increments(ps):
        means = [m for _, m, *_ in ps]
        return [means[i+1]-means[i] for i in range(len(means)-1)]
    c6, c9 = increments(ps6), increments(ps9)
    print("\nPer-pass heading increments (truth: steady, same sign — monotonic turning):")
    print(f"  6-DOF: {[round(x) for x in c6]}")
    print(f"  9-DOF: {[round(x) for x in c9]}")

    # overall drift proxy: mean in-pass heading slope across straight passes
    drift6 = float(np.mean([p[2] for p in ps6]))
    drift9 = float(np.mean([p[2] for p in ps9]))
    print(f"\nMean in-pass drift: 6-DOF {drift6:+.2f} deg/s ; 9-DOF {drift9:+.2f} deg/s")

    out = {
        "node": node,
        "mag_calibration": calib.to_dict(),
        "field_quality": {
            "B_mean_counts": float(Bmag.mean()), "B_std_counts": float(Bmag.std()),
            "B_pct": float(100*Bmag.std()/Bmag.mean()),
            "B_min": float(Bmag.min()), "B_max": float(Bmag.max())},
        "straight_drift": {"n_segments": len(straights),
                           "drift_6dof_dps": d6, "drift_9dof_dps": d9,
                           "verdict": verdict},
        "turnarounds": [{"t_start_s": float(t[s]), "t_end_s": float(t[e]), "deg": ang}
                        for s, e, ang in turns],
        "per_pass": {"6dof": [{"trial": x[0], "mean_deg": x[1], "slope_dps": x[2]} for x in ps6],
                     "9dof": [{"trial": x[0], "mean_deg": x[1], "slope_dps": x[2]} for x in ps9]},
        "per_pass_increments": {"6dof": c6, "9dof": c9},
        "tilt_rms_diff_deg": tilt_rms,
    }
    os.makedirs(cfg["output"]["dir"], exist_ok=True)
    op = os.path.join(cfg["output"]["dir"], f"fusion_check_{node}.json")
    json.dump(out, open(op, "w"), indent=2)
    print(f"\nWrote {op}")

    # optional plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax[0].plot(t, h6, label="6-DOF (no mag)", color="tab:red")
        ax[0].plot(t, h9, label="9-DOF (mag)", color="tab:blue")
        for p in passes:
            ax[0].axvspan(t[p["a"]], t[min(p["b"], len(t)-1)],
                          color="green", alpha=0.15)
        for s, e, ang in turns:
            ax[0].axvspan(t[s], t[e], color="orange", alpha=0.2)
        ax[0].set_ylabel("heading (deg, rel)"); ax[0].legend(); ax[0].set_title(
            f"{subj} {cfg['selection']['task']} {node}: 6-DOF vs 9-DOF heading "
            f"(green=mocap pass, orange=turn)")
        ax[1].plot(t, np.degrees(yr_s), color="purple")
        ax[1].axhline(0, color="k", lw=0.5); ax[1].set_ylabel("vert. yaw rate (deg/s)")
        ax[1].set_xlabel("time (s)")
        fig.tight_layout()
        pngp = os.path.join(cfg["output"]["dir"], f"fusion_check_{node}.png")
        fig.savefig(pngp, dpi=110)
        print(f"Wrote {pngp}")
    except Exception as e:
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    ap.add_argument("--node", default="SA")
    args = ap.parse_args()
    main(args.config, args.node)
