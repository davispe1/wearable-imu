"""
extract.py — Stage 1. Reads the chosen trial's BIN sensors (READ-ONLY), converts to
SI, upsamples the magnetometer 64->256 Hz on each sensor's own BIN clock, delimits the
walking segment, and writes small per-sensor CSV slices under ``data/``.

MULTI-SENSOR TIMING (important)
-------------------------------
The 8 IMUs are SEPARATE BIN files with DIFFERENT RTC start times (spanning ~27 s) and
slightly different sample counts — they are NOT sample-aligned to each other. Each
sensor is therefore aligned INDEPENDENTLY to the optical clock via its own sync_data +
RTC + a constant clock skew (estimated from long content-rich trials). We then express
every sensor on a common **optical-time axis** so downstream stages can compute
cross-sensor joint angles.

CONTRACT NOTES
--------------
* Fusion runs per-sensor on each sensor's BIN-native channels (accel 0x13, gyro 0x14,
  mag 0x15 upsampled on the BIN clock). The common optical-time column is used only to
  align ORIENTATION outputs across sensors / to the markers — never to resample the IMU
  before fusion.
* sync_data is used (inside align.py only) to calibrate per-sensor clock skew. It never
  enters the slices or fusion.
"""
from __future__ import annotations

import argparse
import json
import os
import numpy as np
import yaml

from bin_reader import read_bin
import align


def bin_path(root, subject, session, sensor):
    return os.path.join(root, f"{subject}_{session}", "RAW_DATA",
                        f"{subject}_{session}_{sensor}_Inertial_sensor.BIN")


def to_si(bd, cfg):
    """accel -> m/s^2, gyro -> rad/s, mag -> raw counts (tag 0x18, 256 Hz).

    The magnetometer is left in raw counts: its absolute scale is uncertain and the
    downstream ellipsoid calibration normalises it. All three streams are 256 Hz and
    sample-aligned on the BIN clock (mag tag 0x18), so no upsampling is needed.
    """
    bf = cfg["bin_format"]
    acc = bd.acc_raw.astype(np.float64) * (bf["gravity"] / bf["acc_counts_per_g"])
    gyr = bd.gyr_raw.astype(np.float64) * (np.pi / 180.0 / bf["gyr_counts_per_dps"])
    mag = bd.mag_raw.astype(np.float64)   # raw counts, 256 Hz
    return acc, gyr, mag


def moving_rms(x, win):
    win = max(1, int(win))
    return np.sqrt(np.convolve(x * x, np.ones(win) / win, mode="same"))


def detect_walking_segment(foot_gyr, fs_high, seg_cfg, anchor_idxs):
    g = np.linalg.norm(foot_gyr, axis=1)
    rms_deg = np.degrees(moving_rms(g, seg_cfg["smooth_s"] * fs_high))
    mask = rms_deg > seg_cfg["gyro_rms_threshold_dps"]
    bridge = int(seg_cfg["min_gap_s"] * fs_high)
    m = mask.copy()
    false_start = None
    for j in range(len(m)):
        if not m[j]:
            if false_start is None:
                false_start = j
        else:
            if false_start is not None and (j - false_start) <= bridge and false_start > 0:
                m[false_start:j] = True
            false_start = None
    runs = []
    j = 0
    while j < len(m):
        if m[j]:
            s = j
            while j < len(m) and m[j]:
                j += 1
            runs.append((s, j))
        else:
            j += 1
    covering = [r for r in runs if any(r[0] <= a < r[1] for a in anchor_idxs)]
    if not covering:
        covering = [max(runs, key=lambda r: r[1] - r[0])] if runs else [(anchor_idxs[0], anchor_idxs[-1] + 1)]
    s0 = min(r[0] for r in covering)
    s1 = max(r[1] for r in covering)
    pad = int(seg_cfg["pad_s"] * fs_high)
    s0 = max(0, s0 - pad)
    s1 = min(len(m), s1 + pad)
    cap = int(seg_cfg["max_segment_s"] * fs_high)
    if s1 - s0 > cap:
        s1 = s0 + cap
    return int(s0), int(s1), runs


def refine_intersensor(si, epoch, foot, fs_high, T0, T1, min_corr=0.45):
    """Refine each non-foot sensor's optical-time epoch against the foot using shared
    high-frequency accel impacts (heel strikes propagate to all sensors).

    Returns (refined_epoch, detail). Only confident refinements (corr>=min_corr) are
    applied; others keep the optical-skew epoch (e.g. the pelvis, where impacts are
    damped and the periodic walking signal is stride-ambiguous). IMU-only — markers
    are never used (that would break the raw-data contract / selftest).
    """
    from scipy.signal import butter, filtfilt
    from align import ncc_valid
    bhf, ahf = butter(4, 8/(fs_high/2), "high")

    def impacts(acc, e):
        i0 = max(0, int(round((T0 - e) * fs_high)))
        i1 = int(round((T1 - e) * fs_high))
        i1 = min(len(acc), i1)
        x = np.linalg.norm(acc[i0:i1], axis=1)
        t = e + np.arange(i0, i1) / fs_high
        return t, np.abs(filtfilt(bhf, ahf, x))

    tf, impf = impacts(si[foot][0], epoch[foot])
    refined = dict(epoch)
    detail = {}
    half = 0.6
    W = int(half * fs_high)
    for node in si:
        if node == foot:
            detail[node] = {"lag_ms": 0.0, "corr": 1.0, "applied": True}
            continue
        tn, impn = impacts(si[node][0], epoch[node])
        sig = np.interp(tf, tn, impn)
        r = ncc_valid(np.concatenate([np.zeros(W), sig, np.zeros(W)]), impf)
        k = int(np.argmax(r)); lag = (k - W) / fs_high; corr = float(r[k])
        applied = corr >= min_corr
        if applied:
            refined[node] = epoch[node] - lag    # shift so impacts coincide with foot
        detail[node] = {"lag_ms": lag*1000, "corr": corr, "applied": applied}
    return refined, detail


def run_extract(cfg_path="config/default.yaml"):
    cfg = yaml.safe_load(open(cfg_path))
    ds = cfg["dataset"]; root, subj, sess = ds["root"], ds["subject"], ds["session"]
    sel = cfg["selection"]; nodes = sel["nodes"]; task = sel["task"]
    fs_high = cfg["bin_format"]["fs_high"]; fs_mag = cfg["bin_format"]["fs_mag"]
    foot = sel["foot_node"]

    # 1) decode all node sensors + per-sensor SI + per-sensor optical skew
    print("Decoding BIN sensors and aligning each to the optical clock ...")
    bdata, si, skew, rtc = {}, {}, {}, {}
    for node in nodes:
        sensor = cfg["sensor_map"][node]
        bd = read_bin(bin_path(root, subj, sess, sensor))
        bdata[node] = bd
        si[node] = to_si(bd, cfg)
        rtc[node] = align.bin_rtc_datetime(bd.start_datetime)
        gmag = np.linalg.norm(si[node][1], axis=1)
        sk = align.estimate_session_skew(gmag, fs_high, rtc[node], root, subj, sess, node)
        skew[node] = sk
        print(f"  {node}({sensor}): rtc={bd.start_datetime} skew={sk.skew_s:+.2f}s "
              f"acc{bd.acc_raw.shape} mag{bd.mag_raw.shape} invalid={100*bd.invalid_records/bd.total_records:.3f}%")

    # common optical-time epoch (origin = foot sensor's RTC). For sensor X:
    #   epoch_offset_X = (rtc_X - rtc_foot) - skew_X ;  T_opt_X(i) = epoch_offset_X + i/fs
    epoch = {node: (rtc[node] - rtc[foot]).total_seconds() - skew[node].skew_s for node in nodes}

    # 2) locate optical windows in the FOOT sensor's timeline (foot drives segmentation)
    foot_acc_mag = np.linalg.norm(si[foot][0], axis=1)
    foot_gyr_mag = np.linalg.norm(si[foot][1], axis=1)
    aligns = []
    for trial in align.available_trials(root, subj, sess, task, sel["trials"]):
        res = align.align_trial(foot_acc_mag, foot_gyr_mag, fs_high, rtc[foot],
                                root, subj, sess, task, trial, foot, skew[foot].skew_s)
        aligns.append((trial, res))

    # 3) delimit the walking segment from foot activity (foot native indices)
    anchors = [r.bin_start_idx for _, r in aligns]
    s0f, s1f, _ = detect_walking_segment(si[foot][1], fs_high, cfg["segment"], anchors)
    # optical-time window of the segment (common axis, relative to segment start)
    T0 = epoch[foot] + s0f / fs_high
    T1 = epoch[foot] + s1f / fs_high
    dur = T1 - T0
    print(f"\nWalking segment (optical time): [{T0:.2f},{T1:.2f}]s = {dur:.1f}s, "
          f"covering {len(anchors)} optical anchors")

    # 3b) refine inter-sensor timing against the foot via shared heel-strike impacts
    epoch, refine_detail = refine_intersensor(si, epoch, foot, fs_high, T0, T1)
    print("Inter-sensor impact refinement (vs foot):")
    for node in nodes:
        d = refine_detail[node]
        flag = "applied" if d["applied"] else "KEPT optical-skew (impacts too weak)"
        print(f"  {node}: lag={d['lag_ms']:+.0f}ms corr={d['corr']:.3f} -> {flag}")

    # 4) common 256 Hz optical grid; write per-sensor slices on each sensor's NATIVE
    #    samples covering the window, carrying a common t_opt column for downstream.
    out_dir = os.path.join(cfg["output"]["data_dir"], f"{subj}_{sess}_{task}")
    os.makedirs(out_dir, exist_ok=True)
    sensor_slice_info = {}
    for node in nodes:
        acc, gyr, mag = si[node]      # all 256 Hz, sample-aligned (mag tag 0x18)
        # native index range covering [T0,T1] for this sensor
        i0 = int(round((T0 - epoch[node]) * fs_high))
        i1 = int(round((T1 - epoch[node]) * fs_high))
        i0 = max(0, i0); i1 = min(len(acc), i1)
        idx = np.arange(i0, i1)
        t_native = idx / fs_high
        t_opt = (epoch[node] + idx / fs_high) - T0    # common axis, 0 at segment start
        arr = np.column_stack([t_native, t_opt, acc[i0:i1], gyr[i0:i1], mag[i0:i1]])
        header = "t_native_s,t_opt_s,ax,ay,az,gx,gy,gz,mx,my,mz"
        np.savetxt(os.path.join(out_dir, f"{node}.csv"), arr, delimiter=",",
                   header=header, comments="", fmt="%.7g")
        sensor_slice_info[node] = {"i0": i0, "i1": i1, "n": int(i1 - i0),
                                   "epoch_offset_s": epoch[node]}
    print(f"Wrote {len(nodes)} per-sensor slices -> {out_dir}")

    # 5) report
    report = {
        "subject": subj, "session": sess, "task": task,
        "fs_high": fs_high, "fs_mag": fs_mag,
        "common_time": "t_opt_s column: optical clock, 0 at segment start, shared by all sensors",
        "sensors": {node: {
            "sensor": cfg["sensor_map"][node],
            "rtc": bdata[node].start_datetime,
            "skew_s": skew[node].skew_s,
            "skew_confident": [{"trial": d[0], "corr": d[3], "skew_s": d[4]}
                               for d in skew[node].detail if d[3] >= 0.6],
            "epoch_offset_s": epoch[node],
            "intersensor_refine": refine_detail[node],
            "slice": sensor_slice_info[node],
            "invalid_pct": 100*bdata[node].invalid_records/bdata[node].total_records,
            "acc_mag_ratio": bdata[node].meta["acc_mag_ratio"],
        } for node in nodes},
        "walking_segment": {
            "t0_opt_s": T0, "t1_opt_s": T1, "duration_s": dur,
            "foot_native_idx": [s0f, s1f],
            "delimiting": {"method": "foot-IMU gyro RMS threshold + gap-bridge + pad",
                           "activity_node": foot,
                           "threshold_dps": cfg["segment"]["gyro_rms_threshold_dps"],
                           "smooth_s": cfg["segment"]["smooth_s"],
                           "min_gap_s": cfg["segment"]["min_gap_s"],
                           "pad_s": cfg["segment"]["pad_s"]},
        },
        "optical_windows": [
            {"trial": t,
             "foot_bin_idx": [r.bin_start_idx, r.bin_end_idx],
             "t_opt_start_s": (epoch[foot] + r.bin_start_idx/fs_high) - T0,
             "t_opt_end_s": (epoch[foot] + r.bin_end_idx/fs_high) - T0,
             "n_sync": r.n_sync, "refine_corr": r.corr}
            for t, r in aligns],
        "slice_columns": "t_native_s,t_opt_s,ax..az(m/s^2),gx..gz(rad/s),mx..mz(raw counts,256Hz tag0x18,uncalibrated)",
        "mag_calibration": "NOT applied here; fitted/applied in gaitlib",
        "out_dir": out_dir,
    }
    json.dump(report, open(os.path.join(out_dir, "extract_report.json"), "w"), indent=2)
    print(f"Wrote report -> {os.path.join(out_dir, 'extract_report.json')}")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    args = ap.parse_args()
    run_extract(args.config)
