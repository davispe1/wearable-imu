"""
extract.py — Stage 1 of the pipeline. Reads the chosen trial's BIN sensors
(READ-ONLY), converts to SI, upsamples the magnetometer 64->256 Hz on the BIN's
own sample clock, delimits the walking segment, and writes small per-sensor CSV
slices under ``data/``. Nothing downstream ever touches the original dataset.

CONTRACT NOTES
--------------
* Only BIN-native channels (accel 0x13, gyro 0x14, mag 0x15) are written. The 3rd
  256 Hz channel (0x18) and ``sync_data`` are never written into the slices.
* The magnetometer is upsampled onto the accelerometer/gyro 256 Hz grid by linear
  interpolation in the *shared device-clock time base* (acc sample i -> i/256 s,
  mag sample j -> j/64 s). No cross-source timing is used.
* ``sync_data`` is used (inside align.py only) to calibrate the clock skew and to
  locate the optical windows; it never enters the slices.
"""
from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict
import numpy as np
import yaml

from bin_reader import read_bin
import align


# --------------------------------------------------------------------------- #
def bin_path(root, subject, session, sensor):
    return os.path.join(root, f"{subject}_{session}", "RAW_DATA",
                        f"{subject}_{session}_{sensor}_Inertial_sensor.BIN")


def to_si(bd, cfg):
    """Convert raw int16 counts to SI. Returns dict of arrays on native rates."""
    bf = cfg["bin_format"]
    acc = bd.acc_raw.astype(np.float64) * (bf["gravity"] / bf["acc_counts_per_g"])  # m/s^2
    gyr = bd.gyr_raw.astype(np.float64) * (np.pi / 180.0 / bf["gyr_counts_per_dps"])  # rad/s
    mag = bd.mag_raw.astype(np.float64) * (bf["mag_range_gauss"] / 32768.0)          # Gauss (provisional)
    return acc, gyr, mag


def upsample_mag_to_high(mag64, fs_mag, n_high, fs_high):
    """Linear-interpolate 64 Hz magnetometer onto the 256 Hz accel/gyro grid.

    Shared device clock: mag sample j is at t=j/fs_mag, target sample i at t=i/fs_high.
    """
    t_mag = np.arange(len(mag64)) / fs_mag
    t_high = np.arange(n_high) / fs_high
    out = np.empty((n_high, 3))
    for k in range(3):
        out[:, k] = np.interp(t_high, t_mag, mag64[:, k])
    return out


def moving_rms(x, win):
    win = max(1, int(win))
    k = np.ones(win) / win
    return np.sqrt(np.convolve(x * x, k, mode="same"))


def detect_walking_segment(foot_gyr, fs_high, seg_cfg, anchor_idxs):
    """Delimit the continuous walking bout around the optical anchors from foot-IMU
    activity. Threshold foot gyro RMS, bridge short sub-threshold gaps (stance),
    pad, and return the contiguous run covering the anchors.
    """
    g = np.linalg.norm(foot_gyr, axis=1)                  # rad/s
    rms_deg = np.degrees(moving_rms(g, seg_cfg["smooth_s"] * fs_high))
    mask = rms_deg > seg_cfg["gyro_rms_threshold_dps"]

    # bridge gaps shorter than min_gap_s
    bridge = int(seg_cfg["min_gap_s"] * fs_high)
    m = mask.copy()
    i = 0
    n = len(m)
    # fill False runs shorter than `bridge` that are flanked by True
    false_start = None
    for j in range(n):
        if not m[j]:
            if false_start is None:
                false_start = j
        else:
            if false_start is not None and (j - false_start) <= bridge and false_start > 0:
                m[false_start:j] = True
            false_start = None

    # find contiguous True runs, keep those covering >=1 anchor, merge their span
    runs = []
    j = 0
    while j < n:
        if m[j]:
            s = j
            while j < n and m[j]:
                j += 1
            runs.append((s, j))
        else:
            j += 1
    covering = [r for r in runs if any(r[0] <= a < r[1] for a in anchor_idxs)]
    if not covering:
        # fall back: the longest run
        covering = [max(runs, key=lambda r: r[1] - r[0])] if runs else [(anchor_idxs[0], anchor_idxs[-1] + 1)]
    s0 = min(r[0] for r in covering)
    s1 = max(r[1] for r in covering)
    pad = int(seg_cfg["pad_s"] * fs_high)
    s0 = max(0, s0 - pad)
    s1 = min(n, s1 + pad)
    # safety cap
    cap = int(seg_cfg["max_segment_s"] * fs_high)
    if s1 - s0 > cap:
        s1 = s0 + cap
    return int(s0), int(s1), runs


# --------------------------------------------------------------------------- #
def run_extract(cfg_path="config/default.yaml"):
    cfg = yaml.safe_load(open(cfg_path))
    ds = cfg["dataset"]
    root, subj, sess = ds["root"], ds["subject"], ds["session"]
    sel = cfg["selection"]
    nodes = sel["nodes"]
    task = sel["task"]
    fs_high = cfg["bin_format"]["fs_high"]

    # 1) decode all node sensors
    print("Decoding BIN sensors ...")
    bdata, si = {}, {}
    for node in nodes:
        sensor = cfg["sensor_map"][node]
        bd = read_bin(bin_path(root, subj, sess, sensor))
        bdata[node] = bd
        si[node] = to_si(bd, cfg)
        print(f"  {node}({sensor}): acc{bd.acc_raw.shape} mag{bd.mag_raw.shape} "
              f"invalid={100*bd.invalid_records/bd.total_records:.3f}% dur={bd.duration_s/60:.1f}min")

    rtc = align.bin_rtc_datetime(bdata[nodes[0]].start_datetime)

    # 2) calibrate session clock skew (sync_data used only inside align)
    ref_node = sel["ref_align_node"]
    ref_acc_mag = np.linalg.norm(si[ref_node][0], axis=1)
    ref_gyr_mag = np.linalg.norm(si[ref_node][1], axis=1)
    skew = align.estimate_session_skew(ref_gyr_mag, fs_high, rtc, root, subj, sess, ref_node)
    print(f"\nSession clock skew = {skew.skew_s:+.2f} s ({skew.n_used} confident trials)")

    # 3) locate the optical windows
    aligns = []
    for trial in sel["trials"]:
        res = align.align_trial(ref_acc_mag, ref_gyr_mag, fs_high, rtc,
                                root, subj, sess, task, trial, ref_node, skew.skew_s)
        aligns.append((trial, res))
        print(f"  {task}_{trial}: idx={res.bin_start_idx} t=[{res.t_start_s:.2f},{res.t_end_s:.2f}]s "
              f"refine_corr={res.corr:.3f}")

    # 4) delimit the walking segment from foot-IMU activity
    foot = sel["foot_node"]
    anchors = [r.bin_start_idx for _, r in aligns]
    s0, s1, runs = detect_walking_segment(si[foot][1], fs_high, cfg["segment"], anchors)
    print(f"\nWalking segment: idx[{s0},{s1}] t=[{s0/fs_high:.2f},{s1/fs_high:.2f}]s "
          f"= {(s1-s0)/fs_high:.1f}s, covering {len(anchors)} optical anchors")

    # 5) write SI slices (segment only), mag upsampled on the BIN grid
    out_dir = os.path.join(cfg["output"]["data_dir"], f"{subj}_{sess}_{task}")
    os.makedirs(out_dir, exist_ok=True)
    n_high = s1 - s0
    t_seg = np.arange(n_high) / fs_high
    for node in nodes:
        acc, gyr, mag64 = si[node]
        mag_hi = upsample_mag_to_high(mag64, cfg["bin_format"]["fs_mag"], len(acc), fs_high)
        sl = slice(s0, s1)
        arr = np.column_stack([
            t_seg,
            acc[sl], gyr[sl], mag_hi[sl],
        ])
        header = "t_s,ax,ay,az,gx,gy,gz,mx,my,mz"
        path = os.path.join(out_dir, f"{node}.csv")
        np.savetxt(path, arr, delimiter=",", header=header, comments="",
                   fmt="%.7g")
    print(f"Wrote {len(nodes)} per-sensor slices -> {out_dir}")

    # 6) extract report (optical windows expressed relative to segment start too)
    report = {
        "subject": subj, "session": sess, "task": task,
        "bin_rtc_start": bdata[nodes[0]].start_datetime,
        "fs_high": fs_high, "fs_mag": cfg["bin_format"]["fs_mag"],
        "decode": {node: {"acc_n": int(bdata[node].acc_raw.shape[0]),
                          "mag_n": int(bdata[node].mag_raw.shape[0]),
                          "invalid_pct": 100*bdata[node].invalid_records/bdata[node].total_records,
                          "acc_mag_ratio": bdata[node].meta["acc_mag_ratio"]}
                   for node in nodes},
        "clock_skew_s": skew.skew_s,
        "skew_detail": [{"trial": d[0], "pred_s": d[1], "loc_s": d[2], "corr": d[3], "skew_s": d[4]}
                        for d in skew.detail],
        "optical_windows": [
            {"trial": t,
             "bin_start_idx": r.bin_start_idx, "bin_end_idx": r.bin_end_idx,
             "t_start_s": r.t_start_s, "t_end_s": r.t_end_s,
             "rel_start_idx": r.bin_start_idx - s0, "rel_end_idx": r.bin_end_idx - s0,
             "n_sync": r.n_sync, "refine_corr": r.corr, "refine_shift_s": r.refine_shift_s}
            for t, r in aligns],
        "walking_segment": {
            "bin_start_idx": s0, "bin_end_idx": s1,
            "t_start_s": s0/fs_high, "t_end_s": s1/fs_high,
            "duration_s": (s1-s0)/fs_high,
            "delimiting": {
                "method": "foot-IMU gyro RMS threshold + gap-bridge + pad, run covering optical anchors",
                "activity_node": cfg["segment"]["activity_node"],
                "threshold_dps": cfg["segment"]["gyro_rms_threshold_dps"],
                "smooth_s": cfg["segment"]["smooth_s"],
                "min_gap_s": cfg["segment"]["min_gap_s"],
                "pad_s": cfg["segment"]["pad_s"],
            }},
        "slice_columns": "t_s,ax,ay,az(m/s^2),gx,gy,gz(rad/s),mx,my,mz(Gauss,uncalibrated,upsampled 64->256)",
        "mag_calibration": "NOT applied in extract; fitted from varied-orientation tasks and applied in kincore",
        "out_dir": out_dir,
    }
    rpath = os.path.join(out_dir, "extract_report.json")
    json.dump(report, open(rpath, "w"), indent=2)
    print(f"Wrote report -> {rpath}")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    args = ap.parse_args()
    run_extract(args.config)
