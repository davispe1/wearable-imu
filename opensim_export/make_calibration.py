#!/usr/bin/env python3
"""
opensim_export/make_calibration.py — Build an OpenSense calibration .sto from a
trial's orientations .sto, choosing the static-calibration pose explicitly.

Two modes:

  1) STATIC WINDOW  (use this for your OWN recordings with a standing-still pose)
        python -m opensim_export.make_calibration orientations.sto out_calibration.sto --window 0 5
     Averages every quaternion over [T0, T1] seconds. Record 3-5 s standing
     still in neutral at the start of each session and point --window at it.

  2) AUTO-NEUTRAL   (for datasets with NO explicit static pose, e.g. continuous walking)
        python -m opensim_export.make_calibration orientations.sto out_calibration.sto --auto-neutral
     Looks for a joint-angles CSV next to the orientations file (written by the
     pipeline: <id>_joint_angles.csv or timeseries.csv), finds the instant closest
     to neutral standing AND quasi-static (low joint velocity), and averages a
     small window around it.

The output matches the format OpenSim OpenSense expects: a single data row,
header with DataRate/DataType=Quaternion, scalar-first w,x,y,z per IMU column.
No OpenSim dependency; standard library only.

CLI (run from the gait-opensim/ project root):
    python -m opensim_export.make_calibration <orientations.sto> <out.sto> --window T0 T1
    python -m opensim_export.make_calibration <orientations.sto> <out.sto> --auto-neutral
"""
import argparse
import csv
import glob
import math
import os
import sys


def read_orientations_sto(path):
    """Return (data_rate, columns, times, table) where table[col] = list of (w,x,y,z)."""
    with open(path) as fh:
        lines = fh.readlines()
    data_rate = None
    hdr_i = None
    for i, ln in enumerate(lines):
        s = ln.strip()
        if s.lower().startswith("datarate"):
            data_rate = s.split("=", 1)[1].strip()
        if s == "endheader":
            hdr_i = i
            break
    if hdr_i is None:
        raise ValueError(f"{path}: no 'endheader' line found")
    cols = lines[hdr_i + 1].rstrip("\n").split("\t")
    imu_cols = cols[1:]                         # drop 'time'
    times = []
    table = {c: [] for c in imu_cols}
    for ln in lines[hdr_i + 2:]:
        if not ln.strip():
            continue
        cells = ln.rstrip("\n").split("\t")
        times.append(float(cells[0]))
        for c, cell in zip(imu_cols, cells[1:]):
            w, x, y, z = (float(v) for v in cell.split(","))
            table[c].append((w, x, y, z))
    return data_rate, imu_cols, times, table


def quat_mean(quats):
    """Hemisphere-aligned component mean, renormalized. quats: list of (w,x,y,z)."""
    ref = quats[0]
    acc = [0.0, 0.0, 0.0, 0.0]
    for q in quats:
        dot = sum(a * b for a, b in zip(q, ref))
        s = -1.0 if dot < 0 else 1.0
        for k in range(4):
            acc[k] += s * q[k]
    n = math.sqrt(sum(c * c for c in acc)) or 1.0
    return tuple(c / n for c in acc)


def indices_in_window(times, t0, t1):
    return [i for i, t in enumerate(times) if t0 <= t <= t1]


def _find_angles_csv(orientations_path):
    """Locate the joint-angles CSV next to the orientations .sto.

    Accepts both the new pipeline format (<id>_joint_angles.csv) and the legacy
    timeseries.csv name.
    """
    directory = os.path.dirname(os.path.abspath(orientations_path))
    # New pipeline: <session_id>_joint_angles.csv
    candidates = glob.glob(os.path.join(directory, "*_joint_angles.csv"))
    if candidates:
        return candidates[0]
    # Legacy name
    legacy = os.path.join(directory, "timeseries.csv")
    if os.path.exists(legacy):
        return legacy
    return None


def find_neutral_index(angles_csv, times, vel_weight=0.1):
    """Index (into the orientation grid) of the best calibration instant.

    Cost = |hip| + |knee| + |ankle|  +  vel_weight * (sum of joint velocities)

    The velocity term ensures we pick a quasi-static instant — slow-moving AND
    close to neutral. For a dedicated static-standing window all velocities are ~0
    so this reduces to 'most neutral pose'.
    """
    rows = list(csv.DictReader(open(angles_csv)))
    # Support both t_s (new) and t_opt_s (legacy) time column names
    t_col = "t_s" if "t_s" in rows[0] else "t_opt_s"
    best_cost = None
    best_t = best_ang = best_vel = 0.0
    for r in rows:
        try:
            ang = abs(float(r["hip_deg"])) + abs(float(r["knee_deg"])) + abs(float(r["ankle_deg"]))
        except (KeyError, ValueError):
            continue
        vel = 0.0
        for k in ("hip_vel_dps", "knee_vel_dps", "ankle_vel_dps"):
            if k in r:
                try:
                    vel += abs(float(r[k]))
                except ValueError:
                    pass
        cost = ang + vel_weight * vel
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_t = float(r[t_col])
            best_ang = ang
            best_vel = vel
    best_i = min(range(len(times)), key=lambda i: abs(times[i] - best_t))
    print(f"  selected: t={best_t:.3f} s  angle={best_ang:.1f} deg  velocity={best_vel:.1f} deg/s")
    return best_i, best_t


def write_calibration_sto(path, data_rate, imu_cols, mean_quats):
    with open(path, "w", newline="") as fh:
        fh.write(f"DataRate={data_rate}\n")
        fh.write("DataType=Quaternion\n")
        fh.write("version=3\n")
        fh.write("OpenSimVersion=4.5\n")
        fh.write("endheader\n")
        fh.write("time\t" + "\t".join(imu_cols) + "\n")
        cells = ["0.000000"]
        for c in imu_cols:
            w, x, y, z = mean_quats[c]
            cells.append(f"{w:.8f},{x:.8f},{y:.8f},{z:.8f}")
        fh.write("\t".join(cells) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Build an OpenSense calibration .sto with a chosen calibration pose.")
    ap.add_argument("orientations", help="full-trial orientations .sto (from opensim_export.to_sto)")
    ap.add_argument("output", help="calibration .sto to write")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"),
                   help="average a static standing window [T0 T1] in seconds")
    g.add_argument("--auto-neutral", action="store_true",
                   help="auto-pick the best near-neutral, quasi-static instant from the joint-angles CSV")
    ap.add_argument("--halfwidth", type=float, default=0.05,
                    help="--auto-neutral: +/- seconds averaged around the selected instant (default 0.05 s)")
    args = ap.parse_args(argv)

    data_rate, imu_cols, times, table = read_orientations_sto(args.orientations)

    if args.window:
        t0, t1 = args.window
        idx = indices_in_window(times, t0, t1)
        if not idx:
            sys.exit(f"No samples in window [{t0}, {t1}] s.")
        print(f"STATIC WINDOW  [{t0}, {t1}] s  ->  {len(idx)} samples averaged.")
    else:
        angles_csv = _find_angles_csv(args.orientations)
        if angles_csv is None:
            sys.exit(
                "--auto-neutral needs a joint-angles CSV next to the orientations file.\n"
                "Run the pipeline first:  python -m kinematics.pipeline <session_dir> --csv")
        print(f"AUTO-NEUTRAL  using {os.path.basename(angles_csv)}")
        ci, ct = find_neutral_index(angles_csv, times)
        idx = indices_in_window(times, ct - args.halfwidth, ct + args.halfwidth)
        print(f"  averaging t=[{ct - args.halfwidth:.3f}, {ct + args.halfwidth:.3f}] s  "
              f"-> {len(idx)} samples (+/-{args.halfwidth * 1000:.0f} ms).")

    mean_quats = {c: quat_mean([table[c][i] for i in idx]) for c in imu_cols}
    write_calibration_sto(args.output, data_rate, imu_cols, mean_quats)
    print(f"Wrote  {args.output}")
    print("Columns:", ", ".join(imu_cols))


if __name__ == "__main__":
    raise SystemExit(main())
