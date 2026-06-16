"""
app/pipeline_runner.py — Run the Gait Kinematics pipeline on a recorded session.

This is a thin front-end over the **gaitlib** library (Task 1). The app keeps **no copy of
the kinematic math**: it builds a mounting config + raw-data dict from the session's stored
IMU streams, calls ``gaitlib.compute``, and writes the results into the session folder.
gaitlib is the single source of truth.

Output (cached under <session>/results/):
  timeseries.csv  t_s, <joint>_deg, <joint>_vel_dps, <joint>_acc_dps2, [<joint>_deg_9dof],
                  foot_strike, mid_swing, toe_off, steady_state
  summary.json    per-joint ROM/peaks/reps, gait stats, turnarounds, run metadata
"""
from __future__ import annotations
import json, os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gaitlib
from gaitlib.rawdata import infer_rate

# Node -> body-segment label, for the mounting config (display/metadata only).
from app.session_store import NODE_INFO


# --------------------------------------------------------------------------- #
def _mounting_config(nodes, joints, foot_node, pelvis_node, fs, run_modes,
                     beta6, beta9, joint_tau_s):
    """Assemble a gaitlib.MountingConfig from the session's topology + fusion options."""
    sensors = {n: NODE_INFO.get(n, (n, ""))[0] for n in nodes}
    fusion = gaitlib.FusionParams(run_modes=tuple(run_modes), beta_6dof=beta6,
                                  beta_9dof=beta9, joint_tau_s=joint_tau_s)
    return gaitlib.MountingConfig(
        sensors=sensors,
        joints={k: tuple(v) for k, v in joints.items()},
        foot_node=foot_node, pelvis_node=pelvis_node,
        rates={"imu_hz": fs, "mag_hz": fs}, fusion=fusion)


def run_pipeline(nodes_data, *, joints, foot_node, pelvis_node=None,
                 run_modes=("6dof",), beta6=0.033, beta9=0.05, joint_tau_s=0.3,
                 progress=None):
    """Run gaitlib on per-node IMU streams; return the ``GaitResults``.

    nodes_data: {node: {"t": (N,), "acc": (N,3), "gyr": (N,3), "mag": (N,3)}}
    joints:     {name: [distal_node, proximal_node]}
    """
    def say(msg, frac=None):
        if progress:
            progress(msg, frac)

    nodes = list(nodes_data.keys())
    fs = infer_rate(nodes_data[foot_node]["t"]) or 100.0
    cfg = _mounting_config(nodes, joints, foot_node, pelvis_node, fs, run_modes,
                           beta6, beta9, joint_tau_s)
    say("fusion → joint angles → gait events (gaitlib.compute)", 0.1)
    res = gaitlib.compute(nodes_data, cfg)
    say("done", 1.0)
    return res


# --------------------------------------------------------------------------- #
def write_results(out_dir, res, meta=None):
    """Write gaitlib results into the session folder (timeseries.csv + summary.json)."""
    os.makedirs(out_dir, exist_ok=True)
    res.save_timeseries_csv(os.path.join(out_dir, "timeseries.csv"))
    summary = res.summary()
    if meta:
        summary["meta"] = {**summary.get("meta", {}), **meta}
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    return summary


# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    # smoke test against a session dir created by session_store
    import argparse
    from app.session_store import load_session_streams, default_joint_topology
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir")
    args = ap.parse_args()
    nd, meta = load_session_streams(args.session_dir)
    topo = default_joint_topology(list(nd.keys()))
    res = run_pipeline(nd, joints=topo["joints"], foot_node=topo["foot"],
                       pelvis_node=topo["pelvis"], progress=lambda m, f: print(f"  {m}"))
    s = write_results(os.path.join(args.session_dir, "results"), res, meta)
    print(json.dumps(s["per_joint"], indent=2))
    print("gait:", s["gait"])
