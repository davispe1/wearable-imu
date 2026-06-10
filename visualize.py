"""
visualize.py — 3D animated stick figure + 2D overlays for the right-leg gait result.

Two paths:
  * Interactive (default): pyqtgraph GLViewWidget animates the right-leg stick figure
    (foot -> ankle -> knee -> hip -> pelvis) from the yaw-corrected joint angles, with a
    2D dock of computed-vs-reference angle, angular velocity, and foot accel + step marks.
  * --save: render a matplotlib dashboard PNG (+ a stick-figure montage PNG). Works
    headless; this is also the 2D fallback if the Qt/OpenGL stack is unavailable.

Data come from the outputs CSV/JSON written by run.py.
"""
from __future__ import annotations
import argparse, json, os
import numpy as np

# nominal right-leg segment lengths (m) for the stick figure
L_THIGH, L_SHANK, L_FOOT, PELVIS_W = 0.42, 0.42, 0.20, 0.20


def load(cfg_base):
    csv = cfg_base + "_timeseries.csv"
    js = cfg_base + "_summary.json"
    d = np.genfromtxt(csv, delimiter=",", names=True)
    summary = json.load(open(js)) if os.path.exists(js) else {}
    return d, summary


def leg_points(hip_deg, knee_deg, ankle_deg):
    """Sagittal-plane joint positions (x forward, z up) for one frame.

    Pelvis at top; thigh inclined by hip flexion, shank by knee flexion, foot by ankle.
    Returns array (5,3): pelvis, hip, knee, ankle, toe (y=0, sagittal).
    """
    hip = np.array([0.0, 0.0, 0.0])
    pelvis = hip + np.array([-0.5*PELVIS_W, 0, 0.10])
    th = np.radians(hip_deg)                 # thigh angle from vertical (flexion +fwd)
    knee = hip + L_THIGH * np.array([np.sin(th), 0, -np.cos(th)])
    sh = th - np.radians(knee_deg)           # knee flexes shank backward
    ankle = knee + L_SHANK * np.array([np.sin(sh), 0, -np.cos(sh)])
    ft = sh + np.radians(90 - ankle_deg)     # foot ~perpendicular, modulated by ankle
    toe = ankle + L_FOOT * np.array([np.sin(ft), 0, -np.cos(ft)])
    return np.vstack([pelvis, hip, knee, ankle, toe])


# --------------------------------------------------------------------------- #
def save_dashboard(d, summary, base):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = d["t_opt_s"]
    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(3, 3)
    joints = ["ankle", "knee", "hip"]
    # angle overlays
    for i, j in enumerate(joints):
        ax = fig.add_subplot(gs[i, 0:2])
        ax.plot(t, d[f"{j}_deg"], lw=0.8, color="tab:blue", label="computed 6-DOF")
        if f"{j}_deg_9dof" in d.dtype.names:
            ax.plot(t, d[f"{j}_deg_9dof"], lw=0.5, color="tab:gray", alpha=0.5, label="9-DOF")
        ref = d[f"{j}_ref_deg"]
        m = ~np.isnan(ref)
        ax.plot(t[m], ref[m], ".", ms=2, color="tab:red", label="optical reference")
        ax.set_ylabel(f"{j}\nflexion (deg)")
        if i == 0:
            ax.legend(loc="upper right", fontsize=7, ncol=3)
        rmse = summary.get("validation_rmse_deg", {}).get("6dof", {}).get(j, float("nan"))
        ax.text(0.01, 0.92, f"RMSE={rmse:.1f} deg  ROM={summary.get('joint_rom_deg_steady',{}).get('6dof',{}).get(j,float('nan')):.0f}",
                transform=ax.transAxes, fontsize=8, va="top")
    fig.axes[-1].set_xlabel("time (s)")

    # angular velocity
    axv = fig.add_subplot(gs[0, 2])
    for j in joints:
        axv.plot(t, d[f"{j}_vel_dps"], lw=0.6, label=j)
    axv.set_title("joint angular velocity (deg/s)", fontsize=9); axv.legend(fontsize=7)

    # foot accel + step markers
    axf = fig.add_subplot(gs[1, 2])
    axf.plot(t, d["foot_acc_mag"], lw=0.5, color="k")
    fs = np.where(d["foot_strike"] > 0)[0]
    axf.plot(t[fs], d["foot_acc_mag"][fs], "rv", ms=4, label="foot strike")
    axf.set_title("foot accel magnitude + steps", fontsize=9); axf.legend(fontsize=7)

    # metrics panel
    axm = fig.add_subplot(gs[2, 2]); axm.axis("off")
    g = summary.get("gait", {}); h = summary.get("heading_rmse_vs_optical_deg", {})
    lines = [
        f"Subject {summary.get('subject')} / {summary.get('task')}  ({summary.get('duration_s',0):.0f} s walk)",
        f"cadence: {g.get('cadence_steps_per_min',float('nan')):.1f} steps/min",
        f"stride: {g.get('stride_time_mean_s',float('nan')):.2f}+/-{g.get('stride_time_std_s',float('nan')):.2f} s",
        f"steady strides: {g.get('n_steady_strides','?')}  strikes: {g.get('n_foot_strikes','?')}",
        f"turnarounds: {len(summary.get('turnarounds',[]))}",
        "",
        "RMSE vs optical (deg):",
        "  " + "  ".join(f"{j}={summary.get('validation_rmse_deg',{}).get('6dof',{}).get(j,float('nan')):.1f}" for j in joints),
        f"heading 6-DOF={h.get('6dof',float('nan')):.1f}  9-DOF={h.get('9dof',float('nan')):.1f}",
        "  -> magnetometer does NOT improve heading",
        "     (indoor field distortion)",
    ]
    axm.text(0, 1, "\n".join(lines), va="top", fontsize=8, family="monospace")

    fig.suptitle("Mythos MECH — right-leg gait (6-DOF primary) vs optical reference", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p = base + "_dashboard.png"
    fig.savefig(p, dpi=110); plt.close(fig)
    print(f"Wrote {p}")

    # stick-figure montage over one stride
    fig2, ax2 = plt.subplots(figsize=(11, 4))
    # pick a clean steady-state stride
    steady = np.where(d["steady_state"] > 0)[0]
    i0 = steady[len(steady)//3] if len(steady) else int(len(t)*0.3)
    stride = int(1.1 * 256); nframes = 7
    for k, fi in enumerate(range(i0, i0 + stride, max(1, stride // nframes))):
        pts = leg_points(d["hip_deg"][fi], d["knee_deg"][fi], d["ankle_deg"][fi])
        c = plt.cm.viridis(k / nframes)
        ax2.plot(pts[:, 0] + 0.55*k, pts[:, 2], "-o", color=c, ms=5, lw=2.5)
    ax2.set_aspect("equal"); ax2.set_title("right-leg sagittal stick figure over one steady stride (foot->ankle->knee->hip->pelvis)")
    ax2.set_xlabel("forward (m), frames offset in time ->"); ax2.set_ylabel("up (m)")
    p2 = base + "_stickfigure.png"
    fig2.savefig(p2, dpi=110); plt.close(fig2)
    print(f"Wrote {p2}")


# --------------------------------------------------------------------------- #
def show_3d(d, summary):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    import pyqtgraph.opengl as gl

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = gl.GLViewWidget(); w.setWindowTitle("Mythos MECH — right leg (3D)")
    w.setCameraPosition(distance=2.0, elevation=8, azimuth=70)
    grid = gl.GLGridItem(); grid.scale(0.1, 0.1, 0.1); w.addItem(grid)
    line = gl.GLLinePlotItem(width=4, antialias=True, color=(0.2, 0.6, 1, 1))
    pts = gl.GLScatterPlotItem(size=10, color=(1, 0.3, 0.3, 1))
    w.addItem(line); w.addItem(pts); w.show()

    t = d["t_opt_s"]; n = len(t); state = {"i": int(n*0.3)}

    def update():
        i = state["i"] % n
        P = leg_points(d["hip_deg"][i], d["knee_deg"][i], d["ankle_deg"][i])
        line.setData(pos=P); pts.setData(pos=P)
        state["i"] += 3
    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(12)
    app.exec_() if hasattr(app, "exec_") else app.exec()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    ap.add_argument("--save", action="store_true", help="render PNGs (headless)")
    args = ap.parse_args()
    import yaml
    cfg = yaml.safe_load(open(args.config))
    base = os.path.join(cfg["output"]["dir"],
                        f"{cfg['dataset']['subject']}_{cfg['dataset']['session']}_{cfg['selection']['task']}")
    d, summary = load(base)
    if args.save:
        save_dashboard(d, summary, base)
        return
    try:
        show_3d(d, summary)
    except Exception as e:
        print(f"3D unavailable ({e}); writing 2D dashboard instead.")
        save_dashboard(d, summary, base)


if __name__ == "__main__":
    main()
