#!/usr/bin/env python3
"""
Gait Kinematics — Open3D viewer
================================

Reads a recorded set of joint angles (gait_frames.csv + meta.json) and plays back a
single-leg sagittal skeleton in real time. This is the "quick visualization" layer; it is
fully separate from the gaitlib library and the 2D app (Open3D is NOT a dependency of either).

Input (produced by the app/pipeline exporter):
  gait_frames.csv : columns  t_s, hip_deg, knee_deg, ankle_deg   (one row per sample)
  meta.json       : {"leg": "right", "fs": 100.0,
                     "segment_lengths_m": {"pelvis":0.20,"thigh":0.45,"shank":0.43,"foot":0.20}}

Design notes
  * SAGITTAL plane only (X = forward, Z = up). We drive the figure from the clean,
    yaw-immune flexion angles — NOT full 3D orientation — so there is no yaw drift wobble.
  * ONE leg is drawn, because that is what the system measures. We do NOT invent the other
    leg's motion. Set CONTEXT_LEG = True to draw a static, faded second leg + torso purely
    as visual context (never animated).
  * Real-time playback: the frame shown is chosen by wall-clock time mapped onto t_s, so the
    on-screen cadence matches the real cadence. SPEED scales it (0.5 = slow motion).

Future: swap read_frames_from_csv() for a socket source to drive the same viewer live from
the hub — the per-frame contract (t_s, hip, knee, ankle) stays identical.

Run:  python gait_open3d_viewer.py gait_frames.csv
Requires:  pip install open3d numpy
"""

import sys, json, time, os
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("Open3D is not installed.  Run:  pip install open3d")

# ---------------------------------------------------------------- config
SPEED = 1.0          # 1.0 = real time, 0.5 = half speed, etc.
LOOP = True          # restart when the recording ends
ADVANCE = True       # procedural forward translation so the figure "walks" across the view
CONTEXT_LEG = False  # draw a static faded second leg + torso for body context (not measured)

DEFAULT_LENGTHS = {"pelvis": 0.20, "thigh": 0.45, "shank": 0.43, "foot": 0.20}


# ---------------------------------------------------------------- data loading
def load_meta(csv_path):
    meta_path = os.path.join(os.path.dirname(csv_path) or ".", "meta.json")
    meta = {"leg": "right", "fs": None, "segment_lengths_m": DEFAULT_LENGTHS}
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta.update(json.load(f))
    # fill any missing lengths with defaults
    L = dict(DEFAULT_LENGTHS)
    L.update(meta.get("segment_lengths_m", {}))
    meta["segment_lengths_m"] = L
    return meta


def read_frames_from_csv(csv_path):
    """Returns t_s, hip_deg, knee_deg, ankle_deg as 1-D arrays."""
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = np.asarray(data["t_s"], float)
    hip = np.asarray(data["hip_deg"], float)
    knee = np.asarray(data["knee_deg"], float)
    ankle = np.asarray(data["ankle_deg"], float)
    return t, hip, knee, ankle


# ---------------------------------------------------------------- forward kinematics
def fk_sagittal(hip_deg, knee_deg, ankle_deg, L, x_offset=0.0):
    """
    Sagittal-plane joint positions for ONE leg. X = forward, Z = up, Y = 0.
    Returns 5 points: [pelvis_top, hip, knee, ankle, toe].

    Sign convention is a sane starting point; if the leg bends the wrong way on your data,
    flip the sign of (hip_deg) and/or (knee_deg) below — that is the "neutral pose"
    calibration the Open3D guide refers to.
    """
    hip = np.array([x_offset, 0.0, 0.0])

    th_thigh = np.deg2rad(hip_deg)                       # thigh tilt from vertical
    knee = hip + L["thigh"] * np.array([np.sin(th_thigh), 0.0, -np.cos(th_thigh)])

    th_shank = np.deg2rad(hip_deg - knee_deg)            # knee bends the shank back
    ankle = knee + L["shank"] * np.array([np.sin(th_shank), 0.0, -np.cos(th_shank)])

    th_foot = th_shank + np.deg2rad(90.0 - ankle_deg)    # foot roughly forward-horizontal
    toe = ankle + L["foot"] * np.array([np.sin(th_foot), 0.0, -np.cos(th_foot)])

    pelvis_top = hip + np.array([0.0, 0.0, L["pelvis"]])
    return np.vstack([pelvis_top, hip, knee, ankle, toe])


BONES = np.array([[0, 1], [1, 2], [2, 3], [3, 4]])   # pelvis-hip, hip-knee, knee-ankle, ankle-toe


# ---------------------------------------------------------------- viewer
def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "gait_frames.csv"
    if not os.path.exists(csv_path):
        sys.exit(f"Frame file not found: {csv_path}")

    meta = load_meta(csv_path)
    L = meta["segment_lengths_m"]
    t, hip, knee, ankle = read_frames_from_csv(csv_path)
    duration = float(t[-1] - t[0])
    stride_advance = 0.9   # metres of forward drift per second when ADVANCE is on (visual only)

    # initial skeleton
    pts0 = fk_sagittal(hip[0], knee[0], ankle[0], L)
    lines = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts0),
        lines=o3d.utility.Vector2iVector(BONES),
    )
    lines.colors = o3d.utility.Vector3dVector(np.tile([0.10, 0.45, 0.85], (len(BONES), 1)))
    joints = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts0))
    joints.colors = o3d.utility.Vector3dVector(np.tile([0.95, 0.55, 0.10], (len(pts0), 1)))

    vis = o3d.visualization.Visualizer()
    vis.create_window(f"Gait Kinematics — Open3D  ({meta['leg']} leg)", width=960, height=720)
    vis.add_geometry(lines)
    vis.add_geometry(joints)

    # a faint ground line for reference
    ground = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([[-1, 0, -sum(L[k] for k in ("thigh", "shank"))],
                                           [ 6, 0, -sum(L[k] for k in ("thigh", "shank"))]]),
        lines=o3d.utility.Vector2iVector([[0, 1]]),
    )
    ground.colors = o3d.utility.Vector3dVector([[0.6, 0.6, 0.6]])
    vis.add_geometry(ground)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.07, 0.07, 0.09])
    opt.point_size = 12.0
    opt.line_width = 6.0  # honoured on some backends only

    # lateral camera (look along +Y at the sagittal plane)
    vc = vis.get_view_control()
    vc.set_front([0.0, -1.0, 0.0])
    vc.set_up([0.0, 0.0, 1.0])
    vc.set_lookat([1.5, 0.0, -0.4])
    vc.set_zoom(0.8)

    print(f"Playing {csv_path}: {len(t)} frames, {duration:.1f} s, leg={meta['leg']}, "
          f"speed={SPEED}x.  Close the window to quit.")

    t0 = time.perf_counter()
    running = True
    while running:
        elapsed = (time.perf_counter() - t0) * SPEED
        if elapsed > duration:
            if LOOP:
                t0 = time.perf_counter()
                elapsed = 0.0
            else:
                break
        i = int(np.searchsorted(t - t[0], elapsed))
        i = min(i, len(t) - 1)

        x_off = stride_advance * elapsed if ADVANCE else 0.0
        pts = fk_sagittal(hip[i], knee[i], ankle[i], L, x_offset=x_off)
        lines.points = o3d.utility.Vector3dVector(pts)
        joints.points = o3d.utility.Vector3dVector(pts)
        vis.update_geometry(lines)
        vis.update_geometry(joints)

        # keep the walking figure roughly centred
        if ADVANCE:
            vc.set_lookat([x_off + 0.3, 0.0, -0.4])

        running = vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()


if __name__ == "__main__":
    main()