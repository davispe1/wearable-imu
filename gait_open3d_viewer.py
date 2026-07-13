#!/usr/bin/env python3
"""
Gait Kinematics — Open3D humanoid viewer (v3)
=============================================

Plays a recorded set of joint angles (gait_frames.csv + meta.json) as a solid HUMANOID
mannequin walking in real time.

  * MEASURED leg = real angles, drawn in ORANGE.
  * The 4 real SENSOR positions on the measured leg are marked in GREEN
    (pelvis/sacrum, mid-thigh, mid-shank, foot instep — NOT the toe tip).
  * The other leg is GENERATED in anti-phase (estimated, faded grey) for a natural walk.
    Set SECOND_LEG=False to show only the measured leg.
  * Foot has heel + ankle + toe (not a single spike).
  * Long ground so it never runs out; camera follows the figure.

On-screen controls:  SPACE pause/resume · J slower · K faster · R restart · mouse rotate/zoom

Run:  python gait_open3d_viewer.py data/<session>/results/gait_frames.csv
Needs:  pip install open3d numpy
"""

import sys, os, json, time
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("Open3D is not installed.  Run:  pip install open3d")

# ----------------------------------------------------------- config (edit + rerun)
SPEED       = 1.0
LOOP        = True
ADVANCE     = True
SECOND_LEG  = True
ARMS        = True
SHOW_SENSORS = True

BONE_R, JOINT_R, HEAD_R = 0.035, 0.050, 0.105
SENSOR_R   = 0.055
FOOT_LEN, FOOT_DROP = 0.22, 0.085     # foot length and ankle height above ground
WALK_SPEED = 0.9                       # m/s of forward drift (visual only)

COL_BODY   = [0.78, 0.80, 0.85]
COL_MEAS   = [0.96, 0.55, 0.10]        # measured leg
COL_EST    = [0.45, 0.47, 0.52]        # estimated leg + arms
COL_JOINT  = [0.90, 0.92, 0.96]
COL_SENSOR = [0.10, 0.90, 0.25]        # GREEN sensor markers

DEFAULT_L = {"pelvis": 0.20, "thigh": 0.45, "shank": 0.43, "foot": 0.20}
TORSO_LEN, NECK_GAP = 0.52, 0.07
HIP_W, SHOULDER_W   = 0.20, 0.36
UPPER_ARM, FORE_ARM = 0.30, 0.27


# ----------------------------------------------------------- data
def load_meta(csv_path):
    p = os.path.join(os.path.dirname(csv_path) or ".", "meta.json")
    meta = {"leg": "right", "fs": None, "segment_lengths_m": dict(DEFAULT_L)}
    if os.path.exists(p):
        with open(p) as f:
            meta.update(json.load(f))
    L = dict(DEFAULT_L); L.update(meta.get("segment_lengths_m", {}))
    meta["segment_lengths_m"] = L
    return meta


def load_frames(csv_path):
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    return (np.asarray(d["t_s"], float), np.asarray(d["hip_deg"], float),
            np.asarray(d["knee_deg"], float), np.asarray(d["ankle_deg"], float))


def half_stride_shift(sig, fs):
    x = sig - np.mean(sig)
    ac = np.correlate(x, x, "full")[len(x) - 1:]
    if len(ac) < 5:
        return 0
    d = np.diff(ac)
    zc = np.where((d[:-1] < 0) & (d[1:] >= 0))[0]
    period = int(zc[0]) + 1 if len(zc) else int(0.6 * (fs or 100))
    return max(1, period // 2)


# ----------------------------------------------------------- kinematics
def leg_chain(hip_deg, knee_deg, ankle_deg, hip_pos, L):
    """Joints incl. a real foot: hip, knee, ankle, heel, toe."""
    y = hip_pos[1]
    th = np.deg2rad(hip_deg)
    knee = hip_pos + L["thigh"] * np.array([np.sin(th), 0, -np.cos(th)])
    th2 = np.deg2rad(hip_deg - knee_deg)
    ankle = knee + L["shank"] * np.array([np.sin(th2), 0, -np.cos(th2)])
    # foot: heel below/behind the ankle, toe forward; ankle_deg tilts it
    pitch = np.deg2rad(ankle_deg - 90.0)          # flip sign here if foot tilts wrong way
    fwd = np.array([np.cos(pitch), 0.0, np.sin(pitch)])
    heel = ankle + np.array([0, 0, -FOOT_DROP]) - 0.30 * FOOT_LEN * fwd
    toe = heel + FOOT_LEN * fwd
    out = {"hip": hip_pos, "knee": knee, "ankle": ankle, "heel": heel, "toe": toe}
    for k in out:
        out[k] = np.array([out[k][0], y, out[k][2]])
    return out


def leg_sensors(leg, pelvis):
    """Real 4-sensor positions on the measured leg (green markers)."""
    return {
        "SA": pelvis + np.array([-0.02, 0, 0.05]),               # pelvis / sacrum
        "RT": 0.5 * (leg["hip"] + leg["knee"]),                  # mid thigh
        "RS": 0.5 * (leg["knee"] + leg["ankle"]),                # mid shank
        "RF": 0.60 * leg["ankle"] + 0.40 * leg["toe"],           # foot instep (not the toe)
    }


def arm_chain(shoulder, swing_deg):
    th = np.deg2rad(swing_deg)
    elbow = shoulder + UPPER_ARM * np.array([np.sin(th), 0, -np.cos(th)])
    hand = elbow + FORE_ARM * np.array([np.sin(th), 0, -np.cos(th)])
    return [shoulder, elbow, hand]


def build_skeleton(i, t, hip, knee, ankle, L, meas_side, shift, x_off):
    j = int((i + shift) % len(t))
    sgn = +1 if meas_side == "right" else -1

    pelvis = np.array([x_off, 0.0, 0.0])
    neck = pelvis + np.array([0, 0, TORSO_LEN])
    head = neck + np.array([0, 0, NECK_GAP + HEAD_R])
    sh_m = neck + np.array([0,  sgn * SHOULDER_W / 2, 0])
    sh_o = neck + np.array([0, -sgn * SHOULDER_W / 2, 0])
    hip_m = pelvis + np.array([0,  sgn * HIP_W / 2, 0])
    hip_o = pelvis + np.array([0, -sgn * HIP_W / 2, 0])

    legM = leg_chain(hip[i], knee[i], ankle[i], hip_m, L)
    legO = leg_chain(hip[j], knee[j], ankle[j], hip_o, L) if SECOND_LEG else None

    P, B, S = {}, [], {}
    P["head"], P["neck"], P["pelvis"] = head, neck, pelvis
    B += [("neck", "head", "body"), ("pelvis", "neck", "body")]

    for tag, leg in (("M", legM), ("O", legO)):
        if leg is None:
            continue
        col = "meas" if tag == "M" else "est"
        for k, v in leg.items():
            P[f"{k}{tag}"] = v
        B += [("pelvis", f"hip{tag}", "body"),
              (f"hip{tag}", f"knee{tag}", col), (f"knee{tag}", f"ankle{tag}", col),
              (f"ankle{tag}", f"heel{tag}", col), (f"heel{tag}", f"toe{tag}", col)]

    if SHOW_SENSORS:
        S = leg_sensors(legM, pelvis)

    if ARMS:
        P["shM"], P["shO"] = sh_m, sh_o
        armM = arm_chain(sh_m, -(hip[j] if SECOND_LEG else -hip[i]) * 0.6)
        armO = arm_chain(sh_o, -hip[i] * 0.6)
        P["elbM"], P["hndM"] = armM[1], armM[2]
        P["elbO"], P["hndO"] = armO[1], armO[2]
        B += [("neck", "shM", "body"), ("neck", "shO", "body"),
              ("shM", "elbM", "est"), ("elbM", "hndM", "est"),
              ("shO", "elbO", "est"), ("elbO", "hndO", "est")]
    return P, B, S


# ----------------------------------------------------------- mesh helpers
def R_z_to(d):
    z = np.array([0, 0, 1.0]); d = d / (np.linalg.norm(d) + 1e-12)
    v = np.cross(z, d); s = np.linalg.norm(v); c = float(np.dot(z, d))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def main():
    csv = sys.argv[1] if len(sys.argv) > 1 else "gait_frames.csv"
    if not os.path.exists(csv):
        sys.exit(f"Frame file not found: {csv}")
    meta = load_meta(csv); L = meta["segment_lengths_m"]
    t, hip, knee, ankle = load_frames(csv)
    fs = meta.get("fs") or (1.0 / np.median(np.diff(t)))
    shift = half_stride_shift(knee, fs)
    duration = float(t[-1] - t[0])
    meas_side = meta.get("leg", "right")
    colmap = {"body": COL_BODY, "meas": COL_MEAS, "est": COL_EST}

    cyl0 = o3d.geometry.TriangleMesh.create_cylinder(radius=1.0, height=1.0, resolution=14)
    cyl0_v = np.asarray(cyl0.vertices).copy()
    sph0 = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=12)
    sph0_v = np.asarray(sph0.vertices).copy()

    P0, B0, S0 = build_skeleton(0, t, hip, knee, ankle, L, meas_side, shift, 0.0)

    bone_meshes = []
    for (a, b, tag) in B0:
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(cyl0_v.copy()), cyl0.triangles)
        m.paint_uniform_color(colmap[tag]); m.compute_vertex_normals()
        bone_meshes.append(m)
    joint_meshes = {}
    for name in P0:
        r = HEAD_R if name == "head" else JOINT_R
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(sph0_v.copy()), sph0.triangles)
        is_meas = name[:-1] in ("hip", "knee", "ankle", "heel", "toe") and name.endswith("M")
        m.paint_uniform_color(COL_MEAS if is_meas else COL_JOINT); m.compute_vertex_normals()
        joint_meshes[name] = (m, r)
    sensor_meshes = {}
    for name in S0:
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(sph0_v.copy()), sph0.triangles)
        m.paint_uniform_color(COL_SENSOR); m.compute_vertex_normals()
        sensor_meshes[name] = m

    def set_bone(mesh, p0, p1, radius):
        v = p1 - p0; Lc = np.linalg.norm(v)
        nv = (cyl0_v * np.array([radius, radius, Lc])) @ R_z_to(v).T + (p0 + p1) / 2
        mesh.vertices = o3d.utility.Vector3dVector(nv); mesh.compute_vertex_normals()

    def set_sphere(mesh, r, pos):
        mesh.vertices = o3d.utility.Vector3dVector(sph0_v * r + pos); mesh.compute_vertex_normals()

    # ---- window + controls
    state = {"paused": False, "speed": SPEED, "el": 0.0, "last": time.perf_counter()}
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(f"Gait Kinematics — Open3D  ({meas_side} leg measured)", 1000, 760)

    def k_pause(v): state["paused"] = not state["paused"]; return False
    def k_faster(v): state["speed"] *= 1.25; return False
    def k_slower(v): state["speed"] /= 1.25; return False
    def k_reset(v): state["el"] = 0.0; return False
    vis.register_key_callback(32, k_pause)        # space
    vis.register_key_callback(ord("K"), k_faster)
    vis.register_key_callback(ord("J"), k_slower)
    vis.register_key_callback(ord("R"), k_reset)

    for m in bone_meshes:
        vis.add_geometry(m)
    for (m, _) in joint_meshes.values():
        vis.add_geometry(m)
    for m in sensor_meshes.values():
        vis.add_geometry(m)

    floor_len = WALK_SPEED * duration + 4.0 if ADVANCE else 6.0
    ground = o3d.geometry.TriangleMesh.create_box(floor_len, 4.0, 0.01)
    ground.translate([-2, -2, -(L["thigh"] + L["shank"] + FOOT_DROP + 0.02)])
    ground.paint_uniform_color([0.16, 0.16, 0.19])
    vis.add_geometry(ground)

    ro = vis.get_render_option(); ro.background_color = np.array([0.06, 0.06, 0.08]); ro.light_on = True
    vc = vis.get_view_control()
    side = +1 if meas_side == "right" else -1
    vc.set_front([0.12, -side, 0.10]); vc.set_up([0, 0, 1]); vc.set_lookat([0, 0, 0.0]); vc.set_zoom(0.85)

    print("Controls:  SPACE pause · J slower · K faster · R restart · mouse rotate/zoom")
    print(f"Playing {csv}: {len(t)} frames, {duration:.1f}s, measured leg={meas_side}.")

    running = True
    while running:
        now = time.perf_counter()
        if not state["paused"]:
            state["el"] += (now - state["last"]) * state["speed"]
        state["last"] = now
        el = state["el"]
        if el > duration:
            if LOOP: state["el"] = el = 0.0
            else: break
        i = min(int(np.searchsorted(t - t[0], el)), len(t) - 1)
        x_off = WALK_SPEED * el if ADVANCE else 0.0

        P, B, S = build_skeleton(i, t, hip, knee, ankle, L, meas_side, shift, x_off)
        for mesh, (a, b, tag) in zip(bone_meshes, B):
            set_bone(mesh, P[a], P[b], BONE_R); vis.update_geometry(mesh)
        for name, (mesh, r) in joint_meshes.items():
            if name in P:
                set_sphere(mesh, r, P[name]); vis.update_geometry(mesh)
        for name, mesh in sensor_meshes.items():
            if name in S:
                set_sphere(mesh, SENSOR_R, S[name]); vis.update_geometry(mesh)
        if ADVANCE:
            vc.set_lookat([x_off, 0, 0.0])
        running = vis.poll_events(); vis.update_renderer()
    vis.destroy_window()


if __name__ == "__main__":
    main()