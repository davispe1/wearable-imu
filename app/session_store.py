"""
app/session_store.py — On-disk session store + 9-DOF IMU CSV schema.

A *session* is a folder under the configured data root (default ~/GaitApp/sessions/):

    <data_root>/<session_id>/
        raw/data.csv      canonical imported IMU (long format, all nodes), never mutated
        session.json      metadata: subject, date, duration, fs, nodes, joints, height
        results/          pipeline output (timeseries.csv, summary.json) — cached on run

CSV schema (canonical, one combined file, long format):

    node,t_s,ax,ay,az,gx,gy,gz,mx,my,mz

  node          sensor/segment id (RF/RS/RT/SA, LF/LS/LT, or generic)
  t_s           time, monotonic per node, 0 at start (s)
  ax,ay,az      linear acceleration, sensor frame (m/s^2)
  gx,gy,gz      angular velocity, sensor frame (rad/s)
  mx,my,mz      magnetometer, raw counts (uncalibrated; pipeline fits hard/soft-iron)

Import also accepts a single combined CSV, OR per-node files named <NODE>.csv (the `node`
is then taken from the filename) — the existing Geneva slices load directly this way.
"""
from __future__ import annotations
import csv, json, os, shutil, sys
from datetime import datetime
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Node -> (segment label, body side). Drives default topology + display.
NODE_INFO = {
    "RF": ("right foot", "right"), "RS": ("right shank", "right"), "RT": ("right thigh", "right"),
    "LF": ("left foot", "left"),   "LS": ("left shank", "left"),   "LT": ("left thigh", "left"),
    "SA": ("sacrum / pelvis", "center"),
}
SCHEMA_COLS = ["node", "t_s", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]

# Accepted aliases when parsing arbitrary CSVs.
_T_ALIASES = ("t_s", "t_opt_s", "t", "time", "time_s", "timestamp_s")
_COL_ALIASES = {"ax": ("ax", "acc_x", "a_x"), "ay": ("ay", "acc_y", "a_y"), "az": ("az", "acc_z", "a_z"),
                "gx": ("gx", "gyr_x", "g_x"), "gy": ("gy", "gyr_y", "g_y"), "gz": ("gz", "gyr_z", "g_z"),
                "mx": ("mx", "mag_x", "m_x"), "my": ("my", "mag_y", "m_y"), "mz": ("mz", "mag_z", "m_z")}


# --------------------------------------------------------------------------- #
def app_config(path=None):
    """Load config/app.yaml with sensible defaults."""
    path = path or os.path.join(ROOT, "config", "app.yaml")
    cfg = {}
    if os.path.exists(path):
        cfg = yaml.safe_load(open(path)) or {}
    app = cfg.setdefault("app", {})
    app.setdefault("data_dir", "~/GaitApp/sessions")
    app.setdefault("run_modes", ["6dof"])
    fz = app.setdefault("fusion", {})
    fz.setdefault("beta_6dof", 0.033); fz.setdefault("beta_9dof", 0.05); fz.setdefault("joint_tau_s", 0.3)
    cfg.setdefault("visualization", {"fps": 60, "default_speed": 1.0, "loop": True})
    cfg.setdefault("anthropometry", {"default_height_m": 1.70})
    return cfg


def data_root(cfg):
    p = os.path.expanduser(cfg["app"]["data_dir"])
    os.makedirs(p, exist_ok=True)
    return p


# --------------------------------------------------------------------------- #
def _resolve_columns(header):
    """Map a CSV header to canonical channel names; return (idx_map, t_idx, node_idx)."""
    low = [h.strip().lower() for h in header]
    t_idx = next((low.index(a) for a in _T_ALIASES if a in low), None)
    node_idx = low.index("node") if "node" in low else None
    idx = {}
    for canon, aliases in _COL_ALIASES.items():
        j = next((low.index(a) for a in aliases if a in low), None)
        idx[canon] = j
    return idx, t_idx, node_idx


def parse_imu_csv(path, node_hint=None):
    """Parse one CSV into {node: {t, acc, gyr, mag}}.

    Combined files (with a `node` column) split into nodes; otherwise the node id comes
    from `node_hint` or the filename stem. Missing magnetometer columns -> zeros.
    """
    with open(path, newline="") as f:
        rdr = csv.reader(f)
        header = next(rdr)
        idx, t_idx, node_idx = _resolve_columns(header)
        if t_idx is None:
            raise ValueError(f"{os.path.basename(path)}: no time column (expected one of {_T_ALIASES})")
        rows = {}
        for r in rdr:
            if not r:
                continue
            node = (r[node_idx].strip() if node_idx is not None else
                    (node_hint or os.path.splitext(os.path.basename(path))[0]))
            rows.setdefault(node, []).append(r)
    out = {}
    for node, rr in rows.items():
        arr = np.array(rr, dtype=object)
        get = lambda j: arr[:, j].astype(float) if j is not None else np.zeros(len(arr))
        t = get(t_idx)
        acc = np.column_stack([get(idx["ax"]), get(idx["ay"]), get(idx["az"])])
        gyr = np.column_stack([get(idx["gx"]), get(idx["gy"]), get(idx["gz"])])
        mag = np.column_stack([get(idx["mx"]), get(idx["my"]), get(idx["mz"])])
        order = np.argsort(t)
        out[node] = {"t": t[order], "acc": acc[order], "gyr": gyr[order], "mag": mag[order]}
    return out


def _merge(streams_list):
    merged = {}
    for s in streams_list:
        for node, d in s.items():
            if node in merged:
                raise ValueError(f"node {node!r} appears in more than one imported file")
            merged[node] = d
    return merged


# --------------------------------------------------------------------------- #
def default_joint_topology(nodes):
    """Pick joint chain (distal->proximal), foot and pelvis nodes from available ids."""
    present = set(nodes)
    joints, foot, side = {}, None, None
    if {"RF", "RS", "RT"} <= present:
        side, foot = "right", "RF"
        joints = {"ankle": ["RF", "RS"], "knee": ["RS", "RT"]}
        if "SA" in present:
            joints["hip"] = ["RT", "SA"]
    elif {"LF", "LS", "LT"} <= present:
        side, foot = "left", "LF"
        joints = {"ankle": ["LF", "LS"], "knee": ["LS", "LT"]}
        if "SA" in present:
            joints["hip"] = ["LT", "SA"]
    else:
        # generic: keep any joints we can't infer empty; foot = first node
        foot = nodes[0] if nodes else None
    pelvis = "SA" if "SA" in present else None
    return {"joints": joints, "foot": foot, "pelvis": pelvis, "side": side}


def write_combined_csv(path, streams, nodes=None):
    """Write {node: streams} as the canonical long-format combined CSV."""
    nodes = nodes or list(streams.keys())
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(SCHEMA_COLS)
        for node in nodes:
            d = streams[node]
            t, a, g, m = d["t"], d["acc"], d["gyr"], d["mag"]
            for i in range(len(t)):
                w.writerow([node, f"{t[i]:.6f}",
                            f"{a[i,0]:.6f}", f"{a[i,1]:.6f}", f"{a[i,2]:.6f}",
                            f"{g[i,0]:.6g}", f"{g[i,1]:.6g}", f"{g[i,2]:.6g}",
                            f"{m[i,0]:.6g}", f"{m[i,1]:.6g}", f"{m[i,2]:.6g}"])


# --------------------------------------------------------------------------- #
def create_session(cfg, streams, *, session_id, subject="", date="", task="",
                   height_m=None, source_files=None, overwrite=False):
    """Materialise a session folder from per-node streams; return its path + metadata."""
    sdir = os.path.join(data_root(cfg), session_id)
    if os.path.exists(sdir) and not overwrite:
        return sdir, read_metadata(sdir)
    if os.path.exists(sdir) and overwrite:
        shutil.rmtree(sdir)
    nodes = list(streams.keys())
    write_combined_csv(os.path.join(sdir, "raw", "data.csv"), streams, nodes)
    topo = default_joint_topology(nodes)
    fs = float(1.0 / np.median(np.diff(streams[topo["foot"] or nodes[0]]["t"])))
    dur = float(min(streams[n]["t"][-1] - streams[n]["t"][0] for n in nodes))
    meta = {
        "session_id": session_id, "subject": subject, "date": date, "task": task,
        "side": topo["side"], "nodes": nodes,
        "node_labels": {n: NODE_INFO.get(n, (n, ""))[0] for n in nodes},
        "joints": topo["joints"], "foot_node": topo["foot"], "pelvis_node": topo["pelvis"],
        "fs_hz": round(fs, 3), "duration_s": round(dur, 2),
        "height_m": float(height_m) if height_m else cfg["anthropometry"]["default_height_m"],
        "n_samples": {n: int(len(streams[n]["t"])) for n in nodes},
        "source_files": source_files or [], "imported_at": datetime.now().isoformat(timespec="seconds"),
        "has_results": False,
    }
    json.dump(meta, open(os.path.join(sdir, "session.json"), "w"), indent=2)
    return sdir, meta


def import_csv(cfg, src_paths, *, session_id=None, subject="", task="", height_m=None,
               overwrite=False):
    """Import one combined CSV or several per-node CSVs into a new session."""
    if isinstance(src_paths, str):
        src_paths = [src_paths]
    streams = _merge([parse_imu_csv(p) for p in src_paths])
    if not streams:
        raise ValueError("no IMU rows parsed from the selected file(s)")
    if not session_id:
        stem = os.path.splitext(os.path.basename(src_paths[0]))[0]
        session_id = f"{subject + '_' if subject else ''}{stem}_{datetime.now():%Y%m%d_%H%M%S}"
    session_id = "".join(c if (c.isalnum() or c in "._-") else "_" for c in session_id)
    return create_session(cfg, streams, session_id=session_id, subject=subject, task=task,
                          height_m=height_m, source_files=[os.path.basename(p) for p in src_paths],
                          overwrite=overwrite)


# --------------------------------------------------------------------------- #
def read_metadata(sdir):
    p = os.path.join(sdir, "session.json")
    if not os.path.exists(p):
        return None
    meta = json.load(open(p))
    meta["has_results"] = os.path.exists(os.path.join(sdir, "results", "summary.json"))
    meta["path"] = sdir
    return meta


def list_sessions(cfg):
    root = data_root(cfg)
    out = []
    for name in sorted(os.listdir(root)):
        sdir = os.path.join(root, name)
        if os.path.isdir(sdir):
            m = read_metadata(sdir)
            if m:
                out.append(m)
    return out


def load_session_streams(sdir):
    """Load a session's raw IMU back into {node: {t, acc, gyr, mag}} + metadata."""
    streams = parse_imu_csv(os.path.join(sdir, "raw", "data.csv"))
    return streams, read_metadata(sdir)


def results_dir(sdir):
    return os.path.join(sdir, "results")


def load_results(sdir):
    """Return (timeseries recarray, summary dict) or (None, None) if not computed."""
    rd = results_dir(sdir)
    ts_p = os.path.join(rd, "timeseries.csv"); sj_p = os.path.join(rd, "summary.json")
    if not (os.path.exists(ts_p) and os.path.exists(sj_p)):
        return None, None
    d = np.genfromtxt(ts_p, delimiter=",", names=True)
    summary = json.load(open(sj_p))
    return d, summary


# --------------------------------------------------------------------------- #
def _slice_streams(slice_dir, nodes):
    """Read Geneva per-node slice CSVs (t_opt_s + ax..mz) for the given nodes."""
    streams = {}
    for n in nodes:
        p = os.path.join(slice_dir, f"{n}.csv")
        if os.path.exists(p):
            s = parse_imu_csv(p, node_hint=n)
            streams[n] = s[n]
    return streams


def ensure_test_sessions(cfg):
    """Convert the bundled Geneva slices into app sessions (idempotent). Returns ids."""
    created = []
    specs = [
        ("data/P01_S01_2minWalk", "P01", "S01", "2minWalk", "right", ["RF", "RS", "RT", "SA"]),
        ("data/P01_S01_2minWalk", "P01", "S01", "2minWalk", "left", ["LF", "LS", "LT", "SA"]),
        ("data/P02_S01_2minWalk", "P02", "S01", "2minWalk", "right", ["RF", "RS", "RT", "SA"]),
    ]
    for rel, subj, sess, task, side, nodes in specs:
        slice_dir = os.path.join(ROOT, rel)
        if not all(os.path.exists(os.path.join(slice_dir, f"{n}.csv")) for n in nodes):
            continue
        sid = f"{subj}_{sess}_{task}_{side}"
        sdir = os.path.join(data_root(cfg), sid)
        if os.path.exists(os.path.join(sdir, "session.json")):
            created.append(sid); continue
        streams = _slice_streams(slice_dir, nodes)
        if len(streams) < 2:
            continue
        date = _read_slice_date(slice_dir)
        height = _read_height_mm(subj)
        create_session(cfg, streams, session_id=sid, subject=subj, task=task, date=date,
                       height_m=(height / 1000.0 if height else None),
                       source_files=[f"{n}.csv (Geneva slice)" for n in streams], overwrite=True)
        created.append(sid)
    return created


def _read_slice_date(slice_dir):
    try:
        rep = json.load(open(os.path.join(slice_dir, "extract_report.json")))
        rtc = next(iter(rep.get("sensors", {}).values())).get("rtc", "")
        return rtc.split(" ")[0] if rtc else ""
    except Exception:
        return ""


def _read_height_mm(subject):
    """Subject stature (mm) from the dataset Inputs sheet, if the dataset is present."""
    try:
        import glob, openpyxl
        root = "Human gait and other movements - markers inertial sensors pressure insoles force plates/researchdata"
        hits = glob.glob(os.path.join(ROOT, root, "*Supplementary File 1*"))
        if not hits:
            return None
        ws = openpyxl.load_workbook(hits[0], read_only=True, data_only=True)["Inputs"]
        rows = ws.iter_rows(values_only=True)
        header = [str(c).lower() if c else "" for c in next(rows)]
        hcol = next((i for i, h in enumerate(header) if "height" in h), 7)
        for r in rows:
            if r and str(r[0]).strip() == subject:
                return float(r[hcol])
    except Exception:
        return None
    return None


if __name__ == "__main__":
    cfg = app_config()
    ids = ensure_test_sessions(cfg)
    print("data root:", data_root(cfg))
    for m in list_sessions(cfg):
        print(f"  {m['session_id']:28s} subj={m['subject']} side={m['side']} "
              f"nodes={m['nodes']} dur={m['duration_s']}s fs={m['fs_hz']}")
