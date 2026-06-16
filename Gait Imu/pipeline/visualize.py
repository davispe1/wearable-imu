"""
visualize.py — one interactive window: a lateral (sagittal) mannequin animated from the
CLEAN yaw-immune joint angles, beside synchronised gait panels that a time-cursor sweeps in
lockstep.

Why this design (see docs/user_guide.md §4):
  * The figure is driven by the validated sagittal ankle/knee/hip FLEXION angles in a lateral
    view — NOT the full 3D orientation (whose drifting yaw made the old view look wrong).
  * Playback advances by real wall-clock time mapped through the data's t_s (256 Hz), so 1.0x
    is real-time regardless of render rate (the old view stepped a fixed N samples per frame).
  * The root is orientation-only (fixed) by construction; an OPTIONAL procedural forward
    translation (cadence x step length) walks the figure across the view instead of in place.

Modes:
  * Interactive (default): single window = mannequin (left) + gait panels (right) + a static
    sensor-placement diagram, one QTimer, play/pause + speed (0.25/0.5/1x) + loop.
  * --save: matplotlib stills (dashboard + stick-figure montage + sensor-placement PNG); works
    headless and is the automatic fallback if the Qt/OpenGL stack is unavailable.
  * --shot <png>: render the live window once (offscreen-friendly) and save a screenshot.

Data come from the outputs CSV/JSON written by run.py; segment lengths are scaled from the
subject's stature (dataset "Inputs" sheet, or config override).
"""
from __future__ import annotations
import argparse, json, os
import numpy as np

# ----------------------------------------------------------------------------- #
# Pleasant lateral-mannequin palette.
COL_THIGH = (76, 114, 176)      # blue
COL_SHANK = (85, 168, 104)      # green
COL_FOOT  = (196, 78, 82)       # red
COL_TRUNK = (129, 114, 179)     # purple
COL_JOINT = (40, 40, 45)        # joint pivots
COL_SENSOR = (232, 176, 7)      # RF/RS/RT/SA dots (gold)
COL_SILH  = (150, 150, 160)     # faint context silhouette


# ----------------------------------------------------------------------------- #
def load(cfg_base):
    csv = cfg_base + "_timeseries.csv"
    js = cfg_base + "_summary.json"
    d = np.genfromtxt(csv, delimiter=",", names=True)
    summary = json.load(open(js)) if os.path.exists(js) else {}
    return d, summary


def read_subject_height_mm(cfg):
    """Stature (mm) from the dataset 'Inputs' sheet (Supplementary File 1), by subject.

    Returns None if the file/row cannot be read; the caller falls back to a default.
    """
    root = cfg.get("dataset", {}).get("root", "")
    subj = cfg.get("dataset", {}).get("subject", "")
    try:
        import glob, openpyxl
        hits = glob.glob(os.path.join(root, "*Supplementary File 1*"))
        if not hits:
            return None
        wb = openpyxl.load_workbook(hits[0], read_only=True, data_only=True)
        ws = wb["Inputs"]
        rows = ws.iter_rows(values_only=True)
        header = [str(c).lower() if c else "" for c in next(rows)]
        hcol = next((i for i, h in enumerate(header) if "height" in h), 7)
        for r in rows:
            if r and str(r[0]).strip() == subj:
                return float(r[hcol])
    except Exception:
        return None
    return None


def resolve_anthropometry(cfg):
    """Segment lengths (m) scaled from subject stature; honours config overrides."""
    v = cfg.get("visualization", {})
    a = v.get("anthropometry", {})
    h_mm = a.get("height_mm") or read_subject_height_mm(cfg) or 1700.0
    H = float(h_mm) / 1000.0
    return {
        "H": H,
        "thigh":  a.get("thigh_frac", 0.245) * H,
        "shank":  a.get("shank_frac", 0.246) * H,
        "foot":   a.get("foot_frac", 0.152) * H,
        "pelvis": a.get("pelvis_frac", 0.100) * H,
        "trunk":  a.get("trunk_frac", 0.288) * H,
        "step":   v.get("step_length_frac", 0.45) * H,
    }


# ----------------------------------------------------------------------------- #
def leg_chain(hip_deg, knee_deg, ankle_deg, L, root=(0.0, 0.0)):
    """Sagittal joint positions for one frame, lateral plane (x forward, y up).

    Returns a dict of (x,y) points: pelvis_top, hip, knee, ankle, toe, shoulder.
    Convention matches validation: thigh inclined by hip flexion (+forward), knee flexes the
    shank backward, foot ~perpendicular modulated by ankle flexion.
    """
    rx, ry = root
    hip = np.array([rx, ry])
    th = np.radians(hip_deg)                       # thigh angle from vertical
    knee = hip + L["thigh"] * np.array([np.sin(th), -np.cos(th)])
    sh = th - np.radians(knee_deg)                 # knee flexes shank backward
    ankle = knee + L["shank"] * np.array([np.sin(sh), -np.cos(sh)])
    ft = sh + np.radians(90.0 - ankle_deg)         # foot ~perpendicular to shank
    toe = ankle + L["foot"] * np.array([np.sin(ft), -np.cos(ft)])
    pelvis_top = hip + np.array([-0.04 * L["H"], L["pelvis"]])
    shoulder = pelvis_top + np.array([0.02 * L["H"], L["trunk"]])
    return {"pelvis": pelvis_top, "hip": hip, "knee": knee,
            "ankle": ankle, "toe": toe, "shoulder": shoulder}


def sensor_points(P):
    """Sensor dot positions on their segments: RF foot, RS shank, RT thigh, SA pelvis."""
    mid = lambda a, b, f=0.5: P[a] * (1 - f) + P[b] * f
    return {
        "RF": mid("ankle", "toe", 0.55),
        "RS": mid("knee", "ankle", 0.55),
        "RT": mid("hip", "knee", 0.5),
        "SA": P["pelvis"] + np.array([-0.01, -0.02]),
    }


def silhouette_polygon(L):
    """A faint lateral body outline (head + torso + standing leg), origin at the hip.

    Coarse side profile for context only; translates with the figure root.
    """
    H = L["H"]; tr = L["trunk"]; pv = L["pelvis"]
    head_r = 0.065 * H
    neck_y = pv + tr
    # closed side profile: back of pelvis -> spine -> back of head -> face -> chest -> belly
    pts = [
        (-0.09 * H, 0.0),
        (-0.10 * H, 0.45 * tr),
        (-0.085 * H, 0.85 * tr),
        (-0.07 * H, neck_y - head_r * 0.4),
        (-0.05 * H, neck_y + head_r * 0.7),     # back of head
        (0.02 * H,  neck_y + head_r * 1.6),     # crown
        (0.09 * H,  neck_y + head_r * 0.7),     # forehead
        (0.085 * H, neck_y - head_r * 0.3),     # face/chin
        (0.11 * H,  0.78 * tr),                 # chest
        (0.12 * H,  0.30 * tr),                 # belly
        (0.10 * H,  0.0),                       # front of pelvis
    ]
    return np.array(pts + [pts[0]])


# ----------------------------------------------------------------------------- #
def _round_pen(color, width):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore
    pen = pg.mkPen(color=color, width=width)
    pen.setCapStyle(QtCore.Qt.RoundCap)
    pen.setJoinStyle(QtCore.Qt.RoundJoin)
    return pen


def draw_mannequin(plot, P, S, L, sil_items, seg_items, joint_item, sensor_item,
                   sensor_labels):
    """Update an already-built mannequin (segments/joints/sensors/silhouette) in place."""
    import numpy as np
    seg_items["thigh"].setData([P["hip"][0], P["knee"][0]], [P["hip"][1], P["knee"][1]])
    seg_items["shank"].setData([P["knee"][0], P["ankle"][0]], [P["knee"][1], P["ankle"][1]])
    seg_items["foot"].setData([P["ankle"][0], P["toe"][0]], [P["ankle"][1], P["toe"][1]])
    seg_items["trunk"].setData([P["hip"][0], P["shoulder"][0]], [P["hip"][1], P["shoulder"][1]])
    jx = [P[k][0] for k in ("hip", "knee", "ankle")]
    jy = [P[k][1] for k in ("hip", "knee", "ankle")]
    joint_item.setData(jx, jy)
    sx = [S[k][0] for k in ("RF", "RS", "RT", "SA")]
    sy = [S[k][1] for k in ("RF", "RS", "RT", "SA")]
    sensor_item.setData(sx, sy)
    for k, lab in sensor_labels.items():
        lab.setPos(S[k][0] + 0.02, S[k][1] + 0.02)
    sil = silhouette_polygon(L) + P["hip"]
    sil_items.setData(sil[:, 0], sil[:, 1])


def build_mannequin(plot, L, with_labels=True, head=True):
    """Create the mannequin items on a PlotWidget; return handles for updating."""
    import pyqtgraph as pg
    plot.setAspectLocked(True)
    plot.hideButtons(); plot.setMenuEnabled(False)
    plot.getAxis("left").setStyle(showValues=False)
    plot.getAxis("bottom").setStyle(showValues=False)

    sil = pg.PlotDataItem(pen=pg.mkPen(COL_SILH + (60,), width=1),
                          fillLevel=None, fillBrush=COL_SILH + (35,))
    sil.setFillLevel(0); sil.setBrush(pg.mkBrush(COL_SILH + (35,)))
    plot.addItem(sil)
    if head:
        head_dot = pg.ScatterPlotItem(size=0.13 * L["H"], pxMode=False,
                                      brush=pg.mkBrush(COL_SILH + (55,)),
                                      pen=pg.mkPen(COL_SILH + (90,), width=1))
        plot.addItem(head_dot)
    else:
        head_dot = None

    seg = {}
    for name, col, w in (("trunk", COL_TRUNK, 0.075), ("thigh", COL_THIGH, 0.085),
                         ("shank", COL_SHANK, 0.075), ("foot", COL_FOOT, 0.060)):
        px_w = max(10, w * L["H"] * 220)   # thickness scaled to stature, in px
        it = pg.PlotDataItem(pen=_round_pen(col, px_w))
        plot.addItem(it); seg[name] = it

    joints = pg.ScatterPlotItem(size=16, brush=pg.mkBrush(COL_JOINT),
                                pen=pg.mkPen("w", width=2))
    sensors = pg.ScatterPlotItem(size=13, symbol="s", brush=pg.mkBrush(COL_SENSOR),
                                 pen=pg.mkPen((60, 45, 0), width=1.5))
    plot.addItem(joints); plot.addItem(sensors)

    labels = {}
    if with_labels:
        for k in ("RF", "RS", "RT", "SA"):
            lab = pg.TextItem(k, color=(90, 70, 0), anchor=(0, 1))
            f = lab.textItem.font(); f.setPointSize(8); f.setBold(True)
            lab.setFont(f); plot.addItem(lab); labels[k] = lab
    return {"sil": sil, "head": head_dot, "seg": seg, "joints": joints,
            "sensors": sensors, "labels": labels}


# ----------------------------------------------------------------------------- #
def show_window(d, summary, cfg, base, shot_path=None):
    """The single combined interactive window (mannequin + synced gait panels)."""
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    import time as _time

    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    L = resolve_anthropometry(cfg)
    t = d["t_opt_s"].astype(float)
    n = len(t); t_end = float(t[-1])
    joints = ["hip", "knee", "ankle"]
    jcol = {"hip": COL_THIGH, "knee": COL_SHANK, "ankle": COL_FOOT}

    vcfg = cfg.get("visualization", {})
    speed0 = float(vcfg.get("default_speed", 1.0))
    fps = int(vcfg.get("fps", 60))
    do_loop = bool(vcfg.get("loop", True))
    fwd = bool(vcfg.get("forward_translation", True))
    subj = summary.get("subject", cfg.get("dataset", {}).get("subject", "?"))

    # forward walking speed from cadence x step length (procedural translation)
    cad = summary.get("gait", {}).get("cadence_steps_per_min", 0.0) or 0.0
    fwd_speed = (cad / 60.0) * L["step"] if cad else 1.2     # m/s
    ground_y = -(L["thigh"] + L["shank"]) - 0.03

    # ---- window scaffold: splitter (mannequin | panels) + controls ------------ #
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(f"Gait Kinematics — {subj} lateral gait (clean sagittal)")
    central = QtWidgets.QWidget(); win.setCentralWidget(central)
    vbox = QtWidgets.QVBoxLayout(central)
    split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
    vbox.addWidget(split, 1)

    # left column: animated mannequin (top) + static placement diagram (bottom)
    left = QtWidgets.QWidget(); lv = QtWidgets.QVBoxLayout(left); lv.setContentsMargins(0, 0, 0, 0)
    mann = pg.PlotWidget(title="Lateral mannequin — driven by clean sagittal flexion")
    mann.addItem(pg.InfiniteLine(pos=ground_y, angle=0,
                                 pen=pg.mkPen((180, 180, 180), width=2,
                                              style=QtCore.Qt.DashLine)))
    h = build_mannequin(mann, L, with_labels=True)
    lv.addWidget(mann, 3)
    place = pg.PlotWidget(title="Sensor placement (RF foot · RS shank · RT thigh · SA pelvis)")
    build_placement_plot(place, L)
    lv.addWidget(place, 2)
    split.addWidget(left)

    # right column: gait panels
    panels = pg.GraphicsLayoutWidget()
    split.addWidget(panels)
    split.setSizes([560, 760])

    p_ang = panels.addPlot(row=0, col=0, title="Joint flexion (deg)")
    p_ang.addLegend(offset=(10, 5)); p_ang.showGrid(x=True, y=True, alpha=0.2)
    for j in joints:
        p_ang.plot(t, d[f"{j}_deg"], pen=pg.mkPen(jcol[j], width=1.5), name=j)
    cur_ang = pg.InfiniteLine(angle=90, pen=pg.mkPen((20, 20, 20), width=1.5))
    p_ang.addItem(cur_ang)

    p_vel = panels.addPlot(row=1, col=0, title="Joint angular velocity (deg/s)")
    p_vel.showGrid(x=True, y=True, alpha=0.2); p_vel.setXLink(p_ang)
    for j in joints:
        p_vel.plot(t, d[f"{j}_vel_dps"], pen=pg.mkPen(jcol[j], width=1.0))
    cur_vel = pg.InfiniteLine(angle=90, pen=pg.mkPen((20, 20, 20), width=1.5))
    p_vel.addItem(cur_vel)

    p_foot = panels.addPlot(row=2, col=0, title="Foot accel magnitude + strikes")
    p_foot.showGrid(x=True, y=True, alpha=0.2); p_foot.setXLink(p_ang)
    p_foot.plot(t, d["foot_acc_mag"], pen=pg.mkPen((60, 60, 60), width=0.8))
    fs = np.where(d["foot_strike"] > 0)[0]
    p_foot.plot(t[fs], d["foot_acc_mag"][fs], pen=None, symbol="t",
                symbolBrush=COL_FOOT, symbolSize=9, name="foot strike")
    cur_ft = pg.InfiniteLine(angle=90, pen=pg.mkPen((20, 20, 20), width=1.5))
    p_foot.addItem(cur_ft)
    p_foot.setLabel("bottom", "time (s)")

    # metrics box
    g = summary.get("gait", {}); rw = summary.get("joint_rom_in_window_deg", {})
    rmse = summary.get("validation_rmse_deg", {}).get("6dof", {})
    metrics = (
        "<table cellpadding=3 style='font-family:monospace;font-size:10pt'>"
        f"<tr><td><b>subject</b></td><td>{subj} / {summary.get('task','')}"
        f" ({summary.get('duration_s',0):.0f} s, H={L['H']:.2f} m)</td></tr>"
        f"<tr><td><b>cadence</b></td><td>{g.get('cadence_steps_per_min',float('nan')):.1f} steps/min</td></tr>"
        f"<tr><td><b>stride</b></td><td>{g.get('stride_time_mean_s',float('nan')):.2f}"
        f" &plusmn; {g.get('stride_time_std_s',float('nan')):.2f} s</td></tr>"
        f"<tr><td><b>steps</b></td><td>{g.get('n_foot_strikes','?')} strikes,"
        f" {g.get('n_steady_strides','?')} steady strides</td></tr>"
        "<tr><td><b>ROM (window)</b></td><td>"
        + " ".join(f"{j} {rw.get(j,{}).get('computed',float('nan')):.0f}&deg;" for j in ["ankle","knee","hip"])
        + "</td></tr>"
        "<tr><td><b>RMSE vs opt</b></td><td>"
        + " ".join(f"{j} {rmse.get(j,float('nan')):.1f}&deg;" for j in ["ankle","knee","hip"])
        + "</td></tr></table>"
    )
    lbl = panels.addLabel(metrics, row=3, col=0, justify="left")

    # ---- controls -------------------------------------------------------------- #
    ctl = QtWidgets.QHBoxLayout(); vbox.addLayout(ctl)
    btn_play = QtWidgets.QPushButton("⏸ Pause")
    cmb_speed = QtWidgets.QComboBox(); cmb_speed.addItems(["0.25x", "0.5x", "1x"])
    cmb_speed.setCurrentText("1x" if speed0 >= 1 else ("0.5x" if speed0 >= 0.5 else "0.25x"))
    chk_loop = QtWidgets.QCheckBox("loop"); chk_loop.setChecked(do_loop)
    lbl_t = QtWidgets.QLabel("t = 0.00 s")
    ctl.addWidget(btn_play); ctl.addWidget(QtWidgets.QLabel("speed:")); ctl.addWidget(cmb_speed)
    ctl.addWidget(chk_loop); ctl.addStretch(1); ctl.addWidget(lbl_t)

    # ---- playback clock -------------------------------------------------------- #
    st = {"playing": True, "speed": speed0, "t_anchor": 0.0, "wall": _time.perf_counter()}

    def reanchor(tcur):
        st["t_anchor"] = tcur; st["wall"] = _time.perf_counter()

    def speed_val():
        return {"0.25x": 0.25, "0.5x": 0.5, "1x": 1.0}[cmb_speed.currentText()]

    def on_speed():
        reanchor(current_t()); st["speed"] = speed_val()

    def current_t():
        if not st["playing"]:
            return st["t_anchor"]
        return st["t_anchor"] + (_time.perf_counter() - st["wall"]) * st["speed"]

    def on_play():
        if st["playing"]:
            st["t_anchor"] = current_t(); st["playing"] = False; btn_play.setText("▶ Play")
        else:
            reanchor(st["t_anchor"]); st["playing"] = True; btn_play.setText("⏸ Pause")

    btn_play.clicked.connect(on_play)
    cmb_speed.currentTextChanged.connect(lambda *_: on_speed())

    def update():
        tc = current_t()
        if tc > t_end:
            if chk_loop.isChecked():
                tc = 0.0; reanchor(0.0)
            else:
                tc = t_end; st["t_anchor"] = t_end; st["playing"] = False
                btn_play.setText("▶ Play")
        i = int(np.searchsorted(t, tc))
        i = max(0, min(n - 1, i))
        rx = fwd_speed * tc if fwd else 0.0
        P = leg_chain(d["hip_deg"][i], d["knee_deg"][i], d["ankle_deg"][i], L, root=(rx, 0.0))
        S = sensor_points(P)
        draw_mannequin(mann, P, S, L, h["sil"], h["seg"], h["joints"], h["sensors"], h["labels"])
        if h["head"] is not None:
            h["head"].setData([P["shoulder"][0] + 0.02 * L["H"]],
                              [P["shoulder"][1] + 0.07 * L["H"]])
        mann.setXRange(rx - 0.9, rx + 0.9, padding=0)
        mann.setYRange(ground_y - 0.05, L["pelvis"] + L["trunk"] + 0.18 * L["H"], padding=0)
        for c in (cur_ang, cur_vel, cur_ft):
            c.setPos(tc)
        lbl_t.setText(f"t = {tc:6.2f} / {t_end:.0f} s   ({st['speed']:.2f}x)")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(int(1000 / max(1, fps)))
    update()

    if shot_path:
        # offscreen-friendly: render a couple of frames, then grab the window
        st["t_anchor"] = 0.30 * t_end; st["playing"] = False
        win.resize(1320, 820); win.show()
        for _ in range(8):
            app.processEvents()
        update()
        for _ in range(4):
            app.processEvents()
        win.grab().save(shot_path)
        print(f"Wrote {shot_path}")
        return

    win.resize(1320, 820); win.show()
    (app.exec_ if hasattr(app, "exec_") else app.exec)()


# ----------------------------------------------------------------------------- #
def build_placement_plot(plot, L):
    """Static neutral-pose mannequin annotated with the four sensor locations."""
    import pyqtgraph as pg
    plot.setAspectLocked(True); plot.hideButtons(); plot.setMenuEnabled(False)
    plot.getAxis("left").setStyle(showValues=False)
    plot.getAxis("bottom").setStyle(showValues=False)
    P = leg_chain(0.0, 0.0, 0.0, L, root=(0.0, 0.0))   # neutral standing
    S = sensor_points(P)
    sil = silhouette_polygon(L) + P["hip"]
    poly = pg.PlotDataItem(sil[:, 0], sil[:, 1], pen=pg.mkPen(COL_SILH + (90,), width=1))
    poly.setFillLevel(0); poly.setBrush(pg.mkBrush(COL_SILH + (35,))); plot.addItem(poly)
    for a, b, col, w in (("hip", "knee", COL_THIGH, 0.085), ("knee", "ankle", COL_SHANK, 0.075),
                         ("ankle", "toe", COL_FOOT, 0.060), ("hip", "shoulder", COL_TRUNK, 0.075)):
        it = pg.PlotDataItem([P[a][0], P[b][0]], [P[a][1], P[b][1]],
                             pen=_round_pen(col, max(10, w * L["H"] * 220)))
        plot.addItem(it)
    plot.addItem(pg.ScatterPlotItem([P[k][0] for k in ("hip", "knee", "ankle")],
                                    [P[k][1] for k in ("hip", "knee", "ankle")],
                                    size=14, brush=pg.mkBrush(COL_JOINT), pen=pg.mkPen("w", width=2)))
    plot.addItem(pg.ScatterPlotItem([S[k][0] for k in S], [S[k][1] for k in S],
                                    size=14, symbol="s", brush=pg.mkBrush(COL_SENSOR),
                                    pen=pg.mkPen((60, 45, 0), width=1.5)))
    seg_of = {"RF": "foot", "RS": "shank", "RT": "thigh", "SA": "pelvis"}
    for k in ("RF", "RS", "RT", "SA"):
        lab = pg.TextItem(f"{k} · {seg_of[k]}", color=(70, 55, 0), anchor=(0, 0.5))
        f = lab.textItem.font(); f.setPointSize(8); f.setBold(True); lab.setFont(f)
        lab.setPos(S[k][0] + 0.05, S[k][1]); plot.addItem(lab)
    plot.setXRange(-0.45, 0.55); plot.setYRange(P["toe"][1] - 0.05, P["shoulder"][1] + 0.2 * L["H"])


def save_placement_png(base, L):
    """Static sensor-placement diagram as a standalone PNG (matplotlib, headless)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    P = leg_chain(0.0, 0.0, 0.0, L, root=(0.0, 0.0))
    S = sensor_points(P)
    fig, ax = plt.subplots(figsize=(5.2, 8.4))
    sil = silhouette_polygon(L) + P["hip"]
    ax.fill(sil[:, 0], sil[:, 1], color=(0.6, 0.6, 0.63), alpha=0.18, zorder=0)
    ax.plot(sil[:, 0], sil[:, 1], color=(0.55, 0.55, 0.6), lw=1, zorder=1)
    segs = (("hip", "knee", np.array(COL_THIGH) / 255, 14, "thigh"),
            ("knee", "ankle", np.array(COL_SHANK) / 255, 12, "shank"),
            ("ankle", "toe", np.array(COL_FOOT) / 255, 10, "foot"),
            ("hip", "shoulder", np.array(COL_TRUNK) / 255, 12, "trunk"))
    for a, b, c, w, _ in segs:
        ax.plot([P[a][0], P[b][0]], [P[a][1], P[b][1]], color=c, lw=w,
                solid_capstyle="round", zorder=2)
    for k in ("hip", "knee", "ankle"):
        ax.add_patch(Circle(P[k], 0.018, color=(0.16, 0.16, 0.18), ec="w", lw=1.5, zorder=4))
    seg_of = {"RF": "foot (RF)", "RS": "shank (RS)", "RT": "thigh (RT)", "SA": "pelvis/sacrum (SA)"}
    for k in ("RF", "RS", "RT", "SA"):
        ax.scatter(*S[k], s=120, marker="s", color=np.array(COL_SENSOR) / 255,
                   edgecolor=(0.24, 0.18, 0), lw=1.5, zorder=5)
        ax.annotate(f"{k} — {seg_of[k]}", S[k], xytext=(S[k][0] + 0.07, S[k][1]),
                    va="center", fontsize=10, fontweight="bold", color=(0.27, 0.21, 0),
                    arrowprops=dict(arrowstyle="-", color=(0.5, 0.4, 0), lw=1))
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-0.5, 0.75); ax.set_ylim(P["toe"][1] - 0.06, P["shoulder"][1] + 0.18 * L["H"])
    ax.set_title("Gait Kinematics — IMU sensor placement (lateral)\nright leg: RF · RS · RT · SA",
                 fontsize=11)
    p = base + "_placement.png"
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"Wrote {p}")


# --------------------------------------------------------------------------- #
def save_dashboard(d, summary, base, L):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = d["t_opt_s"]
    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(3, 3)
    joints = ["ankle", "knee", "hip"]
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
        rw = summary.get("joint_rom_in_window_deg", {}).get(j, {})
        ax.text(0.01, 0.92, f"RMSE={rmse:.1f} deg   ROM(window) computed={rw.get('computed',float('nan')):.0f} vs optical={rw.get('optical',float('nan')):.0f}",
                transform=ax.transAxes, fontsize=8, va="top")
    fig.axes[-1].set_xlabel("time (s)")

    axv = fig.add_subplot(gs[0, 2])
    for j in joints:
        axv.plot(t, d[f"{j}_vel_dps"], lw=0.6, label=j)
    axv.set_title("joint angular velocity (deg/s)", fontsize=9); axv.legend(fontsize=7)

    axf = fig.add_subplot(gs[1, 2])
    axf.plot(t, d["foot_acc_mag"], lw=0.5, color="k")
    fs = np.where(d["foot_strike"] > 0)[0]
    axf.plot(t[fs], d["foot_acc_mag"][fs], "rv", ms=4, label="foot strike")
    axf.set_title("foot accel magnitude + steps", fontsize=9); axf.legend(fontsize=7)

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

    fig.suptitle("Gait Kinematics — right-leg gait (6-DOF primary) vs optical reference", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p = base + "_dashboard.png"
    fig.savefig(p, dpi=110); plt.close(fig)
    print(f"Wrote {p}")

    # stick-figure montage over one steady stride (lateral, scaled to anthropometry)
    fig2, ax2 = plt.subplots(figsize=(11, 4))
    steady = np.where(d["steady_state"] > 0)[0]
    i0 = steady[len(steady)//3] if len(steady) else int(len(t)*0.3)
    stride = int(1.1 * 256); nframes = 7
    for k, fi in enumerate(range(i0, i0 + stride, max(1, stride // nframes))):
        P = leg_chain(d["hip_deg"][fi], d["knee_deg"][fi], d["ankle_deg"][fi], L)
        order = ["pelvis", "hip", "knee", "ankle", "toe"]
        xs = [P[o][0] + 0.55*k for o in order]; ys = [P[o][1] for o in order]
        c = plt.cm.viridis(k / nframes)
        ax2.plot(xs, ys, "-o", color=c, ms=5, lw=2.5)
    ax2.set_aspect("equal")
    ax2.set_title("right-leg sagittal stick figure over one steady stride (pelvis->hip->knee->ankle->toe)")
    ax2.set_xlabel("forward (m), frames offset in time ->"); ax2.set_ylabel("up (m)")
    p2 = base + "_stickfigure.png"
    fig2.savefig(p2, dpi=110); plt.close(fig2)
    print(f"Wrote {p2}")


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    ap.add_argument("--save", action="store_true", help="render PNGs (headless)")
    ap.add_argument("--shot", metavar="PNG", help="render the live window once and save a screenshot")
    args = ap.parse_args()
    import yaml
    cfg = yaml.safe_load(open(args.config))
    base = os.path.join(cfg["output"]["dir"],
                        f"{cfg['dataset']['subject']}_{cfg['dataset']['session']}_{cfg['selection']['task']}")
    d, summary = load(base)
    L = resolve_anthropometry(cfg)
    if args.save:
        save_dashboard(d, summary, base, L)
        save_placement_png(base, L)
        return
    if args.shot:
        show_window(d, summary, cfg, base, shot_path=args.shot)
        save_placement_png(base, L)
        return
    try:
        show_window(d, summary, cfg, base)
    except Exception as e:
        print(f"Interactive window unavailable ({e}); writing PNGs instead.")
        save_dashboard(d, summary, base, L)
        save_placement_png(base, L)


if __name__ == "__main__":
    main()
