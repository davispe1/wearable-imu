"""
app/ui_results.py — Results view: consolidated parameters table + overlaid gait cycles
+ export (CSV / JSON / text report) into the session folder.
"""
from __future__ import annotations
import csv, json, os
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from app.analysis import LoadedSession, gait_cycle_params, overlay_cycles, JCOLOR


class ResultsView(QtWidgets.QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.ls = None
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        self.title = QtWidgets.QLabel("<h2>Results</h2><i>load a session from the Sessions tab</i>")
        v.addWidget(self.title)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        v.addWidget(split, 1)

        # left: tables
        left = QtWidgets.QWidget(); lv = QtWidgets.QVBoxLayout(left)
        lv.addWidget(QtWidgets.QLabel("<b>Per-joint parameters</b>"))
        self.tbl = QtWidgets.QTableWidget(0, 6)
        self.tbl.setHorizontalHeaderLabels(["Joint", "ROM full°", "ROM steady°",
                                            "peak min°", "peak max°", "peak |ω|°/s"])
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        lv.addWidget(self.tbl)
        lv.addWidget(QtWidgets.QLabel("<b>Gait parameters</b>"))
        self.gtbl = QtWidgets.QTableWidget(0, 2)
        self.gtbl.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.gtbl.horizontalHeader().setStretchLastSection(True)
        self.gtbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        lv.addWidget(self.gtbl)
        split.addWidget(left)

        # right: overlaid cycles
        right = QtWidgets.QWidget(); rv = QtWidgets.QVBoxLayout(right)
        rv.addWidget(QtWidgets.QLabel("<b>Overlaid gait cycles</b> (steady strides, 0–100 %)"))
        self.cycles = pg.GraphicsLayoutWidget(); self.cycles.setBackground("w")
        rv.addWidget(self.cycles, 1)
        split.addWidget(right)
        split.setSizes([520, 720])

        row = QtWidgets.QHBoxLayout(); row.addStretch(1)
        self.btn_csv = QtWidgets.QPushButton("Export CSV")
        self.btn_json = QtWidgets.QPushButton("Export JSON")
        self.btn_rep = QtWidgets.QPushButton("Export report (.txt)")
        self.btn_o3d = QtWidgets.QPushButton("Export for Open3D")
        self.btn_o3d.setToolTip("Write gait_frames.csv + meta.json for the external Open3D "
                                "viewer (Open3D is not bundled with this app).")
        for b in (self.btn_csv, self.btn_json, self.btn_rep, self.btn_o3d):
            row.addWidget(b)
        v.addLayout(row)
        self.status = QtWidgets.QLabel(""); self.status.setStyleSheet("color:#456"); v.addWidget(self.status)
        self.btn_csv.clicked.connect(lambda: self.export("csv"))
        self.btn_json.clicked.connect(lambda: self.export("json"))
        self.btn_rep.clicked.connect(lambda: self.export("report"))
        self.btn_o3d.clicked.connect(self.export_open3d)

    # -- load ---------------------------------------------------------------- #
    def load(self, sdir):
        self.ls = LoadedSession(sdir)
        m = self.ls.meta
        self.title.setText(f"<h2>Results — {m['session_id']}</h2>"
                           f"<span style='color:#666'>{m.get('subject','?')} · {m.get('side','')} leg · "
                           f"{self.ls.t[-1]:.0f}s</span>")
        self._fill_tables()
        self._plot_cycles()

    def _fill_tables(self):
        pj = self.summary_per_joint()
        self.tbl.setRowCount(len(pj))
        for r, (j, d) in enumerate(pj.items()):
            vals = [j, f"{d['rom_deg_full']:.1f}", f"{d['rom_deg_steady']:.1f}",
                    f"{d['peak_min_deg']:.1f}", f"{d['peak_max_deg']:.1f}", f"{d['peak_abs_vel_dps']:.0f}"]
            for c, val in enumerate(vals):
                self.tbl.setItem(r, c, QtWidgets.QTableWidgetItem(str(val)))

        g = self.ls.summary.get("gait", {})
        gc = gait_cycle_params(self.ls)
        items = [
            ("cadence (steps/min)", f"{g.get('cadence_steps_per_min', float('nan')):.1f}"),
            ("stride time (s)", f"{g.get('stride_time_mean_s', float('nan')):.2f} ± {g.get('stride_time_std_s', float('nan')):.2f}"),
            ("step time (s)", f"{gc['step_time_s']:.2f}"),
            ("stride variability (CV %)", f"{gc['stride_cv_pct']:.1f}"),
            ("stance / swing (%)", f"{gc['stance_pct']:.0f} / {gc['swing_pct']:.0f}"
             if gc['stance_pct'] == gc['stance_pct'] else "—"),
            ("steady strides", f"{g.get('n_steady_strides', '?')}"),
            ("foot strikes", f"{g.get('n_foot_strikes', '?')}"),
            ("turnarounds", f"{self.ls.summary.get('n_turnarounds', 0)}"),
            ("active duration (s)", f"{self._active_duration():.0f}"),
        ]
        self.gtbl.setRowCount(len(items))
        for r, (k, val) in enumerate(items):
            self.gtbl.setItem(r, 0, QtWidgets.QTableWidgetItem(k))
            self.gtbl.setItem(r, 1, QtWidgets.QTableWidgetItem(val))

    def summary_per_joint(self):
        """Per-joint table built from display angles (consistent with the curves)."""
        out = {}
        for j in self.ls.joints:
            disp = self.ls.disp[j]; vel = self.ls.vel[j]; st = self.ls.steady
            out[j] = {
                "rom_deg_full": float(np.nanmax(disp) - np.nanmin(disp)),
                "rom_deg_steady": float(np.nanmax(disp[st]) - np.nanmin(disp[st])) if st.any() else float("nan"),
                "peak_min_deg": float(np.nanmin(disp)),
                "peak_max_deg": float(np.nanmax(disp)),
                "peak_abs_vel_dps": float(np.nanmax(np.abs(vel))),
            }
        return out

    def _active_duration(self):
        return float(self.ls.steady.sum() * self.ls.dt)

    def _plot_cycles(self):
        self.cycles.clear()
        for r, j in enumerate(self.ls.joints):
            p = self.cycles.addPlot(row=r, col=0, title=f"{j} flexion")
            p.showGrid(x=True, y=True, alpha=0.2)
            if r == len(self.ls.joints) - 1:
                p.setLabel("bottom", "gait cycle (%)")
            grid, cyc, mean = overlay_cycles(self.ls, j)
            c = JCOLOR.get(j, (80, 80, 80))
            for arr in cyc:
                p.plot(grid, arr, pen=pg.mkPen(c + (45,), width=1))
            if mean is not None:
                p.plot(grid, mean, pen=pg.mkPen((20, 20, 20), width=2.2))

    # -- export -------------------------------------------------------------- #
    def export(self, kind):
        if not self.ls:
            return
        out_dir = os.path.join(self.ls.sdir, "results")
        os.makedirs(out_dir, exist_ok=True)
        pj = self.summary_per_joint()
        g = self.ls.summary.get("gait", {})
        gc = gait_cycle_params(self.ls)
        if kind == "csv":
            p = os.path.join(out_dir, "parameters.csv")
            with open(p, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["joint", "rom_full_deg", "rom_steady_deg", "peak_min_deg",
                            "peak_max_deg", "peak_abs_vel_dps"])
                for j, d in pj.items():
                    w.writerow([j, f"{d['rom_deg_full']:.3f}", f"{d['rom_deg_steady']:.3f}",
                                f"{d['peak_min_deg']:.3f}", f"{d['peak_max_deg']:.3f}",
                                f"{d['peak_abs_vel_dps']:.3f}"])
                w.writerow([])
                w.writerow(["gait", "value"])
                for k, val in (("cadence_steps_per_min", g.get("cadence_steps_per_min")),
                               ("stride_time_mean_s", g.get("stride_time_mean_s")),
                               ("stride_time_std_s", g.get("stride_time_std_s")),
                               ("stride_cv_pct", gc["stride_cv_pct"]),
                               ("stance_pct", gc["stance_pct"]), ("swing_pct", gc["swing_pct"]),
                               ("n_steady_strides", g.get("n_steady_strides")),
                               ("active_duration_s", self._active_duration())):
                    w.writerow([k, val])
        elif kind == "json":
            p = os.path.join(out_dir, "parameters.json")
            json.dump({"session": self.ls.meta["session_id"], "per_joint": pj,
                       "gait": g, "gait_cycle": gc,
                       "active_duration_s": self._active_duration()},
                      open(p, "w"), indent=2)
        else:
            p = os.path.join(out_dir, "report.txt")
            with open(p, "w", encoding="utf-8") as f:
                m = self.ls.meta
                f.write(f"Gait Kinematics — session report\n{'='*40}\n")
                f.write(f"session : {m['session_id']}\nsubject : {m.get('subject','?')}  "
                        f"side {m.get('side','')}\nduration: {self.ls.t[-1]:.0f} s @ {self.ls.fs:.0f} Hz\n\n")
                f.write("Per-joint (display/anatomical reference):\n")
                for j, d in pj.items():
                    f.write(f"  {j:6s} ROM {d['rom_deg_full']:5.1f}° (steady {d['rom_deg_steady']:5.1f}°)  "
                            f"range [{d['peak_min_deg']:+.0f},{d['peak_max_deg']:+.0f}]°  "
                            f"peak|ω| {d['peak_abs_vel_dps']:.0f}°/s\n")
                f.write(f"\nGait:\n  cadence {g.get('cadence_steps_per_min', float('nan')):.1f} steps/min\n"
                        f"  stride  {g.get('stride_time_mean_s', float('nan')):.2f} ± "
                        f"{g.get('stride_time_std_s', float('nan')):.2f} s  (CV {gc['stride_cv_pct']:.1f}%)\n"
                        f"  stance/swing  {gc['stance_pct']:.0f}/{gc['swing_pct']:.0f} %\n"
                        f"  steady strides {g.get('n_steady_strides','?')}  "
                        f"strikes {g.get('n_foot_strikes','?')}  turns {self.ls.summary.get('n_turnarounds',0)}\n")
        self.status.setText(f"exported → {os.path.relpath(p, self.ls.sdir)}")

    def export_open3d(self):
        """Write the external Open3D viewer's inputs (gait_frames.csv + meta.json).

        Uses the *gaitlib* sagittal flexion stored in the results timeseries
        (``<joint>_deg`` = ``results.joints[<joint>]["flexion"]``) for the single leg —
        not the 3D orientation. Open3D is not a dependency; this only writes the files.
        """
        if not self.ls:
            QtWidgets.QMessageBox.warning(self, "Export for Open3D",
                                          "Load a session first (Sessions tab).")
            return
        try:
            from pipeline.export_open3d import write_open3d_inputs, infer_leg
            names = self.ls.d.dtype.names
            angles = {j: np.asarray(self.ls.d[f"{j}_deg"], float)
                      for j in ("hip", "knee", "ankle") if f"{j}_deg" in names}
            leg = infer_leg(side=self.ls.meta.get("side"),
                            foot_node=self.ls.meta.get("foot_node"))
            out_dir = os.path.join(self.ls.sdir, "results")
            frames, meta_p, meta = write_open3d_inputs(
                out_dir, self.ls.t, angles, leg=leg, fs=self.ls.fs,
                height_m=self.ls.meta.get("height_m"))
        except Exception as e:
            self.status.setText(f"Export for Open3D failed: {e}")
            QtWidgets.QMessageBox.critical(
                self, "Export for Open3D failed", f"{type(e).__name__}: {e}")
            return
        frames_abs, meta_abs = os.path.abspath(frames), os.path.abspath(meta_p)
        self.status.setText(
            f"exported → {frames_abs} + meta.json "
            f"({meta['n_frames']} frames, {leg} leg) for Open3D")
        QtWidgets.QMessageBox.information(
            self, "Export for Open3D",
            f"Wrote Open3D viewer inputs ({meta['n_frames']} frames, {leg} leg):\n\n"
            f"{frames_abs}\n{meta_abs}")
