"""
app/results_gui.py — tkinter GUI for browsing gait kinematics results.

Opens any session folder, runs the pipeline in a background thread, and shows the
result in two tabs:

    Plots       the joint angles, gait-event signal and mean gait cycle, plus an
                at-a-glance parameter panel beside the graphs (the figure)
    Parameters  the full scalar summary as a clean table: ROM, cadence, stride time,
                stance/swing, etc.

The Plots tab is the visual overview (time-series + a quick parameter readout); the
Parameters tab is the detailed, scrollable table. Method references live in the
documentation (docs/kinematics.md), not in this GUI.

    python -m app.results_gui [session_dir]
"""
from __future__ import annotations

import argparse
import os
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from kinematics import analyze_session
from app.viewer import make_figure, _default_window

# Anatomical joint labels for the parameter table.
_JOINT_LABEL = {"hip": "Hip", "knee": "Knee", "ankle": "Ankle"}


# --------------------------------------------------------------------------- #
class ResultsGUI(tk.Tk):
    def __init__(self, session_dir: str | None = None):
        super().__init__()
        self.title("Gait Kinematics — Results Viewer")
        self.geometry("1240x860")
        self.minsize(880, 600)

        self._result = None
        self._fig = None
        self._canvas = None
        self._session_dir: str | None = None
        self._q: queue.Queue = queue.Queue()

        self._build_ui()
        if session_dir:
            self._open(session_dir)

    # ---------------------------------------------------------------------- #
    def _build_ui(self):
        # Top bar
        top = tk.Frame(self, pady=6, padx=10)
        top.pack(side=tk.TOP, fill=tk.X)
        tk.Button(top, text="Open session…", command=self._browse,
                  width=14).pack(side=tk.LEFT, padx=(0, 8))
        self._path_var = tk.StringVar(value="(no session loaded)")
        tk.Label(top, textvariable=self._path_var, anchor="w",
                 fg="#333").pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._recompute_btn = tk.Button(top, text="Recompute", command=self._recompute,
                                         width=10, state=tk.DISABLED)
        self._recompute_btn.pack(side=tk.RIGHT)

        ttk.Separator(self, orient="horizontal").pack(fill=tk.X)

        # Two tabs: Plots | Parameters
        self._nb = ttk.Notebook(self)
        self._nb.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self._plots_tab = tk.Frame(self._nb, bg="#f8f8f8")
        self._params_tab = tk.Frame(self._nb)
        self._nb.add(self._plots_tab, text="  Plots  ")
        self._nb.add(self._params_tab, text="  Parameters  ")

        self._fig_placeholder = tk.Label(
            self._plots_tab,
            text="Open a session folder to see the plots.\n\n"
                 "The folder must contain the per-node CSVs\n"
                 "(RF.csv, RS.csv, RT.csv, SA.csv) or a combined raw/data.csv.",
            fg="#888", font=("Segoe UI", 12), bg="#f8f8f8", justify="center")
        self._fig_placeholder.pack(expand=True)

        self._build_table(self._params_tab)

        # Status bar
        ttk.Separator(self, orient="horizontal").pack(fill=tk.X)
        self._status = tk.StringVar(value="Ready")
        tk.Label(self, textvariable=self._status, anchor="w", fg="#555",
                 font=("Segoe UI", 9), padx=10, pady=3).pack(side=tk.BOTTOM, fill=tk.X)

    def _build_table(self, parent):
        cols = ("group", "parameter", "value")
        frame = tk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self._tree = ttk.Treeview(frame, columns=cols, show="headings",
                                   selectmode="browse")
        for col, head, width, anchor in (
                ("group", "Group", 170, "w"),
                ("parameter", "Parameter", 260, "w"),
                ("value", "Value", 160, "w")):
            self._tree.heading(col, text=head)
            self._tree.column(col, width=width, anchor=anchor, stretch=(col == "value"))
        self._tree.tag_configure("group", background="#eef4fb",
                                 font=("Segoe UI", 10, "bold"))
        scroll_y = ttk.Scrollbar(frame, orient="vertical", command=self._tree.yview)
        self._tree.configure(yscrollcommand=scroll_y.set)
        self._tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)

    # ---------------------------------------------------------------------- #
    def _browse(self):
        d = filedialog.askdirectory(title="Select a session folder")
        if d:
            self._open(d)

    def _open(self, session_dir: str):
        self._session_dir = session_dir
        self._path_var.set(os.path.abspath(session_dir))
        self._run_pipeline(session_dir)

    def _recompute(self):
        if self._session_dir:
            self._run_pipeline(self._session_dir)

    # ---------------------------------------------------------------------- #
    def _run_pipeline(self, session_dir: str):
        self._status.set("Computing…  (this can take a few seconds)")
        self._recompute_btn.config(state=tk.DISABLED)
        self.update_idletasks()
        threading.Thread(target=self._compute, args=(session_dir,),
                         daemon=True).start()
        self.after(150, self._check_queue)

    def _compute(self, session_dir: str):
        try:
            res = analyze_session(session_dir)
            self._q.put(("ok", res))
        except Exception as exc:
            self._q.put(("error", str(exc)))

    def _check_queue(self):
        try:
            kind, payload = self._q.get_nowait()
        except queue.Empty:
            self.after(150, self._check_queue)
            return
        self._recompute_btn.config(state=tk.NORMAL)
        if kind == "ok":
            self._show_results(payload)
        else:
            self._status.set(f"Error: {payload}")
            messagebox.showerror("Could not process the session", payload)

    # ---------------------------------------------------------------------- #
    def _show_results(self, res):
        self._result = res
        self._update_figure(res)
        self._update_table(res)
        modes = ", ".join(sorted(set(res.modes.values()))) if res.modes else "6D"
        self._status.set(
            f"Session: {res.session_id}   leg: {res.side}   "
            f"duration: {res.duration_s:.1f} s   {res.fs:.0f} Hz   VQF {modes}   "
            f"turnarounds: {len(res.turnarounds)}")

    def _update_figure(self, res):
        for widget in self._plots_tab.winfo_children():
            widget.destroy()
        if self._fig is not None:
            plt.close(self._fig)

        t0, t1 = _default_window(res)
        # Keep the at-a-glance parameter panel beside the graphs; drop the references
        # footnote (references live in the docs, and there is a dedicated Parameters tab).
        self._fig = make_figure(res, t0, t1, include_params=True, include_footnote=False)

        toolbar_frame = tk.Frame(self._plots_tab)
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self._canvas = FigureCanvasTkAgg(self._fig, master=self._plots_tab)
        self._canvas.draw()
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self._canvas, toolbar_frame).update()

    def _update_table(self, res):
        for row in self._tree.get_children():
            self._tree.delete(row)

        def group(name):
            self._tree.insert("", "end", values=(name, "", ""), tags=("group",))

        def row(param, val):
            self._tree.insert("", "end", values=("", param, val))

        # Temporal
        tp = res.temporal
        group("Temporal (steady walking)")
        row("Cadence (steps/min)", f"{tp['cadence_steps_per_min']:.1f}")
        row("Stride time (s)",
            f"{tp['stride_time_mean_s']:.3f} ± {tp['stride_time_std_s']:.3f}")
        row("Step time (s)", f"{tp['step_time_s']:.3f}")
        row("Stance (%)", f"{tp['stance_pct']:.1f}")
        row("Swing (%)", f"{tp['swing_pct']:.1f}")
        row("Stride-time CV (%)", f"{tp['stride_time_cv_pct']:.1f}")
        row("Steady strides", f"{tp['n_steady_strides']}")

        # Spatial
        sp = res.spatial
        group("Spatial (foot-ZUPT estimate)")
        row("Stride length (m)", f"~{sp.get('stride_length_m_est', float('nan')):.2f}")
        row("Walking speed (m/s)", f"~{sp.get('walking_speed_mps_est', float('nan')):.2f}")

        # Per joint
        for j in res.joint_names:
            p = res.joints[j]["params"]
            group(f"{_JOINT_LABEL.get(j, j.capitalize())} (cycle-averaged)")
            row("ROM (°)", f"{p.get('rom_cycle_deg', p['rom_deg']):.1f}")
            row("Peak flexion (°)",
                f"{p.get('peak_flexion_cycle_deg', p['peak_flexion_deg']):.1f}")
            row("Peak extension (°)",
                f"{p.get('peak_extension_cycle_deg', p['peak_extension_deg']):.1f}")
            row("Cycles detected", f"{p['cycle_count']}")

        # Session info
        group("Session")
        row("ID", res.session_id)
        row("Measured leg", res.side)
        row("Duration (s)", f"{res.duration_s:.1f}")
        row("Sampling rate (Hz)", f"{res.fs:.0f}")
        row("Turnarounds", f"{len(res.turnarounds)}")
        modes = ", ".join(sorted(set(res.modes.values()))) if res.modes else "6D"
        row("VQF mode", modes)

        if res.warnings:
            group("Warnings")
            for w in res.warnings:
                row("", w)

    # ---------------------------------------------------------------------- #
    def on_close(self):
        if self._fig is not None:
            plt.close(self._fig)
        self.destroy()


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="GUI to browse gait kinematics results for an IMU session.")
    ap.add_argument("session", nargs="?",
                    help="session folder to open on startup (optional)")
    args = ap.parse_args(argv)

    app = ResultsGUI(session_dir=args.session)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
