"""
app/ui_sessions.py — Sessions view: import CSV, list sessions, run the pipeline.
"""
from __future__ import annotations
import os
from pyqtgraph.Qt import QtCore, QtWidgets

from app import session_store as store
from app.session_store import (load_session_streams, default_joint_topology,
                               results_dir, read_metadata)
from app.pipeline_runner import run_pipeline, write_results


class PipelineWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str, float)
    finished_ok = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, sdir, cfg, parent=None):
        super().__init__(parent)
        self.sdir, self.cfg = sdir, cfg

    def run(self):
        try:
            nd, meta = load_session_streams(self.sdir)
            topo = default_joint_topology(list(nd.keys()))
            fz = self.cfg["app"]["fusion"]
            out = run_pipeline(
                nd, joints=topo["joints"], foot_node=topo["foot"], pelvis_node=topo["pelvis"],
                run_modes=tuple(self.cfg["app"]["run_modes"]),
                beta6=fz["beta_6dof"], beta9=fz["beta_9dof"], joint_tau_s=fz["joint_tau_s"],
                progress=lambda m, f: self.progress.emit(m, float(f or 0.0)))
            write_results(results_dir(self.sdir), out, meta)
            self.finished_ok.emit(self.sdir)
        except Exception as e:
            import traceback
            self.failed.emit(f"{e}\n{traceback.format_exc()}")


class SessionsView(QtWidgets.QWidget):
    # (session_dir, target_view)
    openRequested = QtCore.pyqtSignal(str, str)

    COLS = ["Session", "Subject", "Date", "Dur (s)", "Side", "Nodes", "Joints", "Results"]

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._worker = None
        self._build()
        self.refresh()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        head = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("<h2>Sessions</h2>")
        head.addWidget(title); head.addStretch(1)
        self.btn_import = QtWidgets.QPushButton("Import CSV…")
        self.btn_geneva = QtWidgets.QPushButton("Add Geneva test data")
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        for b in (self.btn_import, self.btn_geneva, self.btn_refresh):
            head.addWidget(b)
        v.addLayout(head)

        self.table = QtWidgets.QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setColumnWidth(0, 230); self.table.setColumnWidth(5, 150); self.table.setColumnWidth(6, 150)
        v.addWidget(self.table, 1)

        row = QtWidgets.QHBoxLayout()
        self.lbl_root = QtWidgets.QLabel(f"data folder: {store.data_root(self.cfg)}")
        self.lbl_root.setStyleSheet("color:#666")
        row.addWidget(self.lbl_root); row.addStretch(1)
        self.btn_run = QtWidgets.QPushButton("Run pipeline")
        self.btn_vis = QtWidgets.QPushButton("Open in Visualize ▶")
        self.btn_res = QtWidgets.QPushButton("Open Results")
        for b in (self.btn_run, self.btn_res, self.btn_vis):
            row.addWidget(b)
        v.addLayout(row)

        self.progress = QtWidgets.QProgressBar(); self.progress.setVisible(False)
        self.status = QtWidgets.QLabel(""); self.status.setStyleSheet("color:#444")
        v.addWidget(self.progress); v.addWidget(self.status)

        self.btn_import.clicked.connect(self.on_import)
        self.btn_geneva.clicked.connect(self.on_geneva)
        self.btn_refresh.clicked.connect(self.refresh)
        self.btn_run.clicked.connect(lambda: self.on_open("run"))
        self.btn_vis.clicked.connect(lambda: self.on_open("visualize"))
        self.btn_res.clicked.connect(lambda: self.on_open("results"))
        self.table.doubleClicked.connect(lambda *_: self.on_open("visualize"))

    # -- data ---------------------------------------------------------------- #
    def refresh(self):
        self.sessions = store.list_sessions(self.cfg)
        self.table.setRowCount(len(self.sessions))
        for r, m in enumerate(self.sessions):
            vals = [m["session_id"], m.get("subject", ""), m.get("date", ""),
                    f"{m.get('duration_s', 0):.0f}", m.get("side", "") or "—",
                    ",".join(m.get("nodes", [])), ",".join(m.get("joints", {}).keys()),
                    "✓ computed" if m.get("has_results") else "— not run"]
            for c, val in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(str(val))
                if c == 7 and not m.get("has_results"):
                    it.setForeground(QtCore.Qt.gray)
                self.table.setItem(r, c, it)
        if self.sessions:
            self.table.selectRow(0)

    def _selected(self):
        r = self.table.currentRow()
        if r < 0 or r >= len(self.sessions):
            return None
        return self.sessions[r]

    # -- actions ------------------------------------------------------------- #
    def on_import(self):
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Import 9-DOF IMU CSV (combined, or per-node files)", "",
            "CSV files (*.csv);;All files (*)")
        if not paths:
            return
        subject, ok = QtWidgets.QInputDialog.getText(self, "Subject", "Subject id (optional):")
        try:
            sdir, meta = store.import_csv(self.cfg, paths, subject=subject.strip(), overwrite=False)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Import failed", str(e))
            return
        self.refresh()
        self._select_id(meta["session_id"])
        self._run_pipeline(sdir, then="visualize")

    def on_geneva(self):
        ids = store.ensure_test_sessions(self.cfg)
        self.refresh()
        self.status.setText(f"Geneva test sessions available: {', '.join(ids) or 'none found'}")

    def on_open(self, target):
        m = self._selected()
        if not m:
            return
        sdir = m["path"]
        if target == "run":
            self._run_pipeline(sdir, then=None)
            return
        if not m.get("has_results"):
            self._run_pipeline(sdir, then=target)
        else:
            self.openRequested.emit(sdir, target)

    def _select_id(self, sid):
        for r, m in enumerate(self.sessions):
            if m["session_id"] == sid:
                self.table.selectRow(r); return

    # -- pipeline run -------------------------------------------------------- #
    def _run_pipeline(self, sdir, then="visualize"):
        if self._worker and self._worker.isRunning():
            return
        self._set_busy(True, "Running pipeline …")
        self._then = then
        self._worker = PipelineWorker(sdir, self.cfg)
        self._worker.progress.connect(self._on_progress)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.failed.connect(self._on_fail)
        self._worker.start()

    def _on_progress(self, msg, frac):
        self.progress.setValue(int(frac * 100))
        self.status.setText(f"pipeline: {msg}")

    def _on_done(self, sdir):
        self._set_busy(False, "Pipeline complete.")
        self.refresh()
        self._select_id(os.path.basename(sdir))
        if self._then:
            self.openRequested.emit(sdir, self._then)

    def _on_fail(self, err):
        self._set_busy(False, "")
        QtWidgets.QMessageBox.critical(self, "Pipeline failed", err)

    def _set_busy(self, busy, msg):
        self.progress.setVisible(busy)
        if busy:
            self.progress.setRange(0, 100); self.progress.setValue(0)
        for b in (self.btn_import, self.btn_run, self.btn_vis, self.btn_res, self.btn_geneva):
            b.setEnabled(not busy)
        self.status.setText(msg)
