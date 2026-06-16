"""
app/main.py — Gait Kinematics desktop app (recorded-file viewer).

A windowed PyQt app with a left sidebar that switches between three views over the
gaitlib library (the single source of truth for the kinematics):

    Sessions   import 9-DOF IMU CSVs, list sessions, run the pipeline
    Visualize  synced 2D joint-angle & angular-velocity curves + live parameters + scrubber
    Results    consolidated per-joint / gait parameters, overlaid cycles, export

Sessions live on disk under config/app.yaml -> app.data_dir (default ~/GaitApp/sessions/).
A live-capture mode can later feed the same Visualize view through the same analysis layer.

Run:  python -m app.main          (or: python app/main.py)
"""
from __future__ import annotations
import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from app.session_store import app_config, ensure_test_sessions
from app.ui_sessions import SessionsView
from app.ui_visualize import VisualizeView
from app.ui_results import ResultsView


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.setWindowTitle("Gait Kinematics")
        self.resize(1360, 860)

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central); h.setContentsMargins(0, 0, 0, 0); h.setSpacing(0)

        # sidebar
        self.nav = QtWidgets.QListWidget()
        self.nav.setFixedWidth(180)
        self.nav.setStyleSheet(
            "QListWidget{background:#22272e;color:#cfd6dd;border:none;font-size:13pt;}"
            "QListWidget::item{padding:16px 18px;}"
            "QListWidget::item:selected{background:#2f81f7;color:white;}")
        for name in ("  Sessions", "  Visualize", "  Results"):
            self.nav.addItem(QtWidgets.QListWidgetItem(name))
        h.addWidget(self.nav)

        # stacked views
        self.stack = QtWidgets.QStackedWidget()
        self.sessions = SessionsView(cfg)
        self.visualize = VisualizeView(cfg)
        self.results = ResultsView(cfg)
        for w in (self.sessions, self.visualize, self.results):
            self.stack.addWidget(w)
        h.addWidget(self.stack, 1)

        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.sessions.openRequested.connect(self.open_session)
        self.nav.setCurrentRow(0)

    def open_session(self, sdir, target):
        try:
            if target in ("visualize", "run"):
                self.visualize.load(sdir)
                self.results.load(sdir)
                self.nav.setCurrentRow(1)
            elif target == "results":
                self.results.load(sdir)
                self.visualize.load(sdir)
                self.nav.setCurrentRow(2)
        except Exception as e:
            import traceback
            QtWidgets.QMessageBox.critical(self, "Could not open session",
                                           f"{e}\n\n{traceback.format_exc()}")


def main():
    cfg = app_config()
    try:
        ensure_test_sessions(cfg)        # make Geneva slices available on first launch
    except Exception:
        pass
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow(cfg)
    win.show()
    sys.exit((app.exec_ if hasattr(app, "exec_") else app.exec)())


if __name__ == "__main__":
    main()
