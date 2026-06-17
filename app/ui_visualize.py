"""
app/ui_visualize.py — Visualize view: synchronised 2D joint-angle and angular-velocity
curves with a live-parameter box and a timeline scrubber, driven by a real-time playback
clock. No 3D figure pane — the curves and live parameters are the whole view.
"""
from __future__ import annotations
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from app.analysis import LoadedSession, JCOLOR
from app.ui_common import PlaybackClock


class VisualizeView(QtWidgets.QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.ls = None
        self.clock = None
        self._turn_items = []
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)

    # -- layout -------------------------------------------------------------- #
    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        self.title = QtWidgets.QLabel("<h2>Visualize</h2><i>load a session from the Sessions tab</i>")
        v.addWidget(self.title)

        # curves (joint angle + angular velocity), cursor-swept in lockstep
        pg.setConfigOptions(antialias=True)
        self.plots = pg.GraphicsLayoutWidget()
        self.plots.setBackground("w")
        v.addWidget(self.plots, 3)
        self.p_ang = self.plots.addPlot(row=0, col=0, title="Joint flexion (neutral-referenced, °)")
        self.p_ang.addLegend(offset=(8, 4)); self.p_ang.showGrid(x=True, y=True, alpha=0.2)
        self.p_vel = self.plots.addPlot(row=1, col=0, title="Joint angular velocity (°/s)")
        self.p_vel.showGrid(x=True, y=True, alpha=0.2); self.p_vel.setXLink(self.p_ang)
        self.p_vel.setLabel("bottom", "time (s)")
        self.cur_ang = pg.InfiniteLine(angle=90, pen=pg.mkPen((20, 20, 20), width=1.5))
        self.cur_vel = pg.InfiniteLine(angle=90, pen=pg.mkPen((20, 20, 20), width=1.5))
        self.p_ang.addItem(self.cur_ang); self.p_vel.addItem(self.cur_vel)

        # live parameter box
        self.params = QtWidgets.QTextEdit(); self.params.setReadOnly(True)
        self.params.setFixedHeight(190)
        self.params.setStyleSheet("font-family:Consolas,monospace; font-size:11pt; background:#fafafa;")
        v.addWidget(self.params)

        # transport + scrubber
        transport = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("⏸ Pause")
        self.cmb_speed = QtWidgets.QComboBox(); self.cmb_speed.addItems(["0.25×", "0.5×", "1×", "2×"])
        self.cmb_speed.setCurrentText("1×")
        self.chk_loop = QtWidgets.QCheckBox("loop"); self.chk_loop.setChecked(True)
        transport.addWidget(self.btn_play)
        transport.addWidget(QtWidgets.QLabel("speed")); transport.addWidget(self.cmb_speed)
        transport.addWidget(self.chk_loop); transport.addStretch(1)
        self.lbl_time = QtWidgets.QLabel("t = 0.00 s")
        transport.addWidget(self.lbl_time)
        v.addLayout(transport)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 1000)
        v.addWidget(self.slider)

        self.btn_play.clicked.connect(self._toggle)
        self.cmb_speed.currentTextChanged.connect(self._speed)
        self.chk_loop.stateChanged.connect(lambda *_: self._set_loop())
        self.slider.sliderPressed.connect(lambda: setattr(self, "_scrubbing", True))
        self.slider.sliderReleased.connect(self._scrub_release)
        self.slider.valueChanged.connect(self._scrub_move)
        self._scrubbing = False

    # -- load ---------------------------------------------------------------- #
    def load(self, sdir):
        self.ls = LoadedSession(sdir)
        m = self.ls.meta
        self.title.setText(
            f"<h2>Visualize — {m['session_id']}</h2>"
            f"<span style='color:#666'>{m.get('subject','?')} · {m.get('side','')} leg · "
            f"{self.ls.t[-1]:.0f}s · {self.ls.fs:.0f} Hz</span>")
        # draw static curves (clearPlots removes only PlotDataItems; cursor lines persist)
        self.p_ang.clearPlots(); self.p_vel.clearPlots()
        for j in self.ls.joints:
            c = JCOLOR.get(j, (80, 80, 80))
            self.p_ang.plot(self.ls.t, self.ls.disp[j], pen=pg.mkPen(c, width=1.6), name=j)
            self.p_vel.plot(self.ls.t, self.ls.vel[j], pen=pg.mkPen(c, width=1.1))
        # shade turnarounds (non-steady) lightly
        self._shade_turns()
        self.clock = PlaybackClock(self.ls.t[-1], speed=self._speed_val(),
                                   loop=self.chk_loop.isChecked())
        self.clock.playing = True
        self.btn_play.setText("⏸ Pause")
        self.timer.start(int(1000 / max(1, self.cfg["visualization"].get("fps", 60))))
        self._tick()

    def _shade_turns(self):
        for it in self._turn_items:
            self.p_ang.removeItem(it)
        self._turn_items = []
        steady = self.ls.steady
        if steady.all():
            return
        edges = np.diff(steady.astype(int))
        starts = np.where(edges == -1)[0]; ends = np.where(edges == 1)[0]
        if not steady[0]:
            starts = np.r_[0, starts]
        if not steady[-1]:
            ends = np.r_[ends, len(steady) - 1]
        for s, e in zip(starts, ends):
            br = pg.LinearRegionItem([self.ls.t[s], self.ls.t[min(e, self.ls.n - 1)]],
                                     movable=False, brush=(255, 170, 60, 30))
            br.setZValue(-10)
            self.p_ang.addItem(br)
            self._turn_items.append(br)

    # -- transport ----------------------------------------------------------- #
    def _speed_val(self):
        return {"0.25×": 0.25, "0.5×": 0.5, "1×": 1.0, "2×": 2.0}[self.cmb_speed.currentText()]

    def _toggle(self):
        if not self.clock:
            return
        playing = self.clock.toggle()
        self.btn_play.setText("⏸ Pause" if playing else "▶ Play")

    def _speed(self):
        if self.clock:
            self.clock.set_speed(self._speed_val())

    def _set_loop(self):
        if self.clock:
            self.clock.loop = self.chk_loop.isChecked()

    def _scrub_move(self, val):
        if self._scrubbing and self.clock:
            t = (val / 1000.0) * self.ls.t[-1]
            self.clock.set_time(t)
            self._render_at(t)

    def _scrub_release(self):
        self._scrubbing = False

    # -- per-frame ----------------------------------------------------------- #
    def _tick(self):
        if not self.clock:
            return
        if self.clock.playing and not self._scrubbing:
            t = self.clock.time()
            self._render_at(t)
            if not self.chk_loop.isChecked() and t >= self.ls.t[-1]:
                self.btn_play.setText("▶ Play")
            self.slider.blockSignals(True)
            self.slider.setValue(int(1000 * t / self.ls.t[-1]))
            self.slider.blockSignals(False)

    def _render_at(self, t):
        ls = self.ls
        i = ls.index_at(t)
        for c in (self.cur_ang, self.cur_vel):
            c.setPos(t)
        self.lbl_time.setText(f"t = {t:6.2f} / {ls.t[-1]:.0f} s   ({self.clock.speed:.2f}×)")
        self.params.setHtml(self._param_html(ls.param_snapshot(i)))

    def _param_html(self, p):
        rows = []
        for j in self.ls.joints:
            d = p["joints"][j]
            c = JCOLOR.get(j, (80, 80, 80))
            chip = f"<span style='color:rgb{c}'>●</span>"
            rows.append(
                f"<tr><td>{chip} <b>{j}</b></td>"
                f"<td align=right>{d['angle']:+6.1f}°</td>"
                f"<td align=right>{d['vel']:+6.0f}°/s</td>"
                f"<td align=right>{d['rom']:5.1f}°</td>"
                f"<td align=right>{d['peak_min']:+.0f}…{d['peak_max']:+.0f}°</td>"
                f"<td align=right>{d['peak_vel']:.0f}°/s</td></tr>")
        cad = p["cadence"]
        cad_s = f"{cad:.0f}" if cad == cad else "—"
        return (
            "<table width=100% cellpadding=3 style='font-size:10.5pt'>"
            "<tr style='color:#888'><td>joint</td><td align=right>angle</td>"
            "<td align=right>ang.vel</td><td align=right>ROM</td>"
            "<td align=right>peak range</td><td align=right>peak|ω|</td></tr>"
            + "".join(rows) +
            "</table>"
            f"<div style='margin-top:6px'><b>steps</b> {p['steps']} &nbsp;·&nbsp; "
            f"<b>cadence</b> {cad_s} steps/min &nbsp;·&nbsp; "
            f"<b>t</b> {p['t']:.2f} s</div>")
