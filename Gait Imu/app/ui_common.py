"""
app/ui_common.py — Small shared UI helpers: the real-time playback clock (wall-clock ->
data-time mapping with speed/pause/loop).
"""
from __future__ import annotations
import time
import numpy as np


class PlaybackClock:
    """Maps wall-clock time to data time [0, t_end] with play/pause, speed and loop."""

    def __init__(self, t_end, speed=1.0, loop=True):
        self.t_end = float(t_end)
        self.speed = float(speed)
        self.loop = bool(loop)
        self.playing = True
        self._anchor = 0.0
        self._wall = time.perf_counter()

    def _reanchor(self, t):
        self._anchor = t
        self._wall = time.perf_counter()

    def time(self):
        if not self.playing:
            return self._anchor
        t = self._anchor + (time.perf_counter() - self._wall) * self.speed
        if t > self.t_end:
            if self.loop:
                self._reanchor(0.0)
                return 0.0
            t = self.t_end
            self._anchor = self.t_end
            self.playing = False
        return t

    def set_time(self, t):
        self._reanchor(float(np.clip(t, 0.0, self.t_end)))

    def set_speed(self, s):
        self._reanchor(self.time())
        self.speed = float(s)

    def toggle(self):
        if self.playing:
            self._anchor = self.time()
            self.playing = False
        else:
            self._reanchor(self._anchor)
            self.playing = True
        return self.playing
