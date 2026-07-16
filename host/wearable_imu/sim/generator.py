"""
generator.py — synthetic IMU data source (no hardware).

Produces a physically-consistent stream: each node is a rigid segment slowly
rotating about its Y axis under gravity. The accel reads the gravity vector in the
(rotating) body frame and the gyro reads the angular rate, so a real orientation
filter could actually track it later — not just random noise.

Three interfaces, same data:
    iter_frames()  -> ImuFrame objects directly (fast; bypasses the wire)
    iter_bytes()   -> encoded protocol bytes (exercises the REAL parser)
    write_capture(path, seconds) -> a .bin replayable via ingest.reader.FileReader

stdlib only (math, random) — no numpy needed.
"""

from __future__ import annotations

import math
import random
from collections.abc import Iterator
from pathlib import Path

from .. import config
from ..ingest import protocol
from ..model import ImuFrame, NodeSample

_G = 9.80665


class SyntheticSource:
    """Generates synthetic IMU frames at config.SAMPLE_RATE_HZ."""

    def __init__(self, cfg=config, *, noise_counts: float = 2.0, seed: int | None = 0) -> None:
        self.cfg = cfg
        self.noise = noise_counts
        self.dt = 1.0 / cfg.SAMPLE_RATE_HZ
        self.fmt = protocol.FMT_SFLP_QUAT if cfg.DATA_FORMAT == "sflp" else protocol.FMT_RAW_9DOF
        self._rng = random.Random(seed)
        # Per-node motion: amplitude (rad) and phase, so nodes differ.
        self._phase = {nid: 0.6 * i for i, nid in enumerate(cfg.NODE_IDS)}
        self._amp = 0.5 * math.pi          # ±90° sweep
        self._freq_hz = 0.2                # one sweep every 5 s

    # ── frame generation ────────────────────────────────────────────────────
    def _theta(self, node_id: int, t: float) -> tuple[float, float]:
        """Return (angle θ rad, rate ω rad/s) for a node at time t."""
        w = 2.0 * math.pi * self._freq_hz
        ph = self._phase[node_id]
        theta = self._amp * math.sin(w * t + ph)
        omega = self._amp * w * math.cos(w * t + ph)
        return theta, omega

    def _sample(self, node_id: int, node_seq: int, t: float) -> NodeSample:
        theta, omega = self._theta(node_id, t)
        ts_us = int(round(t * 1e6))

        if self.fmt == protocol.FMT_SFLP_QUAT:
            # Orientation = rotation θ about Y → quaternion (cos θ/2, 0, sin θ/2, 0).
            qw, qy = math.cos(theta / 2.0), math.sin(theta / 2.0)
            return NodeSample(node_id, node_seq, ts_us, quat=(qw, 0.0, qy, 0.0),
                              mag=self._mag_counts(theta))

        # RAW_9DOF: gravity in the rotating body frame is (-g sinθ, 0, g cosθ).
        ax = -_G * math.sin(theta)
        ay = 0.0
        az = _G * math.cos(theta)
        accel = (self._to_count(ax, protocol.ACCEL_COUNT_TO_MPS2),
                 self._to_count(ay, protocol.ACCEL_COUNT_TO_MPS2),
                 self._to_count(az, protocol.ACCEL_COUNT_TO_MPS2))
        gyro = (self._to_count(0.0, protocol.GYRO_COUNT_TO_RADS),
                self._to_count(omega, protocol.GYRO_COUNT_TO_RADS),
                self._to_count(0.0, protocol.GYRO_COUNT_TO_RADS))
        return NodeSample(node_id, node_seq, ts_us, accel=accel, gyro=gyro,
                          mag=self._mag_counts(theta))

    def _mag_counts(self, theta: float) -> tuple[int, int, int]:
        if not self.cfg.MAG_ENABLED:
            return (0, 0, 0)
        # Placeholder constant field; rotated copy could be added at mag bring-up.
        return (1000, 200, -1700)

    def _to_count(self, physical: float, count_to_physical: float) -> int:
        raw = physical / count_to_physical + self._rng.gauss(0.0, self.noise)
        return max(-32768, min(32767, int(round(raw))))

    # ── public iterators ──────────────────────────────────────────────────────
    def iter_frames(self, max_frames: int | None = None) -> Iterator[ImuFrame]:
        seq = 0
        t = 0.0
        while max_frames is None or seq < max_frames:
            samples = [self._sample(nid, seq, t) for nid in self.cfg.NODE_IDS]
            yield ImuFrame(frame_seq=seq & 0xFFFF, fmt=self.fmt, samples=samples)
            seq += 1
            t += self.dt

    def iter_bytes(self, max_frames: int | None = None) -> Iterator[bytes]:
        for frame in self.iter_frames(max_frames):
            yield protocol.encode_imu_frame(frame)

    def write_capture(self, path: str | Path, seconds: float) -> int:
        """Write `seconds` of encoded frames to a .bin file; return bytes written."""
        n = int(round(seconds * self.cfg.SAMPLE_RATE_HZ))
        path = Path(path)
        total = 0
        with path.open("wb") as f:
            for raw in self.iter_bytes(max_frames=n):
                total += f.write(raw)
        return total

    # ── compatibility with main.py's source interface ────────────────────────
    def start(self) -> None:  # pragma: no cover - convenience for manual runs
        for frame in self.iter_frames():
            print(f"[sim] frame {frame.frame_seq:5d}  nodes={len(frame.samples)}")
