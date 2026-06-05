"""
generator.py — synthetic IMU data source.

Generates realistic accel + gyro (+ optional mag) samples for N nodes
following a scripted or random upper-limb motion. Drives the full pipeline
without any hardware.

Emits the same NodeSample objects as the hardware ingest path so the rest
of the pipeline is transport-agnostic.

TODO:
    - Define NodeSample dataclass (node_id, timestamp, ax/ay/az, gx/gy/gz, mx/my/mz).
    - Implement SyntheticSource.start() — spawn background thread, push samples to queue.
    - Implement a simple scripted motion (e.g., elbow flexion sweep 0–90°).
    - Add Gaussian noise scaled to LSM6DSV16B datasheet noise density.
    - Optionally generate synthetic UWB ranges from the scripted geometry.
"""

from __future__ import annotations
import time
import math
from .. import config


class SyntheticSource:
    """Generates synthetic IMU samples at config.SAMPLE_RATE_HZ."""

    def start(self) -> None:
        print(f"[sim] Synthetic source running — {config.NODE_COUNT} nodes "
              f"@ {config.SAMPLE_RATE_HZ} Hz")
        dt = 1.0 / config.SAMPLE_RATE_HZ
        t = 0.0
        try:
            while True:
                for node_id in config.NODE_IDS:
                    sample = self._make_sample(node_id, t)
                    # TODO: push sample onto processing queue
                    _ = sample
                t += dt
                time.sleep(dt)
        except KeyboardInterrupt:
            print("[sim] Stopped.")

    def _make_sample(self, node_id: int, t: float) -> dict:
        """Return a synthetic sample dict for the given node at time t."""
        # Gravity vector (static node pointing up)
        ax, ay, az = 0.0, 0.0, 9.81
        # Slow sinusoidal rotation around Y (simulates elbow flexion)
        omega = 0.5 * math.pi * math.sin(2 * math.pi * 0.2 * t)
        gx, gy, gz = 0.0, omega + node_id * 0.01, 0.0
        mx, my, mz = (25.0, 5.0, -42.0) if config.MAG_ENABLED else (0.0, 0.0, 0.0)
        return {
            "node_id": node_id,
            "timestamp_us": int(t * 1e6),
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "mx": mx, "my": my, "mz": mz,
        }
