"""
config.py — runtime configuration for the host pipeline.

All open items from README §11 live here so nothing is hard-coded in modules.
Edit this file (or override via CLI flags / env vars) before running.
"""

from __future__ import annotations

# ── Sampling ─────────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ: int = 100          # open item: 100 vs 200

# ── Node network ─────────────────────────────────────────────────────────────
NODE_COUNT: int = 3                # open item: 3 → 6–8
NODE_IDS: list[int] = list(range(NODE_COUNT))

# ── Data format ───────────────────────────────────────────────────────────────
# "raw"  → raw 9-DOF (accel + gyro + mag)
# "sflp" → SFLP quaternion + raw mag
DATA_FORMAT: str = "raw"           # open item: "raw" vs "sflp"

# ── Magnetometer ─────────────────────────────────────────────────────────────
MAG_ENABLED: bool = False          # open item: enable after lab characterisation

# ── Transport ─────────────────────────────────────────────────────────────────
# "swo"   → wired SWO/SWV via ST-LINK (bring-up)
# "ble"   → BLE serial (primary wireless)
# "uwb"   → UWB-USB dongle (backup wireless)
TRANSPORT: str = "swo"

# Serial port for SWO or BLE COM-port ingestion
SERIAL_PORT: str = "COM3"          # Windows example; adjust per machine
SERIAL_BAUD: int = 115200

# ── Orientation filter ───────────────────────────────────────────────────────
# "complementary" | "madgwick" | "vqf" | "sflp" (pass-through from node)
ORIENTATION_FILTER: str = "madgwick"   # open item

# ── Body model ────────────────────────────────────────────────────────────────
# Segment lengths in metres — upper-limb model (adjust per subject)
UPPER_ARM_LENGTH_M: float  = 0.30
FOREARM_LENGTH_M: float    = 0.26
HAND_LENGTH_M: float       = 0.10

# ── Visualiser ───────────────────────────────────────────────────────────────
VIZ_FPS: int = 60
