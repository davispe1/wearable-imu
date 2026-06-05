# 03 — Communication

> Expand on README §5.

## Packet format (per sample, per node)

**Option A — raw 9-DOF (preferred for v1):**

| Field | Type | Bytes |
|-------|------|-------|
| node_id | uint8 | 1 |
| seq | uint16 | 2 |
| timestamp_us | uint32 | 4 |
| accel[3] | int16×3 | 6 |
| gyro[3] | int16×3 | 6 |
| mag[3] | int16×3 | 6 |
| _pad | — | 3 |
| **Total** | | **~28 B** |

**Option B — SFLP quaternion + raw mag:** smaller (~20 B), less host work.

Decision is open — see [open items](07-roadmap.md).

## UWB TDMA frame

TODO: define slot structure (data slots + ranging rounds + sync beacon).

Ranging rate: 25–50 Hz (power saving). IMU rate: 100 Hz.

## BLE uplink

Serial-over-BLE (NUS-equivalent GATT service). Aggregated from master.
Bandwidth: ~230 kbps for 8 nodes @ 100 Hz raw — comfortable for one BLE 5.x link.
