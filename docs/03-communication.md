# 03 — Communication

> **Authoritative wire-protocol spec.** The host parser
> (`host/wearable_imu/ingest/protocol.py`) implements this document and is real/tested.
> The firmware serializer is **not implemented yet** — no `packet.c` exists (see
> [07-roadmap.md](07-roadmap.md)); once written, it should be the firmware twin of
> the host parser. If you change the protocol, change it here first, then both ends,
> then bump `PROTO_VERSION`.

## 0. Conventions

- **Endianness:** little-endian for all multi-byte fields (STM32 and x86 are both LE).
- **Integers:** unsigned unless noted; sensor samples are signed `int16` raw counts.
- **Units on the wire:** raw ADC counts (not physical units). The host applies scale +
  bias — keeps nodes thin. Scale factors live in `protocol.py` and must match the
  full-scale settings the firmware configures (see §4).

## 1. Frame structure

Every frame on every transport (SWO, BLE serial, UWB-dongle, file replay) uses this envelope:

```
┌────────────┬──────┬─────────────────────────────────────────────┐
│ Field      │ Size │ Notes                                        │
├────────────┼──────┼─────────────────────────────────────────────┤
│ SYNC0      │ 1    │ 0xAA                                          │
│ SYNC1      │ 1    │ 0x55                                          │
│ VERSION    │ 1    │ PROTO_VERSION = 0x01                          │
│ MSG_TYPE   │ 1    │ see §2                                        │
│ PAYLOAD_LEN│ 2    │ uint16 LE — number of payload bytes          │
│ PAYLOAD    │ var  │ PAYLOAD_LEN bytes (§3)                        │
│ CRC16      │ 2    │ uint16 LE, CRC-16/CCITT-FALSE over            │
│            │      │   VERSION..last payload byte (excludes SYNC)  │
└────────────┴──────┴─────────────────────────────────────────────┘
```

Header is 6 bytes, trailer (CRC) is 2 bytes → 8 bytes of envelope overhead per frame.

**CRC-16/CCITT-FALSE:** poly `0x1021`, init `0xFFFF`, no reflection, no final XOR.
Covers `VERSION, MSG_TYPE, PAYLOAD_LEN, PAYLOAD` so a corrupted length is also caught.

**Resync:** a reader scans for `AA 55`. On CRC failure it discards one byte past the false
`AA 55` and rescans — a corrupted/misaligned stream re-locks within a few bytes.

## 2. Message types

| Value | Name              | Meaning                                  |
|-------|-------------------|------------------------------------------|
| 0x01  | `MSG_IMU`         | One or more node IMU samples (§3.1)      |
| 0x02  | `MSG_RANGE`       | UWB inter-node ranges (§3.2)             |
| 0x10  | `MSG_STATUS`      | Reserved — battery / link stats (future) |

## 3. Payloads

### 3.1 `MSG_IMU`

```
┌────────────┬──────┬───────────────────────────────────────────┐
│ FRAME_SEQ  │ 2    │ uint16 LE — increments per emitted frame    │
│ FORMAT     │ 1    │ 0 = RAW_9DOF, 1 = SFLP_QUAT                  │
│ NODE_COUNT │ 1    │ number of node records that follow          │
│ records[]  │ var  │ NODE_COUNT × node record                    │
└────────────┴──────┴───────────────────────────────────────────┘
```

**Node record — FORMAT 0 (RAW_9DOF), 25 bytes:**

```
NODE_ID      1   uint8
NODE_SEQ     2   uint16 LE   per-node sample counter (per-node loss detection)
TIMESTAMP_US 4   uint32 LE   node UWB-aligned timestamp, microseconds
AX AY AZ     6   int16 LE ×3 raw accelerometer counts
GX GY GZ     6   int16 LE ×3 raw gyroscope counts
MX MY MZ     6   int16 LE ×3 raw magnetometer counts (0 if MAG disabled)
```

**Node record — FORMAT 1 (SFLP_QUAT), 21 bytes:**

```
NODE_ID      1   uint8
NODE_SEQ     2   uint16 LE
TIMESTAMP_US 4   uint32 LE
QW QX QY QZ  8   int16 LE ×4 quaternion, Q15 fixed-point (×32767)
MX MY MZ     6   int16 LE ×3 raw magnetometer counts (0 if MAG disabled)
```

### 3.2 `MSG_RANGE`

```
FRAME_SEQ    2   uint16 LE
TIMESTAMP_US 4   uint32 LE   master time of the ranging round
PAIR_COUNT   1   uint8
records[]    var PAIR_COUNT × range record
```

**Range record, 4 bytes:** `NODE_A(1) NODE_B(1) DIST_MM(uint16 LE)` — distance in mm
(0–65.5 m range). Ranging runs at `RANGING_RATE_HZ` (25–50 Hz), slower than IMU.

## 4. Scale factors (counts → physical)

These depend on the full-scale ranges the firmware configures. **They must match
the firmware's config constants (`IMU_ACCEL_FS_G`, `IMU_GYRO_FS_DPS` — not yet
written) and the constants in `host/wearable_imu/ingest/protocol.py`.** v1 defaults:

| Quantity | Full scale | Sensitivity (LSB) | To physical |
|----------|-----------|-------------------|-------------|
| Accel | ±8 g | 0.244 mg/LSB | `g = raw × 0.244e-3`; `m/s² = g × 9.80665` |
| Gyro | ±2000 dps | 70 mdps/LSB | `dps = raw × 0.070`; `rad/s = dps × π/180` |
| Mag (BMM350) | not yet characterized | not yet characterized | pending mag bring-up — see [06](06-calibration.md) |

> Values are LSM6DSV-family datasheet figures — **verify against the LSM6DSV16B datasheet
> when hardware arrives.**

## 5. Sample rate note

The LSM6DSV16B ODR grid is `… 30, 60, 120, 240 …` Hz — **there is no 100 Hz step.** v1 runs
the sensor at **120 Hz** (closest to the README's 100 Hz target). Because every sample carries
a `TIMESTAMP_US` and the orientation filters integrate by `dt`, the exact nominal rate is not
load-bearing; the host can also decimate 120→100 Hz if a strict rate is ever required.

## 6. Bandwidth check

RAW_9DOF: envelope(8) + IMU header(4) + 25 B/node.
- 1 node @ 120 Hz: `(8+4+25) × 120 = 4.4 kB/s ≈ 35 kbps`
- 8 nodes aggregated @ 120 Hz: `(8+4+8×25) × 120 = 26.6 kB/s ≈ 213 kbps` — comfortable for one BLE 5.x link.
