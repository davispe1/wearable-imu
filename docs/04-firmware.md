# 04 — Firmware

> Node firmware for the STM32WB55CEUx. Expands README §7 (phase 1).

## Status

**Nothing described below is implemented yet.** Only the CubeMX-generated project
skeleton exists (`firmware/Core/`, `Drivers/`, `Middlewares/`, `ThirdParty/`,
`USB_Device/` — see `firmware/README.md`). An earlier scaffolded
`firmware/node/{App,Comms,Drivers,Sensors,Power}` layout, referenced by older
versions of this doc, was pure stub code (empty `TODO` function bodies) and has
been deleted. The architecture below is the **plan** for what gets built next,
not a status report — see [07-roadmap.md](07-roadmap.md) for what's actually done.

## Planned architecture

```
firmware/
├── Core/        # CubeMX-generated (do not edit by hand) — exists today
├── App/         # app_main, role_master, role_sensor — top-level state machines (planned)
├── Drivers/     # device drivers + the port abstraction (port.h) (planned)
├── Comms/       # packet serializer, uwb_tdma, ble_serial, swo_data (planned)
├── Sensors/     # imu, mag, fusion (glue: drivers -> raw counts -> packet) (planned)
└── Power/       # charger (BQ25185), fuel_gauge (MAX17048) (planned)
```

Note: today's real `Drivers/` (CMSIS + STM32WBxx HAL, CubeMX-generated) and the
planned app-layer `Drivers/` above will coexist — the app-layer one is thin
device drivers written against a port abstraction, not the HAL itself.

### Layering (why drivers shouldn't call the HAL directly)

The plan is to write drivers against a `port.h` (a tiny bus/timing abstraction),
**not** the STM32 HAL directly. This would let them compile and be reasoned about
independent of what peripheral/pins CubeMX assigns. Both buses are in play here:
the IMU and magnetometer are **I2C** (one shared bus, I2C1), while the DWM3000
UWB module is the only **SPI** device on the node (SPI1). `port.h` would wrap
`HAL_I2C_Mem_Read`/`HAL_I2C_Mem_Write` for the sensors, `HAL_SPI_TransmitReceive`
for the DWM3000, plus `HAL_Delay` and a 1 MHz `TIM` counter (`port_micros`) —
none of this exists yet.

## v1 sensor data path (planned)

Intended behavior: **sample IMU(+mag) → build one-node frame → emit**, bringing up
over **SWO (wired)** first, later swapped for UWB TDMA transmit in the node's
assigned slot. The node only ever handles **raw int16 counts** — all scaling/bias
is host-side (thin-node philosophy). None of this is written yet.

## Wire protocol

The format is specified in [03-communication.md](03-communication.md) — **that doc
is authoritative; change it before either implementation.** The host decoder
(`host/wearable_imu/ingest/protocol.py`) implements it today and is tested against
a golden vector (`host/tests/test_protocol.py`). The firmware serializer doesn't
exist yet; once written, it should reproduce the same golden vector so the two
encoders can't drift.

## Driver notes — LSM6DSV16BXTR (planned)

- **I2C1, address `0x6B`** (SDO pin pulled high on this board).
- v1 config: **±8 g, ±2000 dps, ODR 120 Hz** (no 100 Hz step exists — see §5 of the
  comms doc). FS settings must match the host scale constants.
- The bring-up smoke test (see [07-roadmap.md](07-roadmap.md)) already talks to this
  sensor at `0x6B` and reads a plausible WHO_AM_I — the address/bus are confirmed
  working; only the sampling/packing pipeline is unwritten.

## Driver notes — BMM350 (planned)

- **I2C1, address `0x14`** (ADSEL pin pulled low on this board) — same bus as the IMU.
- Full-scale / sensitivity: not yet characterized, pending mag bring-up (see
  [03-communication.md](03-communication.md) and [06-calibration.md](06-calibration.md)).
- Vendor driver lives in `firmware/ThirdParty/BMM350/`.

## Bring-up order

1. ~~Generate `Core/` from STM32CubeMX for the STM32WB55CEUx~~ — done. Implement `port.c`.
2. SWD/SWO wired link: verify IMU sampling + frames out via SWO.
3. UWB TDMA (data + sync + ranging).
4. BLE serial-over-BLE uplink.

## TDMA frame (planned — not implemented)

Nothing below is implemented or tested yet. These are pre-hardware-test design
estimates, not decisions — UWB hasn't been brought up at all (see
[07-roadmap.md](07-roadmap.md)), so treat every number here as provisional
until it's actually run on the DWM3000.

- One channel is intended to carry **data slots + ranging rounds + sync** via a
  single TDMA frame.
- Ranging is slow relative to data: **25–50 Hz** is expected to be plenty.
- All-pairs ranging scales as N(N-1)/2 (15 pairs @ 6 nodes, 28 @ 8 nodes). For
  upper-limb joint angles, **adjacent-segment pairs** may suffice — full mesh
  is probably not required, but this hasn't been tested.
- **Sync plan:** the master is intended to be the UWB time reference, with
  nodes sub-ns aligned **on the body, before** data leaves; the master
  timestamps aggregated data.

TODO: define exact frame length, slot assignments, sync beacon timing — none
of this is decided, only sketched above.
