# 04 — Firmware

> Expand on README §7 (phase 1). Document firmware architecture, TDMA schedule, driver APIs.

## Architecture

```
firmware/node/
├── Core/        # CubeMX-generated (do not edit by hand)
├── App/         # app_main, role_master, role_sensor — top-level state machines
├── Drivers/     # thin HAL wrappers: lsm6dsv16b, mmc5983ma, dwm3000, w25q64
├── Comms/       # uwb_tdma, ble_serial, swo_data
├── Sensors/     # imu, mag, fusion (glue between drivers and App)
└── Power/       # charger (BQ25185), fuel_gauge (MAX17048)
```

## Bring-up order

1. SWD/SWO wired link — verify sampling + SWO data out.
2. UWB TDMA on-body — data slots + sync + ranging.
3. BLE serial — parallel to wired path, then wired becomes fallback.

## TDMA frame (TBD)

TODO: define frame length, slot assignments, sync beacon timing.

## Driver notes

TODO: add per-driver SPI/I2C config, register map references.
