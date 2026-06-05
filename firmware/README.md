# firmware/

Node firmware for the STM32WBA55. **Git-versioned — no version folders here.**
Use `git tag imu-vX.Y.Z` to mark releases and tie them to a hardware revision.

## Structure

```
firmware/node/
├── Core/        # CubeMX-generated — do NOT edit by hand; regenerate from .ioc
├── App/         # application entry + role state machines
├── Drivers/     # thin peripheral drivers
├── Comms/       # transport layer (UWB TDMA, BLE serial, SWO data)
├── Sensors/     # sensor glue (IMU, mag, fusion config)
├── Power/       # power management (charger, fuel gauge)
└── config.h     # node ID, role, sample rate, data-format flags
```

## Bring-up order

1. Generate `Core/` from STM32CubeMX for the STM32WBA55CG.
2. Bring up SWD/SWO wired link first.
3. Add UWB TDMA (data + sync + ranging).
4. Add BLE serial-over-BLE uplink.

## Building

TODO: STM32CubeIDE project setup. Toolchain: arm-none-eabi-gcc.
