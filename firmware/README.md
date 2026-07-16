# firmware/

Node firmware for the STM32WBA55. **Git-versioned — no version folders here.**
Use `git tag imu-vX.Y.Z` to mark releases and tie them to a hardware revision.

## Structure

Standard STM32CubeIDE project layout, generated from `wearable IMU.ioc`:

```
firmware/
├── Core/            # CubeMX-generated — do NOT edit by hand; regenerate from .ioc
│   ├── Inc/, Src/, Startup/
├── Drivers/
│   ├── CMSIS/                   # ARM CMSIS
│   └── STM32WBxx_HAL_Driver/    # ST HAL
├── Middlewares/ST/               # ST middleware (e.g. BLE stack support)
├── ThirdParty/
│   ├── BMM350/                  # magnetometer vendor driver
│   └── lsm6dsv16bx/             # IMU vendor driver
├── USB_Device/                   # USB device stack (charging path only — WBA55 has no native USB data)
├── *.ld                          # linker scripts
└── wearable IMU.ioc              # CubeMX config (source of truth for Core/)
```

Application logic (role state machines, UWB TDMA, BLE serial, sensor glue,
power management) isn't written yet — only the CubeMX-generated skeleton
exists so far. An earlier scaffolded `App/`, `Comms/`, `Drivers/{dwm3000,
lsm6dsv16b,...}`, `Sensors/`, `Power/` layout was removed in favor of this
generated structure; expect the application code to land inside `Core/Src/`
or new folders alongside it as it's written.

## Bring-up order

1. Generate `Core/` from STM32CubeMX for the STM32WBA55CG.
2. Bring up SWD/SWO wired link first.
3. Add UWB TDMA (data + sync + ranging).
4. Add BLE serial-over-BLE uplink.

## Building

TODO: STM32CubeIDE project setup. Toolchain: arm-none-eabi-gcc.
