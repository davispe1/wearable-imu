# 02 — Hardware

> Expand on README §4. Add pin mappings, power tree, layout notes here.

## Node BOM (v1.0)

See README §4 for the full table. Key parts:

- MCU: STM32WB55CEUx (UFQFPN48), dual-core Cortex-M4 (64 MHz) + Cortex-M0+ (32 MHz),
  512 KB flash, 256 KB SRAM, native USB
- IMU: LSM6DSV16BXTR (6-axis + onboard SFLP), I2C
- Mag: BMM350 (populated, use open — see open items), I2C
- UWB: DWM3000 (SPI from WB55CEUx — the only SPI sensor on the node)
- No on-node flash IC (confirmed against the fabricated PCB's BOM — not populated)
- Charger: BQ25185 | Fuel gauge: MAX17048 | SMPS: TPSM828224

## Interfaces

| Peripheral | WB55CEUx interface | Notes |
|------------|----------------|-------|
| DWM3000 | SPI1 | IRQ pin not yet assigned in firmware |
| LSM6DSV16BXTR | I2C1, addr `0x6B` | SDO pin pulled high. INT1/INT2 |
| BMM350 | I2C1, addr `0x14` | ADSEL pin pulled low |
| BQ25185 | I2C, bus not yet assigned in firmware | |
| MAX17048 | I2C, bus not yet assigned in firmware | |
| SWD/SWO | SWDIO/SWDCLK/SWO | TC2030-IDC footprint (Tag-Connect) — bare pogo-pin pads, no on-board connector; mates with a separate TC2030 cable/clip |
| USB | Native USB (CDC stack present in firmware) | See README §3 — needs reconciling with the SWD/SWO wired-data-path assumption now that native USB is confirmed |

## Power tree

TODO: fill in from schematic (TPSM828224 → 3V3 rail; BQ25185 USB-C charging).
