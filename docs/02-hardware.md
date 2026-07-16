# 02 — Hardware

> Expand on README §4. Add pin mappings, power tree, layout notes here.

## Node BOM (v1.0)

See README §4 for the full table. Key parts:

- MCU: STM32WBA55CGU7 (UFQFPN48)
- IMU: LSM6DSV16BXTR (6-axis + onboard SFLP), I2C
- Mag: BMM350 (populated, use TBD — see open items), I2C
- UWB: DWM3000 (SPI from WBA55 — the only SPI sensor on the node)
- Flash: W25Q64JVXGIM (8 MB, SPI)
- Charger: BQ25185 | Fuel gauge: MAX17048 | SMPS: TPSM828224

## Interfaces

| Peripheral | WBA55 interface | Notes |
|------------|----------------|-------|
| DWM3000 | SPI1 | IRQ on PXX — TBD |
| LSM6DSV16BXTR | I2C1, addr `0x6B` | SDO pin pulled high. INT1/INT2 |
| BMM350 | I2C1, addr `0x14` | ADSEL pin pulled low |
| W25Q64 | SPI3 — TBD | |
| BQ25185 | I2C — TBD | |
| MAX17048 | I2C — TBD | |
| SWD/SWO | SWDIO/SWDCLK/SWO | TC2030-IDC footprint (Tag-Connect) — bare pogo-pin pads, no on-board connector; mates with a separate TC2030 cable/clip |

## Power tree

TODO: fill in from schematic (TPSM828224 → 3V3 rail; BQ25185 USB-C charging).
