# 02 — Hardware

> Expand on README §4. Add pin mappings, power tree, layout notes here.

## Node BOM (v1.0)

See README §4 for the full table. Key parts:

- MCU: STM32WBA55CGU7 (UFQFPN48)
- IMU: LSM6DSV16BXTR (6-axis + onboard SFLP)
- Mag: MMC5983MA (populated, use TBD — see open items)
- UWB: DWM3000 (SPI from WBA55)
- Flash: W25Q64JVXGIM (8 MB, SPI)
- Charger: BQ25185 | Fuel gauge: MAX17048 | SMPS: TPSM828224

## Interfaces

| Peripheral | WBA55 interface | Notes |
|------------|----------------|-------|
| DWM3000 | SPI1 | IRQ on PXX — TBD |
| LSM6DSV16B | SPI2 or I2C — TBD | INT1/INT2 |
| MMC5983MA | I2C — TBD | |
| W25Q64 | SPI3 — TBD | |
| BQ25185 | I2C — TBD | |
| MAX17048 | I2C — TBD | |
| SWD/SWO | SWDIO/SWDCLK/SWO | 10-pin connector |

## Power tree

TODO: fill in from schematic (TPSM828224 → 3V3 rail; BQ25185 USB-C charging).
