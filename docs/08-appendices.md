# 08 — Appendices

## References

- **Ultra Inertial Poser (UIP)** — SIGGRAPH 2024. Sparse body-worn IMU + UWB ranging,
  fused on host with VQF, per-pair EKF, LSTM + graph-conv network, physics optimizer.
  Direct inspiration for sensing approach and "thin nodes, heavy host" philosophy.

## Glossary

| Term | Meaning |
|------|---------|
| DH | Denavit-Hartenberg — systematic parameterisation of rigid-body kinematic chains |
| SFLP | Sensor Fusion Low Power — onboard quaternion output of LSM6DSV16B |
| TDMA | Time Division Multiple Access — UWB slot schedule |
| VQF | Versatile Quaternion-based Filter (Laidig & Seel, 2022) |
| SWO | Serial Wire Output — one-way MCU → host data channel through ST-LINK |
| NUS | Nordic UART Service — BLE serial-over-GATT convention |

## Data-sheet / resource links

- STM32WBA55: https://www.st.com/en/microcontrollers-microprocessors/stm32wba55.html
- LSM6DSV16B: https://www.st.com/en/mems-and-sensors/lsm6dsv16b.html
- MMC5983MA: https://www.memsic.com/magnetometer-2
- DWM3000: https://www.qorvo.com/products/p/DWM3000
- W25Q64: https://www.winbond.com/hq/product/code-storage-flash-memory/serial-nor-flash/
- BQ25185: https://www.ti.com/product/BQ25185
- MAX17048: https://www.analog.com/en/products/max17048.html
