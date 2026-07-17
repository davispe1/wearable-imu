# 08 — Appendices

## References

- **[Ultra Inertial Poser (UIP)](https://siplab.org/projects/UltraInertialPoser)** —
  SIGGRAPH 2024 ([ACM DL](https://dl.acm.org/doi/10.1145/3641519.3657465),
  [arXiv](https://arxiv.org/abs/2404.19541),
  [code](https://github.com/eth-siplab/UltraInertialPoser)). Sparse body-worn IMU + UWB
  ranging, fused on host with VQF, per-pair EKF, LSTM + graph-conv network, physics
  optimizer. Direct inspiration for sensing approach and "thin nodes, heavy host" philosophy.

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

- STM32WB55CE: https://www.st.com/en/microcontrollers-microprocessors/stm32wb55ce.html
- LSM6DSV16B: https://www.st.com/en/mems-and-sensors/lsm6dsv16b.html
- BMM350: https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/
- DWM3000: https://www.qorvo.com/products/p/DWM3000
- BQ25185: https://www.ti.com/product/BQ25185
- MAX17048: https://www.analog.com/en/products/max17048.html
