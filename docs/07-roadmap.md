# 07 — Roadmap & Open Items

## Build phases

| Phase | Milestone |
|-------|-----------|
| 1 | Firmware + data path — node sampling, UWB TDMA, master aggregation, host ingest over SWD/SWO |
| 2 | Visualization — orientation → joint angles → DH → live 3D (synthetic data first) |
| 3 | BLE uplink — integrate after wired path is proven |
| 4 | EKF — Kalman filter fusing UWB inter-node distances for drift/position |

## Open items

- [ ] **Data format:** raw 9-DOF vs SFLP quaternion + raw mag
- [ ] **Magnetometer:** use or skip for v1 (characterize lab magnetic environment first)
- [ ] **Battery capacity:** 300 mAh vs 600 mAh (final size TBD)
- [ ] **Node count for initial tests:** 3 (single arm) → 6–8
- [ ] **Sample rate:** confirm 100 Hz vs 200 Hz
- [ ] **BLE receiver:** host-native vs nRF52840 USB dongle
- [ ] **UWB backup dongle:** build or defer
- [ ] **Orientation filter:** complementary vs Madgwick vs VQF vs SFLP
- [ ] **EKF scope:** confirm phase-2 (not v1)
