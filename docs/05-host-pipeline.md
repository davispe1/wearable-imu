# 05 — Host Pipeline

> Expand on README §6.

## Stages

| # | Module | Input | Output |
|---|--------|-------|--------|
| 0 | calibration | T/N-pose capture | sensor-to-segment alignment matrices |
| 1 | orientation | accel + gyro (+ mag) | world-frame quaternion per segment |
| 2 | kinematics | adjacent quaternion pairs | joint angles |
| 3 | EKF *(phase 2)* | joint angles + UWB ranges | drift-corrected angles + positions |
| 4 | forward kinematics | joint angles + segment lengths | 3D joint positions |
| 5 | viz | 3D joint positions | live rendered skeleton |

## v1 path

Stages 0 → 1 → 2 → 4 → 5. No EKF needed for a working visualization.

## Orientation filter options

- **Complementary filter** — simplest, good starting point.
- **Madgwick** — well-documented, single tuning parameter (β).
- **VQF** — used by UIP (SIGGRAPH 2024 reference); handles magnetic disturbances well.
- **SFLP** — onboard LSM6DSV16B output; offloads host compute.

Decision open — see [open items](07-roadmap.md).

## Denavit-Hartenberg (DH) forward kinematics

TODO: define DH parameter table for the upper-limb model (shoulder → elbow → wrist).

## Synthetic data

`host/wearable_imu/sim/` generates synthetic IMU streams for pipeline development
before hardware is ready.
