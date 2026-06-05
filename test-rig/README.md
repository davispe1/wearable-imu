# test-rig/

Self-contained sub-project: a motorised arm rig for collecting ground-truth joint
angles to validate the IMU pose estimation pipeline.

Follows the same structure as the main project:
- **Hardware version folders** under `hardware/`.
- **Firmware and control software** are git-versioned (no version folders).
- Tagged with `rig-vX.Y.Z` releases (separate from `imu-vX.Y.Z`).

## Structure

```
test-rig/
├── hardware/
│   ├── electronics/v1.0/   # rig servo driver / controller PCB
│   └── mechanical/v1.0/    # rig 3D files (arm links, servo mounts)
├── firmware/               # rig motion controller (git-versioned)
└── control/                # host-side: command rig + log ground-truth angles
```

## Purpose

1. Drive the rig through known joint angles.
2. Simultaneously record IMU node data.
3. Compare estimated angles to ground-truth → quantify filter error.
