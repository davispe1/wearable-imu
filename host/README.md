# host/

PC-side processing pipeline and 3D visualizer. **Git-versioned — no version folders.**
Tag releases as `imu-vX.Y.Z` alongside the firmware and hardware revision they target.

## Structure

```
host/wearable_imu/
├── ingest/       # serial / BLE / SWO readers → raw sample stream
├── sync/         # multi-node timestamp alignment
├── orientation/  # accel+gyro(+mag) → world-frame quaternion
├── kinematics/   # joint angles + Denavit-Hartenberg forward kinematics
├── ekf/          # phase 2: UWB-distance fusion for drift/position
├── viz/          # live 3D skeleton viewer
├── sim/          # synthetic data generator (develop pipeline without hardware)
├── config.py     # runtime configuration (sample rate, node count, data format …)
└── main.py       # entry point
```

## Setup

```bash
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python host/wearable_imu/main.py
```

## Development order (README §7)

1. Drive pipeline with `sim/` synthetic data — no hardware needed.
2. Add `ingest/` SWO reader for wired bring-up.
3. Add `ingest/` BLE reader once firmware BLE is ready.
