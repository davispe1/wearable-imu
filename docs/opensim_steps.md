# OpenSim OpenSense — step by step

This runs entirely in the **OpenSim 4.5 GUI** (or the `opensense` command line). It consumes the
two `.sto` files written by:

```
python -m opensim_export.to_sto <session_dir>
# -> <session_dir>/results/<id>_orientations.sto
#    <session_dir>/results/<id>_calibration.sto
```

Templates for the two tool setups are in [`../opensim/setups/`](../opensim/setups/). Replace the
`SUBJECT_*` placeholders with your session id and keep `sensor_to_opensim_rotations` identical in
both files. Model background is in [`../opensim/README.md`](../opensim/README.md).

## 0. Gather files

Put these together (e.g. in the session's `results/` folder):

- `Rajagopal_2015.osim` — the Rajagopal model **with IMU frames** (`pelvis_imu`, `femur_*_imu`,
  `tibia_*_imu`, `calcn_*_imu`).
- `<id>_calibration.sto` and `<id>_orientations.sto`.
- `imu_placer.xml`, `imu_ik.xml` (edited copies of the templates).

## 1. Open the model

**File ▸ Open Model…** → `Rajagopal_2015.osim`. Confirm the IMU frames appear on the model
(Navigator ▸ the body's *Components*). The model opens in its default pose.

## 2. Lock the non-measured leg

This is a single-leg rig, so the contralateral leg has no orientation data and would otherwise
hang at its default pose / drift. Before tracking:

- In the **Coordinates** panel, set the non-measured leg's coordinates (e.g. `hip_*_l`,
  `knee_angle_l`, `ankle_angle_l` for a right-leg session) to a neutral value and **lock** them
  (padlock toggle). Lock the lumbar/back and arm coordinates too if you only care about the leg.
- This keeps IK focused on the measured leg and prevents unconstrained joints from wandering.

## 3. Calibrate — IMU Placer (sensor-to-segment registration)

**Tools ▸ Calibrate Model from IMU Data…** (or load `imu_placer.xml`). Set:

- **Model** = `Rajagopal_2015.osim`
- **Orientations file (calibration)** = `<id>_calibration.sto` (the single-row static pose)
- **Base IMU** = `pelvis_imu`, **Base heading axis** = `z`
- **Sensor-to-OpenSim rotations** = `-1.5708 0 0` (the template default; tune if needed)
- **Output model** = `<id>_calibrated.osim`

Run it. The tool aligns the model heading from the base IMU and registers each sensor to its
body using the static pose. **Check the preview**: the model should stand upright and face
forward. If it is rotated 90°/180° or lies on its side, adjust `sensor_to_opensim_rotations` and
re-run. The calibrated `.osim` is the input to IK.

## 4. Track — IMU Inverse Kinematics

**Tools ▸ IMU Inverse Kinematics…** (or load `imu_ik.xml`). Set:

- **Model** = `<id>_calibrated.osim` (from step 3)
- **Orientations file** = `<id>_orientations.sto` (the full trial)
- **Sensor-to-OpenSim rotations** = identical to step 3
- **Time range** = your trial window in seconds (the `.sto` `time` column is 0-based)
- **Report errors** = on → writes `<id>_ik.mot_orientationErrors.sto`
- **Output motion** = `<id>_ik.mot`

Run it. OpenSense solves per frame for the joint angles that best match the measured segment
orientations.

## 5. Visualize / export

- The solved motion auto-loads — press **play** to watch the model walk; scrub to inspect.
- **Plot** joint angles: **Tools ▸ Plot…**, y = `hip_flexion_<r|l>` / `knee_angle_<r|l>` /
  `ankle_angle_<r|l>`, x = time.
- The `.mot` is the joint-angle result; load it elsewhere or post-process as needed.

## 6. Sanity checks

- Open `<id>_ik.mot_orientationErrors.sto`: per-frame, per-IMU tracking error should stay small
  (roughly a few degrees). Large or growing errors point to a placement/calibration problem or a
  wrong `sensor_to_opensim_rotations`.
- Angles should sit in physiological ranges (e.g. knee flexion ~0° near heel strike, ~60° in
  swing). If a segment looks mirrored or offset, re-check its placement and the calibration pose.
- A magnetically disturbed environment degrades 9D heading — if angles look heading-corrupted,
  re-export with `--mode 6D`.
