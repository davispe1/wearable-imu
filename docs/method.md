# Method

A fundamentals-first IMU gait pipeline built from **validated components**. Raw inertial data
becomes joint kinematics in three stages, and the two stages that are easy to get subtly wrong
— orientation estimation and inverse kinematics — are delegated to peer-reviewed, widely used
implementations rather than home-grown math.

```
raw IMU CSV ──► VQF fusion ──► OpenSense orientation .sto ──► OpenSim OpenSense IK ──► joint angles
 (core.rawdata)  (core.fusion_vqf)   (opensim_export.to_sto)     (OpenSim GUI, separate)
```

## Why this rebuild

The earlier pipeline computed orientations with a custom Madgwick filter and joint angles with
custom sensor-to-segment geometry. Both are replaced here:

| Concern | Old (home-grown) | New (validated) |
|---|---|---|
| Orientation fusion | custom Madgwick/MARG | **VQF** (Laidig & Seel 2023) |
| Gyro bias / rest detection | none / ad hoc | VQF (built in) |
| Magnetic disturbance rejection | manual hard/soft-iron + heuristics | VQF (built in, 9D) |
| Sensor-to-segment calibration | custom | **OpenSim OpenSense** IMU Placer |
| Joint angles | custom sagittal projection | **OpenSim OpenSense** IMU Inverse Kinematics |

We keep only the part that was already sound: the **per-node CSV I/O on a shared (hub)
timebase** (`core/rawdata.py`).

## Stage 1 — Raw-data I/O (`core/rawdata.py`)

Per-node 9-DOF CSVs (`RF.csv`, `RS.csv`, `RT.csv`, `SA.csv`) or one combined `raw/data.csv`.
Each sample: time (s), accel (m/s², sensor frame), gyro (rad/s, sensor frame), magnetometer
(arbitrary units, sensor frame). All nodes share **one clock** (`t_opt_s`, 0 at bout start), so
sample *i* of every node is the same instant — the loader trusts this hub contract and only
trims to a common length. No cross-node resampling is performed.

## Stage 2 — Orientation fusion (`core/fusion_vqf.py`)

Each segment's accel/gyro(/mag) is fused **independently** with VQF into a world-frame
orientation quaternion. VQF performs rest detection, gyroscope-bias estimation, and (in 9D)
magnetic-disturbance rejection internally.

**6D vs 9D.**
- *6D* (gyr+acc, magnetometer-free): roll/pitch are gravity-referenced and absolute; heading
  is gyro-integrated with bias correction. Robust, needs **no magnetometer calibration** — the
  **default**.
- *9D* (gyr+acc+mag): heading is additionally tied to magnetic north, giving a consistent
  absolute heading across all segments. Best when magnetometers are calibrated and the
  environment is magnetically clean.

**Different per-channel sample rates.** VQF natively supports gyro, accel and magnetometer at
**different rates**. When the magnetometer arrives on its own clock, the real-time `VQF` filter
is fed each channel at its true rate (`gyrTs`/`accTs`/`magTs` from the config / timestamps) —
there is **no custom resampling**. When all channels are synchronous (the usual case on the hub
timebase), the acausal **`offlineVQF`** variant is used instead for the most accurate offline
estimate, free of startup transients (which matters for the static calibration window).

**Quaternion convention (verified).** VQF's `quat6D`/`quat9D` are **sensor → earth,
scalar-first `[w, x, y, z]`**: rotating the measured gravity vector by the quaternion yields
earth **+Z (up)**. We confirmed this empirically — a known 30° sensor tilt about earth-X is
recovered exactly, and `q ⊗ a_sensor ⊗ q* = [0,0,g]`. This is precisely the convention OpenSense
consumes; the IMU-axes-to-model-axes rotation is applied **inside** OpenSim
(`sensor_to_opensim_rotations`), not here. The fused quaternions are exported unchanged.

## Stage 3 — Export to OpenSense (`opensim_export/`)

`to_sto.py` writes two quaternion tables into `<session>/results/`:

- `<id>_orientations.sto` — the full trial, one quaternion per segment per frame.
- `<id>_calibration.sto` — a **single row**, the mean orientation over the first ~1 s (the
  static pose used to place the model on the subject).

Both carry the exact OpenSense header (`DataRate` / `DataType=Quaternion` / `version=3` /
`OpenSimVersion=4.5` / `endheader`), a `time` column, and one TAB-separated column per measured
segment whose cells are `w,x,y,z` (scalar-first). Column names come from
`segment_map.py` using the measured side (`right → r`):

```
pelvis → pelvis_imu      thigh → femur_<r|l>_imu
shank  → tibia_<r|l>_imu  foot  → calcn_<r|l>_imu
```

Only the measured leg gets columns — the contralateral leg is never fabricated. This step adds
**no OpenSim dependency**; it only writes text.

## Stage 4 — Kinematics in OpenSim OpenSense (separate app)

We do **not** compute joint angles in Python and do **not** implement sensor-to-segment
calibration. OpenSense does both: the **IMU Placer** registers each sensor to its body using
`<id>_calibration.sto`, and **IMU Inverse Kinematics** tracks `<id>_orientations.sto` on the
Rajagopal model to produce joint angles (`.mot`). Setup templates are in `opensim/setups/`;
step-by-step instructions in [`opensim_steps.md`](opensim_steps.md).

## References

1. **VQF** — D. Laidig, T. Seel. *VQF: Highly Accurate IMU Orientation Estimation with Bias
   Estimation and Magnetic Disturbance Rejection.* Information Fusion 91:187–204, 2023.
   doi:10.1016/j.inffus.2022.10.014.
2. **OpenSense** — M. Al Borno, J. O'Day, V. Ibarra, J. Dunne, A. Seth, A. Habib, C. Ong,
   J. Hicks, S. Uhlrich, S. Delp. *OpenSense: An open-source toolbox for inertial-measurement-
   unit-based measurement of lower extremity kinematics over long durations.* Journal of
   NeuroEngineering and Rehabilitation 19:22, 2022. doi:10.1186/s12984-022-01001-x.
3. **Rajagopal model** — A. Rajagopal, C. L. Dembia, M. S. DeMers, D. D. Delp, J. L. Hicks,
   S. L. Delp. *Full-body musculoskeletal model for muscle-driven simulation of human gait.*
   IEEE Transactions on Biomedical Engineering 63(10):2068–2079, 2016.
4. **Dataset** — Grouvel et al., 2023 — the Geneva dataset (Physilog 6S IMUs + optical markers
   + force plates + pressure insoles) used for the bundled example slices.
