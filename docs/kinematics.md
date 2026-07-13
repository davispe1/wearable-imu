# Kinematics — raw IMU to gait parameters (the library's primary product)

This is the **main** pipeline: four IMUs in, **kinematic parameters** out, visualized in Python.
The OpenSim/OpenSense export ([`method.md`](method.md)) is a *separate, optional* downstream
path — not part of this one.

```
raw IMU CSV ─► VQF orientation ─► sagittal joint angles ─► gait events ─► gait parameters ─► viewer
 core.rawdata    core.fusion_vqf   kinematics.joint_angles  kinematics.    kinematics.        app.viewer
                                                            gait_events    parameters
```

Everything that is easy to get subtly wrong — orientation estimation — is delegated to **VQF**, a
peer-reviewed estimator. Every parameter is grounded in published IMU-gait methods, most of them
developed on the **same Physilog hardware** that recorded the bundled Geneva dataset.

## Stage 1 — Load (`core.rawdata`)

Per-node 9-DOF CSVs (`RF.csv`, `RS.csv`, `RT.csv`, `SA.csv`) or a combined `raw/data.csv`, on one
shared clock (`t_opt_s`, 0 at start). Pure I/O — no orientation math. The example slices are
256 Hz, ~135 s, magnetometer present.

## Stage 2 — Orientation (`core.fusion_vqf`)

Each segment is fused **independently** by VQF into a sensor→earth, scalar-first quaternion.
**6D** (gyro+accel) is the default: roll/pitch are gravity-referenced and absolute, heading is
bias-corrected gyro integration. Because the joint angle uses only the **gravity** direction,
heading is irrelevant and 6D needs no magnetometer calibration. **9D** adds a magnetic-north
heading when wanted.

## Stage 3 — Joint angles (`kinematics.joint_angles`)

Sagittal **flexion** of hip, knee and ankle, without sensor-to-segment calibration:

1. **Functional joint axis** — the mediolateral (flexion) axis of each segment is the
   largest-variance direction of its gyroscope during walking (Seel, Raisch & Schauer 2014).
2. **Gravity-projection angle** — flexion is the angle the segment's fused gravity vector has
   swept *about that axis*, relative to a neutral pose. It uses only gravity, so it is
   **yaw-immune and drift-free**; VQF's bias-corrected orientation makes gyro integration
   unnecessary (the optional complementary refinement is off by default — integrating the gyro
   can leak out-of-plane motion into the angle).
3. **Joint flexion = distal tilt − proximal tilt** (hip = thigh−pelvis, knee = shank−thigh,
   ankle = foot−shank), with the two functional axes sign-aligned.

Angles are reported **relative to a neutral pose** (≈0° at quiet stance). The cycle **shape,
timing and ROM** are robust; an *absolute* anatomical offset would require a static calibration
pose — exactly what the separate OpenSim IMU-Placer path provides.

## Stage 4 — Gait events & steady-state (`kinematics.gait_events`)

- **Events from the shank gyroscope.** Each stride shows a large **mid-swing** peak; **initial
  contact** is the reversal just after it, **toe-off** the reversal just before — the ambulatory
  gyroscope method of Aminian et al. (2002) and Salarian et al. (2004). The foot gyro is the
  fallback if no shank is present.
- **Turnarounds → steady-state mask.** ~180° turns are detected from the pelvis vertical-axis
  angular rate and excluded from the gait statistics (which should describe straight walking).

## Stage 5 — Parameters (`kinematics.parameters`)

| Group | Parameters | Basis |
|---|---|---|
| Per joint | ROM, peak flexion/extension, peak & mean angular velocity, cycle count | Seel et al. 2014 |
| Temporal | cadence, stride & step time (mean±SD), stance/swing %, stride-time **CV** | Aminian 2002, Salarian 2004; CV ← Hausdorff 2005 |
| Spatial *(estimate)* | stride length, walking speed | foot-IMU ZUPT (Mariani et al. 2010) |

Per-joint ROM/peaks are reported **cycle-averaged** (over the mean gait cycle), which is robust to
the occasional bad stride near a turn. Stance/swing use the detected initial-contact and toe-off.
Spatial parameters are an **estimate**: world-frame foot acceleration (gravity removed via the
VQF quaternion) is integrated twice with **zero-velocity updates at mid-stance** (foot-flat) to
de-drift each stride; it is the least robust IMU-only output and is labeled as such.

## Output & visualization

`analyze_session(session_dir)` returns a `KinematicResults` and can write into
`<session>/results/`:

- `<id>_joint_angles.csv` — time · per-joint angle & angular velocity · event flags · steady flag
- `<id>_gait_events.csv` — one row per event (`t_s`, `sample`, `event`)
- `<id>_gait_parameters.json` — every parameter (per-joint + temporal + spatial)
- `<id>_kinematics.png` — the viewer figure (`app.viewer`)

```bash
python -m kinematics.pipeline <session_dir> --csv     # parameters → CSV/JSON
python -m app.viewer          <session_dir> --save    # figure + CSV/JSON  (--full, --t0/--t1)
```

## Validation snapshot (bundled example)

| Parameter | P04 | P01 | Normal walking |
|---|---|---|---|
| Hip / Knee / Ankle ROM | 46° / 69° / 34° | 40° / 55° / 32° | ~45° / ~60° / ~30° |
| Cadence | 100 /min | 108 /min | ~100–120 |
| Stride time | 1.20 s | 1.11 s | ~1.0–1.3 s |
| Stance / swing | 57 / 43% | 52 / 48% | ~60 / 40% |
| Stride length / speed *(est.)* | 1.43 m / 1.20 m/s | 1.58 m / 1.42 m/s | ~1.4 m / ~1.2 m/s |

## References

1. **VQF** — D. Laidig, T. Seel. *VQF: Highly Accurate IMU Orientation Estimation…* Information
   Fusion 91:187–204, 2023.
2. **Functional-axis joint angles** — T. Seel, J. Raisch, T. Schauer. *IMU-based joint angle
   measurement for gait analysis.* Sensors 14(4):6891–6909, 2014.
3. **Gyroscope gait events** — K. Aminian et al. *Spatio-temporal parameters of gait measured by
   an ambulatory system using miniature gyroscopes.* J. Biomechanics 35(5):689–699, 2002.
4. **Shank-gyro events** — A. Salarian et al. *Gait assessment in Parkinson's disease: toward an
   ambulatory system for long-term monitoring.* IEEE TBME 51(8):1434–1443, 2004.
5. **Foot-IMU stride length (ZUPT)** — B. Mariani et al. *3D gait assessment in young and elderly
   subjects using foot-worn inertial sensors.* J. Biomechanics 43(15):2999–3006, 2010.
6. **Gait variability (CV)** — J. M. Hausdorff. *Gait variability: methods, modeling and meaning.*
   J. NeuroEngineering and Rehabilitation 2:19, 2005.
7. **Dataset** — Grouvel et al., 2023 — Geneva dataset (Physilog 6S IMUs + optical + force plates).
