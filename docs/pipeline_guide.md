# Mythos MECH — Pipeline Guide

> **See also:** [`methods.md`](methods.md) — full per-stage technical/methods manual
> (algorithms, equations, parameters); [`user_guide.md`](user_guide.md) — usage manual
> (run commands, config knobs, output schemas, reading the visuals/metrics). This file is a
> concise overview; the two manuals are the detailed references.

Wearable lower-limb IMU kinematics pipeline. It performs **9-DOF-capable sensor fusion**
on the Grouvel et al. 2023 (Geneva) dataset — 8 Physilog 6S IMUs — and produces gait /
biomechanical parameters plus a 3D visualization. It is a rehearsal for real wearable
hardware: the kinematic core consumes raw accel + gyro + magnetometer only, exactly as
the live device will provide.

Default selection (all config-driven in `config/default.yaml`): subject **P01**, task
**2minWalk**, **right leg**, nodes **RF/RS/RT/SA** (foot, shank, thigh, pelvis), joints
**ankle (RF–RS), knee (RS–RT), hip (RT–SA)**.

---

## Data flow

```
READ-ONLY DATASET (Pxx/RAW_DATA/*.BIN, *.c3d ; Pxx/SYNC_DATA/*.csv)
        │
   bin_reader.py     decode Physilog .BIN (page-aware) -> accel/gyro/MAG (all 256 Hz) + baro
        │
   align.py          per-sensor optical-clock alignment (timestamp + constant skew);
        │            sync_data used ONLY to calibrate skew & confirm decode (never fused)
   extract.py        SI convert -> delimit walking segment from foot activity ->
        │            inter-sensor impact refinement -> write data/<trial>/<node>.csv slices
        │            (t_native, t_opt, accel m/s^2, gyro rad/s, mag counts)
        │
   ══════ RAW-DATA CONTRACT WALL ══════ (adapters/geneva.py -> IMUTrial.imu)
        │
   kincore/          calibration.py  magnetometer hard/soft-iron ellipsoid + frame align
        │            fusion.py       Madgwick 6-DOF (primary) & 9-DOF, per-sensor BIN clock
        │            angles.py       yaw-immune sagittal joint flexion (gravity about joint
        │                            axis) + complementary filter; angular vel/acc, ROM
        │            segment.py      pelvis turnaround detection + steady-state mask
        │            gait.py         foot-IMU step events, cadence, stride stats
        │
   run.py            orchestrates the above -> outputs/<...>_timeseries.csv + _summary.json
        │
   validation/       reference.py    marker-cluster + joint-centre reference angles (READ
        │                            markers ONLY here); sub-sample RMSE; optical heading
        │                            arbiter; Zeni gait events
        │
   visualize.py      pyqtgraph 3D stick figure + 2D overlays (--save -> PNG dashboard)
   selftest.py       RAW-DATA CONTRACT proof
```

---

## Files

| File | Role |
|------|------|
| `bin_reader.py` | Reverse-engineered GaitUp/Physilog `.BIN` decoder. 512-byte pages, 8-byte records `tag+counter+3×int16 BE`. Tags: `0x13` accel, `0x14` gyro, **`0x18` magnetometer (256 Hz)**, `0x15` barometer (64 Hz). |
| `align.py` | Locates each trial inside the continuous ~69-min BIN via the c3d absolute timestamp + a constant per-sensor clock skew (calibrated from long trials). `sync_data` is confined here. |
| `extract.py` | Per-sensor SI conversion, walking-segment delimiting, inter-sensor impact refinement, writes `data/` slices + `extract_report.json`. |
| `adapters/geneva.py` | `IMUTrial(imu, reference, labels)` and the `GenevaAdapter`. The contract boundary. |
| `kincore/` | Self-contained kinematic core (calibration, fusion, angles, gait, segment). Sees IMU only. |
| `validation/reference.py` | Marker-derived reference angles, RMSE, heading arbiter, Zeni events. The only marker reader. |
| `run.py` | Orchestrator: `compute_core` (IMU-only) + `validate` (markers) + `write_outputs`. |
| `visualize.py` | 3D animated right-leg stick figure + 2D dashboard (`--save` for PNG). |
| `selftest.py` | Proves the core is a pure function of the IMU. |
| `config/default.yaml` | All selection + parameters. |

---

## Run commands

```powershell
# 1. Stage 1 — decode + align + write data/ slices (idempotent, READ-ONLY on dataset)
python extract.py

# 2. Full pipeline — fusion, joint angles, gait, validation, outputs
python run.py
#   -> outputs/P01_S01_2minWalk_timeseries.csv  (computed + reference + error + vel + acc + steps)
#      outputs/P01_S01_2minWalk_validation.csv  (per-window RMSE)
#      outputs/P01_S01_2minWalk_summary.json    (RMSE, ROM, cadence, caveats)

# 3. Visualization
python visualize.py --save          # headless: writes _dashboard.png + _stickfigure.png
python visualize.py                 # interactive pyqtgraph 3D stick figure

# 4. Contract selftest
python selftest.py                  # PASS = core independent of markers/labels

# Everything is config-driven; change subject/task/leg/nodes in config/default.yaml.
```

Dependencies: `numpy scipy pandas ezc3d pyyaml matplotlib` (core) and
`pyqtgraph PyOpenGL PyQt5` (interactive 3D). See `requirements.txt`.

---

## Key results (P01 / 2minWalk / right leg)

- Walking segment: continuous **128.7 s** bout (the real 2-min walk), 7 turnarounds excluded from gait stats.
- Gait: **cadence 109 steps/min**, stride **1.10 ± 0.07 s**, 99 steady strides.
- Joint-angle RMSE vs optical (lag-optimised, offset-removed): **ankle 8.2°, knee 14.0°, hip 8.8°** (knee is fastest-moving, hardest).
- **Magnetometer verdict:** 9-DOF does **not** improve heading. Optical arbiter: pelvis heading RMSE **6-DOF 5.7° vs 9-DOF 5.9°**. The indoor force-plate lab distorts the field (|B| swings 3.7–15% during the walk; inclination ≈ 88° vs Geneva's ~63°). 6-DOF is the primary orientation; 9-DOF is logged for comparison.

---

## Swap point to live hardware

The live device streams raw accel + gyro + magnetometer per sensor. To switch from the
dataset to hardware, replace **only** the data-ingest layer:

1. Implement a new adapter alongside `adapters/geneva.py` that fills `IMUTrial.imu` with
   `{t_native, t_opt, acc (m/s²), gyr (rad/s), mag (counts)}` per node from the live
   stream. **Nothing downstream of the contract wall changes.**
2. `kincore/` runs unchanged (fusion, angles, gait). 6-DOF stays primary; enable 9-DOF
   per sensor once the magnetometer is calibrated and **field quality is gated** (see
   caveats — naïve 9-DOF degraded heading indoors here).
3. `bin_reader.py`, `align.py`, `extract.py` are dataset-specific (file decode + optical
   clock alignment) and are **not needed** with a live device that already provides
   time-synchronized, single-clock samples — the hardest problems this pipeline solved
   (proprietary decode, multi-file clock skew) disappear on real hardware.
4. Validation (`validation/`) is optional on hardware (needs an optical reference); the
   `selftest.py` contract check still applies.

---

## Caveats (read before trusting numbers)

1. **Magnetometer rate.** The dataset documentation lists the magnetometer at 64 Hz, but
   in the `.BIN` the magnetometer (tag `0x18`) arrives at **256 Hz, sample-aligned with
   accel/gyro on the same record stream** — so no 64→256 upsampling is needed and the mag
   is aligned by construction. The 64 Hz channel (`0x15`) is the **barometer** (proven:
   only two scalar fields, cannot be a 3-axis vector). Its useful bandwidth may still be
   ~64 Hz, but every accel/gyro sample has a co-timed mag sample.

2. **Reference angles are derived, not pre-computed.** They come from the c3d 4-marker
   clusters and joint centres (RHJC/RKJC/RAJC), zeroed at the `Static_01` neutral pose.
   They are read **only** in `validation/` and never enter the core (proven by
   `selftest.py`). Validation RMSE is offset-removed (sensor-mounting offset not
   penalised) and sub-sample aligned within ±0.3 s.

3. **Magnetometer calibration.** Hard/soft-iron ellipsoid + magnetometer→accelerometer
   frame alignment, fitted from varied-orientation windows (CalibrationTask + TUG, not
   walking/sitting). The indoor field is distorted (high ellipsoid residual, wrong
   inclination), which is *why* 9-DOF does not help here. For real hardware, gate the
   magnetometer on field quality (|B| deviation, inclination consistency).

4. **Inter-sensor timing.** The 8 IMUs are separate `.BIN` files with different RTC start
   times; each is aligned to the optical clock independently. Adjacent leg sensors
   (RF/RS/RT) refine to 4–23 ms via shared heel-strike impacts. The **pelvis (SA) could
   not be refined below ~0.4 s from IMU alone** (impacts damped, periodic signal
   stride-ambiguous) — and marker-based timing would break the raw-data contract — so the
   **hip** carries that timing uncertainty (still corr 0.92–0.97 vs optical).

5. **Subject-specific.** P01 has no insoles; P05 is missing the LT sensor. The default
   P01 + right leg avoids both. Segment lengths in the stick figure are nominal.
