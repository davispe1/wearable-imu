# gaitlib — Gait Kinematics library

`gaitlib` converts **raw 9-DOF inertial/magnetic sensor data** into **commonly measured
gait parameters**. It is a pure library: *data in, parameters out*. It has **no** hardware,
serial-port, network, or 3D-visualisation dependencies — acquisition and visualisation are
separate layers handled elsewhere.

```python
import gaitlib

results = gaitlib.compute(raw_data, mounting_config)

results.gait["cadence_steps_per_min"]            # bout cadence
results.joints["knee"]["params"]["rom_deg"]      # steady-state knee ROM
results.joints["knee"]["flexion"]                # full flexion trace (deg)
```

---

## 1. Public API

| call | purpose |
|---|---|
| `gaitlib.compute(raw_data, mounting_config=None) -> GaitResults` | the single entry point |
| `gaitlib.default_config(side="right", imu_hz=100.0, mag_hz=None, run_modes=("6dof",))` | the default 4-sensor, one-leg config |
| `gaitlib.both_legs_config(...)` | 7-sensor, two-leg config (6 joints) |
| `gaitlib.MountingConfig(...)` | build any topology by hand |

`compute` knows how Python **calls** the library — it has nothing to do with serial ports,
sockets, pins, or registers. Those belong to the acquisition layer that *produces*
`raw_data`.

### `GaitResults`

| attribute / method | contents |
|---|---|
| `.fs`, `.t` | common sample rate (Hz) and time grid (s) |
| `.joints[name]` | `{"flexion", "ang_vel", "ang_acc", "params"[, "modes"]}` |
| `.gait` | cadence, step/stride time, stance/swing, stride counts |
| `.events` | `foot_strike`, `mid_swing`, `toe_off` (sample indices) |
| `.steady_state` | boolean mask (turnarounds excluded) |
| `.turnarounds`, `.warnings`, `.config`, `.meta` | bout turns, skipped-joint notes, the config used |
| `.summary()` / `.save_summary_json(p)` | all parameters as a JSON-able dict |
| `.save_timeseries_csv(p)` | full per-sample output |
| `.joint_angle_table()` / `.save_joint_angles_csv(p)` | computed joint angles **for manual goniometer comparison** |

---

## 2. Input data contract

`raw_data` is raw 9-DOF samples. Per sample:

| field | meaning | units |
|---|---|---|
| `timestamp` | monotonic per node, any 0-origin | s |
| `node_id` | sensor id; must match a key in the mounting config | — |
| `ax, ay, az` | linear acceleration, sensor frame | m/s² |
| `gx, gy, gz` | angular velocity, sensor frame | rad/s |
| `mx, my, mz` | magnetometer, sensor frame | any consistent units (calibration normalises scale + bias) |

This is the contract the firmware/hub must satisfy. `compute` accepts it in three equivalent
forms (see `gaitlib/rawdata.py`):

1. **Per-node dict** — `{node_id: {"t","acc","gyr","mag"[, "t_mag"]}}` (the canonical form).
2. **Long-format rows** — an iterable of per-sample dicts with the columns above.
3. **Long-format array** — a structured/record numpy array whose field names match (aliases
   such as `acc_x`, `time_s`, `node` are accepted).

### Sample-rate handling

The ideal/default case is **all 9 DOF synchronous at the IMU rate** (e.g. 100 Hz) — then no
resampling happens. The library also accepts a **magnetometer at a different rate**: give it
on its own `t_mag` timeline (or just a different length) and it is linearly interpolated onto
the accel/gyro instants so fusion sees one synchronous 9-DOF stream. Per-channel rates are
declared in the config (`rates.imu_hz`, `rates.mag_hz`).

> **Firmware note.** Configure the magnetometer at the same ODR as the IMU (e.g. 100 Hz) so
> the 9 DOF are already synchronous and no resampling is needed.

---

## 3. Mounting config (sensor-count agnostic)

The mounting config is the key design point: the library is agnostic to the number of
sensors. It declares which node ids are present, each sensor → body segment, and the joint
topology (which two segments form each joint).

```python
cfg = gaitlib.MountingConfig(
    sensors={"RF": "foot", "RS": "shank", "RT": "thigh", "SA": "pelvis"},
    joints={"ankle": ("RF", "RS"),     # (distal, proximal); angle = distal vs proximal
            "knee":  ("RS", "RT"),
            "hip":   ("RT", "SA")},
    foot_node="RF",        # used for gait events
    pelvis_node="SA",      # used for turnaround detection (optional)
    rates={"imu_hz": 100.0, "mag_hz": 100.0},
    strict=False,          # skip joints with missing sensors (default) vs hard-fail
)
```

**Default** (`default_config()`): 4 sensors on one leg — `{foot, shank, thigh, pelvis}` →
joints `{ankle, knee, hip}`. **Both legs** (`both_legs_config()`): add the second leg's
three sensors and three joints — nothing else changes.

**Missing-sensor handling.** By default every joint whose two segments are *both* present is
computed; the rest are skipped and listed in `results.warnings` (no hard fail). Set
`strict=True` to raise instead.

---

## 4. Pipeline stages, equations & symbols

The pipeline reuses the project's validated kinematic core, unchanged in substance. Stages,
in order:

### Notation

| symbol | meaning |
|---|---|
| `q = [w,x,y,z]` | unit quaternion, sensor→earth, scalar-first |
| `a` | accelerometer (specific force), m/s² |
| `ω` | gyroscope, rad/s |
| `m` | magnetometer (sensor frame) |
| `g₀` | neutral (static) gravity direction in sensor frame |
| `R(q)` | rotation matrix from `q` (sensor→earth) |
| `j` | joint mediolateral (flexion) axis, sensor frame |
| `θ_s` | segment sagittal rotation of gravity about `j` from neutral |
| `β` | Madgwick gradient-descent gain |
| `τ` | complementary-filter time constant |
| `fs` | common sample rate (Hz) |

### Calibration — `gaitlib/calibration.py` (9-DOF only)

Hard/soft-iron ellipsoid fit from the recording's own varied-orientation samples: solve the
algebraic quadric `D v = 1` for `Q, n`, then

```
b (hard-iron center) = −Q⁻¹ n
k = 1 + nᵀ Q⁻¹ n
A (soft-iron)        = sqrtm(Q / k)        # symmetric PD root
```

so `A·(m − b)` lies on the unit sphere. A signed axis permutation `P` (48 candidates) aligns
the mag chip axes to the accel frame by minimising the **dip-angle variance** ∠(`P·m̂`, `â`).
Applied as `m_cal = P · A · (m − b)`. Below `min_samples`, an identity calibration is used.
6-DOF (the primary mode) needs none of this.

### Filtering / rate alignment — `gaitlib/rawdata.py`

Per-channel rate alignment (mag → IMU instants, §2) plus resampling of each segment's
accel/gyro onto a common `fs` grid by linear interpolation; quaternions are resampled with
hemisphere-continuous nlerp (`gaitlib/angles.py:slerp_resample`).

### Orientation fusion — `gaitlib/fusion.py`

Madgwick filter, `dt = 1/fs`, rate term `q̇_ω = ½ q ⊗ [0, ω]`:

```
q̇ = q̇_ω − β · ∇f/‖∇f‖ ;   q ← normalize(q + q̇ · dt)
```

*6-DOF* gravity objective (`a` normalised):

```
f = [ 2(q1q3 − q0q2) − ax,  2(q0q1 + q2q3) − ay,  2(½ − q1² − q2²) − az ]
∇f = Jᵀ f
```

*9-DOF MARG* adds the magnetometer objective (earth-field reference `h = q ⊗ [0,m] ⊗ q*`,
`bx = √(h₁²+h₂²)`, `bz = h₃`); mag constrains heading only. Gains: `β_6dof = 0.033`,
`β_9dof = 0.05`. **6-DOF is primary**: sagittal flexion is yaw-immune, so the (often
indoor-distorted) magnetometer is not needed.

### Joint angles — `gaitlib/angles.py`

The joint axis `j` per segment is the **largest-variance gyro direction** (top eigenvector of
`Σ ωωᵀ`), estimated on steady-state samples only (turns excluded). With gravity in the sensor
frame `g = R(q)ᵀ[0,0,1]`, each segment's sagittal rotation from neutral is the signed angle of
`g` about `j` relative to `g₀`:

```
g₀⊥ = normalize(g₀ − (g₀·j) j),  g⊥ = normalize(g − (g·j) j)
θ_s = atan2( (g₀⊥ × g⊥)·j ,  g⊥·g₀⊥ )
flex_grav = unwrap(θ_distal − θ_proximal)         # drift-free, heading-free
```

A complementary filter (`τ = joint_tau_s = 0.3 s`) blends this with the integrated joint rate
`ω_joint = ω_distal·j_d − ω_proximal·j_p` for fast motion:

```
α = τ/(τ+dt)
flex[i] = α·(flex[i−1] + ω_joint[i]·dt) + (1−α)·flex_grav[i]
```

Angular velocity/acceleration are `np.gradient` of flexion; `ROM = max − min`.

### Gait events — `gaitlib/gait.py`

Foot sagittal rate `s = ω · ĵ` (foot largest-variance axis, oriented so swing peaks are
positive). **Mid-swing** = `find_peaks(distance = min_stride_s·fs, height = 0.5·P95(|s|))`;
**foot strike** = min of `s` in the 0.5 s after each mid-swing; **toe-off** = min in the 0.4 s
before. Cadence: stride times `Δt = diff(strike)/fs` (steady mask, `Δt < 2.5 s`),
`cadence = (1/mean Δt)·60·2` steps/min.

### Parameters — `gaitlib/parameters.py`

**Per joint:** ROM (steady & full), peak min/max flexion, peak |angular velocity|, repetition
count (flexion-cycle peaks over the steady trace), active duration (steady samples with
`|ω| > active_vel_dps`). **Gait:** cadence, stride/step time (mean, std), stance/swing %
(from toe-off vs foot-strike), stride counts.

---

## 5. Accuracy validation (manual, not optical)

`gaitlib` **never requires optical or marker reference data** to run. Optical ground truth was
only a past validation aid for the research pipeline. Real-hardware accuracy validation is
**manual**: compare the exported computed joint angle against a **goniometer** or known
reference angles. Export the trace with:

```python
res.save_joint_angles_csv("knee_for_goniometer.csv")   # t_s + <joint>_deg per joint
```

## 6. Equivalence / regression test

`gaitlib/tests/test_geneva_regression.py` runs `compute` on the bundled Geneva slice and
checks it reproduces the previously validated parameters (cadence ~109, steady ROM ankle
~53.3° / knee ~86.4° / hip ~53.8°, 99 strides). This is an **equivalence check** that the
refactor matches the validated pipeline — **not** an accuracy validation.

```
python gaitlib/tests/test_geneva_regression.py
python -m pytest gaitlib/tests/ -q
```
