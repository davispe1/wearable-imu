# Synthetic gait data (ground-truthed fixture)

The pipeline's raw-data contract (per-node 9-DOF CSVs on a shared hub clock; see
`core/rawdata.py`) is satisfied by real Physilog sessions, but real data carries **no ground
truth** — you never know the true hip/knee/ankle angle a subject walked, so you can only *run*
the pipeline, not *score* it.

`tools/make_synthetic_session.py` generates a session that (a) lands exactly in that CSV contract
and (b) ships the **known** ground truth (`ground_truth.json`), so the whole chain can be
validated end-to-end. The shipped instance is **`data/SYN01_S01_straightWalk`**: a short, clean,
straight walk at a normal pace (10 strides, ~109 steps/min) with a quiet-standing lead/tail, one
instrumented right leg (`SA/RT/RS/RF`), a magnetometer, at 100 Hz.

## Why it is reliable: forward simulation (orientation-first)

The signals are **not hand-drawn**. We prescribe the true kinematics — each segment's orientation
`R(t)` (sensor→earth) from a normative sagittal gait model, plus the foot's world trajectory —
then **derive** every sensor channel by the correct physics, in the exact conventions the pipeline
speaks (`kinematics/quaternion.py`):

| channel | formula | units |
|---|---|---|
| gyroscope | `vee(R^T · dR/dt)` (body angular velocity) | rad/s |
| accelerometer | `R^T · (d²x/dt² − g)` (specific force), `g = [0,0,−9.81]` | m/s² |
| magnetometer | `R^T · B` (constant earth field, incl. 60°) | µT |

Because the accelerometer is a genuine specific force and the gyroscope is the true derivative of
`R`, VQF re-fuses them back into `R` (up to the heading it can observe). That round-trip is the
guarantee: the pipeline **must** recover what we put in, so any discrepancy is a real regression.

A short **quiet-standing** lead/tail makes the pipeline's neutral pose (its quietest window) a true
standing pose, so reported flexion reads ~0° at stance and matches the ground truth absolutely. A
small **loading-response transient** at each heel strike supplies the impact feature real IMUs show
(and idealised rigid-body kinematics lack); without it the gyro event detector's initial-contact
lands ~0.1 s late and inflates the swing fraction. It is injected in the *angle* as a zero-integral
wavelet, so joint ROMs are untouched — only the events sharpen.

## Generate & validate

```bash
python -m tools.make_synthetic_session            # writes data/SYN01_S01_straightWalk/
python -m tools.validate_synthetic                # scores the pipeline vs ground_truth.json
```

`validate_synthetic` prints a recovered-vs-truth table and exits non-zero if any metric leaves
tolerance — so it doubles as a **regression anchor**. Representative recovery (100 Hz, clean):

| metric | truth | recovered | err |
|---|---|---|---|
| hip / knee / ankle ROM | 39.7 / 55.1 / 23.6° | 39.7 / 56.4 / 23.5° | ≤1.3° |
| cadence | 109.1 /min | 109.1 /min | 0 |
| stance / swing | 62 / 38 % | 61.8 / 38.2 % | 0.2 |
| stride length / speed | 1.35 m / 1.23 m/s | 1.35 m / 1.23 m/s | ~0 |

That is optical-mocap-level agreement — exactly what IMU gait analysis claims.

## The magnetometer's role (what it does, and does not, change)

Verified on this fixture, honestly:

- **It does not change the joint angles.** 6D and 9D give identical joint ROMs (max diff 0.00°),
  because the sagittal angles are yaw-immune (gravity projection about the functional axis). This
  is why **6D is the safe default** — an uncalibrated/disturbed magnetometer cannot corrupt the
  clinical angles.
- **It adds an absolute heading.** 9D references every segment to the magnetic field, giving one
  consistent, field-anchored heading (segment spread ~0.3°); 6D's heading is an arbitrary
  per-segment zero. This matters for world-frame outputs — the OpenSense export and 3D view.
- **It is not a blind drift-fix.** VQF's magnetic-disturbance rejection means that when a biased
  gyro and the magnetometer disagree *persistently*, VQF treats the field as disturbed and follows
  the gyro. So 9D does **not** magically cancel gyro-heading drift (verified with `--gyro-drift`).

## Options / limitations

`--strides --stride-time --stride-length --heading-deg --fs` shape the bout; `--impact` sets the
heel-strike amplitude; `--gyro-drift` injects a known heading-axis bias; `--noise` adds gaussian
sensor noise (off by default). This is an **idealised** planar-sagittal fixture: no soft-tissue
artefact, and (by default) no sensor noise or mounting misalignment. It is a testing / demo /
regression asset — **not** a substitute for real data or a basis for clinical-accuracy claims.

## Precedent

Synthetic IMU (including magnetometer) from prescribed motion is standard practice: **IMUSim**
(Young et al., 2011) simulates accel+gyro+mag from a trajectory; **OpenSense** derives virtual IMU
orientations from a model's motion; and the mocap→virtual-IMU line for learning (**DIP** 2018,
**TransPose** 2021) synthesises IMU from SMPL/AMASS — the same ecosystem as the vendored `SMPL/`
and `smpl_viewer/`. This generator is a small, fit-for-purpose instance of that method.
