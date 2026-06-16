# Gait Kinematics — Methods Manual

Technical reference for the IMU gait pipeline, ordered by pipeline stage. Every parameter
below is the real value in the source (file + function cited). Equations are written as
implemented.

Default run: subject **P01**, task **2minWalk**, **right** leg, nodes **RF/RS/RT/SA**
(foot, shank, thigh, pelvis), joints **ankle (RF–RS), knee (RS–RT), hip (RT–SA)**.

## Notation

| symbol | meaning |
|---|---|
| `q = [w,x,y,z]` | unit quaternion, sensor→earth, scalar-first |
| `a` | accelerometer (specific force), m/s² |
| `ω` | gyroscope, rad/s |
| `m` | magnetometer, raw counts (tag 0x18) |
| `g₀` | neutral (static) gravity direction in sensor frame |
| `R(q)` | rotation matrix from `q` (sensor→earth) |
| `j` | joint mediolateral (flexion) axis, sensor frame |
| `θ_s` | segment sagittal rotation of gravity about `j` from neutral |
| `β` | Madgwick gradient-descent gain |
| `τ` | complementary-filter time constant |
| `fs` | 256 Hz device rate; marker rate 100 Hz; force plates 1000 Hz |

---

## Stage 1 — BIN decode (`bin_reader.py`)

**Input:** `Pxx_S01_<sensor>_Inertial_sensor.BIN` (GaitUp/Physilog 6S, proprietary, ~28 MB,
one continuous file per sensor for the whole ~69-min session).
**Output:** `BinData` with raw int16 channel arrays (`acc_raw`, `gyr_raw`, `mag_raw`,
`baro_raw`) + metadata.

**Format (reverse-engineered; `read_bin`).** The file is a sequence of **512-byte pages**.
Page 0 is a config header (`50 35`="P5"-framed packets terminated by `ff fe`; a datetime
packet `03 00` carries the RTC start, year big-endian). Every later page = an **8-byte
page header** (`uint32_BE page_index` + `uint32_BE cumulative_sample_counter`) followed by
**63 fixed 8-byte records** (`8 + 63·8 = 512`; constants `PAGE_SIZE=512`, `PAGE_HEADER=8`,
`RECORD=8`, `RECORDS_PER_PAGE=63`).

**Record** = `tag(1) + counter(1) + 3×int16 big-endian (6)`. Channel tags
(`TAG_ACC=0x13`, `TAG_GYR=0x14`, `TAG_MAG=0x18`, `TAG_BARO=0x15`):

| tag | channel | rate | identification |
|---|---|---|---|
| `0x13` | accelerometer | 256 Hz | one axis ≈ +1 g at rest |
| `0x14` | gyroscope | 256 Hz | ≈ 0 at rest |
| `0x18` | **magnetometer** | 256 Hz | stable vector magnitude, gravity-decorrelated (Stage 4) |
| `0x15` | barometer/housekeeping | 64 Hz | two scalar fields, not a 3-vector |

The parse is **page-aware** and fully vectorised: reshape to `(n_pages,512)`, drop page 0,
take bytes `[8:512]` per page as 63 records, read tags and `>i2` payloads. Records whose
tag is not one of the four known tags are dropped (**~0.002 %** in practice).

**Count→SI scaling** (module constants `ACC_TO_MS2`, `GYR_TO_RADS`):
```
accel  [m/s²]  = counts · GRAVITY / ACC_COUNTS_PER_G      = counts · 9.80665 / 2048
gyro   [rad/s] = counts · (1 / GYR_COUNTS_PER_DPS) · π/180 = counts · (1/16.384) · π/180
mag    [counts]= left raw (absolute scale unknown; calibration normalises it)
```
(Accel ±16 g ⇒ 32768/16 = 2048 counts/g; gyro ±2000 °/s ⇒ 32768/2000 = 16.384 counts/(°/s).)

**Assumptions / limits.** Format reverse-engineered (no official Python reader); validated
by physics (accel |a|≈9.79 m/s², gyro≈0 at rest) and by cross-correlation against the
`sync_data` export. The magnetometer's absolute scale/units are unknown — only its
direction is used (calibration normalises magnitude).

---

## Stage 2 — Trial alignment (`align.py`)

**Input:** decoded BIN (per sensor) + the trial's `c3d` (absolute capture time) +
`sync_data` CSV (100 Hz accel/gyro, used here only).
**Output:** `AlignResult` (BIN sample index of the trial window) and a per-sensor clock
skew.

**Why.** The BIN is one 69-min file; a 3-s walking template is *not* unique across it
(blind cross-correlation gives ambiguous ~0.4 peaks). The c3d carries
`TRIAL/TIMECAPTURESTART` (absolute), the BIN header carries the RTC start; they differ by a
small, near-constant **session skew**.

**Skew estimate (`estimate_session_skew`).** For every long `sync_data` trial
(`len ≥ min_len_s = 20 s`): predicted location `pred = (c3d_capture_time − bin_rtc)` [s];
measured location `loc = argmaxₖ NCC(BIN_gyro_mag↓, sync_gyro_mag)` where the BIN gyro
magnitude is band-matched to 100 Hz with `resample_poly(25, 64)` (256→100). The skew is the
**median of `loc − pred` over trials with correlation ≥ `min_corr = 0.6`**. For P01 this is
**−12.14 s** (confident anchors −11.9…−12.7 s). Drift check: the skew holds at ≈ −12 s
across 1481–2720 s of the session (mid-session anchors −12.1 ± 0.3 s; late TUG anchors
−10.4…−11.8 s), i.e. cross-session drift ≲ ±1.5 s worst case.

**Normalized cross-correlation (`ncc_valid`).** Sliding Pearson `r(k)` of a template `t`
over a signal `s`, O(N) via FFT numerator + cumulative-sum window statistics:
```
num(k) = (s ⋆ t_demeaned)[k]                       # fftconvolve, 'valid'
r(k)   = num(k) / sqrt( var_window(k) · Σ(t−t̄)² )
```

**Per-trial localisation (`align_trial`).** `pred_idx = round((capture − rtc + skew)·fs)`.
A refine is attempted in a ±`refine_window_s = 4 s` window but applied **only if its
correlation ≥ `refine_min_corr = 0.5`**; short walking templates do not clear this, so the
timestamp+skew anchor (good to ~±0.4 s) is kept.

**Assumptions / limits.** `sync_data` is confined to this module (skew calibration + decode
confirmation); it never enters fusion. Walking windows rely on the timestamp anchor, not
the (unreliable) waveform refine.

---

## Stage 3 — Extraction & segmentation (`extract.py`)

**Input:** decoded BIN (all node sensors). **Output:** per-sensor SI slices
`data/<subj>_<sess>_<task>/<node>.csv` (`t_native, t_opt, ax..az, gx..gz, mx..mz`) +
`extract_report.json`.

**SI conversion (`to_si`).** Accel→m/s², gyro→rad/s as in Stage 1; **magnetometer kept in
raw counts**. The magnetometer is **tag 0x18 at 256 Hz, sample-aligned with accel/gyro on
the same record stream — no 64→256 upsampling is performed or needed**.

**Per-sensor optical alignment.** Each sensor self-aligns to the optical clock with its own
RTC + skew (`estimate_session_skew`). The common optical-time axis uses, per node X,
`epoch_offset_X = (rtc_X − rtc_foot) − skew_X`, so `t_opt_X(i) = epoch_offset_X + i/fs`.
(The 8 IMUs are separate files with different RTC starts spanning ~27 s; this puts them on
one clock.)

**Walking-bout delimiting (`detect_walking_segment`, config `segment:`).** On the foot
gyro magnitude `‖ω‖`:
```
rms(t)  = sqrt( movavg( ‖ω‖² , smooth_s·fs ) )            # smooth_s = 0.5 s
mask    = degrees(rms) > gyro_rms_threshold_dps           # = 60 °/s
```
Sub-threshold gaps shorter than `min_gap_s = 1.0 s` are bridged; the run(s) covering the
optical anchors are merged, padded by `pad_s = 0.5 s`, and capped at
`max_segment_s = 150 s`. For P01 this yields one continuous **128.7 s** bout.

**Inter-sensor impact refinement (`refine_intersensor`).** Heel strikes propagate to all
sensors. Each non-foot sensor's `epoch_offset` is refined by cross-correlating an
**8 Hz high-pass accel-magnitude impact feature** (`butter(4, 8/(fs/2), 'high')`,
rectified) against the foot, in a ±0.6 s window. The shift is applied only if correlation
≥ `min_corr = 0.45`. Result: RS −4 ms (0.78), RT +16 ms (0.50) applied; **SA (pelvis)
0.29 — kept on optical-skew (~±0.4 s)**, because pelvis impacts are damped and the periodic
walking signal is stride-ambiguous (and marker-based timing would break the raw-data
contract).

**Turnaround detection (`kincore/segment.py:detect_turnarounds`).** On the pelvis (SA):
vertical-axis rate `yr = ω · ĝ`, with gravity `ĝ` from a low-pass of accel
(`_lowpass_grav`, `fc = 0.4 Hz` ⇒ ~2.5 s moving average). Smooth `yr` over
`smooth_s = 0.5 s`; a sample is "turning" if `|yr| > rate_thresh_dps = 50 °/s`; a maximal
same-sign run is a turn if `|∫ yr dt| ≥ min_turn_deg = 120°`. Steady-state mask
(`steady_state_mask`) removes each turn ± `pad_s = 0.7 s`. For P01: **7 turns** of ~130–155°.

**Assumptions / limits.** Optical-time alignment is only as good as the per-sensor skew
(~±0.4 s); adjacent leg sensors are then impact-refined to <25 ms, the pelvis is not.

---

## Stage 4 — Magnetometer calibration (`kincore/calibration.py`)

**Input:** raw mag + accel over varied-orientation windows (CalibrationTask 01–03, TUG
01–02 — config `mag_calibration.source_trials`; Sitting is excluded — no orientation
variety), located via Stage 2.
**Output:** `MagCalibration(b, A, P, …)`; applied as `m_cal = P · A · (m − b)`.

**Hard/soft-iron ellipsoid (`fit_ellipsoid`).** Algebraic quadric fit: solve
`D v = 1` (least squares) for the 9 coefficients of
`a x² + b y² + c z² + 2f yz + 2g xz + 2h xy + 2p x + 2q y + 2r z = 1`, form
`Q = [[a,h,g],[h,b,f],[g,f,c]]`, `n = [p,q,r]`. Then
```
b (hard-iron center) = −Q⁻¹ n
k = 1 + nᵀ Q⁻¹ n
A (soft-iron) = sqrtm(Q / k)      # symmetric PD root via eigendecomposition
```
so `A·(m − b)` lies on the unit sphere. `sphere_residual = std(‖A(m−b)‖ − 1)`.

**Frame alignment (`align_frame`).** The mag chip axes may be permuted/flipped vs the
accel axes. Search all **48 signed axis permutations** `P`; choose the one minimising the
**standard deviation of the dip angle** `∠(P·m̂_cal, â)` across the varied-orientation
samples (a true field has a constant angle to gravity). Returns `(P, dip_mean, dip_std)`.

If fewer than `min_samples = 5000` mag samples are gathered, `identity_calibration()` is
used. For P01 the ellipsoid fit is **poor (sphere residual 0.65–0.83)** and the dip is off,
reflecting indoor distortion.

**World-frame channel-ID test (`tools/mag_worldframe.py`).** Rotating the 0x18 sensor-frame
vector into the world frame via the 6-DOF orientation: the **yaw-invariant world
inclination is constant** — Static_01 hold **−49.5° ± 1.3°**; CalibrationTask rotations
spanning up to 91° give world dip **−48…−57°**. A constant world inclination across large
rotations proves **0x18 is a genuine world-constant magnetometer, not an onboard-derived
artifact**. The local inclination (~50°) vs Geneva's geomagnetic 63° indicates **mild
indoor distortion**, the reason 9-DOF does not help.

**Assumptions / limits.** Calibration windows are themselves inside the distorted lab; the
ellipsoid is under-constrained by uneven orientation coverage. Calibration only sets units
and removes bias — it cannot fix spatial field distortion along the walkway.

---

## Stage 5 — Orientation fusion (`kincore/fusion.py`)

**Input:** per-sensor BIN-native `(ω, a)` and, for 9-DOF, calibrated `m`. **Output:**
quaternion sequence `q(t)` (sensor→earth), per sensor, on the 256 Hz BIN clock.

**Madgwick filter (`run_madgwick`, `dt = 1/fs`).** Common rate-of-change term
`q̇_ω = ½ q ⊗ [0, ω]`. The accel (and mag) define an objective `f` whose normalised
gradient corrects orientation:
```
q̇ = q̇_ω − β · ∇f/‖∇f‖ ;   q ← normalize(q + q̇ · dt)
```

*6-DOF (`_madgwick_6dof`), gravity objective* with `a` normalised:
```
f = [ 2(q1q3 − q0q2) − ax,
      2(q0q1 + q2q3) − ay,
      2(½ − q1² − q2²) − az ]
∇f = Jᵀ f,  J = [[−2q2, 2q3, −2q0, 2q1],
                 [ 2q1, 2q0,  2q3, 2q2],
                 [   0, −4q1, −4q2,  0]]
```

*9-DOF MARG (`_madgwick_9dof`)* adds the magnetometer objective. The earth-frame field
reference is computed each step, `h = q ⊗ [0,m] ⊗ q*`, `bx = √(h₁²+h₂²)`, `bz = h₃`, and
`f`, `J` are extended by three mag rows (exact expressions in source). Mag only constrains
heading by construction.

**Gains (config `fusion:`).** `β = 0.05` (9-DOF), `β_6dof = 0.033`. Initialisation
(`init_quat_from_acc_mag`) uses the first `init_n = 64` samples: roll/pitch from accel,
heading from tilt-compensated mag (or yaw = 0 in 6-DOF).

**Gimbal-lock-free heading (`heading_deg`, `pick_horizontal_axis`).** The pelvis sensor sits
at **pitch ≈ 71–89°**, on the ZYX-Euler yaw singularity, so Euler yaw is unusable. Instead,
pick the body axis whose earth-frame image is most horizontal on average and take its
azimuth:
```
heading(t) = unwrap( atan2( R(q)[1, k], R(q)[0, k] ) )     # k = argmax mean horiz. magnitude
```
A `tilt_deg` check (angle of body-z from earth-vertical) confirms 6-DOF and 9-DOF agree on
tilt (mag should change only heading).

**Assumptions / limits.** Fusion runs **per sensor on its own BIN clock** (contract);
cross-sensor alignment happens later on the orientation outputs. 6-DOF is primary; 9-DOF is
computed for comparison and is degraded by the distorted field.

---

## Stage 6 — Joint angles (`kincore/angles.py`)

**Input:** two sensors' `q(t)` (resampled to a common 256 Hz grid via `slerp_resample`:
linear-interp + renormalize with hemisphere continuity), their segment gyro, and neutral
gravity `g₀` per sensor (from Static_01, `run.py:neutral_gravity`). **Output:** sagittal
flexion (deg), angular velocity, angular acceleration, ROM per joint.

**Yaw-immune sagittal flexion (`joint_angles`).** The joint axis `j` for each segment is
the **largest-variance gyro direction** (`_joint_axis`: top eigenvector of `Σ ωωᵀ`),
estimated on **steady-state samples only** (`axis_mask` excludes turns, so the pelvis axis
is mediolateral, not the turn/yaw axis). Axes are sign-aligned so distal/proximal sagittal
rates correlate positively. With gravity in the sensor frame `g = R(q)ᵀ[0,0,1]`
(`gravity_in_sensor`), each segment's sagittal rotation from neutral
(`_segment_sagittal_rotation`) is the signed angle of `g` about `j` relative to `g₀`:
```
g₀⊥ = normalize(g₀ − (g₀·j) j),  g⊥ = normalize(g − (g·j) j)
θ_s = atan2( (g₀⊥ × g⊥)·j ,  g⊥·g₀⊥ )
flex_grav = unwrap(θ_distal − θ_proximal)
```
This is **drift-free and uses no heading** (flexion is about a gravity-anchored horizontal
axis). Because gravity-projection lags during fast motion, a **complementary filter**
(`_complementary`, `τ = joint_tau_s = 0.3 s`) blends it with the integrated joint rate
`ω_joint = ω_distal·j_d − ω_proximal·j_p`:
```
α = τ/(τ+dt)
flex[i] = α·(flex[i−1] + ω_joint[i]·dt) + (1−α)·flex_grav[i]
```

**Sign, derivatives, ROM.** An anatomical sign per joint (matched to the reference in
Stage 8) is applied to the output (`run.py:write_outputs`). Angular velocity and
acceleration are `np.gradient` of flexion (no extra filtering). `ROM = max − min`; the
summary reports **steady-state ROM** (turns excluded) and the in-window ROM (Stage 8).

**Assumptions / limits.** Flexion only (sagittal); ab/adduction and rotation not reported.
Hip is biased low when the pelvis is mis-timed (the SA ±0.4 s residual). Full-bout ROM
includes some slow complementary-filter drift (knee is most affected).

---

## Stage 7 — Gait events (`kincore/gait.py`)

**Input:** foot (RF) segment gyro on the grid + the steady-state mask. **Output:** mid-swing,
foot-strike, toe-off indices; cadence; stride-time stats.

**Method (`detect_events`).** Foot sagittal rate `s = ω · ĵ` (`ĵ` = foot largest-variance
gyro axis, oriented so swing peaks are positive via the skewness sign). Lightly smoothed
(`0.03 s` moving average). **Mid-swing** peaks: `find_peaks(distance = min_stride_s·fs,
height = 0.5·P95(|s|))` with `min_stride_s = 0.6 s`. **Foot strike** = minimum of `s` in
the 0.5 s after each mid-swing; **toe-off** = minimum in the 0.4 s before.

**Cadence (`cadence_stats`).** Stride times `Δt = diff(strike_idx)/fs`, restricted to the
steady-state mask and to `Δt < 2.5 s` (drop turn gaps). `cadence = (1/mean(Δt))·60·2`
steps/min (two feet ≈ 2× single-foot stride rate). For P01: **109 steps/min**, stride
**1.10 ± 0.07 s**, 99 steady strides.

**Assumptions / limits.** One foot instrumented ⇒ cadence inferred as 2× stride rate.
Gyro-only event detection; IMU foot-strike vs the optical Zeni event differs by ~0.4 s
(Stage 8) due to definition + window-alignment.

---

## Stage 8 — Validation (`validation/reference.py`, `run.py:validate`)

**Input (markers read ONLY here):** the trial `c3d` (markers, joint centres, Zeni events) +
`Static_01` for neutral. **Output:** per-joint RMSE, heading-arbiter RMSE, step-timing
error, reference-on-grid for plotting.

**Reference joint angles (`sagittal_reference`).** Built from the **c3d-provided joint
centres** RHJC/RKJC/RAJC (and pelvis markers), *not* a 4-marker-cluster rigid-body fit — the
joint centres are already computed in the dataset. Segment long-axis vectors:
`thigh = KJC−HJC`, `shank = AJC−KJC`, `foot = TOE−AJC`. The pelvis **mediolateral** axis is
`ml = otherHJC − HJC`; the pelvis **down** axis is `normalize(ml × ap)` with
`ap = midASIS − SACR`, forced to point downward (so the sign convention is leg-independent).
Each joint angle is the **signed angle in the plane ⊥ ml** (`_signed_angle`):
```
u⊥ = u − (u·ml)ml,  v⊥ = v − (v·ml)ml
angle = atan2( (u⊥ × v⊥)·ml , u⊥·v⊥ )
```
with knee = ∠(thigh,shank), ankle = ∠(shank,foot), hip = ∠(pelvis_down,thigh). All are
zeroed at the `Static_01` neutral (median, `neutral_reference`). Side-aware (`side="R"/"L"`).

**Comparison (`best_lag_rmse`, `validate`).** The IMU flexion (256 Hz, optical time) is
interpolated onto the trial's **100 Hz marker timeline** (the only place resampling onto the
marker clock occurs). Then: demean both, sign-match (`sign = sign(Σ a·b)`), search integer
lag within ±`max_lag_s = 0.3 s` (10 ms steps at 100 Hz), report the **minimum RMSE** and the
correlation at that lag. RMSE is therefore **offset-removed** (sensor-mounting offset not
penalised) and lag-optimised. Aggregated as the mean over the 4 windows.

**Heading arbiter.** Optical pelvis heading from `atan2(fwd_y, fwd_x)` with
`fwd = midASIS − SACR`; compared (mean-removed RMSE) to the IMU 6-DOF and 9-DOF pelvis
heading in each window. For P01: **6-DOF 5.7° vs 9-DOF 5.9°** ⇒ mag does not help. *These
windows are only ~3 s — they bound short-term heading error, not long-term yaw drift.*

**Zeni-2008 events (`c3d_events`).** Read from the c3d `EVENT` group (CONTEXTS/LABELS/TIMES).
IMU foot strikes (window-relative) are matched to "Right/Left Foot Strike" times; the mean
of `min|Δt|` for matches < 0.5 s is the step-timing error (~0.4 s for P01).

**Results (in-window, computed vs optical ROM; P01 right):** ankle RMSE 8.2°, ROM 38.6/38.2;
knee RMSE 14.0°, ROM 72.2/70.2; hip RMSE 8.8°, ROM 40.6/53.9. Consistent across P01-left and
P02-right (ankle/knee/hip ≈ 8–15°; mag never helps).

**Assumptions / limits.** Reference depends on the authors' joint-centre computation rather
than an independent cluster fit. RMSE is offset-removed. The ankle "53°" headline is the
full-bout ROM (turns/drift inflated); the in-window 38.6° vs optical 38.2° is the trustworthy
figure. Hip is shape-reliable but ROM-biased low.

---

## Stage 9 — RAW-DATA contract & selftest (`selftest.py`, `adapters/geneva.py`)

**The wall.** `adapters/geneva.py:IMUTrial(imu, reference, labels)`. The kinematic core
(`run.py:compute_core` → `kincore/*`) reads **only** `imu` (BIN-native accel/gyro/mag).
Marker-derived reference angles and Zeni labels are read **only** in `validation/`
(`run.py:validate`).

**Proof (`selftest.py`).** Run the core once; then monkeypatch `read_markers` to
**time-shuffle marker frames** (scramble reference) and `c3d_events` to return `{}` (drop
labels); run the core again. Assert **bit-for-bit identity** (`np.array_equal`) of the
**21 invariants** — for each fusion mode {6dof, 9dof} × joint {ankle, knee, hip} the
`flexion`, `ang_vel`, `rom` (= 18), plus `foot_strike`, `cadence`, `n_strides` (= 21) — and
assert the validation RMSE **changes** (ankle 8.2→11.8, knee 14.0→27.0, hip 8.8→21.9) and
matched steps drop **9 → 0**. PASS ⇒ the core is a pure function of the IMU.

---

## References

1. S. O. H. Madgwick, A. J. L. Harrison, R. Vaidyanathan. *Estimation of IMU and MARG
   orientation using a gradient descent algorithm.* IEEE ICORR, 2011 (filter & MARG update).
2. J. A. Zeni, J. G. Richards, J. S. Higginson. *Two simple methods for determining gait
   events during treadmill and overground walking using kinematic data.* Gait & Posture
   27(4):710–714, 2008 (reference foot-strike/foot-off events in the c3d).
3. V. Renaudin, M. H. Afzal, G. Lachapelle. *Complete triaxis magnetometer calibration in
   the magnetic domain.* J. Sensors, 2010 (hard/soft-iron ellipsoid calibration).
4. T. Seel, J. Raisch, T. Schauer. *IMU-based joint angle measurement for gait analysis.*
   Sensors 14(4):6891–6909, 2014 (yaw-immune sagittal joint angle from gravity + joint axis).
5. Grouvel et al., 2023 — the Geneva dataset (8 Physilog 6S IMUs + optical markers + force
   plates + pressure insoles).
