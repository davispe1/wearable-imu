# Mythos MECH — User Guide

How to run the pipeline and read its outputs. For internals/equations see `methods.md`.

---

## 1. Quickstart

```powershell
# (one-time) install dependencies
python -m pip install -r requirements.txt

# Stage 1: decode BINs, align, write data/ slices (READ-ONLY on the dataset)
python extract.py                       # optional: --config config/p01_left.yaml

# Full pipeline: fusion -> joint angles -> gait -> validation -> outputs/
python run.py                           # prints a console summary; writes 3 output files

# Visualisation
python visualize.py --save              # headless: writes _dashboard.png + _stickfigure.png
python visualize.py                     # interactive 3D stick figure (needs a display)

# RAW-DATA contract selftest (should print PASS)
python selftest.py

# Diagnostics (optional)
python fusion_check.py --node SA        # 6-vs-9-DOF heading deep-dive (writes fusion_check_SA.*)
python tools/mag_worldframe.py RT       # world-frame magnetometer channel-ID test
```

Notes
- Every command accepts `--config <path>`; default is `config/default.yaml`.
- `visualize.py` has **two modes**: with `--save` it renders PNGs (works headless; also the
  automatic fallback if Qt/OpenGL is missing); **without any flag it opens the interactive
  pyqtgraph 3D view**, which needs a display and `pyqtgraph + PyOpenGL + PyQt5`. There is no
  `--view` flag — interactive is the default.
- Run `extract.py` before `run.py` is not strictly required (`run.py` re-reads the BINs), but
  `extract.py` is what writes the reusable `data/` slices and `adapters/geneva.py` reads them.
- Ships with `config/default.yaml` (P01 right), `config/p01_left.yaml`, `config/p02_right.yaml`.

---

## 2. Configuration (`config/default.yaml`)

Everything is config-driven. Knobs and their effect:

**`dataset`** — `root` (path to `researchdata/`, never written), `subject` (`P01`…`P10`),
`session` (`S01`).

**`selection`**
- `task` — trial family, e.g. `2minWalk`, `Gait`, `FastGait`, `TUG`.
- `trials` — list of repetitions to use as optical windows; **auto-filtered** to those that
  exist (P02 has only `01`,`02`).
- `leg` — `right` or `left`; selects the marker side (R/L) for the reference.
- `nodes` — IMU node set, distal→proximal, e.g. `[RF, RS, RT, SA]` (foot/shank/thigh/pelvis).
- `joints` — topology `name: [distal_node, proximal_node]`; angle = proximal vs distal.
- `foot_node` — node used for step detection & walking-bout delimiting.
- `ref_align_node` — node used as the alignment reference (the foot).

**`sensor_map`** — node → BIN sensor short code (the filename token). Change this to remap
which physical IMU is which segment.

**`bin_format`** — `fs_high` (256 Hz accel/gyro/mag), `fs_mag` (64 Hz; the *barometer* rate,
left for reference), `acc_counts_per_g` (2048), `gyr_counts_per_dps` (16.384), `gravity`
(9.80665), `mag_range_gauss` (nominal; calibration normalises so it does not affect results).

**`alignment`** — `min_corr` (0.5, warn threshold), `refine` (enable the local xcorr refine).

**`segment`** (walking-bout delimiting) — `activity_node` (RF), `smooth_s` (0.5 s RMS window),
`gyro_rms_threshold_dps` (60 °/s walking threshold), `min_gap_s` (1.0 s gap-bridge),
`pad_s` (0.5 s), `max_segment_s` (150 s cap). Raise the threshold for faster gait, lower it
for slow/shuffling gait.

**`mag_calibration`** — `source_tasks` / `source_trials` (windows used to fit the ellipsoid;
use varied-orientation tasks, not walking), `min_samples` (5000; below this → identity
calibration).

**`fusion`** — `beta` (0.05, 9-DOF Madgwick gain), `beta_6dof` (0.033, 6-DOF gain),
`joint_tau_s` (0.3 s, joint complementary-filter time constant — smaller trusts gravity more,
larger trusts the gyro more), `run_modes` (`[6dof, 9dof]`; both computed), `init_static_s`.

**`output`** — `dir` (`outputs/`), `data_dir` (`data/`), `angles_unit`.

---

## 3. Output files

All under `outputs/`, prefixed `<subject>_<session>_<task>` (e.g. `P01_S01_2minWalk`).

### `_timeseries.csv` — the full per-sample result (256 Hz, optical time)
One row per sample over the walking bout. Columns (in order):

| column | unit | meaning |
|---|---|---|
| `t_opt_s` | s | common optical-clock time, 0 at bout start |
| `<joint>_deg` | deg | computed **6-DOF** sagittal flexion (anatomically signed) |
| `<joint>_vel_dps` | deg/s | joint angular velocity (d/dt of flexion) |
| `<joint>_acc_dps2` | deg/s² | joint angular acceleration |
| `<joint>_deg_9dof` | deg | computed **9-DOF** flexion (for comparison) |
| `<joint>_ref_deg` | deg | optical reference flexion; **NaN outside the 4 mocap windows** |
| `<joint>_error_deg` | deg | `computed − reference` (NaN outside windows) |
| `foot_acc_mag` | m/s² | foot accelerometer magnitude |
| `foot_strike` | 0/1 | 1 at detected foot-strike samples |
| `steady_state` | 0/1 | 1 = straight walking, 0 = turnaround ± pad |

`<joint>` ∈ {`ankle`, `knee`, `hip`} (the three blocks appear in that order).

### `_validation.csv` — per-window per-joint accuracy
Columns: `trial, t_opt_start_s, joint, rmse_deg, lag_s, corr, ref_rom_deg`
(`rmse_deg` offset-removed & lag-optimised; `corr` = Pearson at best lag; `ref_rom_deg` =
optical range of motion in that window).

### `_summary.json` — headline metrics & caveats
Key fields:
- `walking_segment_s`, `duration_s` — bout bounds.
- `primary_fusion` — `"6dof"`.
- `joint_rom_deg_full_bout` — `{6dof,9dof}{joint}`; steady-state max−min over the whole bout
  (can be inflated by slow drift — see caveats).
- `joint_rom_in_window_deg` — `{joint}{computed, optical}`; **the trustworthy ROM** (same
  windows as the optical reference).
- `joint_rom_confidence` — per-joint note (hip is lower-confidence; see §5).
- `joint_peak_vel_dps_steady` — peak |angular velocity| (steady state).
- `validation_rmse_deg` — `{6dof,9dof}{joint}` mean RMSE vs optical.
- `heading_rmse_vs_optical_deg` — `{6dof,9dof}` pelvis-heading RMSE (the magnetometer test).
- `gait` — `cadence_steps_per_min`, `stride_time_mean_s`, `stride_time_std_s`,
  `n_foot_strikes`, `n_steady_strides`.
- `turnarounds` — list of `{t_start_s, t_end_s, deg}`.
- `intersensor_refine` — `{node:{lag_ms, corr, applied}}` (which sensors were impact-refined).
- `magnetometer` — `channel`, `world_frame_test`, `local_inclination_deg`,
  `geomagnetic_inclination_geneva_deg`, `verdict`, `heading_window_caveat`.
- `caveats` — list of plain-text caveats.

### `_dashboard.png` / `_stickfigure.png` — see §4.

### `fusion_check_SA.json` / `.png` — 6-vs-9-DOF heading deep-dive (`fusion_check.py`)
JSON: `mag_calibration` (b/A/P, dip, sphere residual), `field_quality`
(`B_mean_counts`, `B_std_counts`, `B_pct`, `B_min`, `B_max`), `straight_drift`
(`drift_6dof_dps`, `drift_9dof_dps`, `verdict`), `turnarounds`, `per_pass`,
`per_pass_increments`, `tilt_rms_diff_deg`. PNG: heading(t) 6-vs-9-DOF with passes/turns
shaded + the pelvis vertical yaw-rate.

### `data/<subject>_<session>_<task>/<node>.csv` — the IMU slices (the contract `imu` view)
Columns: `t_native_s, t_opt_s, ax, ay, az` (m/s²), `gx, gy, gz` (rad/s), `mx, my, mz`
(raw mag counts, tag 0x18). Plus `extract_report.json` (skews, segment, window indices).

---

## 4. Reading the visuals

**Dashboard (`_dashboard.png`).**
- **Left column (3 panels):** ankle, knee, hip flexion vs time. **Blue** = computed 6-DOF
  (primary); **grey** = 9-DOF; **red dots** = optical reference (only in the 4 mocap
  windows). Each panel's text gives `RMSE` and the in-window computed-vs-optical ROM.
- **Top-right:** joint angular velocities (deg/s).
- **Middle-right:** foot accelerometer magnitude with red ▼ at detected foot strikes.
- **Bottom-right:** metrics panel — cadence, stride, strides/strikes, turnarounds, per-joint
  RMSE, and the heading 6-vs-9-DOF result.

**Stick figure (`_stickfigure.png`).** The right-leg sagittal chain
**pelvis → hip → knee → ankle → toe** drawn at several frames across one steady stride,
coloured by time (viridis, dark→yellow). Axes: x = forward (m, with a per-frame time
offset so frames don't overlap), z = up (m). Watch the knee bend through swing.

**Interactive 3D (`python visualize.py`).** A pyqtgraph `GLViewWidget` animates the same
right-leg chain in 3D, advancing through the bout in real time (drag to rotate the camera).

---

## 5. Reading the metrics (and their caveats)

- **RMSE (deg)** — root-mean-square difference between computed and optical joint angle over
  the 4 mocap windows, after removing a constant offset and the best lag (±0.3 s). It scores
  *dynamic shape accuracy*, not absolute offset. Typical: ankle ~8°, knee ~14°, hip ~8°.
- **ROM (deg)** — range of motion (max − min). **Use `joint_rom_in_window_deg`** (same
  windows as optical): ankle 38.6 vs 38.2, knee 72.2 vs 70.2, hip 40.6 vs 53.9 (P01). The
  **`joint_rom_deg_full_bout` is larger** (e.g. ankle ~53°) because it spans turnarounds and
  some slow complementary-filter drift — do not read it as physiological ROM.
- **Cadence / stride time** — from foot-strike intervals on steady strides only; cadence is
  reported as both feet (2× single-foot stride rate).

**Known caveats (also in `_summary.json`):**
1. **Hip is lower-confidence.** The pelvis IMU (SA) could not be inter-sensor-aligned below
   ~0.4 s from IMU alone, so thigh/pelvis are paired at slightly different gait phases. The
   hip *waveform shape* is reliable (corr 0.92–0.97) but its **ROM is biased low** (~13°).
2. **Ankle full-bout ROM looks inflated** (~53°) — that is the bout figure; the in-window
   38.6° (≈ optical 38.2°) is correct.
3. **Magnetometer / heading.** 9-DOF does **not** improve heading here (6-DOF 5.7° vs 9-DOF
   5.9° vs optical). The magnetometer (tag 0x18) is a genuine field sensor, but the indoor
   force-plate lab distorts the field (inclination ~50° vs 63°, |B| swings 3.7–15%). **6-DOF
   is primary.** The optical heading windows are only ~3 s, so they bound short-term heading
   error but **cannot measure long-term (minutes) yaw drift** — a longer reference is needed
   for that.
4. **RMSE is offset-removed** (sensor-mounting offset not penalised).

---

## 6. Live-hardware swap point

The kinematic core never touches the dataset files — only `IMUTrial.imu`. To move from the
Geneva dataset to a live device:

1. **Write a new adapter** (alongside `adapters/geneva.py`) that fills `IMUTrial.imu` with
   `{t_native, t_opt, acc (m/s²), gyr (rad/s), mag (counts)}` per node from the live stream.
   Nothing downstream of the contract wall changes.
2. **`kincore/` runs unchanged** (calibration, fusion, angles, gait). Keep **6-DOF primary**;
   enable 9-DOF only once the magnetometer is calibrated **and gated on field quality**
   (|B| deviation, inclination consistency) — naïve 9-DOF degraded heading here.
3. **`bin_reader.py`, `align.py`, `extract.py` are dataset-specific** (proprietary decode +
   multi-file clock-skew alignment). A live device that already streams time-synchronised,
   single-clock samples does not need them — the two hardest problems this pipeline solved
   disappear on real hardware.
4. **`validation/` is optional** on hardware (it needs an optical reference). `selftest.py`
   still applies as a regression guard on the contract.
