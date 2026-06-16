# Gait Kinematics — User Guide

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
python visualize.py                     # interactive single window (mannequin + gait panels)
python visualize.py --save              # headless stills: _dashboard + _stickfigure + _placement
python visualize.py --shot win.png      # render the live window once to a PNG screenshot

# Desktop app (recorded-file viewer): import CSV -> pipeline -> 3D mannequin (see §7)
python -m app.main

# RAW-DATA contract selftest (should print PASS)
python selftest.py

# Diagnostics (optional)
python fusion_check.py --node SA        # 6-vs-9-DOF heading deep-dive (writes fusion_check_SA.*)
python tools/mag_worldframe.py RT       # world-frame magnetometer channel-ID test
```

Notes
- Every command accepts `--config <path>`; default is `config/default.yaml`.
- `visualize.py` modes: **no flag** opens the interactive **single window** (lateral mannequin
  + synchronised gait panels), which needs a display and `pyqtgraph + PyQt5`; **`--save`**
  renders stills (works headless; also the automatic fallback if Qt is missing); **`--shot
  <png>`** renders the live window once to a screenshot (offscreen-friendly).
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

**`visualization`** (interactive window only) — `fps` (animation render rate, 60),
`default_speed` (1.0 = real-time; UI offers 0.25/0.5/1×), `loop` (restart at bout end),
`forward_translation` (procedurally walk the figure across the view), `step_length_frac`
(forward step as a fraction of stature). `anthropometry` scales the mannequin's segment
lengths from the subject's stature: `height_mm` (if `null`, read from the dataset *Inputs*
sheet by subject; else an override) and the Winter ratios `thigh_frac`/`shank_frac`/
`foot_frac`/`pelvis_frac`/`trunk_frac`.

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

### `_dashboard.png` / `_stickfigure.png` / `_placement.png` — see §4.

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
offset so frames don't overlap), y = up (m). Watch the knee bend through swing. Segment
lengths are scaled from the subject's stature.

**Sensor placement (`_placement.png`).** A static lateral mannequin in neutral stance with the
four IMU nodes marked as gold squares: **RF** on the foot, **RS** on the shank, **RT** on the
thigh, **SA** on the pelvis/sacrum. The same diagram appears as a reference panel in the
interactive window.

**Interactive window (`python visualize.py`).** A **single** window, two panes:
- **Left:** a lateral (sagittal) **mannequin** — thigh/shank/foot + a pelvis/trunk stub drawn
  as thick rounded segments with joint pivots, the four sensor dots, and a faint body
  silhouette for context. It is driven by the **clean yaw-immune flexion angles** (not the
  drifting 3D orientation), so the motion is anatomically correct. Below it sits the static
  sensor-placement diagram.
- **Right:** synchronised gait panels — joint flexion, angular velocity, foot-accel with
  strike markers, and a metrics box (cadence, per-joint ROM, stride time, step count). A
  vertical **time-cursor** sweeps all three plots in lockstep with the animation.
- **Playback is real-time:** the animation advances by wall-clock time mapped through the
  data's 256 Hz `t_s` (1.0× = real-time, independent of render rate — the old view stepped a
  fixed number of samples per frame and ran too fast). Controls: **play/pause**, **speed**
  (0.25/0.5/1×), **loop**. With `forward_translation` on, the figure **walks across** the view
  (cadence × step length) instead of flexing in place; the root is otherwise fixed because the
  pipeline computes orientation only, not global position.

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

---

## 7. Desktop app (`app/`) — recorded-file viewer

A windowed PyQt app that imports recorded 9-DOF IMU CSVs into an on-disk session store,
runs the **unchanged** kincore pipeline, and visualises the result with a **shaded 3D
mannequin** beside synchronised gait curves. `run.py`/`visualize.py` remain the
research/validation entry points (optical markers, RMSE); the app is the front-end for
recorded captures and the plug-in point for a future live-capture mode.

```powershell
python -m app.main          # launch the app (needs a display + pyqtgraph, PyQt5, PyOpenGL)
```

On first launch the bundled Geneva slices are converted into ready-to-use sessions, so the
app has real data immediately (P01 right/left, P02 right).

### 7.1 The three views
- **Sessions** — `Import CSV…` copies a recording into the session store and runs the
  pipeline; the table lists every session with subject, date, duration, side, nodes and
  joints, and whether results are computed. `Open in Visualize`/`Open Results` run the
  pipeline first if needed. `Add Geneva test data` (re)creates the bundled sessions.
- **Visualize** — the main screen. *Left:* the 3D mannequin (pelvis · thigh · shank · foot
  as shaded capsules with joint/head spheres, gold sensor pucks on the segments), driven by
  the clean sagittal flexion, with play/pause, speed (0.25–2×), loop, a live time read-out
  and a **timeline scrubber**. *Right:* synchronised joint-flexion and angular-velocity
  curves with a sweeping cursor (turnarounds shaded), and a **live parameter box** showing
  only the in-capture subset — current joint angle, accumulated ROM + peak range, angular
  velocity (current and peak), step count and cadence. Scrubber, cursor and mannequin move
  together.
- **Results** — per-joint table (ROM full/steady, peak min/max, peak |ω|) and gait table
  (cadence, stride/step time, stride variability CV, stance/swing %, steady strides, active
  duration), the **overlaid gait-cycle** plot (every steady stride at 0–100 %, with the mean
  bold), and `Export` to CSV / JSON / a text report written into the session's `results/`.

The mannequin is driven by **neutral-referenced** flexion (per-joint mounting offset removed
so the figure stands like a person); the plotted curves and parameters report the same
display reference. The lateral camera keeps the analysed leg camera-near; the contralateral
leg is drawn faded for body context. 3D backend: **pyqtgraph OpenGL** (`GLViewWidget` +
shaded `GLMeshItem`) — no new dependency, renders interactively and offscreen. A full rigged
human mesh / SMPL is out of scope (a future upgrade).

### 7.2 CSV schema (the app's input contract)
One **combined** CSV per recording (long format, one row per node-sample):

```
node,t_s,ax,ay,az,gx,gy,gz,mx,my,mz
```

| column | unit | meaning |
|---|---|---|
| `node` | — | sensor/segment id: `RF`/`RS`/`RT`/`SA`, `LF`/`LS`/`LT`, or generic ids |
| `t_s` | s | time, monotonic per node, 0 at recording start |
| `ax,ay,az` | m/s² | linear acceleration (sensor frame) |
| `gx,gy,gz` | rad/s | angular velocity (sensor frame) |
| `mx,my,mz` | counts | magnetometer, uncalibrated (pipeline fits hard/soft-iron) |

Import also accepts **per-node files** named `<NODE>.csv` (same columns minus `node`, taken
from the filename) — that is how the existing `data/<…>/<node>.csv` Geneva slices load
(`t_opt_s` → `t_s`). Default joint topology is inferred from the node ids: right leg
`ankle[RF,RS] · knee[RS,RT] · hip[RT,SA]` (left analogous with `L*`); `SA` is the pelvis.

### 7.3 Session-folder layout
Configured by `config/app.yaml → app.data_dir` (default `~/GaitApp/sessions/`):

```
<data_dir>/<session_id>/        e.g. P01_S01_2minWalk_right/
   raw/data.csv                 canonical imported IMU (combined, never mutated)
   session.json                 metadata: subject, date, duration, fs, nodes, joints, height
   results/
      timeseries.csv            t_s, <joint>_deg/_vel_dps/_acc_dps2, foot_acc_mag,
                                foot_strike, mid_swing, toe_off, steady_state
      summary.json              per-joint ROM/peaks + gait stats + turnarounds
      parameters.csv/.json, report.txt   (written by Results → Export)
```

The app's `results/timeseries.csv` mirrors `run.py`'s schema **minus** the optical
reference/error columns (a generic recording has no markers). `config/app.yaml` also sets
`app.run_modes` (default `["6dof"]`; add `"9dof"` to compute the magnetometer-aided angle
too), the Madgwick gains, and the playback defaults.

### 7.4 Notes / limitations
- Stance/swing % is a single-foot-IMU estimate (toe-off from one foot is approximate).
- The pipeline runs on a background thread with a progress bar; a 2-min walk (4 nodes,
  6-DOF) takes ~14 s. The computed numbers match `run.py` to the decimal (ROM/cadence).
