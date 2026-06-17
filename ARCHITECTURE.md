# Architecture

Gait Kinematics turns raw 9-DOF inertial/magnetic sensor data into commonly measured gait
parameters. It is built around one pure library (`gaitlib/`) that is the single source of
truth for all kinematics; everything else (the desktop app, the research pipeline, the
Open3D viewer) is a layer that feeds the library or presents what it returns.

---

## 1. Directory tree

> Folders marked **(git-ignored)** are excluded by `.gitignore`: the read-only dataset,
> recording slices under `data/`, pipeline outputs, and the on-disk app session store.

```
gait-kinematics/
├── README.md                       # project overview: library + app, quick start
├── requirements.txt                # deps split by layer (gaitlib / app / pipeline)
├── .gitignore                      # excludes dataset, data/*, outputs/, GaitApp/, caches
├── gait_open3d_viewer.py           # standalone Open3D playback viewer (Open3D NOT a project dep)
│
├── gaitlib/                        # THE LIBRARY — pure data-in / parameters-out, no hardware deps
│   ├── __init__.py                 # public API: compute(), default_config()/both_legs_config(), MountingConfig, GaitResults
│   ├── pipeline.py                 # compute(): the single entry point; runs all stages in order
│   ├── config.py                   # MountingConfig + FusionParams: sensors→segments, joint topology, rates
│   ├── rawdata.py                  # raw 9-DOF INPUT CONTRACT: load_raw(), per-channel rate alignment, infer_rate()
│   ├── calibration.py              # magnetometer hard/soft-iron + frame alignment (9-DOF only)
│   ├── fusion.py                   # Madgwick 6-/9-DOF orientation per segment; quaternion math
│   ├── angles.py                   # yaw-immune sagittal joint flexion from per-segment orientations
│   ├── gait.py                     # foot-IMU gait events (strike/toe-off/mid-swing), cadence
│   ├── segment.py                  # turnaround detection + steady-state masking (pelvis)
│   ├── parameters.py               # final stage: per-joint ROM/peaks/reps + gait params; overlay_cycles()
│   ├── results.py                  # GaitResults dataclass: traces, params, exporters (CSV/JSON)
│   ├── README.md                   # library reference
│   └── tests/
│       ├── __init__.py
│       └── test_geneva_regression.py   # end-to-end regression on a bundled slice
│
├── app/                            # THE VIEWER APP — PyQt front-end over gaitlib (keeps no kinematic math)
│   ├── __init__.py
│   ├── main.py                     # MainWindow: sidebar + stacked Sessions/Visualize/Results views
│   ├── session_store.py            # on-disk session store; CSV schema; load_session_streams() (both layouts)
│   ├── pipeline_runner.py          # thin bridge: builds MountingConfig, calls gaitlib.compute, writes results
│   ├── analysis.py                 # LoadedSession: reshapes cached results into view-ready quantities
│   ├── ui_sessions.py              # Sessions view: import CSV, list sessions, run pipeline (QThread worker)
│   ├── ui_visualize.py             # Visualize view: synced 2D angle/velocity curves + live params + scrubber
│   ├── ui_results.py               # Results view: param tables, overlaid cycles, exports (CSV/JSON/txt/Open3D)
│   └── ui_common.py                # PlaybackClock: wall-clock→data-time mapping (play/pause/speed/loop)
│
├── pipeline/                       # RESEARCH/DATASET PIPELINE — Geneva BIN → CSV slices → validation
│   ├── run.py                      # end-to-end orchestrator: BIN → SI/align → fusion → angles → gait → validate
│   ├── export_open3d.py            # export a session → gait_frames.csv + meta.json for the Open3D viewer (no recompute if cached)
│   ├── extract.py                  # Stage 1: read BIN (read-only), SI convert, upsample mag, write data/ slices
│   ├── align.py                    # locate a trial's window inside the continuous BIN via RTC + skew
│   ├── bin_reader.py               # reverse-engineered GaitUp/Physilog 6S *.BIN page reader
│   ├── fusion_check.py             # STOP-point: 6-DOF vs 9-DOF heading on the pelvis
│   ├── selftest.py                 # proves computed kinematics are a pure function of IMU data only
│   ├── visualize.py                # research sagittal-mannequin window + synced gait panels
│   ├── adapters/
│   │   ├── __init__.py
│   │   └── geneva.py               # Geneva DatasetAdapter → IMUTrial (imu / reference / labels contract)
│   ├── validation/
│   │   ├── __init__.py
│   │   └── reference.py            # marker-derived reference angles + IMU RMSE comparison (markers read ONLY here)
│   └── tools/                      # one-off CLI investigators (BIN/c3d/mag reverse-engineering)
│       ├── build_methods_docx.py   # generate docs/Gait_Kinematics_Methods.docx
│       ├── find_data_stream.py     # locate the signal stream region in a BIN
│       ├── inspect_bin.py          # dump BIN header/page structure
│       ├── inspect_c3d.py          # inspect optical c3d contents
│       ├── mag_disambiguate.py     # test whether a stream is a true magnetometer
│       ├── mag_worldframe.py       # check mag world-frame consistency under rotation
│       ├── parse_stream.py         # parse candidate packet streams
│       ├── probe_regions.py        # probe byte regions for plausible signals
│       ├── scan_bin_packets.py     # scan/classify BIN packet types
│       ├── tag_inventory.py        # inventory framing tags in a BIN
│       └── verify_pages.py         # verify BIN 512-byte page layout
│
├── config/                         # CONFIGURATION (YAML)
│   ├── app.yaml                    # desktop-app config: data_dir, run_modes, fusion gains, viz, anthropometry
│   ├── default.yaml                # pipeline config: dataset paths, trial selection, nodes/joints, BIN format
│   ├── p01_left.yaml               # per-subject/leg pipeline override (P01, left leg)
│   └── p02_right.yaml              # per-subject/leg pipeline override (P02, right leg)
│
├── docs/                           # DOCUMENTATION
│   ├── methods.md                  # per-stage technical/methods manual (equations)
│   ├── pipeline_guide.md           # how to run the research pipeline
│   ├── user_guide.md               # how to run + read outputs
│   └── assets/                     # figures + rendered equations referenced by the docs (*.png)
│
├── data/                           # DATA LAYOUT — recordings live here (mostly git-ignored)
│   ├── .gitkeep                    # keeps the folder tracked            (tracked)
│   ├── README.md                   # input schema + folder layout        (tracked)
│   ├── schema_example.csv          # tiny fake combined 9-DOF example     (tracked)
│   └── <recording_id>/             # per-recording slices                 (git-ignored: data/*)
│       ├── RF.csv RS.csv RT.csv SA.csv   # per-node 9-DOF CSVs (node id = filename), OR
│       ├── data.csv                      # one combined long-format CSV
│       └── extract_report.json           # pipeline extract provenance (timing/skew)
│
├── outputs/                        # (git-ignored) pipeline-generated CSV/JSON/plots
├── GaitApp/  (~/GaitApp/sessions)  # (git-ignored) app session store: <id>/raw/data.csv + session.json + results/
└── Human gait ... force plates/    # (git-ignored) READ-ONLY Geneva dataset (BIN/c3d source)
```

---

## 2. Section-by-section

### A. `gaitlib/` — the library

**(a) Purpose.** The kinematic engine: raw 9-DOF samples in, gait parameters out. It is
*pure* — no serial ports, sockets, pins, registers, or 3D dependencies. Acquisition and
visualisation are deliberately separate layers. Everything else in the repo treats this
package as the single source of truth for the math.

**(b) Files.**
- **`__init__.py`** — public surface: `compute`, `MountingConfig`, `FusionParams`,
  `default_config`, `both_legs_config`, `GaitResults`, plus the stage submodules.
- **`pipeline.py`** — `compute(raw_data, mounting_config) -> GaitResults`, the one public
  function. Resolves which joints the present sensors allow, builds a common time grid,
  fits per-node neutral gravity (`_neutral_gravity`), runs fusion per node/mode, computes
  joint angles, detects gait events, and assembles parameters.
- **`config.py`** — `MountingConfig` (sensors→segment map, `joints` topology as
  `{name: (distal, proximal)}`, `foot_node`, `pelvis_node`, `rates`, `fusion`, `strict`)
  and `FusionParams` (`run_modes`, `beta_6dof`, `beta_9dof`, `joint_tau_s`). Helpers
  `default_config()` (4 sensors, one leg) and `both_legs_config()` (7 sensors, 6 joints);
  `resolve_joints()` splits joints into computable vs skipped.
- **`rawdata.py`** — the input contract. `load_raw()` normalises a per-node dict, an
  iterable of rows, or a structured array into aligned per-node streams; `align_node()`
  resamples the magnetometer onto the IMU timeline; `infer_rate()` derives Hz from
  timestamps.
- **`calibration.py`** — `MagCalibration`, `fit_mag_calibration()` (ellipsoid hard/
  soft-iron + signed-permutation frame alignment), `identity_calibration()`,
  `gather_orientation_samples()`. 9-DOF only.
- **`fusion.py`** — quaternion math (`q_mult`/`q_conj`/`q_norm`/`q_rotate`) and
  `run_madgwick()` (6-DOF accel+gyro, 9-DOF accel+gyro+mag), plus heading/tilt helpers.
- **`angles.py`** — `joint_angles()`: orientation of the distal segment relative to the
  proximal, sagittal **flexion** about the gravity-anchored mediolateral axis (yaw-immune);
  `slerp_resample`, `derivative`, `rom`.
- **`gait.py`** — `detect_events()` (foot strike/toe-off/mid-swing from foot sagittal
  angular rate), `cadence_stats()`, `principal_axis()`.
- **`segment.py`** — `detect_turnarounds()` and `steady_state_mask()` from the pelvis,
  excluding ~180° turns from steady-state statistics.
- **`parameters.py`** — final stage: `joint_parameters()` (ROM full/steady, peaks, peak
  |ω|, reps, active duration) and `gait_parameters()` (cadence, step/stride time,
  stance/swing); `overlay_cycles()` for cycle averaging.
- **`results.py`** — `GaitResults` dataclass holding `fs`, `t`, `joints`, `gait`,
  `events`, `steady_state`, `turnarounds`, `config`, `warnings`, `meta`; exporters
  `summary()`/`save_summary_json()`, `timeseries_columns()`/`save_timeseries_csv()`, and
  `save_joint_angles_csv()`.

**(c) Connections.** Receives raw 9-DOF data + a `MountingConfig` (from the app's
`pipeline_runner`, the research `pipeline/run.py`, or any caller). Produces a `GaitResults`
object. Depends only on `numpy`/`scipy`. Knows nothing about disk layout, Qt, or Open3D.

### B. `app/` — the viewer app

**(a) Purpose.** A PyQt desktop application to import recordings, run the pipeline, and
review results as 2D curves and parameter tables. It keeps **no copy of the kinematic
math**: it builds inputs for `gaitlib`, calls `compute`, and presents what comes back.

**(b) Files.**
- **`main.py`** — `MainWindow` with a sidebar switching three stacked views (Sessions /
  Visualize / Results); `main()` loads `config/app.yaml` and seeds bundled test sessions.
- **`session_store.py`** — on-disk session store and the app's CSV schema
  (`node,t_s,ax..mz`). `app_config`, `import_csv`/`create_session`, `parse_imu_csv`,
  `default_joint_topology`, `list_sessions`, `load_results`, and `load_session_streams()`
  — which loads **both** layouts: `<session>/raw/data.csv` (app store) and per-node CSVs
  directly in `<session>/` (e.g. `RF.csv`…, `t_opt_s` time column; metadata inferred when
  no `session.json` exists).
- **`pipeline_runner.py`** — `run_pipeline()` assembles a `gaitlib.MountingConfig` from the
  session topology and calls `gaitlib.compute`; `write_results()` writes
  `results/timeseries.csv` + `summary.json`.
- **`analysis.py`** — `LoadedSession` reads cached results and derives neutral-referenced
  display angles, event indices, per-frame live-parameter snapshots, and overlaid cycles
  (no recompute).
- **`ui_sessions.py`** — Sessions view + `PipelineWorker` (QThread) running the pipeline
  off the UI thread.
- **`ui_visualize.py`** — synced angle/velocity curves, live parameters, scrubber driven by
  `PlaybackClock`.
- **`ui_results.py`** — parameter tables, overlaid gait cycles, and exports (CSV/JSON/text;
  **Export for Open3D** writes `gait_frames.csv` + `meta.json`).
- **`ui_common.py`** — `PlaybackClock` (real-time wall-clock → data-time playback).

**(c) Connections.** Receives 9-DOF CSVs (combined or per-node) and the user's actions.
Calls `gaitlib.compute` via `pipeline_runner`. Produces a session folder
(`raw/data.csv` + `session.json` + `results/`) and, on export, the Open3D viewer's input
files. The Visualize/Results views read only the cached results via `LoadedSession`.

### C. `pipeline/` — research/dataset pipeline

**(a) Purpose.** Convert the raw Geneva dataset (proprietary `*.BIN` + optical `*.c3d`)
into the per-node CSV slices the library/app consume, and validate IMU-computed angles
against marker reference. This layer exists for research/dataset prep; production runtime
does not need it. A hard wall is maintained: the kinematic core sees IMU data only; markers
and event labels are read **only** by the validation stage.

**(b) Files.**
- **`run.py`** — end-to-end orchestrator: BIN → SI + optical alignment → mag calibration →
  6/9-DOF orientation → common-grid joint angles → gait events → validation → CSV/JSON.
- **`export_open3d.py`** — separate export step (does not import Open3D): reads a session's
  cached results (or runs gaitlib if absent) and writes `gait_frames.csv` + `meta.json` for
  the viewer; `export_session()` is the CLI, `export_from_results()`/`write_open3d_inputs()`
  back the app's button.
- **`extract.py`** — Stage 1: reads BIN (read-only), converts to SI, upsamples the
  magnetometer 64→256 Hz on each sensor's own clock, delimits the walking segment, writes
  per-sensor `data/<id>/*.csv` slices + `extract_report.json`.
- **`align.py`** — locates a trial's window inside the continuous ~69-min BIN using c3d
  capture time, BIN RTC, and a calibrated session skew.
- **`bin_reader.py`** — reverse-engineered 512-byte-page reader for GaitUp/Physilog 6S BIN.
- **`fusion_check.py`** — STOP-point deliverable comparing 6-DOF vs 9-DOF pelvis heading.
- **`selftest.py`** — proves every IMU-computed quantity is a pure function of IMU data
  (scrambles the reference, asserts computed outputs are bit-identical).
- **`visualize.py`** — research sagittal-mannequin window + synced gait panels.
- **`adapters/geneva.py`** — `DatasetAdapter` mapping dataset files into the
  `IMUTrial(imu, reference, labels)` contract.
- **`validation/reference.py`** — marker-derived sagittal reference angles + IMU RMSE
  scoring (the only place markers/labels are read).
- **`tools/`** — one-off CLI investigators used to reverse-engineer and verify the BIN/c3d
  formats and the magnetometer.

**(c) Connections.** Receives the read-only Geneva dataset. Produces per-node CSV slices
under `data/` (the same per-node format the app's `load_session_streams` reads) and
validation reports/plots under `outputs/`. Its kinematic core reuses the same math as
`gaitlib`; the slices it writes feed straight into Section B and the export path.

### D. `config/` — configuration files

**(a) Purpose.** Keep behaviour config-driven and separate from code: one file for the app
runtime, one default for the research pipeline, and per-subject/leg overrides.

**(b) Files.**
- **`app.yaml`** — read by `app/session_store.py:app_config()`: `app.data_dir`,
  `app.run_modes`, `app.fusion` (`beta_6dof`/`beta_9dof`/`joint_tau_s`), `visualization`,
  `anthropometry.default_height_m`.
- **`default.yaml`** — research-pipeline config: dataset root, subject/session, trial
  selection, `nodes`/`joints`/`foot_node`, sensor map, BIN format, alignment, mag
  calibration, fusion, output, visualization.
- **`p01_left.yaml` / `p02_right.yaml`** — per-subject/leg overrides of `default.yaml`
  (nodes, joints, foot node, calibration windows).

**(c) Connections.** `app.yaml` configures Section B; the others configure Section C. The
fusion gains here mirror the defaults in `gaitlib.FusionParams`.

### E. `docs/` — documentation

**(a) Purpose.** Human-facing manuals: how to run, how to read outputs, and the per-stage
methods/equations.

**(b) Files.** `methods.md` (per-stage technical reference with equations),
`pipeline_guide.md` (running the research pipeline), `user_guide.md` (running + reading
outputs), and `assets/` (figures and rendered-equation PNGs referenced by the manuals;
`tools/build_methods_docx.py` can render a `.docx`, which is git-ignored).

**(c) Connections.** Documentation only — describes Sections A–C; no runtime dependency.

### F. `data/` — data layout

**(a) Purpose.** Where recordings and dataset slices live. Real recordings are **not**
version-controlled; only placeholders ship. The full Geneva dataset and pipeline `outputs/`
are likewise git-ignored.

**(b) Files / layout.**
- Tracked: `.gitkeep`, `README.md` (the input schema), `schema_example.csv` (a tiny fake
  combined 9-DOF example).
- Git-ignored (`data/*` except those placeholders): each `data/<recording_id>/` holds
  either per-node files (`RF.csv RS.csv RT.csv SA.csv`, node id = filename, columns
  `t_native_s,t_opt_s,ax..mz`) **or** one combined `data.csv` (`node,t_s,ax..mz`), plus an
  optional `extract_report.json`.

**(c) Connections.** Produced by `pipeline/extract.py`; consumed by the app
(`load_session_streams` reads either layout) and by the Open3D export CLI. Distinct from the
app's own session store at `~/GaitApp/sessions/` (also git-ignored).

---

## 3. Data flow

A recording's 9-DOF samples flow into `gaitlib.compute`, become a `GaitResults` object,
are cached as `timeseries.csv` + `summary.json`, and from there drive the app's 2D views or
are exported to the Open3D viewer's frame contract. The research pipeline is an upstream
producer of the same per-node CSV format.

```
  Geneva BIN/c3d (read-only)                 9-DOF CSV recording
   │  pipeline/extract.py                      (per-node RF/RS/RT/SA.csv  OR  combined data.csv)
   ▼                                             │
  data/<id>/*.csv  ─────────────────────────────┤
                                                 │  app/session_store.load_session_streams()
                                                 ▼  (accepts raw/data.csv OR per-node layout)
                                       per-node streams  { node: {t, acc, gyr, mag} }
                                                 │  app/pipeline_runner.run_pipeline()
                                                 │  → builds gaitlib.MountingConfig
                                                 ▼
                            ┌─────────────  gaitlib.compute(raw_data, mounting_config)  ─────────────┐
                            │  calibration → rate-align → Madgwick fusion → sagittal flexion →       │
                            │  gait events → parameters                                              │
                            └────────────────────────────────┬────────────────────────────────────-┘
                                                              ▼
                                                       GaitResults
                                                              │  write_results()
                                                              ▼
                                  <session>/results/timeseries.csv  +  summary.json
                                                              │
                            ┌─────────────────────────────────┼──────────────────────────────────┐
                            ▼ app/analysis.LoadedSession                                           ▼ export
                  Visualize view (curves + live params)                        gait_frames.csv + meta.json
                  Results view (tables, overlaid cycles)                                  │
                                                                                          ▼
                                                                       gait_open3d_viewer.py (separate; Open3D not a dep)
```

The research `pipeline/run.py` follows the same fusion → angles → gait path internally and
additionally scores the result against marker reference (`pipeline/validation/`), which the
kinematic core never sees.

---

## 4. Key contracts

Three interfaces must stay stable so any layer can be replaced independently.

### (a) gaitlib input schema — one 9-DOF sample

Accepted as a per-node dict, an iterable of long-format rows, or a structured array (column
aliases tolerated). Per sample:

| field | meaning | units |
|---|---|---|
| `timestamp` | time, monotonic per node, any 0-origin | s (float) |
| `node_id` | sensor id; must match a key in the mounting config | str |
| `ax, ay, az` | linear acceleration, sensor frame | m/s² |
| `gx, gy, gz` | angular velocity, sensor frame | rad/s |
| `mx, my, mz` | magnetometer, sensor frame | any consistent units (calibration normalises scale + bias) |

Canonical per-node dict form:

```
{ node_id: {"t": (N,), "acc": (N,3), "gyr": (N,3), "mag": (N,3) [, "t_mag": (M,)]} }
```

(If the magnetometer is sampled at a different rate, supply its own `t_mag` timeline; it is
aligned to the IMU instants internally.)

### (b) gaitlib.compute() API

```python
results = gaitlib.compute(raw_data, mounting_config)   # mounting_config=None → default 4-sensor, 1-leg
```

- **`raw_data`** — any accepted form above.
- **`mounting_config`** — a `MountingConfig` (or dict, or `None`): `sensors` (node→segment),
  `joints` (`{name: (distal_node, proximal_node)}`), `foot_node`, optional `pelvis_node`,
  `rates` (`imu_hz`/`mag_hz`), `fusion` (`run_modes`, `beta_6dof`, `beta_9dof`,
  `joint_tau_s`), `strict`.
- **returns `GaitResults`** — `fs`, `t (N,)`, `joints[name] = {flexion, ang_vel, ang_acc,
  params[, modes]}`, `gait`, `events {foot_strike, mid_swing, toe_off}`, `steady_state`,
  `turnarounds`, `config`, `warnings`, `meta`. Missing a sensor only skips the joints that
  need it (or raises in `strict` mode).

### (c) gait_frames.csv format (Open3D viewer)

The viewer's per-frame contract — written by the app's "Export for Open3D" and
`pipeline/export_open3d`, consumed by `gait_open3d_viewer.py`. One row per sample, **single
leg**, sagittal flexion (not 3D orientation):

```
t_s,hip_deg,knee_deg,ankle_deg
```

| column | meaning | units |
|---|---|---|
| `t_s` | sample time | s |
| `hip_deg` | hip sagittal flexion | deg |
| `knee_deg` | knee sagittal flexion | deg |
| `ankle_deg` | ankle sagittal flexion | deg |

Sidecar `meta.json`:

```json
{ "leg": "right", "fs": 100.0,
  "segment_lengths_m": {"pelvis": .., "thigh": .., "shank": .., "foot": ..} }
```

---

## 5. Future integration points

**(a) Live hardware → same `compute` call.** `gaitlib` already consumes raw 9-DOF samples
with no hardware coupling, and `MountingConfig` makes it agnostic to sensor count (the
real 4-node hardware = 4 streams). A serial/acquisition layer reads the hub, packages each
sample as `{node_id, timestamp, ax..mz}` (firmware note: run the magnetometer at the same
ODR as the IMU so the 9 DOF are already synchronous), and feeds either the per-node dict or
a row stream into the **same** `gaitlib.compute(raw_data, mounting_config)`. No change to
the library; acquisition stays a separate layer, exactly as the file-based path is today.

**(b) Open3D viewer file → socket.** The viewer is driven entirely by the `gait_frames`
per-frame contract (`t_s, hip_deg, knee_deg, ankle_deg`). The same record can be streamed
over a socket instead of read from `gait_frames.csv`: swap the viewer's
`read_frames_from_csv()` for a socket source while keeping the identical per-frame schema
(and `meta.json` for leg + segment lengths). File playback and live feed differ only in the
source, not the contract.
```
