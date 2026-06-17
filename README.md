# Gait Kinematics

**Gait Kinematics** turns raw 9-DOF inertial/magnetic sensor data into commonly measured
gait parameters — joint angles (ankle/knee/hip flexion), ranges of motion, angular
velocities, cadence, stride/step time, stance/swing — and lets you review them in a desktop
viewer.

It has two parts:

| part | what it is |
|---|---|
| **`gaitlib/`** | the **library** — a pure `data in → parameters out` engine. No hardware, serial, network, or 3D dependencies. |
| **`app/`** | the **viewer app** — imports recordings, runs the pipeline, shows 2D joint-angle / angular-velocity curves and per-joint + gait parameters, and exports results. |

> **The app depends on `gaitlib` as its single source of truth.** All kinematics
> (orientation fusion, joint angles, gait events, parameters) live in `gaitlib`; the app
> keeps no copy of the math — it just feeds the library and presents what comes back.

---

## Install

```bash
python -m venv .venv && source .venv/bin/activate     # (Windows: .venv\Scripts\activate)
pip install -r requirements.txt
```

`gaitlib` itself only needs **numpy** and **scipy**. The viewer app additionally needs
**pyqtgraph**, **PyQt5**, and **pyyaml**. The research/dataset pipeline (`pipeline/`) needs a
few more (ezc3d, pandas, matplotlib, openpyxl) and the Geneva dataset, which is **not**
included.

## Run

**The library:**

```python
import gaitlib

results = gaitlib.compute(raw_data, gaitlib.default_config())   # default: 4 sensors, one leg
print(results.gait["cadence_steps_per_min"])
print(results.joints["knee"]["params"]["rom_deg"])
results.save_joint_angles_csv("knee_angles.csv")               # for manual goniometer checks
```

**The viewer app:**

```bash
python -m app.main
```

Use **Sessions → Import CSV…** to load a recording (or **Add Geneva test data** if the
dataset is present), then **Visualize** (synced 2D curves + live parameters + scrubber) and
**Results** (per-joint & gait parameters, overlaid gait cycles, export).

**Equivalence/regression check** (reproduces the previously validated parameters on the
bundled slice, if present):

```bash
python gaitlib/tests/test_geneva_regression.py
```

## Export for the Open3D viewer (separate step)

A processed session can be exported for an **external Open3D viewer** (a separate project).
**Open3D is not a dependency of this repo** — this is a standalone export step that only reads
results gaitlib already produced and writes two small files into the session's `results/`
folder:

- `gait_frames.csv` — `t_s, hip_deg, knee_deg, ankle_deg`, one row per sample: the clean
  **sagittal flexion** angles from gaitlib (`results.joints[<joint>]["flexion"]`, *not* 3D
  orientation), for a single leg (the one in the mounting config).
- `meta.json` — `{"leg", "fs", "segment_lengths_m": {pelvis, thigh, shank, foot}}`; segment
  lengths are estimated from the subject's stature via Winter's body-segment proportions (a
  default stature is used and noted in the file when none is available).

```bash
python -m pipeline.export_open3d <session_dir>      # or: Results → "Export for Open3D"
```

The `gait_frames` schema is also the **live-streaming contract**: the same per-frame record
(`t_s, hip_deg, knee_deg, ankle_deg`) can later be streamed over a socket from live hardware
instead of written to a file — the viewer's frame schema is identical, so nothing on the
viewer side changes.

---

## Input schema (the 9-DOF raw-data contract)

Per **sample** the firmware/hub must provide:

| field | meaning | units |
|---|---|---|
| `timestamp` | time in seconds; all nodes MUST share a common timebase synchronized by the hub (≤ 1 ms alignment). Do not use per-node independent clocks. | s |
| `node_id` | sensor/segment id (must match the mounting config) | — |
| `ax, ay, az` | linear acceleration, sensor frame | m/s² |
| `gx, gy, gz` | angular velocity, sensor frame | rad/s |
| `mx, my, mz` | magnetometer, sensor frame | any consistent units (calibration normalises) |

> **Note:** The hub is responsible for timestamping and aligning streams before passing them to gaitlib. If nodes have independent clocks, gaitlib results will be wrong.

As a CSV (combined, long format):

```
node,t_s,ax,ay,az,gx,gy,gz,mx,my,mz
```

`gaitlib.compute` also accepts a per-node dict or a structured numpy array (see
`gaitlib/README.md`). A tiny example lives at `data/schema_example.csv`.

**Sample rate.** The default/ideal case is all 9 DOF synchronous at the IMU rate (e.g.
100 Hz) — no resampling. A magnetometer at a different rate is accepted and aligned to the
IMU internally. *Firmware tip:* set the magnetometer ODR equal to the IMU (100 Hz) so the
9 DOF are already synchronous.

## How the data must be organised

A recording is a set of per-node files (or one combined CSV) under `data/`:

```
data/
  <recording_id>/
    RF.csv  RS.csv  RT.csv  SA.csv     # per-node files, OR
    data.csv                           # one combined long-format file
```

The **mounting config** maps node ids → body segments and declares the joint topology. The
default is **4 sensors on one leg**:

```python
gaitlib.default_config(side="right")
# sensors: RF→foot, RS→shank, RT→thigh, SA→pelvis
# joints : ankle=(RF,RS), knee=(RS,RT), hip=(RT,SA)
```

The library is **agnostic to the number of sensors** — declare more sensors/joints for both
legs (`gaitlib.both_legs_config()`), or any subset. A joint whose two segments aren't both
present is skipped with a warning (or raises in `strict=True`).

> **Datasets are not included.** The Geneva reference dataset (Grouvel et al., 2023) and any
> recordings must be obtained separately and placed under `data/` (see `data/README.md`).
> The app imports recordings into its own store at `~/GaitApp/sessions/` (outside the repo).

## Accuracy validation

`gaitlib` never requires optical/marker data to run. Real-hardware accuracy validation is
**manual**: export the computed joint angle (`results.save_joint_angles_csv(...)`) and
compare it against a goniometer or known reference angles. (Optical markers were only a past
validation aid for the research pipeline.)

---

## Repository layout

```
gaitlib/        the Gait Kinematics library (pure; numpy + scipy)
  __init__.py     public API: compute(), default_config(), MountingConfig, ...
  pipeline.py     compute(raw_data, mounting_config) orchestration
  config.py       MountingConfig (sensors → segments → joints, rates, fusion)
  rawdata.py      input contract + parsing + per-channel rate alignment
  fusion.py       Madgwick 6/9-DOF orientation         (validated kinematic core)
  calibration.py  magnetometer hard/soft-iron           (validated kinematic core)
  angles.py       yaw-immune sagittal joint angles      (validated kinematic core)
  gait.py         gait events, cadence                  (validated kinematic core)
  segment.py      turnaround / steady-state masking     (validated kinematic core)
  parameters.py   per-joint + gait parameter extraction
  results.py      GaitResults container + exporters
  README.md       library reference (API, equations, symbols)
  tests/          Geneva equivalence/regression test

app/            the viewer app (depends on gaitlib)
  main.py         windowed app: Sessions / Visualize / Results
  pipeline_runner.py  thin wrapper that calls gaitlib.compute
  session_store.py    on-disk session store + CSV import
  analysis.py         reshapes results for the views (no kinematics)
  ui_*.py             the three views + shared widgets

pipeline/       research/dataset pipeline (needs the Geneva dataset; optional)
  run.py, extract.py, align.py, bin_reader.py, fusion_check.py, selftest.py,
  visualize.py, export_open3d.py, validation/, adapters/, tools/
  (export_open3d.py: standalone Open3D-viewer export; needs no dataset and no open3d)

config/         app.yaml (app) + default/p01/p02 yaml (research pipeline)
docs/           methods + guides (+ figures)
data/           recordings go here — NOT included (placeholders only)
requirements.txt
README.md
```

---

## What to upload to GitHub (and what to leave out)

You are uploading manually. Use this list.

### ✅ Upload

```
README.md
requirements.txt
.gitignore
gaitlib/            (all .py, README.md, tests/)   — but NOT __pycache__
app/                (all .py)                       — but NOT __pycache__
pipeline/           (all .py, validation/, adapters/, tools/) — but NOT __pycache__
config/             (app.yaml, default.yaml, p01_left.yaml, p02_right.yaml)
docs/               (*.md and docs/assets/*.png figures)
data/.gitkeep
data/README.md
data/schema_example.csv
```

### ❌ Do NOT upload

```
Human gait and other movements .../        the Geneva dataset (large, licensed, READ-ONLY)
data/<recording_id>/                        real recordings & dataset slices (e.g. P01_S01_2minWalk/)
outputs/                                     pipeline-generated CSV/JSON/PNG
~/GaitApp/sessions/                          the app's session store (outside the repo anyway)
docs/Gait_Kinematics_Methods.docx            generated docs artifact
**/__pycache__/, *.pyc                       Python caches
.venv/ / venv/ / env/                        virtual environments
.vscode/, .idea/, .DS_Store, Thumbs.db       editor / OS cruft
```

The included `.gitignore` already encodes these rules, so `git add .` will pick up the right
files; the list above is the explicit reference for a manual upload.
