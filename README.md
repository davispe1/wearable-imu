# IIT Gait

IMU gait kinematics, **fundamentals-first**: raw inertial data from 4 body-worn nodes →
**validated** orientation estimation → **kinematic parameters** (joint angles + spatiotemporal
gait), visualized in Python, in an **Open3D** 3D viewer, and (optionally) tracked with
**OpenSim OpenSense** inverse kinematics.

```
raw IMU CSV ─► VQF orientation ─► sagittal joint angles ─► gait events ─► gait parameters ─► viewer
 core.rawdata    core.fusion_vqf   kinematics.joint_angles  kinematics.    kinematics.        app.viewer /
                                                             gait_events    parameters         gait_open3d_viewer.py
```

The kinematic product (joint angles + gait parameters, computed and plotted in Python) is the
core of this repo. Two things build on top of it, both optional:

- **Open3D viewer** (`gait_open3d_viewer.py`) — plays the computed joint angles back as an
  animated 3D mannequin.
- **OpenSim OpenSense** (`opensim_export/`, `opensim/`) — exports the fused sensor orientations
  so OpenSim's own inverse-kinematics solver can track them on a musculoskeletal model.

Orientation fusion — the part that is easy to get subtly wrong — is delegated to a peer-reviewed
estimator (**VQF**, Laidig & Seel 2023); every gait parameter is grounded in published IMU-gait
methods (see [`docs/kinematics.md`](docs/kinematics.md) and [`docs/method.md`](docs/method.md)).

---

## 1. What you need installed

| Component | Required for | Notes |
|---|---|---|
| **Python 3.10+** | everything | |
| `pip install -r requirements.txt` | kinematics pipeline + 2D plots | installs `numpy`, `scipy`, `vqf`, `matplotlib` |
| `pip install open3d` | the 3D mannequin viewer | **not** in `requirements.txt` — only needed if you run `gait_open3d_viewer.py` |
| **OpenSim 4.5** (desktop app) | the optional OpenSense IK path | a separate application, **not** a Python package — download from the [OpenSim project](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53088363/OpenSense+-+Kinematics+with+IMU+Data). Never imported by this repo's Python code. |

```bash
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pip install open3d          # only if you want the 3D viewer
```

Everything below runs **from this folder** (`IIT Gait/`, the project root).

---

## 2. How data must be organized

A recording is **one folder per session** with one CSV per sensor node, all on a shared
timebase set by the hub:

```
data/
  <session_id>/
    SA.csv        ← pelvis
    RT.csv        ← right thigh
    RS.csv        ← right shank
    RF.csv        ← right foot
    (LT/LS/LF.csv for a left-leg rig instead)
```

Each CSV needs these columns (9 DOF + time); column names are resolved automatically
(`t_opt_s`/`t_s`, `acc_x`/`ax`, `gyr_x`/`gx`, etc. — see `core/rawdata.py`):

| Column | Meaning | Units |
|---|---|---|
| `t_opt_s` | shared hub clock, 0 at session start (all nodes MUST share this) | s |
| `ax, ay, az` | linear acceleration, sensor frame | m/s² |
| `gx, gy, gz` | angular velocity, sensor frame | rad/s |
| `mx, my, mz` | magnetic field, sensor frame | a.u. |

A combined long-format file (`<session>/raw/data.csv` with an extra `node` column) is also
accepted. A minimal template is at [`data/schema_example.csv`](data/schema_example.csv).

The default rig is **4 sensors, one leg + pelvis** — the minimum chain for hip/knee/ankle
(`core/config.py`). It is sensor-count agnostic: add nodes for the other leg, or drop to fewer
joints, by changing the config, not the code.

| Node | Segment | OpenSim IMU column |
|---|---|---|
| `SA` | pelvis | `pelvis_imu` |
| `RT`/`LT` | thigh | `femur_<r\|l>_imu` |
| `RS`/`LS` | shank | `tibia_<r\|l>_imu` |
| `RF`/`LF` | foot | `calcn_<r\|l>_imu` |

Only the measured leg is ever exported — the contralateral leg is never fabricated.

---

## 3. Generate results — commands

Run in order, all from `IIT Gait/`:

### 3.1 Kinematic parameters (the core product)

```bash
python -m kinematics.pipeline data/<session_id> --csv
```

Writes into `data/<session_id>/results/`:

- `<id>_joint_angles.csv` — angles + angular velocity + event flags, one row per sample
- `<id>_gait_events.csv` — event table (time, sample, type)
- `<id>_gait_parameters.json` — ROM, cadence, stride/step time, stance/swing, stride length…

`--mode 6D` (default, no magnetometer) / `9D` / `auto`, `--side right|left` to override the
auto-detected leg.

### 3.2 View in 2D (Python plots)

```bash
python -m app.results_gui data/<session_id>     # interactive GUI: plots tab + parameters table
python -m app.viewer data/<session_id> --save    # command-line figure -> also writes the PNG
python -m app.viewer data/<session_id> --full     # whole bout instead of the default 12 s window
```

### 3.3 View in 3D (Open3D mannequin)

The Open3D viewer plays back `gait_frames.csv` + `meta.json`, which the pipeline/viewer step
above already writes into `results/` for the bundled examples. To (re)generate them for a new
session, export explicitly:

```bash
python -m app.viewer data/<session_id> --save     # also produces results/gait_frames.csv + meta.json
python gait_open3d_viewer.py data/<session_id>/results/gait_frames.csv
```

Controls: `SPACE` pause/resume · `J` slower · `K` faster · `R` restart · mouse to rotate/zoom.

### 3.4 OpenSim OpenSense (optional inverse kinematics)

```bash
# 1. Export fused orientations for OpenSense
python -m opensim_export.to_sto data/<session_id>
#    -> data/<session_id>/results/<id>_orientations.sto   (full trial)
#    -> data/<session_id>/results/<id>_calibration.sto    (static pose, ~1 s)

# 2. (optional) Build a calibration pose from a different window
python -m opensim_export.make_calibration <orientations.sto> <out_calib.sto> --window 0 5
python -m opensim_export.make_calibration <orientations.sto> <out_calib.sto> --auto-neutral
```

Then, in the **OpenSim 4.5 GUI** (or `opensense` CLI), follow
[`docs/opensim_steps.md`](docs/opensim_steps.md): open the model in
[`data/Rajagopal_OpenSense/Rajagopal2015_opensense.osim`](data/Rajagopal_OpenSense/Rajagopal2015_opensense.osim)
(the Rajagopal 2015 model with IMU frames — already included, no separate download needed),
calibrate with `<id>_calibration.sto` (templates in [`opensim/setups/`](opensim/setups/)), then
run IMU IK against `<id>_orientations.sto` to produce `<id>_ik.mot`.

Self-check the Python side of this path end-to-end (no OpenSim install required — only checks
the `.sto` files this repo writes):

```bash
python verify_chain.py
```

### 3.5 Synthetic ground-truthed data (optional, for validation)

Real recordings have no ground truth. `tools/make_synthetic_session.py` forward-simulates a
session with a **known** hip/knee/ankle trajectory, so the whole chain can be scored:

```bash
python -m tools.make_synthetic_session      # writes data/SYN01_S01_straightWalk/
python -m tools.validate_synthetic          # scores the pipeline vs ground_truth.json
```

See [`docs/synthetic_data.md`](docs/synthetic_data.md) for the method and options
(`--strides`, `--fs`, `--gyro-drift`, `--noise`, …).

---

## 4. Bundled examples

Two sessions ship with **pre-computed results**, so you can try every viewer immediately
without running the pipeline first:

- **`data/P04_S01_2minWalk`** — a real ~2-minute walking recording (Physilog, 256 Hz), the
  dataset used for development/validation. `results/` has the full chain: joint angles, gait
  events/parameters, the Python kinematics figure, the Open3D frames, and one OpenSense IK run
  (`P04_S01_calibrated.osim` + `P04_S01_ik.mot`).
- **`data/SYN01_S01_straightWalk`** — the synthetic, ground-truthed session from §3.5
  (`ground_truth.json` has the true angles/parameters to compare against). `results/` likewise
  has the full chain including its OpenSense IK run.

Try, with no setup beyond `pip install -r requirements.txt`:

```bash
python -m app.viewer data/P04_S01_2minWalk
python gait_open3d_viewer.py data/P04_S01_2minWalk/results/gait_frames.csv   # needs: pip install open3d
```

---

## 5. Repository layout

```
IIT Gait/
  core/
    rawdata.py        per-node CSV loader + shared-timebase (hub) contract
    config.py          MountingConfig — 4-sensor one-leg default (pelvis, thigh, shank, foot)
    fusion_vqf.py       VQF wrapper -> per-segment sensor->earth quaternions (6D/9D, multi-rate)
  kinematics/          the kinematic product (raw IMU -> gait parameters)
    quaternion.py       small quaternion utilities (gravity, world-rotation)
    joint_angles.py     functional-axis, gravity-projection sagittal flexion
    gait_events.py      shank-gyro events + pelvis turnaround / steady mask
    parameters.py       per-joint + temporal + spatial(estimate) + cycle overlay
    pipeline.py         analyze_session() -> KinematicResults   (CLI)
    results.py          KinematicResults + CSV/JSON exporters
  app/
    viewer.py            matplotlib figure: angles / events / mean cycle / parameters   (CLI)
    results_gui.py       tkinter GUI: interactive figure + parameter table              (CLI)
  gait_open3d_viewer.py  Open3D 3D mannequin viewer, plays back results/gait_frames.csv  (CLI)
  opensim_export/        SEPARATE optional path -> OpenSense .sto
    segment_map.py        node/segment -> OpenSim IMU column name (measured side only)
    to_sto.py             write *_orientations.sto + *_calibration.sto                  (CLI)
    make_calibration.py   build a custom calibration .sto from a window or auto-neutral  (CLI)
  opensim/
    setups/               IMU Placer + IMU IK setup .xml templates (OpenSim 4.5)
    README.md              where the Rajagopal OpenSense model comes from
  tools/
    make_synthetic_session.py   forward-simulated ground-truthed session generator       (CLI)
    validate_synthetic.py       scores the pipeline against that ground truth            (CLI)
  docs/
    kinematics.md          the kinematic pipeline + parameter references   <- primary
    method.md               the optional OpenSim/OpenSense path + references
    sensor_placement.md
    opensim_steps.md        step-by-step OpenSense GUI workflow
    synthetic_data.md        method behind the synthetic fixture
    quickstart.md            condensed command walkthrough (Spanish)
    arquitectura.md          architecture notes
  data/
    schema_example.csv           minimal per-node 9-DOF CSV template
    Rajagopal_OpenSense/          Rajagopal 2015 model with IMU frames, for OpenSense
    P04_S01_2minWalk/             real example session + full precomputed results
    SYN01_S01_straightWalk/       synthetic ground-truthed example + full precomputed results
  verify_chain.py         end-to-end self-check (OpenSim export path, no OpenSim install needed)
  requirements.txt
```

---

## References

- **VQF** — D. Laidig, T. Seel. *VQF: Highly Accurate IMU Orientation Estimation with Bias
  Estimation and Magnetic Disturbance Rejection.* Information Fusion 91:187–204, 2023.
  doi:10.1016/j.inffus.2022.10.014.
- **OpenSense** — M. Al Borno et al. *OpenSense: An open-source toolbox for IMU-based measurement
  of lower-extremity kinematics over long durations.* J. NeuroEngineering and Rehabilitation
  19:22, 2022. doi:10.1186/s12984-022-01001-x.
- **Rajagopal model** — A. Rajagopal et al. *Full-body musculoskeletal model for muscle-driven
  simulation of human gait.* IEEE TBME 63(10):2068–2079, 2016.

Full reference list in [`docs/method.md`](docs/method.md).
