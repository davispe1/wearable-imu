# OpenSim / OpenSense assets

This folder holds the OpenSim-side configuration. **OpenSim is not a Python dependency** — it
runs as its own GUI application (or command-line `opensense`), consuming the `.sto` files that
`python -m opensim_export.to_sto` writes.

```
opensim/
  setups/
    imu_placer.xml   IMU Placer (calibration) setup template
    imu_ik.xml       IMU Inverse Kinematics setup template
  README.md          (this file)
```

## The model: Rajagopal with IMU frames

OpenSense places virtual IMUs on a musculoskeletal model and tracks their measured
orientations. You need the **Rajagopal 2015 full-body model carrying IMU frames** — physical
offset frames named exactly:

```
pelvis_imu   femur_r_imu   tibia_r_imu   calcn_r_imu
             femur_l_imu   tibia_l_imu   calcn_l_imu
```

These names are exactly what `opensim_export/segment_map.py` emits as `.sto` column headers,
so the orientation columns bind to the right bodies automatically.

**Where to get it (ships with OpenSim):**

- It is distributed with the **OpenSense example / getting-started dataset** for OpenSim 4.x,
  available from the [OpenSim OpenSense documentation](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53088363/OpenSense+-+Kinematics+with+IMU+Data)
  (SimTK project *opensim-models* / the "OpenSenseExample" download). The model file is
  `Rajagopal_2015.osim` with IMU frames attached.
- A ready calibrated copy also ships inside the OpenSim install at
  `…/OpenSim 4.5/sdk/Python/opensim/tests/calibrated_model_imu.osim` — useful for confirming
  the IMU frame names, though for a real subject you run the IMU Placer yourself.

Put the model next to the `.sto` files (in the session's `results/` folder) or give the full
path in the setup XML, then follow [`../docs/opensim_steps.md`](../docs/opensim_steps.md).

## Single-leg note

This rig instruments **one leg + pelvis** (4 IMUs). `to_sto` only writes columns for the
measured leg, so OpenSense will only place/track those IMUs; the contralateral leg keeps its
default model pose. Before tracking, lock (or ignore) the non-measured leg's coordinates as
described in the steps doc so its joints don't drift.

## The `sensor_to_opensim_rotations` placeholder

Both setups carry `sensor_to_opensim_rotations = -1.5707963267948966 0 0` (≈ `-1.5708 0 0`,
i.e. −π/2 about X). VQF reports orientations in a **Z-up** world; OpenSim ground is **Y-up**,
so this single rotation reconciles the two frames. It is a starting point — if your model
faces sideways or upside-down in the IMU Placer preview, adjust this vector (and keep the
Placer and IK values identical).
