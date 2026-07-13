# Sensor placement

The default rig is **4 IMUs on one leg + the pelvis** — the minimum that closes the
pelvis → thigh → shank → foot chain OpenSense needs for hip, knee and ankle angles. Instrument
the side you want to analyse (right or left); the pipeline mirrors automatically from the node
ids and emits only that leg's columns.

| Node | Segment | Body landmark | Mounting |
|------|---------|---------------|----------|
| `SA` | pelvis  | **sacrum**, over S1–S2, midline posterior | flat on the bony sacrum, between the posterior superior iliac spines |
| `RT`/`LT` | thigh | **anterior** distal third of the thigh | on the flat anterior surface above the patella, away from bulk muscle belly |
| `RS`/`LS` | shank | **anterior** mid/distal shank | flat on the subcutaneous medial face of the tibia (shin) |
| `RF`/`LF` | foot  | **dorsum** of the foot | flat on the dorsal midfoot, over the metatarsals, clear of the ankle |

General rules:

- **Rigid, bony, low-tissue sites.** Sacrum, anterior tibia and foot dorsum are close to bone,
  which minimises soft-tissue artefact (the IMU should move with the segment, not the muscle).
- **Firm attachment.** Double-sided tape plus an elastic wrap; no rocking or sliding. Relative
  motion between sensor and segment is the dominant error source and is exactly what the
  OpenSense calibration pose cannot fully remove.
- **Consistent, near-anatomical axes.** Align each IMU roughly with the segment's long axis and
  keep the orientation consistent across subjects. The exact sensor-to-segment offset does not
  need to be measured — the IMU Placer estimates it from the static calibration pose — but a
  repeatable, sensible placement keeps that offset small and well-conditioned.
- **Static calibration pose.** Record ~1–2 s of a still, known posture (upright standing) at the
  start of the trial; `to_sto` averages the first ~1 s into `*_calibration.sto`. Stillness here
  directly sets the quality of model placement.

## Literature basis

These sites follow common IMU gait-analysis practice and the source dataset's protocol:

- **OpenSense** validated lower-limb IMU kinematics with sensors on pelvis, thigh, shank and
  foot, registered to the Rajagopal model via a static pose — the workflow this project targets
  (Al Borno et al., 2022).
- Gravity-referenced, segment-mounted inertial sensing for sagittal joint angles is
  well-established for the foot/shank/thigh chain (Seel et al., 2014).
- The bundled example slices come from the Geneva dataset, which mounts Physilog 6S units on
  these same lower-limb segments alongside optical-marker ground truth (Grouvel et al., 2023).

See [`method.md`](method.md) for full citations.
