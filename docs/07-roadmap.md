# 07 — Roadmap & Open Items

## Build phases

| Phase | Milestone |
|-------|-----------|
| 1 | Firmware + data path — node sampling, UWB TDMA, master aggregation, host ingest over SWD/SWO |
| 2 | Visualization — orientation → joint angles → DH → live 3D (synthetic data first) |
| 3 | BLE uplink — integrate after wired path is proven |
| 4 | EKF — Kalman filter fusing UWB inter-node distances for drift/position |

## Phase 1 progress

Done (host side — real, tested code):
- [x] Wire protocol spec ([03-communication.md](03-communication.md)) — authoritative.
- [x] Host codec + streaming parser with resync (`ingest/protocol.py`, 233 lines); tests + golden vector (`host/tests/test_protocol.py`).
- [x] Host ingest (`ingest/reader.py`): file replay + serial; `make_source` factory.
- [x] Synthetic source emitting real wire bytes (`sim/generator.py`); capture/replay via `main.py`.

Done (hardware/tooling):
- [x] CubeMX project generated for the STM32WB55CEUx (`firmware/Core/`, `Drivers/`, `Middlewares/`, `ThirdParty/`, `USB_Device/`, STM32CubeIDE project files). Replaces an earlier scaffolded `firmware/node/{App,Comms,Drivers,Sensors,Power}` stub layout (all TODO placeholders, no logic) — that layout was removed.
- [x] IMU + magnetometer bring-up smoke test: firmware reads both sensors and
      prints raw values over serial, viewed with a serial monitor app. No
      sampling pipeline, packet format, or buffering yet — this is a "sensors
      are alive and readable" check, not a data path. **Needs confirming:** this is
      very likely read over native USB-CDC (the MCU has native USB — see
      [02-hardware.md](02-hardware.md)), not SWO as earlier docs assumed;
      reconcile once confirmed.

Not started (correcting a previous "done" claim — these files never had real
implementations and the stub versions have since been deleted):
- [ ] **Firmware packet serializer** — no `packet.c` exists (never did); needs writing
      against the wire protocol spec, golden-vector-matched to the host decoder.
- [ ] **Port abstraction (I2C for sensors, SPI for DWM3000) + LSM6DSV16BXTR driver** —
      not implemented; the old `Drivers/lsm6dsv16b.c` was a stub (`TODO` bodies only).
      Bus/address confirmed by the bring-up smoke test: I2C1, `0x6B`.
- [ ] **Node sample→pack→emit path** — not implemented; the old
      `App/role_sensor.c` was a stub (`for (;;) {}` with TODO comments), now removed.

Next in Phase 1:
- [ ] Implement the three items above against the generated CubeMX `Core/`.
- [ ] SWO/ITM data output; host SWO ingest (SWV → bytes).
- [ ] UWB TDMA schedule + DWM3000 driver; master aggregation.
- [ ] RAM ring buffer for jitter/retransmit (no on-node flash IC — the fabricated
      v1.0 board doesn't populate one; RAM buffering is the only local insurance).

## Parallel workstream — 5-DOF arm (teleoperation + calibration rig)

A 5-DOF robotic arm mirroring human upper-limb kinematics, used both as a
teleoperation demo and as a calibration/validation reference for the IMU
pipeline. Kinematics code lives in [`simulation/scripts/`](../simulation/scripts/)
(not yet folded into `test-rig/`). Mechanical CAD for the rig itself has
started, under [`test-rig/hardware/mechanical/v1.0/`](../test-rig/hardware/mechanical/v1.0/README.md).

- [x] **Forward kinematics** — `shoulder_arm_fk.m`. DH-parameterized, MATLAB GUI
      with joint sliders, real-time 3D visualization, physiological joint-limit
      checking. Validated.
- [x] **Inverse kinematics — implemented, UNTRUSTED / not validated.** `shoulder_arm_ik.m`
      (675 lines): analytical solver using wrist-center decoupling (Pieper-style)
      for position (θ1, θ2, θ4) + orientation matching (θ3, θ5), enumerates up to
      4 candidate solutions (elbow-up/down × sign branches), filters by joint
      limits, ranks by geodesic orientation error. Design doc:
      [`simulation/scripts/ik_design.md`](../simulation/scripts/ik_design.md).
      **Do not rely on this for calibration/ground-truth work until it's validated
      against known FK solutions — treat every output as unverified for now.**
- [ ] **End-effector adapter** — no piece exists yet to mount a wearable IMU
      node at the rig's end effector (needed for both teleop and
      calibration/ground-truth use). Not designed.
- [ ] **Wearable IMU rig variant ("fork")** — a modified wearable IMU enclosure
      built to attach to the rig exists, but isn't in this repo yet; CAD needs
      to be added.
- [ ] **Reprint the `arm` piece** — the physical part currently on the lab rig
      needs a fresh print.

## Hardware revisions

- **v1.0 (current):** first bring-up board. See `hardware/electronics/CHANGELOG.md`.
  Known issue: the DWM3000 UWB antenna placement/keepout doesn't carefully follow
  Qorvo's layout guidelines (clearance, ground pour keepout, feed line length),
  which may affect range/link margin.
- **Decision pending v1.0 evaluation:** first see how well v1.0 actually performs
  (range, link margin, general bring-up) before committing to a fix path.
  - If v1.0 is good enough or only needs the antenna corrected: **v1.1**, a
    placement-only bug-fix spin, no other redesign.
  - If v1.0 falls short in ways that justify a bigger change: skip v1.1 and fold
    the antenna fix into **v2.0** (AoA UWB with custom PCB antennas) instead —
    no point doing two board spins if a larger redesign is coming anyway.

## Open items

- [ ] **Data format:** raw 9-DOF vs SFLP quaternion + raw mag. What this actually
      decides: whether each node transmits raw accel+gyro+mag counts (host does
      100% of the sensor fusion — maximum flexibility, ~40 B/sample) or the
      LSM6DSV16B's onboard SFLP quaternion + raw mag (device does the accel+gyro
      fusion itself, smaller/faster over the air, but less flexible if the fusion
      algorithm ever needs to change). Not yet decided.
- [x] **Magnetometer:** **using it for v1.** Firmware bring-up so far is a serial
      print smoke test only (see Phase 1 above) — no sampling pipeline or packet
      path yet.
- [x] **Battery capacity: 120 mAh.**
- [ ] **Node count for initial tests:** 2 confirmed (wrist, clavicle), chest possible
      as a 3rd → 6–8
- [x] **Sample rate:** **120 Hz** — the LSM6DSV16B ODR grid has no 100 Hz step; 120 Hz
      is the closest to the README target. Samples are timestamped so the exact rate is
      not load-bearing; host can decimate 120→100 if ever needed.
- [x] **BLE receiver: host-native.** The PC's own Bluetooth receives the uplink; no extra receiver hardware.
- [ ] **Orientation filter:** complementary vs Madgwick vs VQF vs SFLP. Not reached
      yet — the pipeline currently only exists as far as forward kinematics +
      visualization (the MATLAB arm GUI); no real orientation-filter work has
      started on either the arm or the wearable side.
- [ ] **EKF scope:** confirm phase-2 (not v1)
- [ ] **IK validation:** confirm `shoulder_arm_ik.m` against known FK solutions (see above)
- [ ] **v1.0 evaluation → v1.1 vs v2.0 decision:** bring up and test v1.0 (range,
      link margin), then decide whether the DWM3000 antenna issue gets a v1.1
      placement-only fix or gets folded into a v2.0 redesign (see "Hardware
      revisions" above)
- [ ] **Onboarding doc for new students** — not written yet. Needs to cover
      at least: STM32CubeProgrammer (flashing, updating the FUS / BLE stack),
      Serial Monitor Pro (viewing the sensor bring-up smoke-test output).
- [ ] **Battery connector:** currently JST-SH 1.0mm 2-pin (`JST_SH_SM02B-SRSS-TB`
      in the v1.0 schematic). Consider switching to **JST-XH 2.54mm 2-pin** for
      v1.1/v2.0 — the connector that ships on most off-the-shelf LiPo packs,
      avoiding re-terminating batteries by hand.
- [ ] **Mechanical BOMs missing:** no bill of materials exists yet for either the
      wearable case (`hardware/mechanical/v1.0/`) or the rig
      (`test-rig/hardware/mechanical/v1.0/`) — fasteners, standoffs, print
      material, and any off-the-shelf hardware (e.g. rig servos) aren't
      documented anywhere. Needs the actual parts list from whoever built them.
- [ ] **Mechanical fabrication process undocumented:** unlike the PCB (fabbed via
      JLCPCB — see `hardware/electronics/v1.0/README.md`), the 3D-printing
      process for the case and rig (printer/service, material, settings) isn't
      written down anywhere.
- [ ] **Tools list undocumented:** no written list of the tools used to build and
      assemble the hardware (wire stripper is the only one mentioned so far) —
      would help new students know what to have on hand.
