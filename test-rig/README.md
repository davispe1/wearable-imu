# test-rig/

Self-contained sub-project: a motorised 5-DOF arm rig serving **two purposes** —
(1) a teleoperation demo, driven live from the wearable IMU nodes, and (2) a
calibration/ground-truth reference, moved through known joint angles to validate
the IMU pose estimation pipeline against a known reference.

**Status: mostly not started.** Mechanical design of the rig arm is underway
(see `hardware/mechanical/v1.0/`); firmware, control software, and the servo
driver PCB don't exist yet. The empty placeholder folders that used to sit
under this one (`control/`, `firmware/`, `hardware/electronics/v1.0/`) were
removed to keep the repo tree honest — they held nothing but stubs. They'll be
recreated once work actually starts on each part.

## Purpose

**Teleoperation:**
1. Read live wearable IMU node data.
2. Drive the rig's joints to mirror the estimated human arm pose in real time.

**Calibration / ground truth:**
1. Drive the rig through known joint angles.
2. Simultaneously record IMU node data.
3. Compare estimated angles to ground truth to quantify filter error.

## Structure

| Folder | Status | Contents |
|---|---|---|
| [`hardware/mechanical/v1.0/`](hardware/mechanical/v1.0/README.md) | **In progress** | Rig 3D files (arm links, servo mounts) — Fusion 360 source + isometric render. |
| `hardware/electronics/v1.0/` | Planned, not created | Rig servo driver / controller PCB. MCU + servo driver choice: TBD. |
| `firmware/` | Planned, not created | Rig motion controller firmware. Git-versioned, no version folders — tag releases as `rig-vX.Y.Z`. |
| `control/` | Planned, not created | Host-side: drive the rig through scripted motions, log ground-truth joint angles (timestamped CSV), and compare against the IMU pipeline's estimated angles (error plot). |

Follows the same versioning convention as the main project: hardware gets
version folders, firmware/control are git-tagged (`rig-vX.Y.Z`, separate from
the main system's `imu-vX.Y.Z`).

## Related work

The 5-DOF arm forward/inverse kinematics — used both as a teleoperation demo
and as this rig's calibration/validation reference — currently lives in
[`simulation/scripts/`](../simulation/scripts/) (`shoulder_arm_fk.m`,
`shoulder_arm_ik.m`), not yet inside this sub-project's structure. FK is
implemented and validated; IK is implemented (`shoulder_arm_ik.m`,
analytical 5-DOF solver) but not yet validated against known solutions.
