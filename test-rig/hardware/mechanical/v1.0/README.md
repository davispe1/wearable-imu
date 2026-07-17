# test-rig / mechanical / v1.0

5-DOF arm rig — mechanical design, revision 1.0.

## Contents

- `RIG.f3z` — Fusion 360 archive (native source)
- `isometric_rig_view.png` — isometric render

![Rig isometric view](isometric_rig_view.png)

Servo-driven links matching the 5-DOF DH model used in
[`simulation/scripts/shoulder_arm_fk.m`](../../../../simulation/scripts/shoulder_arm_fk.m)
and `shoulder_arm_ik.m`.

TODO: STEP/STL export for non-Fusion viewers, dimensions writeup, servo/mount BOM.

## Known gaps

- **End-effector adapter (missing):** no piece exists yet to physically mount a
  wearable IMU node at the rig's end effector — needed for both the teleop and
  calibration/ground-truth use cases. Not designed.
- **Wearable IMU rig variant ("fork") — not yet in this repo:** a modified
  version of the wearable IMU enclosure exists (built to attach to the rig
  instead of being worn), but its CAD/design files haven't been added here yet.
- **`arm` piece needs reprinting:** the physical `arm` part currently on the lab
  rig needs a fresh print (current one is out of date / not usable as-is).

## Bill of materials

**Not yet filled in — no BOM exists for this rig.** Servos (5-DOF implies at
least 5, but model/count isn't documented), mounting hardware, and printed
parts all need to be listed here from the actual physical build.

| Category | Item | Qty | Notes |
|----------|------|-----|-------|
| Printed | Links / joints (per `RIG.f3z`) | ? | Not itemized per-part yet |
| Actuation | Servo(s) | ? | Model not documented |
| Fasteners | *(not documented)* | ? | |
| Electronics | Servo driver / controller | ? | See `test-rig/hardware/electronics/v1.0/` — planned, not created |

## Fabrication

**Not yet documented** — printing method (printer/service), material, and
print settings used for the rig parts aren't written down anywhere.
