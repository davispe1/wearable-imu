# mechanical / v1.0

Node enclosure — revision 1.0.

## Contents

- `PCB.step` — PCB STEP file (for ECAD/enclosure co-design)
- `CASE.stl`, `case1.stl` — enclosure, 3D-printable

## Assembly

| Orthographic (front/top/side) | Exploded (isometric) | Exploded (lateral) |
|---|---|---|
| ![Assembly orthographic](assembly-views-orthographic.png) | ![Assembly exploded](assembly-exploded-view.png) | ![Assembly exploded lateral](assembly-exploded-lateral.png) |

**On device:**

![Worn on wrist](device-worn-on-wrist.jpg)

## Design notes

3-part shell (top cover + base) with strap slots, USB-C cutout, and a
power-button cutout. Case + battery + PCB assemble as shown above.

**Note:** two STL files exist (`CASE.stl`, `case1.stl`) and it isn't documented
which is the current/final one vs. an earlier iteration — needs clarifying.

TODO: dimensions writeup, mounting/strap interface details, connector cutout
tolerances.

## Bill of materials

**Not yet filled in — no BOM exists for this assembly.** Template below; needs
the actual parts list from whoever built the physical unit.

| Category | Item | Qty | Notes |
|----------|------|-----|-------|
| Printed | Case top cover | 1 | Which STL is current? (see note above) |
| Printed | Case base | 1 | |
| Fasteners | *(not documented)* | ? | Screw sizes/type not written down anywhere |
| Electronics | PCB assembly | 1 | See [`hardware/electronics/v1.0/`](../../electronics/v1.0/README.md) |
| Electronics | LiPo battery, 120 mAh | 1 | JST-SH 1.0mm 2-pin connector (see roadmap open item on switching to JST-XH) |
| Other | Wrist strap | 1 | Material/source not documented |

## Fabrication

**Not yet documented** — printing method (printer/service), material, and print
settings used for `CASE.stl`/`case1.stl` aren't written down anywhere. Compare
to [`hardware/electronics/v1.0/README.md`](../../electronics/v1.0/README.md#fabrication)
for the equivalent PCB fabrication write-up.
