# electronics / v1.0

Node PCB — revision 1.0. 4-layer board (F.Cu, In1.Cu, In2.Cu, B.Cu).

> **Known issue, fix path pending v1.0 evaluation:** the DWM3000 UWB antenna
> placement doesn't fully follow Qorvo's layout guidelines (clearance/keepout,
> feed line length), which may affect range/link margin. Whether this becomes a
> v1.1 placement-only fix or gets folded into a v2.0 redesign depends on how
> v1.0 performs in bring-up. See `../CHANGELOG.md` and `docs/07-roadmap.md`.

## Contents

- `wearable_IMU.kicad_sch` — schematic (source)
- `wearable_IMU.kicad_pcb` — layout (source)
- `wearable_IMU.kicad_pro` / `.kicad_prl` — project files
- `documentation/` — exported images (below) + `schematic.pdf`
- `jlcpcb/` — fab outputs: `gerber/` (raw Gerbers + drill files), `production_filesV1.0/`
  (BOM, pick-and-place, zipped Gerbers — the JLCPCB submission set), `project.db`
- `fabrication-toolkit-options.json` — KiCad fabrication-toolkit plugin export settings
- The PCB's 3D model (`PCB.step`) lives in `../../mechanical/v1.0/` alongside the
  enclosure — kept in one place instead of duplicated here

Cleaned up (untracked, safe to regenerate/ignore): KiCad's local autosave/history
folders (`.history/`, `.history_trim/` — an embedded git history plugin, ~290 MB),
`wearable_IMU-backups/` (autosave zips), a stale `production/` test export
("prueba1"), an empty `jlcpcb/production_files/`, 0-byte `.rpt` temp files, and
KiCad's `.lck` lock files. Added patterns to `.gitignore` so these don't come back.

## Key parts

| Function | Part |
|----------|------|
| MCU | STM32WBA55CGU7 |
| IMU | LSM6DSV16BXTR |
| Mag | BMM350 |
| UWB | DWM3000 |
| Flash | W25Q64JVXGIM |
| SMPS | TPSM828224 |
| Charger | BQ25185 |
| Fuel gauge | MAX17048G+T10 |
| Power btn | STM6601BM2DDM6F |

## Board renders & layers

**3D views**

| Top | Bottom |
|---|---|
| ![PCB 3D top](documentation/pcb-3d-top.png) | ![PCB 3D bottom](documentation/pcb-3d-bottom.png) |

**Copper layers** (top → bottom of stack)

| F.Cu (top) | In1.Cu | In2.Cu | B.Cu (bottom) |
|---|---|---|---|
| ![Top copper](documentation/pcb-layer-top-copper.png) | ![Inner 1](documentation/pcb-layer-inner1.png) | ![Inner 2](documentation/pcb-layer-inner2.png) | ![Bottom copper](documentation/pcb-layer-bottom-copper.png) |

**Schematic:** [`documentation/schematic.pdf`](documentation/schematic.pdf)
