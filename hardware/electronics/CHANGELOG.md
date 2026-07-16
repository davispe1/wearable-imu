# Electronics CHANGELOG

## v1.1 — decision pending v1.0 evaluation

- Known issue: DWM3000 UWB antenna placement/clearance/keepout doesn't fully
  follow Qorvo's layout guidelines, which may affect range/link margin.
- Not yet decided whether this becomes a v1.1 placement-only fix or gets folded
  into a v2.0 redesign (AoA UWB + custom PCB antennas) instead — depends on how
  well v1.0 performs in bring-up. See `docs/07-roadmap.md`.

## v1.0 — initial bring-up revision

- First layout. STM32WBA55 + DWM3000 + LSM6DSV16B + BMM350 + W25Q64.
- USB-C charging only (BQ25185). No MCU USB.
- TC2030-IDC footprint (Tag-Connect) for debug/data — bare pogo-pin pads on the PCB, no
  soldered connector; mates with a separate TC2030 cable/clip.
