# 01 — System Overview

> Expand on README §1–2. Fill in as design decisions are finalised.

## Goals

- Live upper-limb pose estimation from body-worn IMU nodes.
- 100 Hz sample rate, sub-millisecond on-body time sync.
- Thin nodes (sample + transmit only); all fusion/kinematics/viz on the host PC.

## Topology

See README §2 (Mermaid diagrams). Key points:

- N identical STM32WB55CEUx + DWM3000 nodes. One is designated **master** in firmware.
- On-body UWB bus: data + time-sync + inter-node ranging in a TDMA frame.
- Single BLE uplink from master to host (primary). UWB-USB dongle = backup path.

## Design philosophy

- **Thin nodes, heavy host** — avoids reflashing for algorithm changes.
- **Identical hardware** — any node can be master; simplifies spares and testing.
- **Wired-first bring-up** — SWD/SWO before BLE; de-risks each layer independently.
