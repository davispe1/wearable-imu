# Contributing

## Branch convention

`main` is **protected** — no direct pushes. All changes land via pull request.

Create short-lived branches prefixed by subsystem:

| Prefix | Use for |
|--------|---------|
| `firmware/<task>` | Node firmware (e.g. `firmware/uwb-tdma`) |
| `host/<task>` | PC pipeline/viz (e.g. `host/orientation-filter`) |
| `hardware/<task>` | Schematic/layout/case (e.g. `hardware/node-rev1.1`) |
| `rig/<task>` | Test-rig firmware or control (e.g. `rig/motion-control`) |
| `docs/<task>` | Documentation only |

Open a PR into `main`, get a review, merge, delete the branch. No long-lived
`develop` branch — this scale doesn't need Git Flow.

## Release tags

Semantic versioning via git tags. Two tag namespaces:

- `imu-vX.Y.Z` — wearable system (firmware + host + matching `hardware/*/vX.Y/` folders)
- `rig-vX.Y.Z` — test rig

A tag snapshots the matched set: the code state is permanently tied to the
physical hardware revision folders that belong to it.

## Versioning rules

- **Hardware** (`hardware/electronics/`, `hardware/mechanical/`) — discrete fabricated
  revisions. Use **version folders** (`v1.0/`, `v1.1/`, …) so you can keep multiple
  board revs side by side.
- **Firmware + host** — continuous iteration. Git *is* the version history.
  **No** `v1/`, `v2/` folders inside `firmware/` or `host/`. Use tags to mark releases.
