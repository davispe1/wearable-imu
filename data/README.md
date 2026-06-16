# `data/` — where recordings go (datasets are NOT included)

This folder is intentionally (almost) empty in the repository. Real recordings and dataset
slices are **not** version-controlled and are **not** uploaded. Only two placeholders ship:

- `.gitkeep` — keeps the empty folder in version control.
- `schema_example.csv` — a tiny, fake example of the canonical 9-DOF input schema.

## Input schema (one combined long-format CSV)

```
node,t_s,ax,ay,az,gx,gy,gz,mx,my,mz
```

| column | meaning | units |
|---|---|---|
| `node` | sensor/segment id (e.g. `RF`,`RS`,`RT`,`SA`) | — |
| `t_s` | time, monotonic per node, 0 at start | s |
| `ax,ay,az` | linear acceleration, sensor frame | m/s² |
| `gx,gy,gz` | angular velocity, sensor frame | rad/s |
| `mx,my,mz` | magnetometer, sensor frame | any consistent units |

Per-node files (`RF.csv`, `RS.csv`, …) with a `t_s`/`t_opt_s` column and no `node` column
are also accepted — the node id is then taken from the filename.

## Folder layout for a recording

```
data/
  <recording_id>/
    RF.csv   RS.csv   RT.csv   SA.csv      # per-node files, OR
    data.csv                              # one combined long-format file
```

The app imports these into its own session store at `~/GaitApp/sessions/` (outside the
repo). See the top-level `README.md` for how mounting configs map nodes to body segments
and joints.
