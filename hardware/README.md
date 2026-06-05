# hardware/

Physical design files. **This is the only place in the repo with version folders.**

Hardware revisions are discrete fabricated artifacts — you'll have v1.0 boards in hand
while designing v1.1, so both must coexist. Version folders (`v1.0/`, `v1.1/`, …) make
that explicit. Git tags tie a code release to the hardware revision that belongs with it.

## Structure

```
hardware/
├── electronics/
│   ├── v1.0/          ← schematic, layout, BOM, fab files for board rev 1.0
│   └── CHANGELOG.md
└── mechanical/
    ├── v1.0/          ← STEP / STL / CAD files for case rev 1.0
    └── CHANGELOG.md
```

## Adding a new hardware revision

1. Copy the previous version folder (`cp -r v1.0 v1.1`).
2. Make your changes inside `v1.1/`.
3. Add an entry to `CHANGELOG.md`.
4. When a firmware/host release is tied to this hardware rev, create a git tag
   `imu-vX.Y.Z` that documents which hardware folder it targets.
