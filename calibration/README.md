# calibration/

Calibration routines and saved calibration profiles.

## Saved profiles

Profiles are stored as JSON: `<subject>_<date>.json`

```json
{
  "subject": "subject_01",
  "date": "2026-01-01",
  "nodes": {
    "0": { "R_sensor_to_segment": [[...], [...], [...]], "accel_bias": [...], "gyro_bias": [...] },
    "1": { ... }
  },
  "mag_calibration": { "hard_iron": [...], "soft_iron": [[...], [...], [...]] }
}
```

## Procedure

See [docs/06-calibration.md](../docs/06-calibration.md) for the full T/N-pose procedure.
Run `tools/calibrate.py` to execute interactively.
