# 06 — Calibration

> Sensor-to-segment alignment and saved calibration profiles.

## T-pose / N-pose procedure

TODO: define exact pose, duration, acceptance criteria.

## Outputs

- Per-node rotation matrix aligning sensor frame to segment frame.
- Saved to `calibration/<session_id>.json`.

## Magnetometer calibration

- Ellipsoid fitting for hard/soft iron correction.
- Required only if magnetometer is used (open item).

## Accelerometer / gyroscope

- Static bias estimation during the calibration pose.
- Gyro bias re-estimated at every power-on (device still for first N samples).
