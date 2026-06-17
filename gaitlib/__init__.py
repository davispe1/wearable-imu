"""
gaitlib — Gait Kinematics library.

Converts raw 9-DOF inertial/magnetic sensor data into commonly measured gait parameters.
The library is **pure**: data in, parameters out. It has no hardware, serial, network, or
3D-visualisation dependencies — acquisition and visualisation are separate layers.

Quick start
-----------
>>> import gaitlib
>>> results = gaitlib.compute(raw_data, gaitlib.default_config())   # default: 4 sensors, 1 leg
>>> results.gait["cadence_steps_per_min"]
>>> results.joints["knee"]["params"]["rom_deg"]

``raw_data`` is per-sample 9-DOF data (timestamp, node_id, ax..az, gx..gz, mx..mz) in any
form accepted by :func:`gaitlib.rawdata.load_raw`. The mounting config declares which
sensors are present, each sensor's body segment, the joint topology, and the per-channel
sample rates — making the library agnostic to the number of sensors. Missing a sensor only
skips the joints that need it (or raises, in strict mode).

The pipeline reuses the project's validated kinematic core (orientation fusion, yaw-immune
sagittal joint angles, gait-event detection), exposed here as submodules:
``gaitlib.fusion``, ``gaitlib.calibration``, ``gaitlib.angles``, ``gaitlib.gait``,
``gaitlib.segment``.
"""
from __future__ import annotations

from .pipeline import compute
from .config import (MountingConfig, FusionParams, default_config, both_legs_config)
from .results import GaitResults
from . import fusion, calibration, angles, gait, segment, parameters, rawdata

__version__ = "1.0.0"

__all__ = [
    "compute",
    "MountingConfig", "FusionParams", "default_config", "both_legs_config",
    "GaitResults",
    "fusion", "calibration", "angles", "gait", "segment", "parameters", "rawdata",
    "__version__",
]
