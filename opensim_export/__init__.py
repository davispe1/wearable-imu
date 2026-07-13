"""
opensim_export — write OpenSense orientation .sto files from fused segment orientations.

No OpenSim dependency: this package only produces the ``.sto`` text files that the OpenSim
OpenSense GUI/tools consume separately.

Modules
-------
to_sto              write *_orientations.sto + *_calibration.sto  (CLI: python -m opensim_export.to_sto)
make_calibration    build a custom calibration pose from a static window or auto-neutral
                    (CLI: python -m opensim_export.make_calibration)
segment_map         node/segment -> OpenSim IMU column name
"""
from .segment_map import imu_column, ordered_columns, side_char, OPENSIM_STEM, KIND_ORDER
from .to_sto import export_session

__all__ = ["imu_column", "ordered_columns", "side_char", "OPENSIM_STEM", "KIND_ORDER",
           "export_session"]
