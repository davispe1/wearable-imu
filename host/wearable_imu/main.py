"""
main.py — host pipeline entry point.

Wires together: ingest → sync → orientation → kinematics → viz.
Run with:  python -m wearable_imu.main
           python -m wearable_imu.main --sim      (synthetic data, no hardware)

TODO:
    - Add argparse: --sim, --transport, --port, --filter, --nodes
    - Start ingest thread (sim or hardware reader).
    - Start orientation + kinematics processing loop.
    - Start viz window.
    - Graceful shutdown on Ctrl-C.
"""

from __future__ import annotations

import sys
from . import config


def main() -> None:
    sim_mode = "--sim" in sys.argv

    print(f"Wearable IMU host — {'SIM' if sim_mode else config.TRANSPORT.upper()} mode")
    print(f"  nodes={config.NODE_COUNT}  rate={config.SAMPLE_RATE_HZ} Hz"
          f"  format={config.DATA_FORMAT}  filter={config.ORIENTATION_FILTER}")

    if sim_mode:
        from .sim.generator import SyntheticSource
        source = SyntheticSource()
    else:
        from .ingest.reader import HardwareSource
        source = HardwareSource()

    # TODO: build pipeline and start viz
    print("Pipeline stub — implement ingest → sync → orientation → kinematics → viz")
    source.start()


if __name__ == "__main__":
    main()
