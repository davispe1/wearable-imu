"""
reader.py — hardware data source (SWO / BLE / UWB).

Reads framed sample packets from the configured transport, decodes them,
and emits NodeSample objects onto a queue for the sync stage.

TODO:
    - Implement SwoReader: open serial port (SWV/SWO), read framed bytes.
    - Implement BleReader: connect to BLE serial service, read characteristic notifications.
    - Implement UwbReader: read from UWB-USB dongle COM port.
    - Implement frame parser: sync byte → packet type → node_id, timestamp, payload.
    - Factory: HardwareSource selects reader based on config.TRANSPORT.
"""

from __future__ import annotations
from .. import config


class HardwareSource:
    """Thin wrapper that selects the right transport reader from config."""

    def start(self) -> None:
        # TODO: instantiate reader, start background thread
        raise NotImplementedError(
            f"Hardware ingest not yet implemented for transport={config.TRANSPORT!r}. "
            "Run with --sim for synthetic data."
        )
