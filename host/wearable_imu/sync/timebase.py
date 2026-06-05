"""
timebase.py — multi-node timestamp alignment.

Nodes are sub-nanosecond aligned on-body via UWB sync before data leaves.
The master stamps the aggregated packet with a single host-received timestamp.
This module converts per-node UWB timestamps to a common host timebase.

TODO:
    - Implement TimebaseAligner: track per-node clock offset + drift.
    - Apply linear correction to convert node UWB timestamps → host time.
    - Handle node dropouts (gap detection, re-sync on reconnect).
"""

from __future__ import annotations


class TimebaseAligner:
    """Aligns per-node UWB timestamps to a common host timebase."""

    def __init__(self) -> None:
        # TODO: per-node offset/drift state
        pass

    def align(self, node_id: int, uwb_ts_us: int) -> float:
        """Return corrected host timestamp (seconds) for a node's UWB timestamp."""
        # TODO: apply clock correction
        return uwb_ts_us * 1e-6
