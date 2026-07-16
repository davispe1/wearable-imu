"""
Protocol round-trip and robustness tests. Pure stdlib + pytest-free so they run
with plain `python test_protocol.py` (also discoverable by pytest).

Run:  py host/tests/test_protocol.py
"""

from __future__ import annotations

import os
import sys

# Make `import wearable_imu` work when run directly from anywhere.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from wearable_imu.ingest import protocol as P
from wearable_imu.model import ImuFrame, NodeSample, RangeFrame, RangePair


def _checks() -> None:
    yield_count = 0

    # ── CRC known-answer: CRC-16/CCITT-FALSE("123456789") == 0x29B1 ──────────
    assert P.crc16_ccitt(b"123456789") == 0x29B1, "CRC KAT failed"

    # ── RAW_9DOF round-trip ──────────────────────────────────────────────────
    frame = ImuFrame(frame_seq=7, fmt=P.FMT_RAW_9DOF, samples=[
        NodeSample(0, 100, 123456, accel=(-30, 0, 16700), gyro=(0, 1234, 0), mag=(10, -20, 30)),
        NodeSample(1, 101, 123456, accel=(40, -5, 16600), gyro=(0, -300, 0), mag=(0, 0, 0)),
    ])
    raw = P.encode_imu_frame(frame)
    parser = P.FrameParser()
    out = list(parser.feed(raw))
    assert len(out) == 1, f"expected 1 frame, got {len(out)}"
    got = out[0]
    assert isinstance(got, ImuFrame) and got.frame_seq == 7 and got.fmt == P.FMT_RAW_9DOF
    assert len(got.samples) == 2
    assert got.samples[0].accel == (-30, 0, 16700)
    assert got.samples[0].gyro == (0, 1234, 0)
    assert got.samples[1].node_id == 1 and got.samples[1].mag == (0, 0, 0)

    # ── SFLP_QUAT round-trip (Q15 quantization tolerance) ────────────────────
    qf = ImuFrame(frame_seq=9, fmt=P.FMT_SFLP_QUAT, samples=[
        NodeSample(3, 5, 9999, quat=(0.7071, 0.0, 0.7071, 0.0), mag=(1, 2, 3)),
    ])
    qout = list(P.FrameParser().feed(P.encode_imu_frame(qf)))
    assert len(qout) == 1
    qw, qx, qy, qz = qout[0].samples[0].quat
    assert abs(qw - 0.7071) < 1e-3 and abs(qy - 0.7071) < 1e-3

    # ── RANGE round-trip ─────────────────────────────────────────────────────
    rf = RangeFrame(frame_seq=2, timestamp_us=555, pairs=[
        RangePair(0, 1, 312), RangePair(1, 2, 287),
    ])
    rout = list(P.FrameParser().feed(P.encode_range_frame(rf)))
    assert len(rout) == 1 and isinstance(rout[0], RangeFrame)
    assert rout[0].pairs[0].dist_mm == 312 and rout[0].pairs[1].node_b == 2

    # ── Fragmentation: feed the frame one byte at a time ─────────────────────
    p2 = P.FrameParser()
    collected = []
    for b in raw:
        collected.extend(p2.feed(bytes([b])))
    assert len(collected) == 1, "byte-at-a-time reassembly failed"
    assert collected[0].samples[0].accel == (-30, 0, 16700)

    # ── Resync: leading garbage + a corrupted first frame, then a good one ───
    corrupt = bytearray(raw)
    corrupt[8] ^= 0xFF                      # flip a payload byte → CRC fail
    stream = b"\x00\x11\xAA garbage " + bytes(corrupt) + raw
    p3 = P.FrameParser()
    good = list(p3.feed(stream))
    assert len(good) == 1, f"resync should recover exactly the 1 good frame, got {len(good)}"
    assert p3.crc_errors >= 1, "expected a CRC error to be counted"

    # ── Two frames back to back ──────────────────────────────────────────────
    p4 = P.FrameParser()
    both = list(p4.feed(raw + P.encode_range_frame(rf)))
    assert len(both) == 2 and isinstance(both[1], RangeFrame)

    # ── Golden vector: locks the wire format so firmware can't silently drift ─
    # The same bytes are embedded in firmware/node/Comms/packet_selftest.c.
    golden = bytes.fromhex(
        "aa55010136000100000200020003000000640038ff0040010002000300000000"
        "000000010400050000000a0014001e00fffffefffdff070008000900d631"
    )
    gframe = ImuFrame(frame_seq=1, fmt=P.FMT_RAW_9DOF, samples=[
        NodeSample(0, 2, 3, accel=(100, -200, 16384), gyro=(1, 2, 3), mag=(0, 0, 0)),
        NodeSample(1, 4, 5, accel=(10, 20, 30), gyro=(-1, -2, -3), mag=(7, 8, 9)),
    ])
    assert P.encode_imu_frame(gframe) == golden, "wire format drifted from golden vector"

    # ── counts_to_si sanity: az≈+1g should be ≈ +9.81 m/s² ───────────────────
    s = NodeSample(0, 0, 0, accel=(0, 0, round(1.0 / P.ACCEL_LSB_G)), gyro=(0, 0, 0))
    (_, _, az), _ = P.counts_to_si(s)
    assert abs(az - 9.80665) < 0.05, f"az={az}"

    print("all protocol checks passed")


if __name__ == "__main__":
    _checks()
