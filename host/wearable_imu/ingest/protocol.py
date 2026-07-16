"""
protocol.py — wire-protocol codec (encoder + streaming decoder).

Implements the spec in docs/03-communication.md. This is the host-side twin of
firmware/node/Comms/packet.c — the two MUST stay byte-for-byte compatible.

Stdlib only (struct), so the whole ingest path is testable without numpy/hardware.

Public API:
    crc16_ccitt(data)            -> int
    encode_imu_frame(frame)      -> bytes
    encode_range_frame(frame)    -> bytes
    FrameParser().feed(chunk)    -> iterator of ImuFrame | RangeFrame
    counts_to_si(sample)         -> (accel_mps2, gyro_rads) helper for downstream
"""

from __future__ import annotations

import math
import struct
from collections.abc import Iterator

from ..model import ImuFrame, NodeSample, RangeFrame, RangePair

# ── Protocol constants (mirror packet.h) ─────────────────────────────────────
SYNC0 = 0xAA
SYNC1 = 0x55
PROTO_VERSION = 0x01

MSG_IMU = 0x01
MSG_RANGE = 0x02
MSG_STATUS = 0x10

FMT_RAW_9DOF = 0
FMT_SFLP_QUAT = 1

HEADER_LEN = 6          # SYNC0 SYNC1 VER TYPE LEN(2)
CRC_LEN = 2
RAW_RECORD_LEN = 25
SFLP_RECORD_LEN = 21
RANGE_RECORD_LEN = 4

# Safety bound so a corrupt length can't make us buffer forever.
MAX_PAYLOAD_LEN = 4096

# ── Scale factors (counts → physical). MUST match firmware FS config. ────────
# See docs/03-communication.md §4. v1: accel ±8 g, gyro ±2000 dps.
_G = 9.80665
ACCEL_LSB_G = 0.244e-3            # g per LSB at ±8 g
GYRO_LSB_DPS = 0.070             # dps per LSB at ±2000 dps
ACCEL_COUNT_TO_MPS2 = ACCEL_LSB_G * _G
GYRO_COUNT_TO_RADS = GYRO_LSB_DPS * (math.pi / 180.0)
Q15 = 32767.0


# ── CRC ──────────────────────────────────────────────────────────────────────
def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def _wrap(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in the frame envelope (header + CRC)."""
    body = struct.pack("<BBH", PROTO_VERSION, msg_type, len(payload)) + payload
    crc = crc16_ccitt(body)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc)


# ── Encoders (used by the sim and by tests; on the wire the firmware does this) ─
def encode_imu_frame(frame: ImuFrame) -> bytes:
    payload = struct.pack("<HBB", frame.frame_seq & 0xFFFF, frame.fmt, len(frame.samples))
    for s in frame.samples:
        if frame.fmt == FMT_RAW_9DOF:
            ax, ay, az = s.accel or (0, 0, 0)
            gx, gy, gz = s.gyro or (0, 0, 0)
            mx, my, mz = s.mag
            payload += struct.pack(
                "<BHI9h", s.node_id, s.node_seq & 0xFFFF, s.timestamp_us & 0xFFFFFFFF,
                ax, ay, az, gx, gy, gz, mx, my, mz,
            )
        elif frame.fmt == FMT_SFLP_QUAT:
            qw, qx, qy, qz = s.quat or (1.0, 0.0, 0.0, 0.0)
            mx, my, mz = s.mag
            payload += struct.pack(
                "<BHI4h3h", s.node_id, s.node_seq & 0xFFFF, s.timestamp_us & 0xFFFFFFFF,
                _q15(qw), _q15(qx), _q15(qy), _q15(qz), mx, my, mz,
            )
        else:
            raise ValueError(f"unknown IMU format {frame.fmt}")
    return _wrap(MSG_IMU, payload)


def encode_range_frame(frame: RangeFrame) -> bytes:
    payload = struct.pack(
        "<HIB", frame.frame_seq & 0xFFFF, frame.timestamp_us & 0xFFFFFFFF, len(frame.pairs)
    )
    for p in frame.pairs:
        payload += struct.pack("<BBH", p.node_a, p.node_b, p.dist_mm & 0xFFFF)
    return _wrap(MSG_RANGE, payload)


def _q15(x: float) -> int:
    return max(-32768, min(32767, round(x * Q15)))


# ── Streaming decoder ─────────────────────────────────────────────────────────
class FrameParser:
    """Feed it arbitrary byte chunks; it yields fully-validated decoded frames.

    Handles partial reads, mid-stream join, and corruption (resyncs on the next
    AA 55 after a bad CRC or unknown version). Stateful — one instance per stream.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        # Diagnostics — useful during bring-up to see link quality.
        self.crc_errors = 0
        self.resyncs = 0
        self.version_errors = 0

    def feed(self, chunk: bytes) -> Iterator[ImuFrame | RangeFrame]:
        self._buf += chunk
        yield from self._drain()

    def _drain(self) -> Iterator[ImuFrame | RangeFrame]:
        buf = self._buf
        while True:
            # Find frame start (AA 55).
            start = buf.find(b"\xAA\x55")
            if start < 0:
                # Keep only a trailing byte that might be a split AA.
                del buf[: max(0, len(buf) - 1)]
                return
            if start > 0:
                del buf[:start]  # drop garbage before sync

            if len(buf) < HEADER_LEN:
                return  # header not all here yet

            version = buf[2]
            msg_type = buf[3]
            payload_len = buf[4] | (buf[5] << 8)

            if version != PROTO_VERSION or payload_len > MAX_PAYLOAD_LEN:
                # Bogus header — skip this sync and look for the next.
                self.version_errors += version != PROTO_VERSION
                self.resyncs += 1
                del buf[:2]
                continue

            frame_len = HEADER_LEN + payload_len + CRC_LEN
            if len(buf) < frame_len:
                return  # whole frame not here yet

            body = bytes(buf[2 : HEADER_LEN + payload_len])           # VER..payload
            crc_rx = buf[HEADER_LEN + payload_len] | (buf[HEADER_LEN + payload_len + 1] << 8)
            if crc16_ccitt(body) != crc_rx:
                self.crc_errors += 1
                self.resyncs += 1
                del buf[:2]  # false sync; resync past it
                continue

            payload = bytes(buf[HEADER_LEN : HEADER_LEN + payload_len])
            del buf[:frame_len]

            frame = self._decode(msg_type, payload)
            if frame is not None:
                yield frame
            # else: known-envelope but unhandled type — skip silently.

    def _decode(self, msg_type: int, payload: bytes) -> ImuFrame | RangeFrame | None:
        if msg_type == MSG_IMU:
            return _decode_imu(payload)
        if msg_type == MSG_RANGE:
            return _decode_range(payload)
        return None  # MSG_STATUS / unknown — ignore for now


def _decode_imu(payload: bytes) -> ImuFrame:
    frame_seq, fmt, count = struct.unpack_from("<HBB", payload, 0)
    off = 4
    samples: list[NodeSample] = []
    for _ in range(count):
        if fmt == FMT_RAW_9DOF:
            (node_id, node_seq, ts, ax, ay, az, gx, gy, gz, mx, my, mz) = struct.unpack_from(
                "<BHI9h", payload, off
            )
            off += RAW_RECORD_LEN
            samples.append(NodeSample(
                node_id=node_id, node_seq=node_seq, timestamp_us=ts,
                accel=(ax, ay, az), gyro=(gx, gy, gz), mag=(mx, my, mz),
            ))
        elif fmt == FMT_SFLP_QUAT:
            (node_id, node_seq, ts, qw, qx, qy, qz, mx, my, mz) = struct.unpack_from(
                "<BHI4h3h", payload, off
            )
            off += SFLP_RECORD_LEN
            samples.append(NodeSample(
                node_id=node_id, node_seq=node_seq, timestamp_us=ts,
                quat=(qw / Q15, qx / Q15, qy / Q15, qz / Q15), mag=(mx, my, mz),
            ))
        else:
            raise ValueError(f"unknown IMU format {fmt}")
    return ImuFrame(frame_seq=frame_seq, fmt=fmt, samples=samples)


def _decode_range(payload: bytes) -> RangeFrame:
    frame_seq, ts, count = struct.unpack_from("<HIB", payload, 0)
    off = 7
    pairs: list[RangePair] = []
    for _ in range(count):
        a, b, dist = struct.unpack_from("<BBH", payload, off)
        off += RANGE_RECORD_LEN
        pairs.append(RangePair(node_a=a, node_b=b, dist_mm=dist))
    return RangeFrame(frame_seq=frame_seq, timestamp_us=ts, pairs=pairs)


# ── Convenience: counts → SI, for downstream stages ──────────────────────────
def counts_to_si(s: NodeSample) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (accel_mps2, gyro_rads) from a RAW_9DOF sample's int16 counts."""
    if s.accel is None or s.gyro is None:
        raise ValueError("counts_to_si requires a RAW_9DOF sample")
    ax, ay, az = s.accel
    gx, gy, gz = s.gyro
    return (
        (ax * ACCEL_COUNT_TO_MPS2, ay * ACCEL_COUNT_TO_MPS2, az * ACCEL_COUNT_TO_MPS2),
        (gx * GYRO_COUNT_TO_RADS, gy * GYRO_COUNT_TO_RADS, gz * GYRO_COUNT_TO_RADS),
    )
