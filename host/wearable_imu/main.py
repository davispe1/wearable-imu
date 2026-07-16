"""
main.py — host pipeline entry point.

Phase 1 wiring: ingest → (sync → orientation → kinematics → viz are stubs).
For now it ingests frames from the sim or hardware and prints a live summary, so
the data path is verifiable end-to-end before the processing stages exist.

    python -m wearable_imu.main --sim            # synthetic data, no hardware
    python -m wearable_imu.main --sim --capture out.bin --seconds 5
    python -m wearable_imu.main --replay out.bin # replay a capture file
    python -m wearable_imu.main                  # live: config.TRANSPORT serial
"""

from __future__ import annotations

import argparse

from . import config
from .model import ImuFrame, RangeFrame


def _summarize(frames, *, every: int = 50) -> None:
    imu = rng = 0
    for frame in frames:
        if isinstance(frame, ImuFrame):
            imu += 1
            if imu % every == 0:
                s = frame.samples[0]
                lead = f"node{s.node_id} seq{s.node_seq}"
                detail = f"accel={s.accel}" if s.accel else f"quat={s.quat}"
                print(f"[imu] frames={imu:6d} nodes={len(frame.samples)} {lead} {detail}")
        elif isinstance(frame, RangeFrame):
            rng += 1
    print(f"done - imu_frames={imu} range_frames={rng}")


def main() -> None:
    ap = argparse.ArgumentParser(prog="wearable_imu")
    ap.add_argument("--sim", action="store_true", help="use the synthetic source")
    ap.add_argument("--replay", metavar="FILE", help="replay a captured .bin stream")
    ap.add_argument("--capture", metavar="FILE", help="(with --sim) write a capture and exit")
    ap.add_argument("--seconds", type=float, default=5.0, help="capture/sim duration")
    args = ap.parse_args()

    print(f"Wearable IMU host - nodes={config.NODE_COUNT} rate={config.SAMPLE_RATE_HZ}Hz "
          f"format={config.DATA_FORMAT} filter={config.ORIENTATION_FILTER}")

    if args.sim and args.capture:
        from .sim.generator import SyntheticSource
        n = SyntheticSource().write_capture(args.capture, args.seconds)
        print(f"wrote {n} bytes -> {args.capture}")
        return

    if args.sim:
        from .sim.generator import SyntheticSource
        src = SyntheticSource()
        n = int(args.seconds * config.SAMPLE_RATE_HZ)
        # Round-trip through the real wire codec so --sim exercises the parser too.
        from .ingest.protocol import FrameParser
        parser = FrameParser()

        def frames():
            for raw in src.iter_bytes(max_frames=n):
                yield from parser.feed(raw)

        _summarize(frames())
        return

    if args.replay:
        from .ingest.reader import FileReader
        _summarize(FileReader(args.replay).iter_frames())
        return

    # Live hardware.
    from .ingest.reader import HardwareSource
    try:
        _summarize(HardwareSource().iter_frames())
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
