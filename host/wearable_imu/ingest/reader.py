"""
reader.py — byte-stream sources that yield decoded protocol frames.

Each source exposes `iter_frames()` -> Iterator[ImuFrame | RangeFrame], feeding raw
bytes through a single FrameParser. The transport (file / serial / BLE / UWB dongle)
only differs in where the bytes come from; parsing + resync is shared.

    FileReader   — replay a captured binary stream (no hardware). Great for tests/dev.
    SerialReader — read a COM port: SWO-via-bridge, BLE serial, or UWB-USB dongle.
    make_source(config) — factory selecting the reader from config.TRANSPORT.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from .. import config
from ..model import ImuFrame, RangeFrame
from .protocol import FrameParser


class FileReader:
    """Replays a binary capture file through the parser. Finite — stops at EOF."""

    def __init__(self, path: str | Path, chunk_size: int = 4096) -> None:
        self.path = Path(path)
        self.chunk_size = chunk_size
        self.parser = FrameParser()

    def iter_frames(self) -> Iterator[ImuFrame | RangeFrame]:
        with self.path.open("rb") as f:
            while True:
                chunk = f.read(self.chunk_size)
                if not chunk:
                    return
                yield from self.parser.feed(chunk)


class SerialReader:
    """Reads a serial/COM port (pyserial) and yields frames. Runs until closed."""

    def __init__(self, port: str, baud: int, read_size: int = 256) -> None:
        self.port = port
        self.baud = baud
        self.read_size = read_size
        self.parser = FrameParser()
        self._ser = None

    def open(self) -> None:
        try:
            import serial  # pyserial — imported lazily so file/sim paths need no dep
        except ImportError as e:  # pragma: no cover
            raise RuntimeError(
                "pyserial is required for SerialReader. Install: pip install pyserial"
            ) from e
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)

    def iter_frames(self) -> Iterator[ImuFrame | RangeFrame]:
        if self._ser is None:
            self.open()
        assert self._ser is not None
        try:
            while True:
                chunk = self._ser.read(self.read_size)
                if chunk:
                    yield from self.parser.feed(chunk)
        finally:
            self._ser.close()

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


def make_source(cfg=config):
    """Return a frame source based on cfg.TRANSPORT ('swo' | 'ble' | 'uwb')."""
    transport = cfg.TRANSPORT.lower()
    if transport in ("swo", "ble", "uwb"):
        # All three currently arrive as a serial/COM stream on the host.
        return SerialReader(cfg.SERIAL_PORT, cfg.SERIAL_BAUD)
    raise ValueError(f"unknown transport {cfg.TRANSPORT!r}")


class HardwareSource:
    """Thin adapter kept for main.py; delegates to the configured serial reader."""

    def __init__(self, cfg=config) -> None:
        self._reader = make_source(cfg)

    def iter_frames(self) -> Iterator[ImuFrame | RangeFrame]:
        yield from self._reader.iter_frames()
