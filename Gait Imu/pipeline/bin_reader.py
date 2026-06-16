"""
bin_reader.py — Reader for GaitUp / Physilog 6S ``*_Inertial_sensor.BIN`` files.

The official reader is MATLAB (Gait Up Research ToolKit). No public Python reader
exists, so this layout was reverse-engineered directly from the Geneva dataset
(see docs/pipeline_guide.md and the memory note `geneva-bin-format`).

LAYOUT (verified on P01 RF/RS, identical structure)
----------------------------------------------------
* The file is a sequence of **512-byte pages**.
  - Page 0 (bytes 0x000..0x1FF) is the **config header**: ``50 35`` ("P5")-framed
    packets terminated by ``ff fe`` (datetime, ranges, scales). We skip it for the
    signal stream (a light datetime parse is attempted for reporting only).
  - Every later page begins with an **8-byte page header**
    ``uint32_BE page_index`` + ``uint32_BE cumulative_sample_counter`` and is
    followed by exactly **63 fixed 8-byte records** (8 + 63*8 = 512).
* **Record = tag(1) + counter(1) + 3 x int16 BIG-ENDIAN (6 bytes)**.
  Channel tags (one device clock — all share the page sample counter):
    0x13  accelerometer   256 Hz   +/-16 g     -> 2048 counts / g
    0x14  gyroscope       256 Hz   +/-2000 dps -> 16.384 counts / (deg/s)
    0x18  MAGNETOMETER    256 Hz   (3-axis; |B| ~1093 counts, 5%% stable over the
                                    whole session, uncorrelated with gravity)
    0x15  barometer/housekeeping  64 Hz  (packed non-int16x3 layout; NOT used for fusion)
  IMPORTANT: the magnetometer is delivered at 256 Hz on the SAME interleaved record
  stream as accel/gyro (verified by physics: stable vector magnitude while direction
  rotates). It is therefore time-aligned to accel/gyro *by construction* and needs NO
  64->256 upsampling. (The dataset documentation lists the mag as "64 Hz"; here it
  arrives already at 256 Hz — its useful bandwidth may still be ~64 Hz, but each accel/
  gyro sample has a co-timed mag sample.) The 64 Hz channel 0x15 is the barometer.

This module returns **raw int16 counts** plus metadata. SI conversion, magnetometer
calibration and 64->256 Hz upsampling are done downstream in ``extract.py`` so that
the contract boundary (raw IMU only) stays explicit.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

# --- Format constants -------------------------------------------------------
PAGE_SIZE = 512
PAGE_HEADER = 8
RECORD = 8
RECORDS_PER_PAGE = (PAGE_SIZE - PAGE_HEADER) // RECORD  # 63

TAG_ACC = 0x13
TAG_GYR = 0x14
TAG_MAG = 0x18  # magnetometer, 256 Hz, sample-aligned with accel/gyro
TAG_BARO = 0x15  # 64 Hz barometer/housekeeping (packed; not used for fusion)
KNOWN_TAGS = (TAG_ACC, TAG_GYR, TAG_MAG, TAG_BARO)

FS_HIGH = 256.0  # accel, gyro AND magnetometer sampling rate (Hz)
FS_BARO = 64.0   # barometer/housekeeping channel rate (Hz)


@dataclass
class BinData:
    """BIN-native sensor streams (raw int16 counts) on the device clock."""
    acc_raw: np.ndarray          # (N, 3) int16, 256 Hz
    gyr_raw: np.ndarray          # (N, 3) int16, 256 Hz
    mag_raw: np.ndarray          # (N, 3) int16, 256 Hz (tag 0x18, sample-aligned)
    baro_raw: np.ndarray         # (Nb, 3) int16, 64 Hz (tag 0x15; not used for fusion)
    fs_high: float = FS_HIGH
    fs_baro: float = FS_BARO
    # acc/gyro/mag all share the 256 Hz device clock: sample i -> i / fs_high.
    n_pages: int = 0
    invalid_records: int = 0
    total_records: int = 0
    start_datetime: str | None = None
    meta: dict = field(default_factory=dict)

    @property
    def t_high(self) -> np.ndarray:
        return np.arange(len(self.acc_raw)) / self.fs_high

    @property
    def duration_s(self) -> float:
        return len(self.acc_raw) / self.fs_high


def _parse_header_datetime(page0: bytes) -> str | None:
    """Best-effort: pull the ``03 00`` datetime packet from the P5 header.

    Packet form observed: ``50 35 03 00 07 <ss mm hh DD MM YYYY_BE> ff fe`` with
    year stored big-endian (e.g. 0x07E5 = 2021). Returns an ISO-ish string or None.
    """
    i = page0.find(b"\x50\x35\x03\x00")
    if i < 0 or i + 12 > len(page0):
        return None
    L = page0[i + 4]
    p = page0[i + 5 : i + 5 + L]
    if len(p) < 7:
        return None
    ss, mn, hh, dd, mo = p[0], p[1], p[2], p[3], p[4]
    yr = (p[5] << 8) | p[6]
    try:
        return f"{yr:04d}-{mo:02d}-{dd:02d} {hh:02d}:{mn:02d}:{ss:02d}"
    except Exception:
        return None


def read_bin(path: str) -> BinData:
    """Decode a Physilog ``*_Inertial_sensor.BIN`` into raw int16 channel streams.

    Page-aware and fully vectorised. Records whose tag is not one of the four
    known tags are counted (``invalid_records``) and dropped — they should be a
    tiny fraction (page-boundary/anomaly artifacts).
    """
    raw = np.fromfile(path, dtype=np.uint8)
    n_full_pages = raw.size // PAGE_SIZE
    if n_full_pages < 2:
        raise ValueError(f"{path}: too small to contain data pages")

    pages = raw[: n_full_pages * PAGE_SIZE].reshape(n_full_pages, PAGE_SIZE)
    start_dt = _parse_header_datetime(bytes(pages[0].tobytes()))

    # Drop page 0 (config header). Each remaining page: skip 8-byte page header,
    # then 63 contiguous 8-byte records.
    body = pages[1:, PAGE_HEADER:]                      # (P, 504) uint8
    recs = np.ascontiguousarray(body).reshape(-1, RECORD)  # (R, 8) uint8
    total_records = recs.shape[0]

    tags = recs[:, 0]
    # 3 x int16 BIG-ENDIAN from payload bytes [2:8]
    payload = np.ascontiguousarray(recs[:, 2:8])        # (R, 6) uint8
    vals = payload.view(">i2").reshape(-1, 3)           # (R, 3) int16

    valid = np.isin(tags, KNOWN_TAGS)
    invalid = int((~valid).sum())

    acc = vals[tags == TAG_ACC]
    gyr = vals[tags == TAG_GYR]
    mag = vals[tags == TAG_MAG]
    baro = vals[tags == TAG_BARO]

    bd = BinData(
        acc_raw=acc.astype(np.int16),
        gyr_raw=gyr.astype(np.int16),
        mag_raw=mag.astype(np.int16),
        baro_raw=baro.astype(np.int16),
        n_pages=n_full_pages - 1,
        invalid_records=invalid,
        total_records=total_records,
        start_datetime=start_dt,
    )
    # Sanity: acc, gyro, mag are all 256 Hz -> counts should match closely.
    bd.meta["acc_mag_ratio"] = (len(acc) / len(mag)) if len(mag) else float("nan")
    bd.meta["acc_baro_ratio"] = (len(acc) / len(baro)) if len(baro) else float("nan")
    return bd


# --- Default scale constants (counts -> SI), used by extract.py -------------
# Accelerometer: +/-16 g over int16 -> 32768/16 = 2048 counts/g.
G = 9.80665
ACC_COUNTS_PER_G = 2048.0
ACC_TO_MS2 = G / ACC_COUNTS_PER_G            # m/s^2 per count
# Gyroscope: +/-2000 deg/s over int16 -> 32768/2000 = 16.384 counts/(deg/s).
GYR_COUNTS_PER_DPS = 16.384
DEG2RAD = np.pi / 180.0
GYR_TO_RADS = (1.0 / GYR_COUNTS_PER_DPS) * DEG2RAD  # rad/s per count
# Magnetometer: scale assumed/validated in extract.py (see docs). Provisional:
# +/-50 mT range maps to int16; units carried as Gauss. Calibration (hard/soft
# iron) removes scale/bias, so absolute scale only sets units, not fusion result.
MAG_RANGE_GAUSS = 500.0                      # +/-50 mT = +/-500 mGauss? documented assumption
MAG_TO_GAUSS = MAG_RANGE_GAUSS / 32768.0     # provisional; refined/validated downstream


if __name__ == "__main__":
    import sys
    bd = read_bin(sys.argv[1])
    print(f"file: {sys.argv[1]}")
    print(f"start_datetime: {bd.start_datetime}")
    print(f"pages: {bd.n_pages}  records: {bd.total_records}  invalid: {bd.invalid_records} "
          f"({100*bd.invalid_records/bd.total_records:.3f}%)")
    print(f"acc: {bd.acc_raw.shape} @256Hz   gyr: {bd.gyr_raw.shape}   "
          f"mag: {bd.mag_raw.shape} @256Hz   baro: {bd.baro_raw.shape} @64Hz")
    print(f"acc:mag ratio = {bd.meta['acc_mag_ratio']:.4f} (expect ~1.0; both 256 Hz)")
    magn = np.linalg.norm(bd.mag_raw.astype(float), axis=1)
    print(f"mag |B| = {magn.mean():.0f} +/- {magn.std():.0f} counts "
          f"({100*magn.std()/magn.mean():.1f}% — field magnitude, should be roughly stable)")
    print(f"duration: {bd.duration_s:.1f} s ({bd.duration_s/60:.2f} min)")
    # quick physical sanity: accel magnitude near 1 g at rest-ish (median over file)
    acc_ms2 = bd.acc_raw.astype(np.float64) * ACC_TO_MS2
    mag_g = np.linalg.norm(acc_ms2, axis=1)
    print(f"accel |a| median = {np.median(mag_g):.2f} m/s^2 (expect ~9.81 when not moving)")
    print(f"gyro raw median = {np.median(bd.gyr_raw, axis=0)} counts (expect ~0 at rest)")
