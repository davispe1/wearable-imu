"""World-frame magnetometer test: is 0x18 a real (world-constant) field, or onboard-derived?

A raw magnetometer measures Earth's field: at a FIXED location, when the sensor rotates,
the field vector is CONSTANT in the world frame (steady direction; inclination ~63 deg at
Geneva). We rotate the 0x18 sensor-frame vector into the world frame via the 6-DOF
orientation and check (a) the yaw-invariant world inclination (dip) and (b) its constancy
during a rotation. Done on Static_01 (clean static hold) and a slow CalibrationTask
rotation. READ-ONLY.
"""
import os, sys, numpy as np
sys.path.insert(0, ".")
import yaml, align
from bin_reader import read_bin
from kincore import fusion as F

cfg = yaml.safe_load(open("config/default.yaml"))
root = cfg["dataset"]["root"]; subj, sess = "P01", "S01"
node = sys.argv[1] if len(sys.argv) > 1 else "RT"
sensor = cfg["sensor_map"][node]
bd = read_bin(os.path.join(root, f"{subj}_{sess}", "RAW_DATA",
                           f"{subj}_{sess}_{sensor}_Inertial_sensor.BIN"))
fs = 256.0
acc = bd.acc_raw.astype(float) * (9.80665/2048.0)
gyr = bd.gyr_raw.astype(float) * (np.pi/180/16.384)
mag = bd.mag_raw.astype(float)
rtc = align.bin_rtc_datetime(bd.start_datetime)
gmag = np.linalg.norm(gyr, axis=1)
skew = align.estimate_session_skew(gmag, fs, rtc, root, subj, sess, node).skew_s


def window(task, trial, dur_guess=None):
    cap = align.c3d_capture_datetime(align.c3d_path(root, subj, sess, task, trial))
    i0 = int(round(((cap - rtc).total_seconds() + skew) * fs))
    return max(0, i0)


def sensor_frame_dip(a, m):
    """Yaw-invariant inclination: angle between mag and gravity (accel), minus 90 deg."""
    au = a / np.linalg.norm(a, axis=1, keepdims=True)
    mu = m / np.linalg.norm(m, axis=1, keepdims=True)
    ang = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", au, mu), -1, 1)))
    return 90.0 - ang     # inclination below horizontal (accel points up at rest)


# --- Static_01: clean static hold ---
i0 = window("Static", "01")
sa = acc[i0:i0+int(0.4*fs)]; sm = mag[i0:i0+int(0.4*fs)]
dip_static = sensor_frame_dip(sa, sm)
Bs = np.linalg.norm(sm, axis=1)
print(f"[{node}] STATIC_01 (clean hold): |B|={Bs.mean():.0f}+/-{Bs.std():.0f} counts "
      f"({100*Bs.std()/Bs.mean():.1f}%)")
print(f"   inclination(dip) = {dip_static.mean():+.1f} +/- {dip_static.std():.1f} deg "
      f"(raw mag at Geneva ~ +/-63 deg)")

# --- slow CalibrationTask rotation: world-frame constancy ---
print("\nCalibrationTask rotations — world-frame inclination via 6-DOF (yaw-invariant):")
for tr in ["01", "02", "03", "05"]:
    try:
        i0 = window("CalibrationTask", tr)
    except Exception:
        continue
    n = int(20 * fs)
    a = acc[i0:i0+n]; g = gyr[i0:i0+n]; m = mag[i0:i0+n]
    if len(a) < n//2:
        continue
    # restrict to moderate, non-violent rotation for clean 6-DOF gravity tracking
    q = F.run_madgwick(g, a, None, fs, beta=0.08)
    Rs = np.array([F.rot_matrix(qq) for qq in q])     # sensor->earth
    mw = np.einsum("nij,nj->ni", Rs, m)
    mwz = mw[:, 2] / np.linalg.norm(mw, axis=1)
    dip_w = np.degrees(np.arcsin(np.clip(mwz, -1, 1)))
    gspan = np.degrees(np.arccos(np.clip(
        (a/np.linalg.norm(a,axis=1,keepdims=True)) @ (a.mean(0)/np.linalg.norm(a.mean(0))), -1, 1))).max()
    sfd = sensor_frame_dip(a, m)
    print(f"  CalT_{tr}: world dip = {dip_w.mean():+.1f} +/- {dip_w.std():.1f} deg ; "
          f"sensor-frame dip {sfd.mean():+.1f}+/-{sfd.std():.1f} ; orient. span {gspan:.0f} deg "
          f"(constant dip + nonzero span => world-constant field)")
