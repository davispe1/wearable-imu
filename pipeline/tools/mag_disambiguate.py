"""Disambiguate the magnetometer channel: world-frame / dip-constancy test (READ-ONLY).

A TRUE magnetometer measures Earth's field: in quasi-static poses (accel ~ gravity,
gyro ~ 0) the angle between the field vector and gravity is CONSTANT across all
orientations (= 90 - inclination ~ 27 deg at Geneva, i.e. ~153 deg to specific-force-up).
A sensor-fixed non-field vector would show a VARYING angle as the body rotates.
"""
import sys, numpy as np

root = r'.\Human gait and other movements - markers inertial sensors pressure insoles force plates\researchdata'
sensor = sys.argv[1] if len(sys.argv) > 1 else 'RT'
raw = np.fromfile(root + rf'\P01_S01\RAW_DATA\P01_S01_{sensor}_Inertial_sensor.BIN', dtype=np.uint8)
PS, HD = 512, 8
npg = raw.size // PS
body = np.ascontiguousarray(raw[:npg*PS].reshape(npg, PS)[1:, HD:]).reshape(-1, 8)
tags = body[:, 0]

def chan(tag):
    r = body[tags == tag]
    return np.ascontiguousarray(r[:, 2:8]).view('>i2').reshape(-1, 3).astype(float)

acc = chan(0x13) * (9.80665/2048.0)      # m/s^2
gyr = chan(0x14) * (np.pi/180/16.384)    # rad/s
m18 = chan(0x18)

n = min(len(acc), len(gyr), len(m18))
acc, gyr, m18 = acc[:n], gyr[:n], m18[:n]

# quasi-static mask: |accel| near g and gyro small
amag = np.linalg.norm(acc, axis=1)
gmag = np.linalg.norm(gyr, axis=1)
qs = (np.abs(amag - 9.80665) < 0.4) & (gmag < np.radians(5))
print(f"sensor {sensor}: {qs.sum()} quasi-static samples of {n}")

def dip_test(vec, name):
    a = acc[qs]; v = vec[qs]
    au = a / np.linalg.norm(a, axis=1, keepdims=True)
    vu = v / (np.linalg.norm(v, axis=1, keepdims=True) + 1e-9)
    ang = np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', au, vu), -1, 1)))
    # spread of gravity directions to confirm we span orientations
    grav_spread = np.degrees(np.arccos(np.clip(au @ au.mean(0)/np.linalg.norm(au.mean(0)), -1, 1)))
    vm = np.linalg.norm(v, axis=1)
    print(f"\n[{name}] angle-to-gravity: mean={ang.mean():.1f} std={ang.std():.1f} deg "
          f"(TRUE mag: tight std<~5, mean~27 or ~153)")
    print(f"    |vector|: mean={vm.mean():.0f} std={vm.std():.0f} ({100*vm.std()/vm.mean():.1f}%)")
    print(f"    orientation coverage (gravity dir spread): up to {grav_spread.max():.0f} deg")
    # bin angle-to-gravity vs gravity-orientation to see if angle is constant across poses
    return ang

dip_test(m18, "0x18 (candidate mag)")

# world-frame constancy via 6-DOF over a rotating window (uses gaitlib fusion)
sys.path[:0] = ['.', 'pipeline']   # run from repo root: gaitlib (.) + pipeline siblings
from gaitlib import fusion as F
# pick a 30 s window with lots of rotation but quasi-static-ish (slow): high gyro variance
w = 30*256
best_i, best_var = 0, 0
for i in range(0, n-w, w):
    v = gyr[i:i+w]
    var = np.linalg.norm(v, axis=1).mean()
    # prefer moderate rotation (not walking-violent): 0.2-1.0 rad/s
    if 0.2 < var < 1.0 and var > best_var:
        best_var, best_i = var, i
i = best_i
print(f"\nWorld-frame test on window t=[{i/256:.0f},{(i+w)/256:.0f}]s (mean|gyro|={best_var:.2f} rad/s)")
q = F.run_madgwick(gyr[i:i+w], acc[i:i+w], None, 256, beta=0.1)
Rs = np.array([F.rot_matrix(qq) for qq in q])
mw = np.einsum('nij,nj->ni', Rs, m18[i:i+w])   # mag in world frame (6-DOF, yaw arbitrary)
mwz = mw[:,2]/np.linalg.norm(mw,axis=1)         # vertical component (yaw-invariant)
dip = np.degrees(np.arcsin(np.clip(mwz,-1,1)))
print(f"  world-frame VERTICAL inclination (yaw-invariant): mean={dip.mean():.1f} std={dip.std():.1f} deg")
print(f"  (TRUE mag at Geneva: ~+/-63 deg, tight std; azimuth will wander due to 6-DOF yaw)")
