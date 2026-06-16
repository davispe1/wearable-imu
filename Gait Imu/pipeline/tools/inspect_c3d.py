"""Phase-A throwaway inspector for a representative .c3d (READ-ONLY)."""
import sys
import ezc3d

path = sys.argv[1]
c = ezc3d.c3d(path)

params = c["parameters"]
point = params["POINT"]
print(f"FILE: {path}")
print(f"POINT rate: {point['RATE']['value']}")
print(f"POINT frames: {point['FRAMES']['value']}")
labels = point["LABELS"]["value"]
print(f"N markers: {len(labels)}")
dur = float(point['FRAMES']['value'][0]) / float(point['RATE']['value'][0])
print(f"Duration (point): {dur:.2f} s")

print("\n=== Marker labels ===")
print(", ".join(labels))

# Right-leg clusters & joint centers
rl = [l for l in labels if l.startswith(("RT", "RS", "RF", "RHJC", "RKJC", "RAJC", "SA", "RASI", "RPSI"))]
print("\n=== Right-leg / pelvis related labels ===")
print(", ".join(rl))

# Analog / force plate
if "ANALOG" in params:
    an = params["ANALOG"]
    print(f"\nANALOG rate: {an['RATE']['value']}")
    alabels = an["LABELS"]["value"]
    print(f"N analog channels: {len(alabels)}")
    print("Analog labels:", ", ".join(alabels))

# Force platform
if "FORCE_PLATFORM" in params:
    fp = params["FORCE_PLATFORM"]
    used = fp.get("USED", {}).get("value", "?")
    print(f"\nFORCE_PLATFORM USED: {used}")
    if "TYPE" in fp:
        print("FP TYPE:", fp["TYPE"]["value"])

# Events
print("\n=== EVENTS ===")
if "EVENT" in params:
    ev = params["EVENT"]
    for k in ev:
        try:
            print(f"EVENT.{k}: {ev[k]['value']}")
        except Exception as e:
            print(f"EVENT.{k}: <{e}>")
else:
    print("No EVENT parameter group")
if "EVENT_CONTEXT" in params:
    print("EVENT_CONTEXT keys:", list(params["EVENT_CONTEXT"].keys()))

print("\n=== Parameter groups present ===")
print(", ".join(params.keys()))
