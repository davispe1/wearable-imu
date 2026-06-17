"""Phase-A: tag-driven streaming parse of the bulk data, aligned to 0x208."""
import sys, struct
from collections import Counter

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()
n = len(data)

DATA_KNOWN = {0x13, 0x14, 0x15, 0x18}
pos = 0x208
tag_counts = Counter()
unknown_events = []
chan_first = {t: [] for t in DATA_KNOWN}
ctr_seq = {t: [] for t in DATA_KNOWN}
records = 0
resyncs = 0

while pos + 8 <= n:
    tag = data[pos]
    if tag in DATA_KNOWN:
        ctr = data[pos+1]
        ax = struct.unpack(">hhh", data[pos+2:pos+8])
        tag_counts[tag] += 1
        if len(chan_first[tag]) < 5:
            chan_first[tag].append((ctr, ax))
        if len(ctr_seq[tag]) < 30:
            ctr_seq[tag].append(ctr)
        pos += 8
        records += 1
    else:
        # record drift / unknown tag; log and try to resync to next 0x13/14/15/18 frame
        if len(unknown_events) < 25:
            ctx = data[pos:pos+12].hex()
            unknown_events.append((pos, tag, ctx))
        resyncs += 1
        pos += 1

print(f"Records parsed as known tags: {records}, resync/unknown bytes: {resyncs}")
print(f"Clean fraction: {100*records*8/(records*8+resyncs):.3f}%")
print("\n=== Known-tag counts & implied rate (ref tag with max count = 256 Hz) ===")
mx = max(tag_counts.values())
for t in sorted(tag_counts):
    print(f"  tag 0x{t:02x}: {tag_counts[t]:8d}  rate~{256*tag_counts[t]/mx:6.1f} Hz")
dur = mx / 256.0
print(f"\nImplied duration: {dur:.1f} s ({dur/60:.2f} min)  [continuous whole-session recording]")

print("\n=== First samples per tag (counter, x,y,z int16 BE) ===")
for t in sorted(chan_first):
    print(f"  tag 0x{t:02x}: {chan_first[t]}")

print("\n=== Counter sequences (first 30) ===")
for t in sorted(ctr_seq):
    print(f"  tag 0x{t:02x}: {ctr_seq[t]}")

print("\n=== First unknown/drift events (offset, tag, 12-byte ctx) ===")
for off, tag, ctx in unknown_events[:25]:
    print(f"  @0x{off:08x} tag=0x{tag:02x} ctx={ctx}")
