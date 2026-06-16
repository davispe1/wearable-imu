"""Phase-A: confirm 8-byte record layout, enumerate channel tags and their rates."""
import sys, struct
from collections import Counter

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()
n = len(data)

DATA_START = 0x203  # first data byte after header (empirically)
# Assume 8-byte records: tag(1) ctr(1) + 3x int16 BE
tag_counts = Counter()
# decode a window for value ranges per tag
samples = {}
end = n - (n - DATA_START) % 8
pos = DATA_START
# align: find a position where tags look consistent; we trust 0x203
count = 0
while pos + 8 <= n:
    tag = data[pos]
    tag_counts[tag] += 1
    ax = struct.unpack(">hhh", data[pos+2:pos+8])
    if tag not in samples:
        samples[tag] = []
    if len(samples[tag]) < 4:
        samples[tag].append((data[pos+1], ax))
    pos += 8
    count += 1

print(f"Records (8-byte) parsed: {count}")
print("\n=== Tag frequencies (top 12) ===")
total = sum(tag_counts.values())
for tag, c in tag_counts.most_common(12):
    print(f"  tag 0x{tag:02x}: count={c:9d}  ({100*c/total:5.2f}%)  rate~{256*c/tag_counts.most_common(1)[0][1]:.1f}Hz-equiv")

print("\n=== First samples per dominant tag (counter, 3x int16 BE) ===")
for tag in sorted(samples):
    if tag_counts[tag] > total * 0.01:
        print(f"  tag 0x{tag:02x}: {samples[tag]}")

# Estimate duration if 0x13 is 256 Hz
top = tag_counts.most_common(1)[0]
print(f"\nDominant tag 0x{top[0]:02x} count={top[1]} -> if 256Hz, duration ~ {top[1]/256:.1f} s ({top[1]/256/60:.2f} min)")
