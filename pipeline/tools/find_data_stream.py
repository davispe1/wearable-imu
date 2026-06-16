"""Phase-A: locate the bulk data stream after the header and probe record stride."""
import sys
from collections import Counter

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()
n = len(data)

# Count all 50 35 occurrences in whole file
sync_positions = []
i = 0
while True:
    j = data.find(b"\x50\x35", i)
    if j < 0:
        break
    sync_positions.append(j)
    i = j + 1
print(f"Total 0x5035 occurrences in file: {len(sync_positions)}")
print(f"First 20 positions: {sync_positions[:20]}")
if len(sync_positions) > 1:
    diffs = [sync_positions[k+1]-sync_positions[k] for k in range(len(sync_positions)-1)]
    dc = Counter(diffs)
    print("Most common gaps between 0x5035 occurrences:")
    for g, c in dc.most_common(12):
        print(f"   gap={g}  count={c}")

# Header end: find first long zero run after offset 0xc0
def hexdump(off, length):
    out = []
    for k in range(off, min(off+length, n), 16):
        chunk = data[k:k+16]
        hx = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"{k:06x}  {hx:<47}  {asc}")
    return "\n".join(out)

# Find the end of the leading zero pad: scan from 0xc8 for first region that becomes non-zero again
start = 0xc8
z = start
while z < n and data[z] == 0:
    z += 1
print(f"\nLeading zero pad after header runs to offset {z} (0x{z:x}), then data resumes.")
print("\n=== Dump around data resume ===")
print(hexdump(z, 256))
