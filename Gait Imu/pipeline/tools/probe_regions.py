"""Phase-A: map where non-zero data lives in the .BIN and probe record stride."""
import sys
from collections import Counter

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()
n = len(data)

# First non-zero byte after the header (offset 0x100)
k = 0x100
while k < n and data[k] == 0:
    k += 1
print(f"Header zero-pad ends; first non-zero after 0x100 at offset {k} (0x{k:x})")

# Occupancy histogram: fraction non-zero per 1 MB block
print("\n=== Non-zero fraction per 1 MB block ===")
MB = 1 << 20
b = 0
while b < n:
    block = data[b:b+MB]
    nz = sum(1 for x in block if x != 0)
    frac = nz / len(block)
    bar = "#" * int(frac * 40)
    print(f"  block {b//MB:2d} (0x{b:08x}): {frac:6.3f} {bar}")
    b += MB

# Dump the region where data first starts
def hexdump(off, length):
    out = []
    for kk in range(off, min(off+length, n), 16):
        chunk = data[kk:kk+16]
        hx = " ".join(f"{x:02x}" for x in chunk)
        asc = "".join(chr(x) if 32 <= x < 127 else "." for x in chunk)
        out.append(f"{kk:08x}  {hx:<47}  {asc}")
    return "\n".join(out)

print(f"\n=== Dump at first data offset {k} ===")
print(hexdump(k, 256))
