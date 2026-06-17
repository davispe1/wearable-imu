"""Phase-A: verify 512-byte page framing and page-header layout."""
import sys, struct
from collections import Counter

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()
n = len(data)

def hexdump(off, length):
    out = []
    for k in range(off, min(off+length, n), 16):
        chunk = data[k:k+16]
        hx = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"{k:08x}  {hx:<47}  {asc}")
    return "\n".join(out)

print("=== Page transition 0x3f0..0x430 ===")
print(hexdump(0x3f0, 0x40))
print("\n=== Page transition 0x5f0..0x630 ===")
print(hexdump(0x5f0, 0x40))

# Page header hypothesis: at each 0x200 boundary, first 4 bytes = big-endian page index
print("\n=== First 4 bytes at each 0x200 boundary (page index BE?) ===")
for p in range(0x400, 0x400 + 0x200*12, 0x200):
    idx = struct.unpack(">I", data[p:p+4])[0]
    print(f"  @0x{p:08x}: {data[p:p+8].hex()}  -> BE uint32={idx}")
