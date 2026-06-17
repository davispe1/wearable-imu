"""Phase-A throwaway inspector for GaitUp/Physilog .BIN files (READ-ONLY)."""
import sys, struct

path = sys.argv[1]
with open(path, "rb") as f:
    head = f.read(4096)
    f.seek(0, 2)
    size = f.tell()

print(f"FILE: {path}")
print(f"SIZE: {size} bytes")

# ASCII view of first 1024 bytes
def ascii_view(b):
    return "".join(chr(c) if 32 <= c < 127 else "." for c in b)

print("\n=== First 256 bytes HEX ===")
for i in range(0, 256, 16):
    chunk = head[i:i+16]
    hx = " ".join(f"{c:02x}" for c in chunk)
    print(f"{i:04x}  {hx:<47}  {ascii_view(chunk)}")

# Look for printable ASCII runs (possible text header / config strings)
print("\n=== Printable ASCII runs (>=4 chars) in first 4096 bytes ===")
run = b""
runs = []
for c in head:
    if 32 <= c < 127:
        run += bytes([c])
    else:
        if len(run) >= 4:
            runs.append(run.decode())
        run = b""
if len(run) >= 4:
    runs.append(run.decode())
for r in runs[:60]:
    print(repr(r))
