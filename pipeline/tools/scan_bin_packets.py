"""Phase-A: tally GaitUp .BIN packet types to identify channels by rate (READ-ONLY)."""
import sys, struct
from collections import Counter, defaultdict

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()

SYNC = b"\x50\x35"
TERM = b"\xff\xfe"

n = len(data)
i = 0
# skip leading zeros
typ_counter = Counter()      # (A,B,L) -> count
chan_payloads = defaultdict(list)  # (A,B) -> list of first few payloads
total_pkts = 0
bad = 0
first_counter = {}
last_counter = {}

# Packet guess: SYNC(2) A(1) B(1) L(1) payload(L) TERM(2)
while i < n - 2:
    if data[i] == 0x50 and data[i+1] == 0x35:
        if i + 5 > n:
            break
        A = data[i+2]; B = data[i+3]; L = data[i+4]
        pstart = i + 5
        pend = pstart + L
        if pend + 2 <= n and data[pend] == 0xff and data[pend+1] == 0xfe:
            payload = data[pstart:pend]
            typ_counter[(A, B, L)] += 1
            key = (A, B)
            if len(chan_payloads[key]) < 3:
                chan_payloads[key].append(payload.hex())
            # track 16-bit LE counter at payload start for high-rate data packets
            if L >= 4:
                ctr = payload[0] | (payload[1] << 8)
                if key not in first_counter:
                    first_counter[key] = ctr
                last_counter[key] = ctr
            total_pkts += 1
            i = pend + 2
            continue
        else:
            bad += 1
            i += 1
    else:
        i += 1

print(f"FILE size: {n} bytes, total packets parsed: {total_pkts}, resync misses: {bad}")
print("\n=== Packet (A,B,Length) tallies, sorted by count ===")
for (A, B, L), c in sorted(typ_counter.items(), key=lambda x: -x[1]):
    print(f"  A=0x{A:02x} B=0x{B:02x} L={L:3d}  count={c}")

print("\n=== Sample payloads (hex) per (A,B) ===")
for key, samples in sorted(chan_payloads.items()):
    print(f"  A=0x{key[0]:02x} B=0x{key[1]:02x}:")
    for s in samples:
        print(f"      {s}")
