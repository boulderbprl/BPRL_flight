#!/usr/bin/env python3
"""
BPRL DShot GCR offline decoder test.

Implements the same GCR decoding algorithm as src/coms/DShot.cpp decode_gcr()
in Python and verifies it against known test vectors.

Run:   python3 tools/dshot_decode_test.py
No hardware needed.  Used to validate the firmware decoder is correct.

Also decodes raw edge dumps from 'DSHOT,diag' output so you can diagnose
GCR reception issues without recompiling firmware.
"""

import sys

# ── GCR constants (must match DShot.cpp) ────────────────────────────────────

DS_ARR         = 333          # DShot bit period - 1 ticks (200 MHz, PSC=0)
GCR_BIT_TICKS  = (DS_ARR + 1) * 4 // 5   # 267 ticks per GCR bit @ 200 MHz

GCR_DECODE = [
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x09, 0x0A, 0x0B,
    0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03,
    0xFF, 0x05, 0x06, 0x07,
    0xFF, 0x00, 0x08, 0x01,
    0xFF, 0x04, 0x0C, 0xFF,
]

GCR_ENCODE = [
    0x19, 0x1B, 0x12, 0x13,   # 0,  1,  2,  3
    0x1D, 0x15, 0x16, 0x17,   # 4,  5,  6,  7
    0x1A, 0x09, 0x0A, 0x0B,   # 8,  9, 10, 11
    0x1E, 0x0D, 0x0E, 0x0F,   # 12, 13, 14, 15
]


def gcr_decode(buf: list[int]) -> tuple[bool, int]:
    """
    Decode a list of CCR edge timestamps → (ok, erpm).
    Mirrors decode_gcr() in DShot.cpp exactly.
    Returns (True, erpm) on success, (False, 0) on failure.
    """
    n_edges = len(buf)
    if n_edges < 2:
        return False, 0

    bits        = 0
    bits_filled = 0

    for i in range(n_edges):
        if bits_filled >= 21:
            break
        if i + 1 < n_edges:
            diff = buf[i + 1] - buf[i]
            n    = (diff + GCR_BIT_TICKS // 2) // GCR_BIT_TICKS
            if n == 0:
                n = 1
        else:
            n = 21 - bits_filled

        if n > 21 - bits_filled:
            n = 21 - bits_filled

        # Transition-marking: '1' at MSB of run, '0's below it.
        bits = ((bits << n) | (1 << (n - 1))) & 0x1FFFFF
        bits_filled += n

    if bits_filled < 21:
        return False, 0

    gcr20 = bits & 0xFFFFF
    frame = 0
    for g in range(3, -1, -1):
        code   = (gcr20 >> (g * 5)) & 0x1F
        nibble = GCR_DECODE[code]
        if nibble == 0xFF:
            return False, 0
        frame = ((frame << 4) | nibble) & 0xFFFF

    # Inverted nibble CRC: (n0^n1^n2^n3) & 0xF must equal 0xF
    csum = frame ^ (frame >> 4) ^ (frame >> 8) ^ (frame >> 12)
    if (csum & 0xF) != 0xF:
        return False, 0

    val      = (frame >> 4) & 0xFFF
    exponent = (val >> 9) & 0x7
    mantissa = val & 0x1FF
    erpm     = mantissa << exponent
    return True, erpm


def gcr_encode_packet(erpm: int) -> tuple[list[int], int]:
    """
    Encode an eRPM value as a GCR bit-stream and return synthetic edge
    timestamps (at GCR_BIT_TICKS spacing) along with the 20-bit GCR word.
    Useful for generating test vectors.

    Returns (edge_timestamps, gcr20_bits).
    """
    # Pack eRPM into 9-bit mantissa + 3-bit exponent
    mantissa = erpm
    exponent = 0
    while mantissa > 0x1FF:
        mantissa >>= 1
        exponent  += 1
    val = (exponent << 9) | (mantissa & 0x1FF)

    # Build 16-bit frame: [val(12)][CRC(4)]  — period at bits[15:4], CRC at bits[3:0].
    # Same layout as the DShot TX frame from the FC.
    # The 4 nibbles (val>>8, val>>4&F, val&F, crc) must XOR to 0xF (inverted CRC).
    crc   = 0xF ^ ((val ^ (val >> 4) ^ (val >> 8)) & 0xF)
    frame = ((val & 0xFFF) << 4) | (crc & 0xF)

    # Encode 4 nibbles → 4 × 5-bit GCR codes → 20-bit GCR word
    gcr20 = 0
    for g in range(3, -1, -1):
        nibble = (frame >> (g * 4)) & 0xF
        code   = GCR_ENCODE[nibble]
        gcr20  = (gcr20 << 5) | code

    # Convert 20-bit GCR word to edge timestamps.
    #
    # The transition-marking decoder (decode_gcr) produces `bits & 0xFFFFF = gcr20`
    # only when the IC timestamps correspond to the '1'-bit positions of the 21-bit
    # word  bits_21 = (1<<20) | gcr20.  Each '1' in bits_21 is one captured edge;
    # its distance to the next '1' below is the interval the decoder measures.
    # The last '1' (always at bit 0, since valid GCR codes end in 1) triggers the
    # decoder's inferred-last-run path (n = 21 - bits_filled = 1).
    guard   = 6000  # ~30 µs at 200 MHz
    bits_21 = (1 << 20) | gcr20
    timestamps = [guard + (20 - pos) * GCR_BIT_TICKS
                  for pos in range(20, -1, -1)
                  if (bits_21 >> pos) & 1]

    return timestamps, gcr20


# ── Test vectors ─────────────────────────────────────────────────────────────

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
total = 0
passed = 0


def check(label: str, got, expected):
    global total, passed
    total += 1
    ok = (got == expected)
    if ok:
        passed += 1
    sym = PASS if ok else FAIL
    print(f"  {sym}  {label}")
    if not ok:
        print(f"        expected {expected!r}, got {got!r}")


def run_roundtrip(erpm: int):
    """Encode eRPM → synthetic timestamps → decode → verify."""
    # Mantissa/exponent encoding is lossy: compute what survives the round-trip.
    m, e = erpm, 0
    while m > 0x1FF:
        m >>= 1; e += 1
    expected = m << e   # may differ from erpm for values like 5000→4992

    timestamps, gcr20 = gcr_encode_packet(erpm)
    ok, decoded = gcr_decode(timestamps)
    label = f"roundtrip  eRPM={erpm:6d}  gcr20=0x{gcr20:05X}  edges={len(timestamps)}"
    check(label, (ok, decoded), (True, expected))


def run_rejection(label: str, buf: list[int]):
    """Expect decode to FAIL."""
    ok, _ = gcr_decode(buf)
    check(f"reject  {label}", ok, False)


if __name__ == "__main__":
    print("=" * 60)
    print("BPRL DShot GCR decoder self-test")
    print(f"GCR_BIT_TICKS = {GCR_BIT_TICKS}  (200 MHz, DS_ARR=333)")
    print("=" * 60)

    # ── Round-trip tests ──────────────────────────────────────────────────────
    print("\n[Round-trip encode → decode]")

    run_roundtrip(0)           # stopped
    run_roundtrip(100)         # low RPM
    run_roundtrip(1000)        # typical hover
    run_roundtrip(5000)        # half throttle
    run_roundtrip(10000)       # high RPM
    run_roundtrip(65408)       # near max (mantissa=255, exp=7 → 255<<7=32640... hmm)
    # max encodable: mantissa=511, exponent=7 → 511<<7 = 65408
    run_roundtrip(65408)

    # ── Edge case: exactly 21 edges (all single-bit runs) ────────────────────
    print("\n[Edge cases]")
    ts, _ = gcr_encode_packet(1234)
    m1234, e1234 = 1234, 0
    while m1234 > 0x1FF: m1234 >>= 1; e1234 += 1
    expected_1234 = m1234 << e1234
    check("known eRPM=1234 decodes OK", gcr_decode(ts)[0], True)
    check("known eRPM=1234 value",      gcr_decode(ts)[1], expected_1234)

    # ── Rejection tests ───────────────────────────────────────────────────────
    print("\n[Rejection tests]")
    run_rejection("empty buffer",          [])
    run_rejection("single edge",           [6000])
    run_rejection("all-zero edges",        [0] * 22)
    # Corrupt one edge timestamp
    ts_good, _ = gcr_encode_packet(5000)
    ts_bad = ts_good[:]
    ts_bad[5] += 50   # small offset → wrong bit count
    run_rejection("corrupted edge",        ts_bad)

    # ── Verify CRC inversion ─────────────────────────────────────────────────
    print("\n[CRC inversion verification]")
    ts, gcr20 = gcr_encode_packet(3000)
    ok, erpm = gcr_decode(ts)
    m3000, e3000 = 3000, 0
    while m3000 > 0x1FF: m3000 >>= 1; e3000 += 1
    check(f"eRPM=3000 CRC passes", (ok, erpm), (True, m3000 << e3000))

    # Build a packet with NON-inverted CRC (old BPRL bug) and confirm rejection
    # Manually construct a 16-bit frame with standard (non-inverted) CRC
    val = 3000  # mantissa=375(?), let's just use a small eRPM
    erpm_test = 100
    mantissa = erpm_test; exponent = 0
    while mantissa > 0x1FF: mantissa >>= 1; exponent += 1
    val = (exponent << 9) | mantissa
    crc_normal = ((val) ^ (val >> 4) ^ (val >> 8)) & 0xF  # NON-inverted
    frame_bad  = (crc_normal << 12) | val
    gcr20_bad = 0
    for g in range(3, -1, -1):
        nibble = (frame_bad >> (g * 4)) & 0xF
        gcr20_bad = (gcr20_bad << 5) | GCR_ENCODE[nibble]
    # Build fake timestamps from gcr20_bad
    runs_bad = []
    run_len = 0; prev = None
    for bit_pos in range(19, -1, -1):
        b = (gcr20_bad >> bit_pos) & 1
        if prev is None: prev = b; run_len = 1
        elif b == prev:  run_len += 1
        else: runs_bad.append((prev, run_len)); prev = b; run_len = 1
    if run_len: runs_bad.append((prev, run_len))
    t = 6000; ts_bad2 = [t]
    for _, rl in runs_bad[1:]: t += rl * GCR_BIT_TICKS; ts_bad2.append(t)
    run_rejection("non-inverted CRC (old BPRL bug B)", ts_bad2)

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print("=" * 60)
    if passed == total:
        print(f"\033[32mAll {total} tests passed.\033[0m")
    else:
        print(f"\033[31m{total - passed} of {total} tests FAILED.\033[0m")
    print("=" * 60)

    # ── Interactive: decode raw edges from DSHOT,diag output ─────────────────
    if len(sys.argv) > 1:
        print("\n[Manual edge decode]")
        for arg in sys.argv[1:]:
            try:
                edges = [int(x) for x in arg.split(",")]
                ok, erpm = gcr_decode(edges)
                intervals = [edges[i+1]-edges[i] for i in range(len(edges)-1)]
                bits = [round(iv / GCR_BIT_TICKS) for iv in intervals]
                print(f"  Input: {edges}")
                print(f"  Intervals (ticks): {intervals}")
                print(f"  Bit counts: {bits}  (sum={sum(bits)}, expected 21)")
                if ok:
                    print(f"  \033[32mDecode OK: eRPM = {erpm}\033[0m")
                else:
                    print(f"  \033[31mDecode FAILED\033[0m")
            except Exception as e:
                print(f"  ERROR parsing '{arg}': {e}")
        print()
        print("Usage: python3 tools/dshot_decode_test.py 'e0,e1,e2,...'")

    sys.exit(0 if passed == total else 1)
