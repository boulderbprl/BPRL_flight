#!/usr/bin/env python3
"""
wing_encoder_test.py - poll the AS5600 wing encoder over the flight controller's USB serial port.

The firmware answers the text command "WING,read" with one line:

    WING,<s1>,<s2>,0x<STATUS>,<RAW>,<ANGLE>,<AGC>,<MAG>

    s1, s2   ChibiOS status of the two I2C reads (0 = MSG_OK, anything else = bus error/timeout)
    STATUS   AS5600 register 0x0B   (bit5 MD magnet detected, bit4 ML too weak, bit3 MH too strong)
    RAW      12-bit raw angle, 0..4095   (register 0x0C-0x0D)
    ANGLE    12-bit angle after chip scaling/filter (register 0x0E-0x0F)
    AGC      automatic gain control (register 0x1A)
    MAG      CORDIC magnitude (register 0x1B-0x1C)

Usage (Windows, from the repo root):
    py tools\\wing_encoder_test.py --port COM10
    py tools\\wing_encoder_test.py --port COM10 --rate 20 --csv wing.csv
    py tools\\wing_encoder_test.py --list

Needs pyserial:   py -m pip install pyserial
Stop with Ctrl+C.
"""

import argparse
import csv
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing. Install it with:  py -m pip install pyserial")

COMMAND = b"WING,read\n"
PREFIX = "WING,"

MD_BIT = 0x20   # magnet detected
ML_BIT = 0x10   # magnet too weak
MH_BIT = 0x08   # magnet too strong


def parse_line(line):
    """Parse one 'WING,...' line. Returns a dict, or None if the line isn't a valid reply."""
    line = line.strip()
    if not line.startswith(PREFIX):
        return None
    parts = line[len(PREFIX):].split(",")
    if len(parts) != 7:
        return None
    try:
        s1, s2 = int(parts[0]), int(parts[1])
        status = int(parts[2], 16)
        raw, angle, agc, mag = (int(p) for p in parts[3:7])
    except ValueError:
        return None
    return {
        "s1": s1, "s2": s2, "status": status,
        "raw": raw, "angle": angle, "agc": agc, "mag": mag,
        "raw_deg": raw * 360.0 / 4096.0,
        "angle_deg": angle * 360.0 / 4096.0,
    }


def magnet_text(status):
    if status & MD_BIT:
        if status & ML_BIT:
            return "magnet detected but WEAK"
        if status & MH_BIT:
            return "magnet detected but TOO STRONG"
        return "magnet OK"
    if status & ML_BIT:
        return "magnet too weak"
    if status & MH_BIT:
        return "magnet too strong"
    return "NO MAGNET"


def read_sample(ser, budget_lines=20):
    """Send the command and wait for a WING reply, skipping any other lines on the port."""
    ser.write(COMMAND)
    for _ in range(budget_lines):
        raw = ser.readline()
        if not raw:                      # timed out
            return None
        sample = parse_line(raw.decode(errors="replace"))
        if sample is not None:
            return sample
    return None


def format_sample(s):
    ok = "OK" if (s["s1"] == 0 and s["s2"] == 0) else f"I2C ERR ({s['s1']},{s['s2']})"
    return (f"raw={s['raw']:4d} ({s['raw_deg']:6.2f} deg)  "
            f"angle={s['angle']:4d} ({s['angle_deg']:6.2f} deg)  "
            f"agc={s['agc']:3d}  mag={s['mag']:4d}  "
            f"status=0x{s['status']:02x} {magnet_text(s['status']):<28s} [{ok}]")


def main():
    ap = argparse.ArgumentParser(description="Poll the AS5600 wing encoder through the flight controller.")
    ap.add_argument("--port", default="COM10", help="serial port of the running firmware (default COM10)")
    ap.add_argument("--rate", type=float, default=10.0, help="polls per second (default 10)")
    ap.add_argument("--csv", metavar="FILE", help="also log every sample to this CSV file")
    ap.add_argument("--scroll", action="store_true", help="print one line per sample instead of updating in place")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}: {p.description}")
        return 0

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.5)   # baud is ignored by USB CDC
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}")
        print("Is the board powered and running the firmware (not the bootloader port)?")
        print("Is another program (serial monitor, i2c_tools) holding the port?")
        return 1

    csv_file = writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t_s", "s1", "s2", "status", "raw", "angle", "agc", "mag", "raw_deg", "angle_deg"])

    print(f"Connected: {args.port}   (Ctrl+C to stop)")
    ser.reset_input_buffer()
    period = 1.0 / max(args.rate, 0.1)
    t0 = time.time()
    misses = 0
    last_len = 0
    try:
        while True:
            t_loop = time.time()
            s = read_sample(ser)
            if s is None:
                misses += 1
                msg = (f"no WING reply ({misses}) - firmware needs the WING,read command, "
                       f"built for a board with I2C1 enabled")
                print("\r" + msg.ljust(last_len), end="" if not args.scroll else "\n", flush=True)
                last_len = len(msg)
            else:
                misses = 0
                text = format_sample(s)
                if args.scroll:
                    print(text)
                else:
                    print("\r" + text.ljust(last_len), end="", flush=True)
                    last_len = len(text)
                if writer:
                    writer.writerow([f"{t_loop - t0:.3f}", s["s1"], s["s2"], f"0x{s['status']:02x}",
                                     s["raw"], s["angle"], s["agc"], s["mag"],
                                     f"{s['raw_deg']:.3f}", f"{s['angle_deg']:.3f}"])
            time.sleep(max(0.0, period - (time.time() - t_loop)))
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        return 1
    finally:
        ser.close()
        if csv_file:
            csv_file.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())