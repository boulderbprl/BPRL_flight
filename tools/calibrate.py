#!/usr/bin/env python3
"""
BPRL IMU Calibration — collect static bias, write to flash.

Requires a firmware built with -DBPRL_DEBUG.
The $IMU telemetry stream (emitted by DebugThread) is needed for data collection.

Usage:
    python3 tools/calibrate.py                        # default: calibrate (30 s collection)
    python3 tools/calibrate.py calibrate [--duration N]
    python3 tools/calibrate.py clear                  # clear stored calibration, leave at zero

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
    --duration N          Collection time in seconds (default: 30)
"""

import argparse
import time

from bprl_common import (
    console, open_port, add_port_args, SerialReader,
    parse_imu_line,
)

from rich.progress import Progress
from rich.table import Table


def _wait_for_line(ser, want_prefix, timeout, carry=b""):
    """Read lines until one starting with want_prefix arrives, a CAL,ERR
    line arrives, or timeout elapses. Returns (line, leftover) — line is the
    matching/error line or None on timeout; leftover is whatever unconsumed
    bytes were already read past that line. If firmware ever sends two
    responses back-to-back with no gap, both can land in the same read, so
    a match here can carry bytes belonging to the *next* expected response.
    Pass `leftover` in as the next call's `carry` in that case, rather than
    discarding it — otherwise those bytes are silently lost with it."""
    deadline = time.monotonic() + timeout
    buf = carry
    while time.monotonic() < deadline:
        while b"\n" in buf:
            line_b, buf = buf.split(b"\n", 1)
            resp = line_b.decode("ascii", errors="replace").strip()
            if resp.startswith(want_prefix) or resp.startswith("CAL,ERR"):
                return resp, buf
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        else:
            time.sleep(0.05)
    return None, buf


def cmd_calibrate(ser, args):
    duration = getattr(args, "duration", 30)

    console.print("\n[bold]IMU Static Bias Calibration[/bold]")
    console.print("[yellow]Requires -DBPRL_DEBUG firmware build.[/yellow]")
    console.print("Place the drone on a [bold]level surface[/bold] and do not move it.")
    try:
        input("Press [Enter] to begin collection...")
    except EOFError:
        pass

    # Same reasoning as the reset_input_buffer() call before the write phase
    # below: the FC has likely been streaming $TEL/$EKFL/$IMU/$POS the whole
    # time the user was reading the prompt above, and nobody's drained it.
    ser.reset_input_buffer()

    # The $IMU stream reports g_imu[i], which is already bias-corrected
    # (raw - currently stored calibration) on the firmware side. If a prior
    # calibration is already saved, sampling it directly would only capture
    # the *residual* bias and then overwrite the stored calibration with
    # that residual, silently discarding most of the original correction.
    # Clearing first guarantees the stream reflects true raw sensor data
    # regardless of what was previously saved, so every run computes the
    # full absolute bias.
    console.print("Clearing any existing calibration so this run starts from raw sensor data...")
    ser.write(b"CAL,clear\n")
    ser.flush()
    resp, _ = _wait_for_line(ser, "CAL,OK", 2.0)
    if resp != "CAL,OK":
        if resp is None:
            console.print("[red]No response to CAL,clear — aborting (check connection).")
        else:
            console.print(f"[red]CAL,clear failed: {resp} — aborting.")
        return

    # Only start the background reader now, for the passive collection phase.
    # Starting it earlier would race the direct ser.read() calls above (and
    # the CAL,set/commit/query calls below) for bytes on the same serial
    # object — pyserial doesn't fan out one read() to multiple threads, so
    # whichever reader's blocking read happens to be live "wins" a given
    # chunk and the other sees nothing.
    reader = SerialReader(ser)

    console.print(f"Collecting {duration} s of IMU data...")

    sums_gyro  = [[0.0]*3 for _ in range(3)]
    sums_accel = [[0.0]*3 for _ in range(3)]
    counts     = [0, 0, 0]
    t_end      = time.monotonic() + duration

    with Progress(transient=True) as prog:
        task = prog.add_task("Collecting...", total=duration)
        while time.monotonic() < t_end:
            for line in reader.pop_lines():
                s = parse_imu_line(line)
                if s is None:
                    continue
                for i in range(3):
                    if not s.valid[i]:
                        continue
                    counts[i] += 1
                    for k in range(3):
                        sums_gyro[i][k]  += s.gyro[i][k]
                        sums_accel[i][k] += s.accel[i][k]
            elapsed = duration - (t_end - time.monotonic())
            prog.update(task, completed=min(elapsed, duration))
            time.sleep(0.05)

    reader.stop()

    # Not every board populates all 3 IMU slots (e.g. Orqa has only 2 physical
    # IMUs — g_imu[2] is never written, so counts[2] stays 0 forever). Only
    # require/compute bias for slots that actually reported samples.
    active = [i for i in range(3) if counts[i] > 0]
    if not active:
        console.print(f"[red]No IMU samples received at all (got {counts}). Check connection and DEBUG build.")
        return
    if min(counts[i] for i in active) < 10:
        console.print(f"[red]Too few samples (got {counts}). Check connection and DEBUG build.")
        return

    gyro_bias  = {i: [sums_gyro[i][k]  / counts[i] for k in range(3)] for i in active}
    accel_bias = {i: [sums_accel[i][k] / counts[i] for k in range(3)] for i in active}
    # EKF.cpp's predict() is explicit about the convention this firmware
    # actually uses: "NED z-down: sensor reads -g at hover" — a
    # bias-corrected resting accel Z should read -9.80665, not +9.80665.
    # Subtracting +9.80665 here (as this line used to) computes a bias
    # against the opposite, wrong assumption — harmless-looking on paper,
    # but on real data it turns a small ~0.1 m/s^2 sensor offset into a
    # ~19.5 m/s^2 bias that masks a sign inversion instead of correcting a
    # real one. Confirmed against the Orqa port 2026-08-12: raw accel Z at
    # rest reads ~-9.7 (correct per this convention), and calibration
    # computed with "-= 9.80665" was producing exactly the ~-19.5 residual
    # seen in the field.
    for i in active:
        accel_bias[i][2] += 9.80665

    tbl = Table(title="Computed Biases", show_header=True)
    tbl.add_column("IMU")
    tbl.add_column("gx (rad/s)", justify="right")
    tbl.add_column("gy (rad/s)", justify="right")
    tbl.add_column("gz (rad/s)", justify="right")
    tbl.add_column("ax (m/s²)", justify="right")
    tbl.add_column("ay (m/s²)", justify="right")
    tbl.add_column("az (m/s²)", justify="right")
    for i in active:
        tbl.add_row(
            f"IMU{i}",
            *[f"{gyro_bias[i][k]:+.5f}"  for k in range(3)],
            *[f"{accel_bias[i][k]:+.5f}" for k in range(3)],
        )
    console.print(tbl)
    if len(active) < 3:
        missing = [i for i in range(3) if i not in active]
        console.print(f"[dim]IMU slot(s) {missing} reported no data (not present on this board) — skipped.")
    console.print(f"  Samples: " + "  ".join(f"IMU{i}={counts[i]}" for i in range(3)))

    try:
        ans = input("\nWrite to flight controller flash? [y/N] ").strip().lower()
    except EOFError:
        ans = "n"
    if ans != "y":
        console.print("[yellow]Calibration not written.")
        return

    # DebugThread keeps streaming $TEL/$EKFL/$IMU/$POS at 10Hz the whole time
    # we were printing the table and waiting on the prompt above — nothing
    # tells it we stopped reading. That backlog is sitting in the OS-level
    # serial receive buffer; without discarding it here, _wait_for_line()
    # below would have to drain all of it before ever reaching the actual
    # CAL,SET response, easily blowing its timeout and looking exactly like
    # "no response" even though the command went through fine.
    ser.reset_input_buffer()

    for i in active:
        gx, gy, gz = gyro_bias[i]
        ax, ay, az = accel_bias[i]
        cmd = f"CAL,set,{i},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f}"
        ser.write((cmd + "\n").encode())
        ser.flush()
        resp, _ = _wait_for_line(ser, f"CAL,SET,{i},OK", 2.0)
        if resp != f"CAL,SET,{i},OK":
            if resp is None:
                console.print(f"[red]No response to CAL,set for IMU{i} — aborting.")
            else:
                console.print(f"[red]CAL,set for IMU{i} failed: {resp} — aborting.")
            return

    ser.write(b"CAL,commit\n")
    ser.flush()
    resp, _ = _wait_for_line(ser, "CAL,OK", 3.0)
    if resp != "CAL,OK":
        if resp is None:
            console.print("[red]No response to CAL,commit — write may have failed (device may have reset or hung; check connection and try again).")
        else:
            console.print(f"[red]CAL,commit failed: {resp} — write did not complete.")
        return

    console.print("[green]Calibration written to flash.")

    ser.write(b"CAL,query\n")
    ser.flush()
    resp, _ = _wait_for_line(ser, "CAL,DATA,", 3.0)
    if resp is None:
        console.print("[yellow]No CAL,DATA response from query.")
    else:
        console.print(f"[dim]Stored: {resp}")


def cmd_clear(ser, args):
    """Clear the stored calibration and leave it at zero — no collection,
    no write. Use this to back out a bad calibration."""
    console.print("\n[bold]Clear IMU Calibration[/bold]")
    ser.write(b"CAL,clear\n")
    ser.flush()
    resp, _ = _wait_for_line(ser, "CAL,OK", 2.0)
    if resp != "CAL,OK":
        if resp is None:
            console.print("[red]No response to CAL,clear — check connection.")
        else:
            console.print(f"[red]CAL,clear failed: {resp}")
        return
    console.print("[green]Calibration cleared — flight controller now uses zero bias for all IMUs.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL IMU calibration — requires -DBPRL_DEBUG firmware build")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    cal_p = sub.add_parser("calibrate",
                            help="Collect IMU static bias and write to flash (DEBUG build required)")
    cal_p.add_argument("--duration", type=int, default=30,
                       help="Collection time in seconds (default: 30)")

    sub.add_parser("clear",
                    help="Clear stored calibration (zero bias) without recalibrating (DEBUG build required)")

    args = parser.parse_args()
    if args.command is None:
        args.command = "calibrate"
        args.duration = 30

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "clear":
            cmd_clear(ser, args)
        else:
            cmd_calibrate(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
