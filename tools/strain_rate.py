#!/usr/bin/env python3
"""
BPRL Strain Rate Monitor — live display of strain-rate sensor data.

Reads 4 signed int16 values from the strain rate sensor at CAN ID 0x69,
one value per arm. In development.

Works on any firmware build.

Usage:
    python3 tools/strain_rate.py                      # default: live strain-rate display
    python3 tools/strain_rate.py strain-rate

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time
from typing import Optional

from bprl_common import console, open_port, add_port_args

from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

ARM_LABELS = ["Arm 0 (FR)", "Arm 1 (RL)", "Arm 2 (FL)", "Arm 3 (RR)"]


def _poll_strain_rate(ser) -> Optional[tuple]:
    """Send STRAIN_RATE,read and return (val0, val1, val2, val3, valid) or None."""
    ser.reset_input_buffer()
    ser.write(b"STRAIN_RATE,read\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        m = re.match(r"STRAIN_RATE,(-?\d+),(-?\d+),(-?\d+),(-?\d+),(\d+)", line)
        if m:
            return (int(m.group(1)), int(m.group(2)),
                    int(m.group(3)), int(m.group(4)),
                    bool(int(m.group(5))))
    return None


def cmd_strain_rate(ser, _args):
    vals    = [0, 0, 0, 0]
    valid   = False
    last_rx = time.monotonic()

    def build_panel() -> Panel:
        age   = time.monotonic() - last_rx
        stale = age > 2.0
        v_tag = "[green]● valid[/green]" if valid and not stale else \
                ("[yellow](stale)[/yellow]" if stale else "[dim]○ no data[/dim]")

        tbl = Table.grid(padding=(0, 3))
        tbl.add_column(min_width=14)
        tbl.add_column(min_width=10, justify="right")
        for i, lbl in enumerate(ARM_LABELS):
            col = "cyan" if valid and not stale else "dim"
            tbl.add_row(f"[bold]{lbl}[/bold]",
                        f"[{col}]{vals[i]:6d}[/{col}]")

        body = Table.grid()
        body.add_row(tbl)
        return Panel(body,
                     title=f"Strain Rate Sensor (CAN 0x69)  {v_tag}",
                     border_style="cyan")

    console.print("[dim]Polling STRAIN_RATE,read at ~5 Hz — Ctrl-C to exit[/dim]")
    try:
        with Live(build_panel(), refresh_per_second=10, console=console) as live:
            while True:
                result = _poll_strain_rate(ser)
                if result is not None:
                    vals[0], vals[1], vals[2], vals[3], valid = result
                    last_rx = time.monotonic()
                live.update(build_panel())
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL strain rate monitor — live display from CAN ID 0x69")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("strain-rate",
                   help="Live strain-rate sensor monitor (CAN 0x69, in development)")

    args = parser.parse_args()
    if args.command is None:
        args.command = "strain-rate"

    ser = open_port(args.port, args.baud)
    try:
        cmd_strain_rate(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
