#!/usr/bin/env python3
"""
BPRL Encoder RPM Monitor — live display of shaft-angle encoder RPM nodes.

Reads mechanical RPM + raw shaft angle from up to two Feather M4 + AS5047P
CAN nodes (CAN 0x70 = node 0, 0x71 = node 1), broadcast per
Strain_CAN/Feather_Code/Feather_Code.ino and decoded on the FC by
src/sensors/EncoderRPM.cpp.

Works on any firmware build (no BPRL_DEBUG required).

Usage:
    python3 tools/encoder_rpm.py                  # default: live encoder RPM display
    python3 tools/encoder_rpm.py encoder-rpm

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

NUM_NODES = 2
ENC_RE = re.compile(r"ENC,(\d+),(-?[\d.]+),(\d+),(\d+),(\d+)")


def _poll_encoder_rpm(ser) -> Optional[dict]:
    """Send ENC,read and return {node: (rpm, angle_raw, error_flag, valid)}."""
    ser.reset_input_buffer()
    ser.write(b"ENC,read\r\n")
    result = {}
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and len(result) < NUM_NODES:
        line = ser.readline().decode("ascii", errors="replace").strip()
        m = ENC_RE.match(line)
        if m:
            node = int(m.group(1))
            result[node] = (float(m.group(2)), int(m.group(3)),
                             int(m.group(4)), bool(int(m.group(5))))
    return result or None


def cmd_encoder_rpm(ser, _args):
    nodes   = {i: (0.0, 0, 0, False) for i in range(NUM_NODES)}
    last_rx = {i: 0.0 for i in range(NUM_NODES)}

    def build_panel() -> Panel:
        now = time.monotonic()
        tbl = Table.grid(padding=(0, 3))
        tbl.add_column(min_width=8)
        tbl.add_column(min_width=12, justify="right")
        tbl.add_column(min_width=12, justify="right")
        tbl.add_column(min_width=10, justify="right")
        tbl.add_column(min_width=10)
        tbl.add_row("[bold]Node[/bold]", "[bold]RPM[/bold]", "[bold]Angle (deg)[/bold]",
                    "[bold]Err[/bold]", "[bold]Status[/bold]")
        for i in range(NUM_NODES):
            rpm, angle_raw, error_flag, valid = nodes[i]
            age   = now - last_rx[i]
            stale = age > 2.0 or last_rx[i] == 0.0
            col   = "cyan" if valid and not stale else "dim"
            status = "[green]● valid[/green]" if valid and not stale else \
                     ("[yellow](stale)[/yellow]" if valid else "[dim]○ no data[/dim]")
            angle_deg = angle_raw * (360.0 / 16384.0)
            tbl.add_row(f"[bold]{i}[/bold]",
                        f"[{col}]{rpm:8.1f}[/{col}]",
                        f"[{col}]{angle_deg:7.2f}[/{col}]",
                        f"[{'red' if error_flag else col}]{error_flag}[/{'red' if error_flag else col}]",
                        status)

        return Panel(tbl, title="Encoder RPM (CAN 0x70/0x71)", border_style="cyan")

    console.print("[dim]Polling ENC,read at ~5 Hz — Ctrl-C to exit[/dim]")
    try:
        with Live(build_panel(), refresh_per_second=10, console=console) as live:
            while True:
                result = _poll_encoder_rpm(ser)
                if result is not None:
                    now = time.monotonic()
                    for node, vals in result.items():
                        nodes[node]   = vals
                        last_rx[node] = now
                live.update(build_panel())
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL encoder RPM monitor — live display via ENC,read USB command")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("encoder-rpm",
                   help="Live encoder RPM monitor")

    args = parser.parse_args()
    if args.command is None:
        args.command = "encoder-rpm"

    ser = open_port(args.port, args.baud)
    try:
        cmd_encoder_rpm(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
