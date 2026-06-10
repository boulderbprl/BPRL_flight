#!/usr/bin/env python3
"""
BPRL Motor Test — spin individual motors and view RPM feedback.

Works on any firmware build. Remove props before use.

Usage:
    python3 tools/motor_test.py motor-test

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)

Motor layout (top view):
    FL [2]       FR [0]
         \\       /
         [  body  ]
         /       \\
    RL [1]       RR [3]
"""

import argparse
import queue
import threading
import time

from bprl_common import (
    console, open_port, add_port_args, SerialReader, send_cmd,
    TelState, parse_tel_line,
)

from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

MOTOR_LABELS = ["FR[0]", "RL[1]", "FL[2]", "RR[3]"]


def build_motor_panel(state: TelState, active_motor: int, active_pct: int) -> Panel:
    rpm_grid = Table.grid(padding=(0, 2))
    rpm_grid.add_column(min_width=26)
    rpm_grid.add_column(min_width=26)
    for row in ((0, 1), (2, 3)):
        cells = []
        for i in row:
            dot = "●" if state.rpm[i] > 0 else "○"
            col = ("bold green" if i == active_motor and active_pct > 0
                   else ("green" if state.rpm[i] > 0 else "dim"))
            cells.append(Text(f"  {MOTOR_LABELS[i]}: {state.rpm[i]:6d} rpm {dot}",
                              style=col))
        rpm_grid.add_row(*cells)

    note = ""
    if active_motor >= 0 and active_pct > 0:
        note = f"  Active: {MOTOR_LABELS[active_motor]} @ {active_pct}%"

    body = Table.grid()
    body.add_row(rpm_grid)
    if note:
        body.add_row(Text(note, style="bold yellow"))

    return Panel(body,
                 title="[bold red]BPRL Motor Test  ⚠  REMOVE PROPS BEFORE TESTING[/bold red]",
                 border_style="red")


def cmd_motor_test(ser, _args):
    state        = TelState()
    active_motor = -1
    active_pct   = 0
    reader       = SerialReader(ser)
    resp_q       = queue.Queue()
    stop_bg      = threading.Event()

    console.print("[bold red]⚠  MOTOR TEST MODE — REMOVE PROPS BEFORE PROCEEDING[/bold red]")
    console.print("Commands: [cyan]motor <0-3> <pct%>[/cyan]  |  "
                  "[cyan]stop[/cyan]  |  [cyan]quit[/cyan]")
    console.print("[dim]Tip: 'motor 0 15' spins FR motor at 15%[/dim]\n")

    with Live(build_motor_panel(state, active_motor, active_pct),
              refresh_per_second=10, console=console, vertical_overflow="visible") as live:

        def bg_loop():
            while not stop_bg.is_set():
                for line in reader.pop_lines():
                    if line.startswith("MT,"):
                        resp_q.put(line)
                    else:
                        parse_tel_line(line, state)
                live.update(build_motor_panel(state, active_motor, active_pct))
                time.sleep(0.05)

        bg_t = threading.Thread(target=bg_loop, daemon=True)
        bg_t.start()

        try:
            while True:
                try:
                    cmd = input("> ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break

                if cmd in ("quit", "q", "exit"):
                    send_cmd(ser, "MT,stop")
                    break

                if cmd in ("stop", "s"):
                    send_cmd(ser, "MT,stop")
                    active_motor = -1
                    active_pct   = 0
                    continue

                parts = cmd.split()
                if len(parts) == 3 and parts[0] == "motor":
                    try:
                        motor = int(parts[1])
                        pct   = int(parts[2])
                    except ValueError:
                        console.print("[yellow]Usage: motor <0-3> <0-100>")
                        continue
                    if motor < 0 or motor > 3:
                        console.print("[yellow]Motor index must be 0-3.")
                        continue
                    if pct < 0 or pct > 100:
                        console.print("[yellow]Throttle must be 0-100%.")
                        continue
                    send_cmd(ser, f"MT,{motor},{pct}")
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        try:
                            resp = resp_q.get(timeout=0.1)
                        except queue.Empty:
                            continue
                        if "ERR,armed" in resp:
                            console.print("[bold red]REJECTED: Drone is armed! Disarm first.")
                        elif "OK" in resp:
                            active_motor = motor
                            active_pct   = pct
                        break
                    continue

                if cmd:
                    console.print("[dim]Unknown command. Try: motor 0 20 | stop | quit")

        except KeyboardInterrupt:
            pass
        finally:
            stop_bg.set()
            bg_t.join(timeout=1.0)

    send_cmd(ser, "MT,stop")
    reader.stop()
    console.print("[green]Motor test ended. All motors stopped.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL motor test — spin motors individually and view RPM")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("motor-test", help="Interactive motor test with RPM feedback")

    args = parser.parse_args()
    ser  = open_port(args.port, args.baud)
    try:
        cmd_motor_test(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
