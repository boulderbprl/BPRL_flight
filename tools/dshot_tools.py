#!/usr/bin/env python3
"""
BPRL DShot Diagnostic — bidirectional DShot telemetry snapshot.

Works on any firmware build.

Usage:
    python3 tools/dshot_tools.py dshot-diag

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args

MOTOR_NAMES = ["M0 FR (TIM1/CC3)", "M1 RL (TIM1/CC1)", "M2 FL (TIM4/CC2)", "M3 RR (TIM1/CC2)"]


def cmd_dshot_diag(ser, _args):
    ser.write(b"DSHOT,diag\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line.startswith("DSHOT,DIAG,"):
            continue

        rest = line[len("DSHOT,DIAG,"):]
        kv   = dict(tok.split("=") for tok in rest.split(",") if "=" in tok)

        dma0  = int(kv.get("dma_tc", "0/0").split("/")[0])
        dma1  = int(kv.get("dma_tc", "0/0").split("/")[1])
        isr0  = int(kv.get("cc_isr", "0/0").split("/")[0])
        isr1  = int(kv.get("cc_isr", "0/0").split("/")[1])
        ecnts = [int(x) for x in kv.get("edges", "0/0/0/0").split("/")]

        raw_edges = []
        for key in ("e0", "e1", "e2", "e3"):
            if key in kv:
                raw_edges.append([int(x) for x in kv[key].split("/")])
            else:
                raw_edges.append([0] * 5)

        console.print("\n[bold]DShot Bidirectional Diagnostic[/bold]")
        console.print(f"  DMA TC fires  : TIM1={dma0}  TIM4={dma1}")
        console.print(f"  CC ISR decode : TIM1={isr0}  TIM4={isr1}")
        console.print()

        for i, name in enumerate(MOTOR_NAMES):
            ec = ecnts[i] if i < len(ecnts) else 0
            es = raw_edges[i] if i < len(raw_edges) else [0] * 5
            first_us = es[0] / 200.0 if es[0] > 0 else 0.0
            spacing  = (es[1] - es[0]) if ec >= 2 and es[1] > es[0] else 0
            status   = ("[green]OK[/green]"      if ec >= 2 else
                        "[red]NO EDGES[/red]"    if ec == 0 else
                        "[yellow]PARTIAL[/yellow]")
            console.print(f"  {name}: edges={ec}/21  {status}")
            if ec > 0:
                console.print(
                    f"    First edge: {es[0]} ticks ({first_us:.1f} µs after TX)  "
                    f"spacing: {spacing} ticks (~{spacing/267:.2f}× expected)")
                console.print(f"    Raw[0..4]: {es}")

        console.print()
        if dma0 == 0 and dma1 == 0:
            console.print("[red]FAIL: DMA TC never fires — DShot TX not completing (check DMAMUX or DMA init)[/red]")
        elif isr0 == 0 and isr1 == 0:
            console.print("[red]FAIL: CC ISR never decodes — input capture never triggered (check NVIC/DIER)[/red]")
        elif all(ec == 0 for ec in ecnts):
            console.print("[yellow]INFO: ISR fires but no edges captured — ESC not responding")
            console.print("  → Is the motor spinning? (ESC only responds after arming)")
            console.print("  → Check bidirectional DShot wiring (pull-up resistor on motor wire?)[/yellow]")
        elif all(ec >= 2 for ec in ecnts):
            console.print("[green]INFO: Edges captured on all motors — GCR decode is failing[/green]")
            console.print("  → Check bit_ticks (~267 expected), CRC inversion, edge polarity")
        else:
            console.print("[yellow]INFO: Partial edge capture — some motors responding, some not[/yellow]")
        return

    console.print("[red]No DSHOT,DIAG response — flash latest firmware and retry[/red]")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL DShot diagnostic — bidirectional telemetry snapshot")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("dshot-diag",
                   help="One-shot DShot bidirectional telemetry snapshot (edge counts, timing)")

    args = parser.parse_args()
    ser  = open_port(args.port, args.baud)
    try:
        cmd_dshot_diag(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
