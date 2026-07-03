#!/usr/bin/env python3
"""
BPRL MAVLink Tools — TELEM2/SD3 RX diagnostics.

Works on any firmware build (no -DBPRL_DEBUG needed).

Usage:
    python3 tools/mav_tools.py                # default: mav-diag
    python3 tools/mav_tools.py mav-diag        # one-shot counter snapshot
    python3 tools/mav_tools.py mav-diag --watch --interval 1  # repeat every N s

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args


def cmd_mav_diag_once(ser) -> bool:
    ser.reset_input_buffer()
    ser.write(b"MAV,diag\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        m = re.match(
            r"MAV,DIAG,bytes_rx=(\d+),frames_ok=(\d+),frames_bad_crc=(\d+),"
            r"heartbeat_rx=(\d+),param_req_rx=(\d+),vision_pos_rx=(\d+),"
            r"vision_speed_rx=(\d+),unknown_rx=(\d+)", line)
        if m:
            (bytes_rx, frames_ok, frames_bad_crc, heartbeat_rx, param_req_rx,
             vision_pos_rx, vision_speed_rx, unknown_rx) = (int(g) for g in m.groups())

            console.print("[bold]MAVLink (SD3/TELEM2) RX counters[/bold]")
            console.print(f"  bytes_rx        = {bytes_rx}")
            console.print(f"  frames_ok       = {frames_ok}")
            console.print(f"  frames_bad_crc  = {frames_bad_crc}")
            console.print(f"  heartbeat_rx    = {heartbeat_rx}")
            console.print(f"  param_req_rx    = {param_req_rx}")
            console.print(f"  vision_pos_rx   = {vision_pos_rx}")
            console.print(f"  vision_speed_rx = {vision_speed_rx}")
            console.print(f"  unknown_rx      = {unknown_rx}")
            console.print()
            if bytes_rx == 0:
                console.print("[red]bytes_rx stuck at 0 → nothing is reaching SD3 at all. "
                              "Check the radio link / MAVProxy master port, not this firmware.[/red]")
            elif frames_bad_crc > 0 and frames_ok == 0:
                console.print("[red]Bytes arriving but every frame fails CRC → dialect/CRC_EXTRA "
                              "mismatch between the sender and third_party/mavlink here.[/red]")
            elif heartbeat_rx > 0 and vision_pos_rx == 0 and vision_speed_rx == 0:
                console.print("[yellow]Heartbeats parsing fine but no VISION_POSITION_ESTIMATE/"
                              "VISION_SPEED_ESTIMATE seen — check the mocap bridge is actually "
                              "sending them (Motive streaming? mocap_callback firing?).[/yellow]")
            elif vision_pos_rx > 0:
                console.print("[green]Vision data is reaching the FC — g_mocap.valid should be true. "
                              "Check LOCAL_POSITION_NED in your GCS to confirm end-to-end.[/green]")
            return True
    console.print("[red]No MAV,DIAG response — firmware may not support this command "
                  "(rebuild/reflash) or nothing is connected on USB.[/red]")
    return False


def cmd_mav_diag(ser, args):
    if not getattr(args, "watch", False):
        cmd_mav_diag_once(ser)
        return
    interval = getattr(args, "interval", 1.0)
    try:
        while True:
            cmd_mav_diag_once(ser)
            console.print(f"[dim]-- refreshing every {interval}s, Ctrl+C to stop --[/dim]\n")
            time.sleep(interval)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(
        description="BPRL MAVLink tools — TELEM2/SD3 RX diagnostics")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    diag_p = sub.add_parser("mav-diag", help="Read MAVLinkThread RX counters (bytes/frames/per-message)")
    diag_p.add_argument("--watch", action="store_true", help="Repeat until Ctrl+C")
    diag_p.add_argument("--interval", type=float, default=1.0, help="Seconds between refreshes with --watch")

    args = parser.parse_args()
    if args.command is None:
        args.command = "mav-diag"

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "mav-diag":
            cmd_mav_diag(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
