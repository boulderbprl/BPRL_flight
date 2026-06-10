#!/usr/bin/env python3
"""
BPRL Log Tools — SD card log access and offline decoder.

Works on any firmware build.

Usage:
    python3 tools/logs.py logs list              # list log files on SD card
    python3 tools/logs.py logs download [FILE]   # download a log file (default: latest completed)
    python3 tools/logs.py logs decode FILE.bin   # decode binary log to CSV
    python3 tools/logs.py logs erase             # erase completed log files
    python3 tools/logs.py log-status             # query SD card logger status

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import struct
import time
from pathlib import Path
from typing import Optional

from bprl_common import (
    console, open_port, add_port_args, send_cmd,
    _read_line, _read_line_ex,
)

from rich.progress import Progress, BarColumn, TransferSpeedColumn, DownloadColumn
from rich.table import Table


# ── Log list ──────────────────────────────────────────────────────────────────

def cmd_logs_list(ser, _args):
    ser.write(b"LOG,list\n")
    ser.flush()
    files = []
    while True:
        line = _read_line(ser, timeout=5.0)
        if line is None:
            console.print("[red]Timeout waiting for log list.")
            break
        if line.startswith("LOG,ERR,"):
            console.print(f"[red]Error: {line}")
            break
        if line == "LOG,LIST,END":
            break
        if line.startswith("LOG,FILE,"):
            parts = line.split(",")
            if len(parts) >= 4:
                files.append((parts[2], int(parts[3])))

    if not files:
        console.print("[yellow]No log files found (or no SD card).")
        return

    table = Table(title="SD Card Log Files", show_header=True)
    table.add_column("Filename", style="cyan")
    table.add_column("Size", justify="right")
    for fname, fsize in files:
        table.add_row(fname, f"{fsize / 1_048_576:.1f} MB  ({fsize:,} B)")
    console.print(table)
    console.print(f"  [dim]Latest: {files[-1][0]}")


# ── Log download ──────────────────────────────────────────────────────────────

def cmd_logs_download(ser, args):
    fname = args.file
    if fname is None:
        ser.write(b"LOG,list\n")
        ser.flush()
        files = []
        while True:
            line = _read_line(ser, timeout=5.0)
            if line is None or line == "LOG,LIST,END":
                break
            if line.startswith("LOG,FILE,"):
                parts = line.split(",")
                if len(parts) >= 4:
                    files.append(parts[2])
        if not files:
            console.print("[red]No log files found.")
            return
        # Highest-numbered file is still being written; take the previous one.
        fname = files[-2] if len(files) >= 2 else files[-1]
        console.print(f"[dim]Downloading latest completed log: {fname}")

    ser.write(f"LOG,get,{fname}\n".encode())
    ser.flush()

    # Skip any $TEL lines while waiting for the firmware's LOG,SIZE response.
    size_line = None
    leftover  = b""
    deadline  = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line, leftover = _read_line_ex(ser, timeout=deadline - time.monotonic())
        if line is None:
            break
        if line.startswith("LOG,SIZE,") or line.startswith("LOG,ERR,"):
            size_line = line
            break

    if size_line is None:
        console.print("[red]Timeout waiting for LOG,SIZE response.")
        return
    if size_line.startswith("LOG,ERR,"):
        console.print(f"[red]Error: {size_line}")
        return

    total    = int(size_line.split(",")[2])
    out_path = Path(fname)
    console.print(f"Downloading {fname} ({total / 1_048_576:.1f} MB) → {out_path}")

    with Progress(
        "[progress.description]{task.description}",
        BarColumn(),
        DownloadColumn(),
        TransferSpeedColumn(),
        console=console,
    ) as progress:
        task     = progress.add_task(fname, total=total)
        received = 0
        with open(out_path, "wb") as f:
            if leftover:
                f.write(leftover)
                received += len(leftover)
                progress.update(task, advance=len(leftover))
            ser.timeout = 2.0
            while received < total:
                want  = min(2048, total - received)
                chunk = ser.read(want)
                if not chunk:
                    console.print(f"\n[red]Timeout: received {received}/{total} bytes.")
                    break
                f.write(chunk)
                received += len(chunk)
                progress.update(task, advance=len(chunk))
        ser.timeout = 0.1

    if received == total:
        console.print(f"[green]Saved {out_path} ({received:,} bytes).")
        if getattr(args, "decode", False):
            _decode_log(out_path)
    else:
        console.print(f"[yellow]Partial download: {received}/{total} bytes.")


# ── Log erase ─────────────────────────────────────────────────────────────────

def cmd_logs_erase(ser, _args):
    answer = input("Erase all completed log files from SD card? [y/N]: ").strip().lower()
    if answer != "y":
        console.print("[dim]Aborted.")
        return
    ser.write(b"LOG,erase\n")
    ser.flush()
    deadline = time.monotonic() + 10.0
    line = None
    while time.monotonic() < deadline:
        candidate = _read_line(ser, timeout=deadline - time.monotonic())
        if candidate is None:
            break
        if candidate.startswith("LOG,ERASED,") or candidate.startswith("LOG,ERR,"):
            line = candidate
            break
    if line is None:
        console.print("[red]Timeout waiting for erase response.")
    elif line.startswith("LOG,ERASED,"):
        console.print(f"[green]Erased {int(line.split(',')[2])} file(s).")
    elif line.startswith("LOG,ERR,"):
        console.print(f"[red]Error: {line}")
    else:
        console.print(f"[yellow]{line}")


# ── Log status ────────────────────────────────────────────────────────────────

def cmd_log_status(ser, _args):
    ser.reset_input_buffer()
    send_cmd(ser, "LOG,status")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        resp = _read_line(ser, timeout=deadline - time.monotonic())
        if resp is None:
            console.print("(no response)")
            return
        if resp.startswith("LOG,STATUS"):
            console.print(resp)
            return


# ── Binary log decoder (offline — no serial needed) ───────────────────────────

LOG_MSG_FMT   = 0x80
LOG_MSG_IMU   = 0x01
LOG_MSG_STATE = 0x02

_IMU_FMT   = "<I6fB6fB6fB7fB"
_STATE_FMT = "<I10fB"

_IMU_FIELDS = [
    "TimeMS",
    "AX0","AY0","AZ0","GX0","GY0","GZ0","V0",
    "AX1","AY1","AZ1","GX1","GY1","GZ1","V1",
    "AX2","AY2","AZ2","GX2","GY2","GZ2","V2",
    "QW","QX","QY","QZ","CanP","CanQ","CanR","CV",
]
_STATE_FIELDS = [
    "TimeMS","Roll","Pitch","Yaw","P","Q","R",
    "ZPos","ZVel","ZAcc","Thr","Armed",
]

_FMT_BODY   = 71
_IMU_BODY   = struct.calcsize(_IMU_FMT)
_STATE_BODY = struct.calcsize(_STATE_FMT)

_BODY_SIZES = {
    LOG_MSG_FMT:   _FMT_BODY,
    LOG_MSG_IMU:   _IMU_BODY,
    LOG_MSG_STATE: _STATE_BODY,
}


def _decode_log(src: Path, out_dir: Optional[Path] = None) -> None:
    if not src.exists():
        console.print(f"[red]File not found: {src}")
        return

    dest       = out_dir if out_dir is not None else Path.cwd()
    stem       = src.stem
    imu_path   = dest / f"{stem}_imu.csv"
    state_path = dest / f"{stem}_state.csv"

    imu_rows   = []
    state_rows = []
    skipped    = 0
    data       = src.read_bytes()
    pos        = 0
    n          = len(data)

    while pos < n - 3:
        if data[pos] != 0xA3 or data[pos + 1] != 0x95:
            pos     += 1
            skipped += 1
            continue

        msg_id   = data[pos + 2]
        body_len = _BODY_SIZES.get(msg_id)
        if body_len is None:
            pos += 1
            continue

        start = pos + 3
        end   = start + body_len
        if end > n:
            break

        body = data[start:end]
        pos  = end

        if msg_id == LOG_MSG_IMU:
            try:
                imu_rows.append(struct.unpack(_IMU_FMT, body))
            except struct.error:
                pass
        elif msg_id == LOG_MSG_STATE:
            try:
                state_rows.append(struct.unpack(_STATE_FMT, body))
            except struct.error:
                pass

    import csv
    with open(imu_path, "w", newline="") as f:
        csv.writer(f).writerows([_IMU_FIELDS] + list(imu_rows))
    console.print(f"[green]Decoded {len(imu_rows):,} IMU records   → {imu_path}")

    with open(state_path, "w", newline="") as f:
        csv.writer(f).writerows([_STATE_FIELDS] + list(state_rows))
    console.print(f"[green]Decoded {len(state_rows):,} STATE records → {state_path}")

    if skipped:
        console.print(f"[dim]Skipped {skipped} non-sync bytes.")


def cmd_logs_decode(_, args):
    out_dir = Path(args.out_dir) if args.out_dir else None
    _decode_log(Path(args.decode_file), out_dir)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL log tools — SD card access and offline binary decoder")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    logs_p   = sub.add_parser("logs", help="SD card log commands")
    logs_sub = logs_p.add_subparsers(dest="logs_cmd", required=True)

    logs_sub.add_parser("list", help="List log files on SD card")

    dl_p = logs_sub.add_parser("download", help="Download a log file")
    dl_p.add_argument("file", nargs="?", default=None,
                      help="Filename (default: latest completed log)")
    dl_p.add_argument("--decode", action="store_true",
                      help="Decode the log to CSV after download")

    dec_p = logs_sub.add_parser("decode", help="Decode a local binary log to CSV")
    dec_p.add_argument("decode_file", metavar="FILE.bin")
    dec_p.add_argument("--out-dir", default=None, metavar="DIR",
                       help="Output directory (default: current directory)")

    logs_sub.add_parser("erase", help="Erase completed log files from SD card")

    sub.add_parser("log-status", help="Query SD card logger status")

    args = parser.parse_args()

    # decode is offline — no serial needed
    if args.command == "logs" and args.logs_cmd == "decode":
        cmd_logs_decode(None, args)
        return

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "log-status":
            cmd_log_status(ser, args)
        elif args.command == "logs":
            if args.logs_cmd == "list":
                cmd_logs_list(ser, args)
            elif args.logs_cmd == "download":
                cmd_logs_download(ser, args)
            elif args.logs_cmd == "erase":
                cmd_logs_erase(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
