#!/usr/bin/env python3
"""
BPRL Log Tools — SD card log access and offline decoder.

Works on any firmware build.

Usage:
    python3 tools/logs.py                        # default: list log files on SD card
    python3 tools/logs.py logs list              # list log files on SD card
    python3 tools/logs.py logs download [FILE]   # download a log file (default: latest completed)
    python3 tools/logs.py logs decode FILE.bin   # decode binary log to CSV
    python3 tools/logs.py logs check FILE.bin    # integrity report: corrupt/torn records, gaps, rates
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
    _read_line,
)

from rich.progress import Progress, BarColumn, TransferSpeedColumn, DownloadColumn
from rich.table import Table


# ── Log list ──────────────────────────────────────────────────────────────────

def _read_log_list(ser, timeout=5.0):
    """Read LOG,FILE lines until LOG,LIST,END or timeout. Returns list of (fname, size)."""
    files = []
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        while b"\n" in buf:
            idx  = buf.index(b"\n")
            line = buf[:idx].decode("ascii", errors="replace").strip()
            buf  = buf[idx + 1:]
            if not line:
                continue
            if line == "LOG,LIST,END":
                return files, None
            if line.startswith("LOG,ERR,"):
                return files, line
            if line.startswith("LOG,FILE,"):
                parts = line.split(",")
                if len(parts) >= 4:
                    files.append((parts[2], int(parts[3])))
        if not chunk:
            time.sleep(0.01)
    return files, "timeout"


def cmd_logs_list(ser, _args):
    ser.reset_input_buffer()
    ser.write(b"LOG,list\n")
    ser.flush()
    files, err = _read_log_list(ser)
    if err == "timeout":
        console.print("[red]Timeout waiting for log list.")
    elif err:
        console.print(f"[red]Error: {err}")

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
        ser.reset_input_buffer()
        ser.write(b"LOG,list\n")
        ser.flush()
        entries, err = _read_log_list(ser)
        if not entries:
            console.print("[red]No log files found.")
            return
        files = [e[0] for e in entries]
        # Highest-numbered file is still being written; take the previous one.
        fname = files[-2] if len(files) >= 2 else files[-1]
        console.print(f"[dim]Downloading latest completed log: {fname}")

    ser.reset_input_buffer()
    ser.write(f"LOG,get,{fname}\n".encode())
    ser.flush()

    # Read until LOG,SIZE (or LOG,ERR), buffering bytes so interleaved $TEL
    # lines are skipped without discarding any binary data that immediately
    # follows the SIZE header.
    size_line = None
    leftover  = b""
    deadline  = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            leftover += chunk
        while b"\n" in leftover:
            idx  = leftover.index(b"\n")
            line = leftover[:idx].decode("ascii", errors="replace").strip()
            leftover = leftover[idx + 1:]
            if line.startswith("LOG,SIZE,") or line.startswith("LOG,ERR,"):
                size_line = line
                break
        if size_line is not None:
            break
        if not chunk:
            time.sleep(0.01)

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
#
# Reads ArduPilot DataFlash format (.bin):
#   Header: N × FMT records (89 bytes each, [0xA3][0x95][0x80] + schema fields)
#   Body:   data records [0xA3][0x95][msg_id][packed body]
#
# FMT layout (89 bytes):
#   [sync1][sync2][0x80][type_u8][length_u8][name_4][format_16][labels_64]
#   length = 3 + body_size  (total record size including 3-byte data header)

LOG_MSG_FMT = 0x80
_FMT_HDR_SIZE = 89  # fixed size of every FMT record

# ArduPilot format code → Python struct code
_FMT_CODE = {
    'Q': 'Q', 'q': 'q',   # uint64 / int64
    'I': 'I', 'i': 'i',   # uint32 / int32
    'H': 'H', 'h': 'h',   # uint16 / int16
    'B': 'B', 'b': 'b',   # uint8  / int8
    'f': 'f', 'd': 'd',   # float32 / float64
    'n': '4s', 'N': '16s', 'Z': '64s',
}


def _fmt_to_struct(fmt_str: str) -> str:
    """Convert ArduPilot format string (e.g. 'QHffff') to Python struct format."""
    return '<' + ''.join(_FMT_CODE.get(c, '') for c in fmt_str if c and c != '\x00')


def _parse_log(data: bytes):
    """Parse a BPRL (DataFlash-format) log.

    Returns (schema, rows, stats). schema maps type_id -> (name, body_size,
    struct_fmt, labels); rows maps type_id -> list of decoded tuples; stats has
    'skipped' (non-sync bytes stepped over), 'first_skip' (offset of the first
    one, or None) and 'end' (offset just past the last good record).
    """
    n    = len(data)
    pos  = 0

    # Step 1: Read FMT records from the file header.
    # schema maps type_id → (name, body_size, struct_fmt, labels)
    schema = {}
    while pos + _FMT_HDR_SIZE <= n:
        if data[pos:pos+3] != b'\xa3\x95\x80':
            break
        type_id    = data[pos + 3]
        length     = data[pos + 4]           # total data record size including 3-byte header
        name       = data[pos+5:pos+9].decode('ascii', errors='replace').rstrip('\x00 ')
        fmt_str    = data[pos+9:pos+25].decode('ascii', errors='replace').rstrip('\x00')
        labels_raw = data[pos+25:pos+89].decode('ascii', errors='replace').rstrip('\x00')
        labels     = [l.strip() for l in labels_raw.split(',') if l.strip()]
        body_size  = length - 3
        schema[type_id] = (name, body_size, _fmt_to_struct(fmt_str), labels)
        pos += _FMT_HDR_SIZE

    # Step 2: Decode data records.
    rows: dict[int, list] = {t: [] for t in schema}
    skipped = 0
    first_skip = None
    end_good = pos
    while pos < n - 3:
        if data[pos] != 0xA3 or data[pos + 1] != 0x95:
            if first_skip is None:
                first_skip = pos
            pos     += 1
            skipped += 1
            continue
        msg_id = data[pos + 2]
        if msg_id == LOG_MSG_FMT:
            pos += _FMT_HDR_SIZE
            continue
        if msg_id not in schema:
            pos += 1
            continue
        _name, body_size, struct_fmt, _labels = schema[msg_id]
        start = pos + 3
        end   = start + body_size
        if end > n:
            break
        try:
            rows[msg_id].append(struct.unpack(struct_fmt, data[start:end]))
            end_good = end
        except struct.error:
            pass
        pos = end

    return schema, rows, {'skipped': skipped, 'first_skip': first_skip, 'end': end_good}


def _decode_log(src: Path, out_dir: Optional[Path] = None) -> None:
    if not src.exists():
        console.print(f"[red]File not found: {src}")
        return

    dest = out_dir if out_dir is not None else Path.cwd()
    schema, rows, stats = _parse_log(src.read_bytes())
    skipped = stats['skipped']
    if not schema:
        console.print("[red]No FMT records found — may be old log format (pre-UAV-Log-Viewer).")
        return

    # Step 3: Write one CSV per message type.
    import csv
    total_written = 0
    for type_id, (name, _bs, _sf, labels) in schema.items():
        if not rows[type_id]:
            continue
        out_path = dest / f"{src.stem}_{name.lower()}.csv"
        with open(out_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(labels)
            w.writerows(rows[type_id])
        console.print(f"[green]Decoded {len(rows[type_id]):,} {name} records → {out_path}")
        total_written += len(rows[type_id])

    if skipped:
        console.print(f"[dim]Skipped {skipped} non-sync bytes.")
    if not total_written:
        console.print("[red]No records decoded — check log format or file integrity.")


def cmd_logs_decode(_, args):
    out_dir = Path(args.out_dir) if args.out_dir else None
    _decode_log(Path(args.decode_file), out_dir)


# ── Log health check ──────────────────────────────────────────────────────────

def _check_log(src: Path) -> bool:
    """Integrity report for a local log. Returns False if something looks wrong."""
    import numpy as np
    if not src.exists():
        console.print(f"[red]File not found: {src}")
        return False
    data = src.read_bytes()
    schema, rows, st = _parse_log(data)
    if not schema:
        console.print("[red]No FMT records found — not a BPRL log, or the header is corrupt.")
        return False
    ok = True
    n = len(data)
    tail = data[st['end']:]
    console.print(f"[bold]{src}[/]  {n:,} bytes, records parse to byte {st['end']:,} "
                  f"({100.0 * st['end'] / n:.1f} %)")
    if st['skipped']:
        mid = st['first_skip'] is not None and st['first_skip'] < st['end']
        where = f"first at byte {st['first_skip']:,}" if st['first_skip'] is not None else ""
        console.print(f"  skipped {st['skipped']:,} non-sync bytes ({where})"
                      + ("  [yellow]— inside the recorded data: corrupt or torn records[/]" if mid else ""))
        ok &= not mid
    zeros = tail.count(0) == len(tail)
    console.print(f"  after the last record: {len(tail):,} bytes"
                  + (" (all zero — unused preallocated space)" if zeros and tail else
                     " (not zero — stale data in the preallocated file, normal; or unflushed records if the board"
                     " lost power mid-write)" if tail else ""))

    # Per message
    t_ref = None
    tbl = Table(show_header=True, header_style="bold")
    for c in ("msg", "records", "span [s]", "rate [Hz]", "dup t", "back t", "non-finite"):
        tbl.add_column(c, justify="right" if c != "msg" else "left")
    arrays = {}
    for tid, (name, _bs, _sf, labels) in schema.items():
        r = rows[tid]
        if not r:
            continue
        a = np.array([[float(x) if not isinstance(x, (bytes, str)) else np.nan for x in row] for row in r])
        arrays[name] = (a, labels)
        t = a[:, 0] / 1e6
        dt = np.diff(t)
        span = t[-1] - t[0] if len(t) > 1 else 0.0
        nonfin = int((~np.isfinite(a[:, 1:])).sum())
        back = int((dt < 0).sum())
        tbl.add_row(name, f"{len(r):,}", f"{span:.1f}", f"{(len(t) - 1) / span:.1f}" if span > 0 else "-",
                    str(int((dt == 0).sum())), f"[red]{back}[/]" if back else "0",
                    f"[yellow]{nonfin}[/]" if nonfin else "0")
        ok &= back == 0
        if t_ref is None or len(t) > len(t_ref[1]):
            t_ref = (name, t)        # the per-tick message with the most records (ATT on most logs)
    console.print(tbl)

    # Timing gaps on that message
    if t_ref is not None and len(t_ref[1]) > 10:
        ref_name, tu = t_ref[0], np.unique(t_ref[1])
        dt = np.diff(tu) * 1e3
        med = float(np.median(dt))
        big = dt > 1.5 * med
        console.print(f"  {ref_name}: median step {med:.1f} ms; steps > 1.5x median: {int(big.sum())}, "
                      f"> 50 ms: {int((dt > 50).sum())}, longest {dt.max():.0f} ms at t = {tu[np.argmax(dt)] - tu[0]:.1f} s")
        ok &= dt.max() < 250

    # Armed window
    if "RCIN" in arrays and "Armed" in arrays["RCIN"][1]:
        a, labels = arrays["RCIN"]
        arm = a[:, labels.index("Armed")] > 0.5
        t = a[:, 0] / 1e6 - a[0, 0] / 1e6
        edges = np.nonzero(np.diff(arm.astype(int)))[0]
        if arm.any():
            console.print(f"  armed {arm.mean() * 100:.0f} % of the log; arm/disarm at "
                          + ", ".join(f"{t[i + 1]:.1f} s" for i in edges))
        else:
            console.print("  never armed")
    console.print("[green]Looks healthy." if ok else "[yellow]Problems found — see above.")
    return ok


def cmd_logs_check(_, args):
    all_ok = True
    for f in args.check_files:
        all_ok &= _check_log(Path(f))
    raise SystemExit(0 if all_ok else 1)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL log tools — SD card access and offline binary decoder")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")

    logs_p   = sub.add_parser("logs", help="SD card log commands")
    logs_sub = logs_p.add_subparsers(dest="logs_cmd")

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

    chk_p = logs_sub.add_parser("check", help="Integrity report for local binary log(s)")
    chk_p.add_argument("check_files", metavar="FILE.bin", nargs="+")

    logs_sub.add_parser("erase", help="Erase completed log files from SD card")

    sub.add_parser("log-status", help="Query SD card logger status")

    args = parser.parse_args()

    # Default: list log files on SD card
    if args.command is None:
        args.command = "logs"
    if args.command == "logs" and getattr(args, "logs_cmd", None) is None:
        args.logs_cmd = "list"

    # decode is offline — no serial needed
    if args.command == "logs" and args.logs_cmd == "decode":
        cmd_logs_decode(None, args)
        return
    if args.command == "logs" and args.logs_cmd == "check":
        cmd_logs_check(None, args)
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
