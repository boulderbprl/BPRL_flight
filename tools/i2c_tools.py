#!/usr/bin/env python3
"""
BPRL I2C Tools — scan the I2C bus for responding devices.

Sends I2C,scan to the firmware, which probes every valid address (0x08–0x77)
and reports which ones ACK. Works like i2cdetect -y.

Works on any firmware build with I2C enabled.

Usage:
    python3 tools/i2c_tools.py              # default: i2c-scan
    python3 tools/i2c_tools.py i2c-scan

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time

from bprl_common import console, open_port, add_port_args

from rich.table import Table
from rich.text import Text

# Known I2C device addresses for helpful annotation
_KNOWN_DEVICES = {
    0x11: "Strain Rate Sensor",
    0x1E: "HMC5883 Magnetometer",
    0x1C: "HMC5883 Magnetometer (alt)",
    0x3C: "SSD1306 OLED / 0x3C",
    0x3D: "SSD1306 OLED / 0x3D",
    0x40: "INA219 Power Monitor",
    0x48: "ADS1115 ADC",
    0x68: "MPU-6050 / DS1307 RTC",
    0x69: "MPU-6050 (alt addr)",
    0x76: "MS5611 / BMP280 Baro",
    0x77: "BMP180 / BMP280 Baro (alt)",
}


def cmd_i2c_scan(ser, _args):
    ser.reset_input_buffer()
    ser.write(b"I2C,scan\r\n")

    found = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line == "I2C,SCAN,END":
            break
        m = re.match(r"I2C,SCAN,(0x[0-9a-fA-F]{2}),ack", line)
        if m:
            found.append(int(m.group(1), 16))
    else:
        console.print("[red]Timeout waiting for I2C,SCAN,END — is firmware running and I2C enabled?[/red]")
        return

    # ── i2cdetect-style address grid ─────────────────────────────────────────
    found_set = set(found)
    grid = Table(show_header=True, header_style="bold dim", box=None, padding=(0, 1))
    grid.add_column("   ", style="dim", min_width=4)
    for col in range(16):
        grid.add_column(f"+{col:x}", justify="right", min_width=4)

    for row in range(8):
        base = row * 16
        cells = [f"{base:02x}:"]
        for col in range(16):
            addr = base + col
            if addr < 0x08 or addr > 0x77:
                cells.append("[dim]  --[/dim]")
            elif addr in found_set:
                cells.append(f"[bold green]{addr:02x}[/bold green]")
            else:
                cells.append("[dim]  .[/dim]")
        grid.add_row(*cells)

    title = f"I2C Bus Scan — {len(found)} device{'s' if len(found) != 1 else ''} found"
    console.print()
    console.rule(title)
    console.print(grid)

    if not found:
        console.print("\n[yellow]No devices found. Check wiring and pull-up resistors.[/yellow]")
        return

    # ── Identified devices table ──────────────────────────────────────────────
    tbl = Table(show_header=True, header_style="bold")
    tbl.add_column("Address", style="cyan", min_width=10)
    tbl.add_column("Device", min_width=30)

    for addr in sorted(found):
        name = _KNOWN_DEVICES.get(addr, "[dim]Unknown[/dim]")
        tbl.add_row(f"0x{addr:02x}", name)

    console.print()
    console.print(tbl)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL I2C tools — scan bus for responding devices")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("i2c-scan", help="Probe all I2C addresses and display a grid of responding devices")

    args = parser.parse_args()
    if args.command is None:
        args.command = "i2c-scan"

    ser = open_port(args.port, args.baud)
    try:
        cmd_i2c_scan(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
