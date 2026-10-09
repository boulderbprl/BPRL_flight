#!/usr/bin/env python3
"""
BPRL trajectory commands — send a point / circle / origin / stop command to
the flight controller over MAVLink and print its answer.

Unlike the other tools this does not use the USB port: it speaks MAVLink to
the FC's telemetry UART, so point --master at whatever reaches that link
(a MAVProxy --out UDP port, or the serial adapter itself).

    python3 tools/traj_cmd.py --master udp:127.0.0.1:14551 origin
    python3 tools/traj_cmd.py --master udp:127.0.0.1:14551 point --n 1.0
    python3 tools/traj_cmd.py --master udp:127.0.0.1:14551 circle 1.0 --speed 0.5
    python3 tools/traj_cmd.py --master udp:127.0.0.1:14551 circle 0.75 --focus 0.5 --dir -1
    python3 tools/traj_cmd.py --master /dev/ttyUSB0,115200 stop

All positions are NED offsets in metres from the origin (D positive down, so
1 m above the origin is --d -1). Anything left out is sent as NaN = "not
given" and takes the firmware's default: N/E 0, D the current altitude, a
plain circle, positive direction (clockwise seen from above), default speed.
See src/controllers/TrajectoryTracker.hpp and src/coms/MAVLink.hpp.

The vehicle only acts on point/circle while flying in POS_HOLD with a valid
mocap position and an origin set; moving the roll, pitch or yaw stick cancels.

Requires: pip install pymavlink
"""

import argparse
import sys
import time

from pymavlink import mavutil

CMD_SET_ORIGIN, CMD_POINT, CMD_CIRCLE, CMD_STOP = 31010, 31011, 31012, 31013   # MAV_CMD_USER_1..4
NAN = float("nan")

RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED — not now: origin not set, not flying in POS_HOLD, no valid "
       "position, or a trajectory is active (set-origin)",
    2: "DENIED — bad parameters (radius missing or out of range, offset too large, "
       "or altitude below the take-off floor)",
    3: "UNSUPPORTED — firmware does not know this command (reflash?)",
    4: "FAILED — the control loop did not pick the command up (motor test running?)",
}


def opt(value):
    return NAN if value is None else value


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--master", required=True, help="pymavlink connection string, e.g. udp:127.0.0.1:14551 or /dev/ttyUSB0,115200")
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds to wait for the heartbeat and for the answer")
    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("origin", help="set the origin to where the vehicle is now")
    sub.add_parser("stop", help="cancel the trajectory; the vehicle holds position")

    p = sub.add_parser("point", help="fly to a point and hold")
    p.add_argument("--n", type=float, help="North offset from the origin [m] (default 0)")
    p.add_argument("--e", type=float, help="East offset [m] (default 0)")
    p.add_argument("--d", type=float, help="Down offset [m], negative = above the origin (default: current altitude)")

    c = sub.add_parser("circle", help="fly a circle or ellipse, nose along the path")
    c.add_argument("radius", type=float, help="radius [m]; for an ellipse, focus-to-near-end distance")
    c.add_argument("--n", type=float, help="centre North offset from the origin [m] (default 0)")
    c.add_argument("--e", type=float, help="centre East offset [m] (default 0)")
    c.add_argument("--d", type=float, help="Down offset [m], negative = above the origin (default: current altitude)")
    c.add_argument("--focus", type=float, help="ellipse focus distance from the centre [m]: >0 long axis North, <0 East (default 0 = circle)")
    c.add_argument("--dir", type=float, help="+1 clockwise seen from above (default), -1 counter-clockwise")
    c.add_argument("--speed", type=float, help="speed [m/s] (default: firmware's V_DEFAULT)")

    args = ap.parse_args()

    if args.command == "origin":
        cmd, params = CMD_SET_ORIGIN, [0.0] * 7
    elif args.command == "stop":
        cmd, params = CMD_STOP, [0.0] * 7
    elif args.command == "point":
        cmd, params = CMD_POINT, [opt(args.n), opt(args.e), opt(args.d), NAN, NAN, NAN, NAN]
    else:
        cmd, params = CMD_CIRCLE, [args.radius, opt(args.n), opt(args.e), opt(args.d),
                                   opt(args.focus), opt(args.dir), opt(args.speed)]

    link = mavutil.mavlink_connection(args.master, source_system=255)
    if link.wait_heartbeat(timeout=args.timeout) is None:
        print(f"No heartbeat on {args.master} within {args.timeout} s — is the link up?")
        return 1

    link.mav.command_long_send(link.target_system, link.target_component, cmd, 0, *params)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=deadline - time.monotonic())
        if ack is not None and ack.command == cmd:
            print(f"{args.command}: {RESULTS.get(ack.result, f'result {ack.result}')}")
            return 0 if ack.result == 0 else 1
    print(f"{args.command}: no COMMAND_ACK within {args.timeout} s — command may not have arrived")
    return 1


if __name__ == "__main__":
    sys.exit(main())
