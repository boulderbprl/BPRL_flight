#!/usr/bin/env python3
"""
Plot one scenario CSV written by tools/sim/run.sh.

    python3 tools/sim/plot.py OUT_DIR/circle_r1_v05_cw.csv          # writes a .png next to the CSV
    python3 tools/sim/plot.py OUT_DIR/ellipse_north.csv --show      # interactive window instead

Four panels: the path seen from above, speed over the ground, heading, and
height. "Reference" is what the trajectory tracker asked for; it is only
drawn while a trajectory is active (traj_state != 0).
"""
import argparse
import csv
import math

import matplotlib

VEHICLE, REFERENCE = "#2a78d6", "#eb6834"   # fixed order: vehicle first, reference second
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def masked(values, active):
    return [v if a else math.nan for v, a in zip(values, active)]


def unwrap_deg(rad):
    out, offset, prev = [], 0.0, None
    for a in rad:
        if prev is not None and not math.isnan(a):
            if a - prev > math.pi:
                offset -= 2 * math.pi
            elif a - prev < -math.pi:
                offset += 2 * math.pi
        if not math.isnan(a):
            prev = a
        out.append(math.degrees(a + offset))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--show", action="store_true", help="open a window instead of writing a PNG")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = load(args.csv)
    t = d["t"]
    active = [s != 0 for s in d["traj_state"]]
    flying_hold = [not math.isnan(y) and y != 0.0 for y in d["yaw_tgt"]]

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
                         "grid.color": GRID, "axes.grid": True, "lines.linewidth": 2})
    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    name = args.csv.rsplit("/", 1)[-1].rsplit(".", 1)[0]
    fig.suptitle(name, x=0.01, ha="left", fontsize=12, fontweight="bold")

    def pair(a, veh, ref, title, ylabel, xlabel="time [s]", x_veh=t, x_ref=t):
        a.plot(x_veh, veh, color=VEHICLE, label="vehicle")
        a.plot(x_ref, ref, color=REFERENCE, linestyle=(0, (4, 2)), label="reference")
        a.set_title(title, loc="left")
        a.set_xlabel(xlabel)
        a.set_ylabel(ylabel)
        a.legend(frameon=False)

    # Path from above: East across, North up.
    pair(ax[0][0], d["n"], masked(d["ref_n"], active), "Path seen from above", "North [m]",
         xlabel="East [m]", x_veh=d["e"], x_ref=masked(d["ref_e"], active))
    ax[0][0].set_aspect("equal", adjustable="datalim")

    speed = [math.hypot(a, b) for a, b in zip(d["vn"], d["ve"])]
    pair(ax[0][1], speed, masked(d["path_speed"], active), "Speed over the ground", "m/s")

    pair(ax[1][0], unwrap_deg(d["yaw"]), unwrap_deg(masked(d["yaw_tgt"], flying_hold)),
         "Heading (reference = heading target in POS_HOLD)", "deg")

    pair(ax[1][1], [-v for v in d["d"]], [-v for v in masked(d["ref_d"], active)], "Height above the ground", "m")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if args.show:
        plt.show()
    else:
        out = args.csv.rsplit(".", 1)[0] + ".png"
        fig.savefig(out, dpi=110)
        print(out)


if __name__ == "__main__":
    main()
