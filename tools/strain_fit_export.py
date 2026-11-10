#!/usr/bin/env python3
"""
BPRL Strain Fit Export — turns the MATLAB strain -> p_dot/q_dot weights into
the firmware's static table.

Reads the weights .mat saved by gust_calibration.m (gust_fit_A.mat / gust_fit_B.mat /
gust_fit_weights.mat — strain, optional signed-square strain inputs s|s|/scale (use_sq)
and optional motor-RPM inputs), strain_fitting_new.m (strain_fitting_new_weights.mat)
or the older strain_pdot_pipeline.m (strain_pdot_weights.mat) in the strain
calibration folder's weights/ folder and writes src/sensors/StrainGaugeFit.hpp, which
StrainAngAccel (src/sensors/StrainGauge.cpp) applies every control tick.
Re-run it, rebuild and reflash after every refit.

Separate pitch fit (--pitch): p_dot comes from the first .mat, q_dot from the
second. The pitch fit may use its own strain high-pass, signed-square scales and
RPM use (it must share the gauge groups/gains, taps, tap spacing and low-pass);
the firmware then runs a second high-pass/low-pass block on the 24 group
signals that only q_dot reads (STRAIN_FIT_Q_SEPARATE 1). The pitch fit may not
use RPM taps the roll fit doesn't have.

Usage:
    python3 tools/strain_fit_export.py path/to/gust_fit_weights.mat
    python3 tools/strain_fit_export.py W.mat --out src/sensors/StrainGaugeFit.hpp
    python3 tools/strain_fit_export.py roll_W.mat --pitch pitch_W.mat

Needs scipy (scipy.io.loadmat).
"""

import argparse
from pathlib import Path

import numpy as np
import scipy.io as sio

N_GROUPS   = 12   # 4 arms x 3 axes
MAX_MEMBER = 3    # gauges per arm/axis group (offsets 0, 3, 6)
N_MOTORS   = 4    # RPM inputs RPM0..RPM3 = RPMS log order = the firmware's logical rpm[] order


def node_of(ch: int) -> int:
    # Gauge node along the arm, as group_signals() in strain_fitting_new.m
    return (ch % 9) // 3


def f(v: float) -> str:
    return f"{float(v): .9e}f"


def load_fit(path: Path) -> dict:
    """Read one weights .mat and check its input layout against what the firmware computes."""
    W = sio.loadmat(path, squeeze_me=True, struct_as_record=False)["W"]
    n_taps = int(W.n_taps)
    groups = list(np.atleast_1d(W.groups))
    assert len(groups) == N_GROUPS, f"{path.name}: expected {N_GROUPS} groups, got {len(groups)}"

    # Fits the firmware can't run: ATT.P/Q inputs, a complementary filter, taps
    # further apart than the grid.
    assert not bool(getattr(W, "use_att_pq", False)), f"{path.name}: fit uses ATT.P/Q inputs - firmware has no such path"
    assert not bool(getattr(W, "use_comp", False)), f"{path.name}: fit uses the complementary filter - firmware has no such path"
    assert int(getattr(W, "tap_stride", 1)) == 1, f"{path.name}: tap_stride != 1 - firmware taps are STRAIN_FIT_TAP_DT_S apart"

    # Inputs are [12 means, 12 gradients] per tap (+ with use_sq [12 means_sq,
    # 12 gradients_sq] = s|s|/sq_scale of the same signals), taps k, k-1, ...,
    # then (use_rpm) [RPM0..RPM3] per RPM tap — check the names rather than
    # trusting the layout.
    names = list(np.atleast_1d(W.inputs))
    use_sq = bool(getattr(W, "use_sq", False))
    n_base = 2 * N_GROUPS
    n_sig = n_base * (2 if use_sq else 1)
    if use_sq:
        sq_scale = np.asarray(W.sq_scale, dtype=float).ravel()
        assert sq_scale.size == n_base and np.all(sq_scale > 0), f"{path.name}: sq_scale must be {n_base} positive values"
    else:
        sq_scale = np.ones(n_base)
    use_rpm    = bool(getattr(W, "use_rpm", False))
    rpm_taps   = int(W.rpm_taps) if use_rpm else 0
    rpm_square = bool(getattr(W, "rpm_square", False)) if use_rpm else False
    rpm_scale  = float(getattr(W, "rpm_scale", 1e4))
    n_strain   = n_sig * n_taps
    assert len(names) == n_strain + N_MOTORS * rpm_taps, \
        f"{path.name}: input count {len(names)} doesn't match {n_taps} x {n_sig} strain + {rpm_taps} x 4 RPM"
    for j in range(n_taps):
        for k, g in enumerate(groups):
            assert names[j * n_sig + k] == f"{g.name}_mean[k-{j}]", names[j * n_sig + k]
            assert names[j * n_sig + N_GROUPS + k] == f"{g.name}_grad[k-{j}]"
            if use_sq:
                assert names[j * n_sig + n_base + k] == f"{g.name}_mean_sq[k-{j}]", names[j * n_sig + n_base + k]
                assert names[j * n_sig + n_base + N_GROUPS + k] == f"{g.name}_grad_sq[k-{j}]"
    for j in range(rpm_taps):
        for m in range(N_MOTORS):
            assert names[n_strain + j * N_MOTORS + m] == f"RPM{m}[k-{j}]", names[n_strain + j * N_MOTORS + m]

    def axis(ax):
        M = getattr(W, ax)
        w_all = np.asarray(M.w, dtype=float).ravel()
        w = w_all[:n_strain].reshape(n_taps, n_sig)
        w_rpm = w_all[n_strain:].reshape(rpm_taps, N_MOTORS) if rpm_taps else np.zeros((0, N_MOTORS))
        return dict(b=float(M.b), w=w, w_rpm=w_rpm, R2=float(M.R2))

    return dict(path=path, W=W, n_taps=n_taps, dt=float(W.dt), groups=groups, use_sq=use_sq, n_sig=n_sig,
                sq_scale=sq_scale, hp=float(W.hp_hz), lp=float(W.fc_strain), use_rpm=use_rpm, rpm_taps=rpm_taps,
                rpm_square=rpm_square, rpm_scale=rpm_scale, dead=[int(d) for d in np.atleast_1d(W.dead)],
                p_dot=axis("p_dot"), q_dot=axis("q_dot"))


def same_groups(a: dict, b: dict) -> bool:
    for ga, gb in zip(a["groups"], b["groups"]):
        ma, mb = np.atleast_1d(ga.members).astype(int), np.atleast_1d(gb.members).astype(int)
        if ga.name != gb.name or not np.array_equal(ma, mb):
            return False
        if ma.size and not np.allclose(np.atleast_1d(ga.gain).astype(float), np.atleast_1d(gb.gain).astype(float), rtol=0, atol=1e-12):
            return False
    return True


def describe(F: dict) -> str:
    W = F["W"]
    s = getattr(W, "fit_name", "") or F["path"].name
    return (f"{s}; hp {F['hp']:g} Hz, {'signed squares, ' if F['use_sq'] else ''}"
            f"{F['rpm_taps']} RPM taps, gust rows x{float(getattr(W, 'gust_weight', 1)):g}, onset "
            f"'{getattr(W, 'onset_def', 'gt')}' x{float(getattr(W, 'onset_weight', 1)):g}, "
            f"mu_hf {float(getattr(W, 'mu_hf', 0)):g}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mat", type=Path, help="weights .mat (p_dot, and q_dot unless --pitch is given)")
    ap.add_argument("--pitch", type=Path, default=None,
                    help="separate pitch-fit weights .mat: its q_dot replaces the first .mat's q_dot")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).resolve().parent.parent / "src/sensors/StrainGaugeFit.hpp")
    args = ap.parse_args()

    R = load_fit(args.mat)
    Q = load_fit(args.pitch) if args.pitch else R
    sep = args.pitch is not None
    if sep:
        # What the two blocks share in firmware: the group combination, the tap ring and the low-pass.
        assert same_groups(R, Q), "pitch fit uses different gauge groups / gains than the roll fit"
        assert R["n_taps"] == Q["n_taps"], "pitch fit tap count differs from the roll fit"
        assert R["dt"] == Q["dt"], "pitch fit tap spacing differs from the roll fit"
        assert R["lp"] == Q["lp"], "pitch fit low-pass differs from the roll fit"
        assert R["dead"] == Q["dead"], "pitch fit dead-channel list differs from the roll fit"
        if Q["rpm_taps"]:
            assert (Q["rpm_taps"] <= R["rpm_taps"] and Q["rpm_scale"] == R["rpm_scale"]
                    and Q["rpm_square"] == R["rpm_square"]), "pitch fit RPM inputs don't match the roll fit's"

    n_taps, rpm_taps = R["n_taps"], R["rpm_taps"]
    n_base = 2 * N_GROUPS
    W = R["W"]

    lines = []
    lines.append("#pragma once")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("/*")
    lines.append(" * GENERATED by tools/strain_fit_export.py — do not edit by hand. Refit in")
    lines.append(" * gust_calibration.m (or strain_fitting_new.m), then re-run the export.")
    lines.append(" *")
    if sep:
        lines.append(f" * Roll (p_dot) source: {args.mat.name}, trained on {W.train_flight}")
        lines.append(f" * Pitch (q_dot) source: {args.pitch.name}, trained on {Q['W'].train_flight}")
    else:
        lines.append(f" * Source: {args.mat.name}, trained on {W.train_flight}")
    lines.append(f" * R^2 on the fit flights (stacked): p_dot {R['p_dot']['R2']:.3f}, q_dot {Q['q_dot']['R2']:.3f}")
    dead = ", ".join(f"D{d}" for d in R["dead"])
    lines.append(f" * Channels left out (dead when fitted): {dead or 'none'}")
    lines.append(" *")
    if hasattr(W, "az"):
        lines.append(f" * (a_z model also in the .mat, R^2 {float(W.az.R2):.3f} — not exported, firmware has no a_z path)")
        lines.append(" *")
    lines.append(" * Inputs, per tap: 12 arm/axis group means, then the 12 gradients, in")
    lines.append(" * kStrainFitGroups order. Tap j is the signal STRAIN_FIT_TAP_DT_S * j ago.")
    lines.append(" * Weights act on the filtered group signals in raw counts.")
    lines.append(" * With signed squares (USE_SQ) the same 24 filtered signals follow as")
    lines.append(" * s * |s| * inv_scale[i] (fixed per-signal scale = the in-air std at fit time).")
    if sep:
        lines.append(" * Pitch has its own block (STRAIN_FIT_Q_SEPARATE): the same 24 group signals")
        lines.append(" * through STRAIN_FIT_Q_HPF_HZ + the shared low-pass, its own squares; only q_dot")
        lines.append(" * reads it. Roll's block is unchanged.")
    lines.append(f" * Roll fit:  {describe(R)}.")
    if sep:
        lines.append(f" * Pitch fit: {describe(Q)}.")
    if rpm_taps:
        lines.append(" *")
        lines.append(f" * Plus motor RPM: per RPM tap, |rpm[0..3]| x STRAIN_FIT_RPM_SCALE"
                     f"{' squared' if R['rpm_square'] else ''}, unfiltered,")
        lines.append(" * logical motor order (= RPMS.RPM0..3)."
                     + (" Pitch uses none (kStrainFitRpmWQ all 0)." if sep and not Q["rpm_taps"] else ""))
    lines.append(" */")
    lines.append("")
    lines.append(f"#define STRAIN_FIT_N_TAPS     {n_taps}")
    lines.append(f"#define STRAIN_FIT_TAP_DT_S   {R['dt']!r}f   // tap spacing the fit used [s]")
    lines.append(f"#define STRAIN_FIT_N_GROUPS   {N_GROUPS}")
    lines.append(f"#define STRAIN_FIT_USE_SQ     {int(R['use_sq'])}      // 1 = signed-square inputs follow the 24 signals in every tap")
    lines.append(f"#define STRAIN_FIT_N_SIG      {R['n_sig']}     // inputs per tap = 2 x N_GROUPS x (1 + USE_SQ)")
    lines.append(f"#define STRAIN_FIT_HPF_HZ     {R['hp']!r}f    // high-pass the fit was made with (1st order)")
    lines.append(f"#define STRAIN_FIT_LPF_HZ     {R['lp']!r}f   // low-pass the fit was made with (single pole only)")
    lines.append(f"#define STRAIN_FIT_RPM_TAPS   {rpm_taps}      // motor-RPM taps (0 = strain-only fit)")
    lines.append(f"#define STRAIN_FIT_RPM_SCALE  {f(1.0 / R['rpm_scale'])}   // RPM input = |rpm| x this")
    lines.append(f"#define STRAIN_FIT_RPM_SQUARE {int(R['rpm_square'])}      // 1 = input is (|rpm| x scale)^2")
    lines.append("// Pitch block: 1 = q_dot reads its own filtered signals (below), 0 = the shared ones above")
    lines.append(f"#define STRAIN_FIT_Q_SEPARATE {int(sep)}")
    lines.append(f"#define STRAIN_FIT_Q_HPF_HZ   {Q['hp']!r}f    // pitch-block high-pass (= STRAIN_FIT_HPF_HZ when not separate)")
    lines.append(f"#define STRAIN_FIT_Q_USE_SQ   {int(Q['use_sq'])}")
    lines.append(f"#define STRAIN_FIT_Q_N_SIG    {Q['n_sig']}     // q_dot inputs per tap")
    lines.append("")
    lines.append("struct StrainFitGroup {")
    lines.append("    uint8_t n;                  // live gauges in this group (0 = group unused)")
    lines.append(f"    uint8_t ch[{MAX_MEMBER}];              // channel indices (StrainGaugeRaw::data), first .. last node")
    lines.append(f"    float   inv_gain[{MAX_MEMBER}];        // 1 / gauge gain relative to the first gauge")
    lines.append("    float   inv_steps;          // gradient = (first - last) * inv_steps; 0 if n < 2")
    lines.append("};")
    lines.append("")
    lines.append(f"static constexpr StrainFitGroup kStrainFitGroups[STRAIN_FIT_N_GROUPS] = {{")
    for g in R["groups"]:
        mem  = [int(m) for m in np.atleast_1d(g.members)]
        gain = [float(x) for x in np.atleast_1d(g.gain)] if mem else []
        n = len(mem)
        inv_steps = 0.0
        if n >= 2:
            steps = node_of(mem[-1]) - node_of(mem[0])
            assert steps > 0, f"{g.name}: members {mem} on the same node"
            inv_steps = 1.0 / steps
        ch  = mem + [0] * (MAX_MEMBER - n)
        inv = [1.0 / x for x in gain] + [0.0] * (MAX_MEMBER - n)
        lines.append(f"    {{ {n}, {{ {', '.join(f'{c:2d}' for c in ch)} }}, "
                     f"{{ {', '.join(f(x) for x in inv)} }}, {f(inv_steps)} }},  // {g.name}")
    lines.append("};")

    def sq_table(name, F, what):
        lines.append("")
        lines.append(f"// 1 / scale of the {what} signed-square inputs, per signal (means, then gradients);"
                     + (" unused (1)" if not F["use_sq"] else ""))
        lines.append(f"static constexpr float {name}[2 * STRAIN_FIT_N_GROUPS] = {{")
        inv_sq = 1.0 / F["sq_scale"]
        for r in range(0, n_base, 4):
            lines.append("    " + ", ".join(f(x) for x in inv_sq[r:r + 4]) + ",")
        lines.append("};")
    sq_table("kStrainFitSqInvScale", R, "roll-block (shared when not separate)")
    sq_table("kStrainFitSqInvScaleQ", Q, "pitch-block")

    for ax, F, nsig_macro in (("p_dot", R, "STRAIN_FIT_N_SIG"), ("q_dot", Q, "STRAIN_FIT_Q_N_SIG")):
        M = F[ax]
        tag = "P" if ax == "p_dot" else "Q"
        w = M["w"]
        w_rpm = np.zeros((max(rpm_taps, 1), N_MOTORS))
        w_rpm[:M["w_rpm"].shape[0]] = M["w_rpm"]
        lines.append("")
        lines.append(f"// {ax} [rad/s^2] = bias + sum over taps/signals of w * input"
                     + (" (pitch block)" if tag == "Q" and sep else ""))
        lines.append(f"static constexpr float kStrainFitBias{tag} = {f(M['b'])};")
        lines.append(f"static constexpr float kStrainFitW{tag}[STRAIN_FIT_N_TAPS][{nsig_macro}] = {{")
        for j in range(n_taps):
            lines.append(f"    {{  // tap k-{j}")
            for r in range(0, w.shape[1], 4):
                lines.append("        " + ", ".join(f(x) for x in w[j, r:r + 4]) + ",")
            lines.append("    },")
        lines.append("};")
        lines.append(f"// RPM taps k, k-1, ...; {'unused (all 0)' if not M['w_rpm'].shape[0] else 'motors 0..3'}")
        lines.append(f"static constexpr float kStrainFitRpmW{tag}[STRAIN_FIT_RPM_TAPS > 0 ? STRAIN_FIT_RPM_TAPS : 1][{N_MOTORS}] = {{")
        for j in range(w_rpm.shape[0]):
            lines.append("    { " + ", ".join(f(x) for x in w_rpm[j]) + f" }},  // tap k-{j}")
        lines.append("};")

    args.out.write_text("\n".join(lines) + "\n")
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
