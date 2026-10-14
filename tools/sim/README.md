# Host flight simulation

Runs the real controller sources on the PC against a simple quadrotor model, so
changes to POS_HOLD, the heading controller and the trajectory tracker can be
checked before a flight. Nothing here is built into the firmware.

It is a logic and sanity check — sign errors, target jumps, hand-over bumps,
command rules, gross mistuning. The plant is too simple to trust for final
gains; those come from flight logs.

## Run it

```bash
tools/sim/run.sh                      # Drone3 gains, CSVs into a temp directory
tools/sim/run.sh Drone2 /tmp/simout   # another drone's gains, chosen output directory
python3 tools/sim/plot.py /tmp/simout/ellipse_north.csv          # writes ellipse_north.png
python3 tools/sim/plot.py /tmp/simout/point.csv --show           # interactive window
```

Needs `g++`; the plot script needs `matplotlib`. `run.sh` prints one line per
check (`[ ok ]` / `[FAIL]`, the measured value and its limit), ends with
`ALL PASSED` or `FAILED`, and exits non-zero on any failure.

```bash
tools/sim/regress.sh df2b243          # stick paths unchanged since that commit?
```

`regress.sh` is a separate, stricter test for refactors: it builds the same
scripted POS_HOLD/ALT_HOLD stick flight against the sources at a git ref and
against the working tree and requires every controller output to be
identical.

## What is in here

| File | What it is |
|---|---|
| `sim.cpp` | The plant, the simulated pilot, and every scenario with its pass/fail limits |
| `run.sh` | Builds `sim.cpp` with the real controller sources and runs it |
| `plot.py` | Plots one scenario's CSV: path from above, speed, heading, height |
| `regress.cpp`, `regress.sh` | Before/after identity check of `PosControl::update()` and `AltControl::alt_hold()` |
| `stubs/` | Stand-ins for the three ChibiOS-dependent headers the controllers include |

### How the real code is compiled on the PC

The controller sources include `ch.h`, `src/threads.hpp` and `src/uptime.hpp`,
which need ChibiOS. `stubs/` holds minimal replacements, and `run.sh` puts
`stubs/` ahead of the repository on the include path, so
`#include "src/threads.hpp"` finds the stub. Everything else — `PID`,
`AltControl`, `PosControl`, `HeadingControl`, `TrajectoryTracker`,
`FlightStateMachine`, the attitude controllers, the drone's
`drone_config.cpp` — is the firmware's own file, unmodified.

Time is simulated: `stubs/src/uptime.hpp` returns `g_sim_time_us`, which the
simulation advances by one control period (2.5 ms) per tick. If
`CONTROL_PERIOD_US` changes in `src/threads.hpp`, change it in
`stubs/src/threads.hpp` too.

### The plant (`struct Plant` in `sim.cpp`)

- Torque command → angular acceleration (`K_RP`, `K_YAW`), through a 30 ms motor lag.
- Thrust along body −z; hovers at the drone config's `hover_thr`.
- Linear drag and a steady wind (0.25 m/s² N, −0.15 m/s² E) so integrators have work to do.
- Ground at D = 0. `stuck = true` pins the vehicle in N/E for the "can't keep up" test.

### The pilot and the scenarios (`struct Sim`)

`Sim` owns a `Plant` and a `FlightStateMachine`, and exposes the sticks
(`thr`, `roll_stick`, `pitch_stick`, `yaw_stick`), the mode switch, `armed`
and `pos_valid`. `takeoff_to(h)` arms in POS_HOLD and climbs; `run(seconds)`
advances time; `cmd(...)` sends a trajectory command exactly as ControlThread
does (unset parameters are NaN = "not given").

| Scenario | Checks |
|---|---|
| `heading_unit` | `HeadingControl` alone: 90° step, and 170° → −170° goes the short way |
| `heading_stick` | POS_HOLD yaw stick: turns the right way, stops cleanly, holds heading and position |
| `point` | Origin rules, default point, 2 m move, explicit D, throttle offset, floor, stick override, falling behind |
| `circle_*`, `ellipse_*` | Approach, path error, nose along the path, lean, rotation sense, slowdown at tight ends, stop |
| `circle_from_3m_off` | Joining a circle from well outside it |
| `commands` | Every rejection rule and cancel condition, landing afterwards, origin surviving a disarm |
| `degraded_link` | Position hold and a point move with the position 100 ms late and refreshed at 23 Hz — a stability check for the position gains on a poor mocap link |

### CSV columns

`t`, position `n e d`, velocity `vn ve vd`, `roll pitch yaw` (rad),
PosControl's `pos_n_tgt pos_e_tgt` and lean targets `roll_tgt pitch_tgt`,
then the tracker's `traj_state` (0 idle, 1 moving, 2 holding, 3 on the
circle), `ref_n ref_e ref_d`, `yaw_tgt`, `yaw_rate_cmd`, `h_off`,
`path_speed`, and `thrust`, `phase` (0 disarmed, 1 ground idle, 2 active).

## Adding a scenario

Write a function in `sim.cpp` that creates a `Sim("name")`, flies it, and
calls `check(condition, "what", measured, limit)`; call it from `main()`.
The CSV is written automatically as `name.csv`.
