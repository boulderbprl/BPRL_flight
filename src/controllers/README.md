# Controllers

Flight control algorithms for the BPRL quadcopter firmware.
Called at 400 Hz from `ControlThread` in `src/threads.cpp`.

---

## Architecture

```
RC input[6]  [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode, indi_stk]
     │
     ▼
FlightStateMachine  (400 Hz)
     │
     ├─ FLIGHT_MODE < -0.33  ───► STABILIZE
     │    Attitude ──────────────────────────────────────────────────────┐
     │    AltControl::compute_throttle()                                 │
     │                                                                    │
     ├─ -0.33 ≤ FLIGHT_MODE ≤ +0.33 ─► ALT_HOLD                        │
     │    Attitude                                                        │
     │    AltControl::alt_hold()                                         │
     │      stick → alt target + climb rate → thrust_out                │
     │                                                                    │
     └─ FLIGHT_MODE > +0.33  ───► POS_HOLD                              │
          reference source — sticks (default) or TrajectoryTracker:      │
            sticks:     PosControl::update()   AltControl::alt_hold()    │
                        HeadingControl::update_rate()                    │
            trajectory: PosControl::track()    AltControl::track()       │
                        HeadingControl::update_heading()                 │
          → lean angles, thrust_out, yaw rate                            │
          Attitude (lean angles override roll/pitch targets,             │
                    HeadingControl's rate overrides the yaw target)  ─────┤
                                                                          │
     Attitude block (shared, dispatched by FlightStateMachine's           │
     config-driven controller list — see below):                         │
       list index 0 (default) → AttitudePID::update()               ◄────┘
       next, if indi_enabled → AttitudeINDI::update()
                    └─ Unmixer::compute() (RPM → torque N·m)
       next, if pid_pi_enabled → AttitudePIDPI::update()
     │
     ▼
MotorMixer  [roll_tq, pitch_tq, yaw_tq, thrust] → motor commands [0..1000]
```

---

## FlightStateMachine (`FlightStateMachine.hpp/.cpp`)

Top-level dispatcher. Manages two orthogonal concepts.

### Flight phase

| Phase | Condition | Behaviour |
|---|---|---|
| DISARMED | arm switch low | Zero torque and thrust; all controllers reset |
| GROUND_IDLE | just armed (or just landed) | Controller outputs discarded outright, motors held at idle floor, regardless of what the cascades compute |
| ACTIVE | armed and spooled up | Full controller stack running |

Arming always goes `DISARMED → GROUND_IDLE`, never straight to `ACTIVE` (mirrors ArduPilot's spool-state machine). Leaving `GROUND_IDLE` requires a sustained, deliberate stick push past a mode-dependent threshold:

- **STABILIZE:** `thr_stick > 0.10` (any deliberate raise off idle — the stick there is a direct thrust command).
- **ALT_HOLD / POS_HOLD:** `thr_stick > 0.50` (the stick there is a signed climb-rate command centered on a hold-altitude deadband, so intent-to-fly means crossing past that center).

Once past threshold, it must stay there for `TAKEOFF_DEBOUNCE_TICKS = 100` (0.25 s @ 400 Hz) before transitioning to `ACTIVE`. On entering `ACTIVE`, thrust is linearly ramped from 0 over `SPOOL_UP_TICKS = 200` (0.5 s) rather than stepping straight to whatever the cascade commands.

**Landed detection** drops back to `GROUND_IDLE` automatically: once spooled up, if commanded thrust stays below `LANDED_THR_THRESHOLD = 0.15` **and** vertical speed stays below `LANDED_VEL_THRESHOLD = 0.2 m/s` for `LANDED_DEBOUNCE_TICKS = 400` (1.0 s), the phase reverts — this exists so a future cascade bug can't idle-wind-up while sitting on the ground.

Transition to `DISARMED` (from any phase) resets all controller integrators and the position-hold targets.

### Flight mode (3-position RC switch on `input[InputIdx::FLIGHT_MODE]`)

| `input[4]` value | Mode | Attitude target | Throttle |
|---|---|---|---|
| < −0.33 | STABILIZE | Pilot stick (rad) | Expo passthrough with tilt boost |
| −0.33 to +0.33 | ALT_HOLD | Pilot stick (rad) | `AltControl::alt_hold()` — stick → climb rate, altitude held in the stick deadband |
| > +0.33 | POS_HOLD | PosControl lean angles (rad) | `AltControl::alt_hold()` — identical to ALT_HOLD |

Mode changes reset all controllers.

In POS_HOLD the three outer controllers (`PosControl`, `AltControl`, `HeadingControl`) track either the pilot's sticks or, after a MAVLink trajectory command, the reference from `TrajectoryTracker` — see [TrajectoryTracker](#trajectorytracker-trajectorytrackerhppcpp). There is no separate trajectory flight mode.

### Attitude controller selection

The attitude controller is independent of the flight mode and is selected by `set_active_controller(int radio_switch_pos)` at runtime. `FlightStateMachine` holds a config-driven list of controllers (see `configs/DroneConfig.hpp`'s `ControllersConfig`) behind the common `AttitudeController` interface — every controller in the list runs every tick (shadow mode); only the one at the resolved list index drives the output:

List indices are assigned in fixed order — PID always 0; INDI, if enabled, always takes the next index; PID+PI, if enabled, always comes last (so PID+PI is index 1, not 2, on a drone that enables it without INDI):

| List index | Controller |
|---|---|
| 0 (always present, default) | `AttitudePID` — cascade P + PID |
| next, if `ControllersConfig::indi_enabled` | `AttitudeINDI` — incremental NDI roll/pitch, PID yaw |
| next, if `ControllersConfig::pid_pi_enabled` | `AttitudePIDPI` — outer P + rate-PID ("SLC") + inner PI on measured angular acceleration for roll/pitch, PID yaw |

With all three controllers present, the 3-position switch maps one-to-one to list index. With only two present it keeps the legacy mapping (low/mid → index 0, high → index 1) — see `FlightStateMachine::set_active_controller()`.

---

## AttitudePID (`Attitude_PID.hpp/.cpp`)

Standard two-loop cascade for roll, pitch, and yaw.

```
roll_tgt [rad]  ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[0]
pitch_tgt [rad] ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[1]
yaw_rate_tgt ───────────────────────────────────────► rate PID  ──► out_cmds[2]
```

Each loop's target/error/derivative filtering matches ArduPilot's `AC_PID` structure (target LPF → error = filtered target − measurement → error LPF → derivative of filtered error → D LPF); the rate loops' target-filter (`FLTT`) and yaw's error-filter (`FLTE`) are set to match `AC_AttitudeControl_Multi`'s actual defaults — see the D-filter/T-filter/E-filter columns below. Integrators are anti-windup clamped to `imax` (see table).

Gains live per-drone in `configs/<Drone>/drone_config.cpp` (`AttitudePidGains`, see `configs/DroneConfig.hpp`) rather than hardcoded in `Attitude_PID.cpp` — the table below shows representative current values; check the relevant drone's `drone_config.cpp` for the authoritative numbers.

### Gains

| Loop | Kp | Ki | Kd | T-filter | E-filter | D-filter | Imax |
|---|---|---|---|---|---|---|---|
| Roll attitude | 4.00 | 0 | 0 | off | off | 30 Hz | 0.5 |
| Pitch attitude | 4.00 | 0 | 0 | off | off | 30 Hz | 0.5 |
| Roll rate | 0.08 | 0.05 | 0.002 | 20 Hz | off | 30 Hz | 0.5 |
| Pitch rate | 0.09 | 0.06 | 0.002 | 20 Hz | off | 30 Hz | 0.5 |
| Yaw rate | 0.18 | 0.018 | 0 | 20 Hz | 2.5 Hz | 5 Hz | 0.5 |
| Yaw hold (heading-lock trim) | 0.60 | 0.05 | 0 | off | off | 30 Hz | 0.3 |

T-filter/E-filter are the target/error low-pass stages ahead of the derivative (`PID`'s `filt_target_hz`/`filt_error_hz`); "off" means disabled (passthrough), matching ArduPilot's own default for that specific gain (e.g. roll/pitch rate `FLTE=0`, but all rate axes get `FLTT=20Hz`, and yaw alone gets `FLTE=2.5Hz`).

`AttitudePidGains::yaw_stick_gain` scales the yaw rate target before the rate PID — a distinct field from `AttitudeIndiGains::yaw_gain`, not necessarily the same value; see the note in the AttitudeINDI section below.

---

## AttitudeINDI (`Attitude_INDI.hpp/.cpp`)

Incremental Nonlinear Dynamic Inversion for roll and pitch; yaw falls back to standard rate PID.

INDI uses the measured angular acceleration (`p_dot`, `q_dot` from `g_state[P_DOT/Q_DOT]`) to close feedback around the actual dynamics rather than a model.

### Control loop (roll shown, pitch symmetric)

```
1. Outer P:   angle_error [rad]    → rate_tgt [rad/s]
2. Inner PID: rate_error [rad/s]   → accel_cmd [rad/s²]
3. INDI step:
     delta_torque  = (accel_cmd − p_dot_measured) × kappa × G1_hat  [N·m]
     total_torque  = current_torque_Nm + delta_torque               [N·m]
     out_cmds[0]   = clamp(total_torque / T_MAX_NM, −1, 1)
```

`current_torque_Nm` is estimated from live DShot RPM telemetry by the **Unmixer**, using per-drone bench-fit motor constants (see the Unmixer section below) — `current_torque` is no longer always zero on a drone with those constants characterized.

### Live G(x) adaptation — `G1_hat`

`G1_hat` (~1/Ixx, 1/Iyy — the airframe-effectiveness term the old static `indi_gain_roll/pitch` used to be) is no longer a fixed constant. It's a live per-axis estimate, seeded from an offline-identified value and continuously refined online by a normalized-LMS (NLMS) adapter:

```
tau_f            = current_torque[axis] through the SAME filter chain
                    (STATEMGR_LP_PQRDOT_HZ + optional extra 1st-order stage)
                    StateManager applies to p_dot/q_dot — delay-matches the
                    regressor to the measured acceleration it's compared against.
                    Runs every 400 Hz tick, to hold its designed cutoff.

every NLMS_DECIMATION ticks (8 = 20 ms, not every 2.5 ms tick — a per-tick
delta of a ~20 Hz-filtered signal is dominated by filter ripple/noise; flight
data showed doublet excitation visible at 50 Hz resolution but not at 400 Hz):
    delta_tau_f      = tau_f[k] − tau_f[k-1]              # k = decimated step index
    delta_omegadot_f = p_dot_measured[k] − p_dot_measured[k-1]

    if |delta_tau_f| >= NLMS_EXCITATION_MIN_NM:      # else: freeze (uninformative regression)
        e      = delta_omegadot_f − G1_hat × delta_tau_f
        step   = mu × e × delta_tau_f / (delta_tau_f² + NLMS_EPS)
        step   = clamp(step, ±NLMS_MAX_STEP_FRAC × |seed|)          # per-step limiter
        G1_hat = clamp(G1_hat + step, seed ± NLMS_MAX_DRIFT_FRAC × |seed|)  # drift limiter
```

This whole block runs every tick `AttitudeINDI::update()` is called — i.e. every tick INDI is in the drone's controller list, whether or not it's the *active* controller (shadow mode, like every other controller in `FlightStateMachine`'s list) — but only the `tau_f` filter update happens every tick; the decimated regressor/step above only fires once every `NLMS_DECIMATION` ticks.

`mu` is mode-dependent: `nlms_mu_pid` (aggressive) while INDI is running in shadow behind PID/PID+PI, `nlms_mu_indi` (slow/trickle) once INDI is actually driving the mixer — set via `set_indi_active(bool)`, called once per tick by `FlightStateMachine::run_attitude()`. `G1_hat` persists across arm/disarm cycles within a boot (only the filter/differencing memory resets in `reset_all()`) — the learned estimate is never thrown away just because the vehicle disarmed.

`kappa` (`indi_output_gain_roll/pitch`) is a separate, static, per-axis output-authority gain, decoupled from `G1_hat` so tuning control authority never silently retunes the physical-effectiveness estimate (or vice versa). `kappa = 1.0` reproduces the pre-adaptation behavior exactly.

`get_g1(float g1[2])` exposes the live `G1_hat` estimate for comparison against its seed — logged in the `INDI` message's `G1R`/`G1P` fields (`LogMsgINDI::g1_roll/g1_pitch`, `src/logging/LogMessages.hpp`), alongside `UnmixR`/`UnmixP` (`current_torque`, a different quantity — don't conflate the two).

`accel_cmd` (the rate-PID's output, before the INDI increment) and `delta_torque` are no longer separate `update()` output parameters — that would break the common `AttitudeController` interface every controller in `FlightStateMachine`'s list now shares (see `src/controllers/AttitudeController.hpp`). `AttitudeINDI::update()` stores them internally instead; `get_diag(delta_torque[2], accel_cmd[2])` reads them back for the `INDI` log's `AccR`/`AccP` fields — see [SD Card Logging](../../README.md#5-sd-card-logging).

### Gains

The outer angle-P loops match `AttitudePID`'s attitude gains (4.00/0/0). The rate loops do **not** match `AttitudePID`'s rate gains — INDI's rate loop output is a commanded acceleration (rad/s²), not a torque, so it runs much higher gain and a much larger integrator clamp. Representative current values (roll/pitch rate gains are typically shared across drones; yaw-rate and the NLMS seed/output values below are tuned per-airframe — check the relevant drone's `drone_config.cpp` for authoritative numbers):

| Loop | Kp | Ki | Kd | T-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Roll attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Pitch attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Roll rate | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Pitch rate | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Yaw rate | 0.065–0.18 | 0.018–0.02 | 0 | off–20 Hz | 5–30 Hz | 0.5 |
| Yaw hold (heading-lock trim) | 0.60 | 0.05 | 0 | off | 30 Hz | 0.3 |

`AttitudeIndiGains` fields beyond the PID loops (`configs/DroneConfig.hpp`, values per-drone in `configs/<Drone>/drone_config.cpp`):

| Field | Meaning |
|---|---|
| `g1_seed_roll` / `g1_seed_pitch` | Offline-identified `G1_hat` seed (N·m·s²/rad) — also the NLMS drift-clamp reference; airframe-specific, re-derive per drone |
| `indi_output_gain_roll/pitch` (kappa) | Static output-authority gain, decoupled from `G1_hat`. `1.0` reproduces pre-adaptation behavior |
| `nlms_mu_pid` | Adaptation rate while INDI is in shadow (PID/PID+PI active) — needs bench/flight tuning |
| `nlms_mu_indi` | Adaptation rate while INDI is active (slow/trickle) — needs bench/flight tuning |
| `yaw_gain` | Scales the yaw rate target before the rate PID — **not** the same field as `AttitudePidGains::yaw_stick_gain`; two distinct fields in two distinct gain structs, easy to conflate, and not necessarily the same value per drone |

`NLMS_EXCITATION_MIN_NM` (0.02 N·m), `NLMS_MAX_STEP_FRAC` (0.10), and `NLMS_MAX_DRIFT_FRAC` (0.50) are adaptation-safety constants (`Attitude_INDI.hpp`, `static constexpr`) rather than `DroneConfig` fields — flight-tuning constants, not per-drone identification data (same convention as `AttitudeINDI`'s `YAW_HOLD_MAX_RATE`). All three are flagged in-source as needing bench tuning.

---

## AttitudePIDPI (`Attitude_PID_PI.hpp/.cpp`)

Three-stage cascade for roll and pitch; yaw falls back to standard rate PID (same as `AttitudePID`/`AttitudeINDI`). Ported from the BPRL ArduPilot fork (`Documents/ardupilot`, `AC_AttitudeControl_Multi::rate_controller_run_dt`, the "switched to PID" commit `8a1c0e33fb`) — that commit replaced ArduPilot's earlier INDI-style running-total inner loop with a plain PI closed directly on measured angular acceleration.

Unlike `AttitudeINDI`, the inner loop's output **replaces** the rate loop's output outright rather than adding an incremental torque correction on top of `current_torque` — this is a plain nested PI on measured angular acceleration, not a dynamic-inversion step, so it needs no `Unmixer`/`current_torque` feedback (like `AttitudePID`, it builds its own 6-element state locally).

### Control loop (roll shown, pitch symmetric)

```
1. Outer P:      angle_error [rad]   → rate_tgt [rad/s]
2. SLC (rate PID): rate_error [rad/s] → accel_tgt [rad/s²]
3. Inner PI:     accel_error [rad/s²] → out_cmds[0], accel_error = accel_tgt − p_dot_measured
```

`p_dot`/`q_dot` come from `g_state[P_DOT/Q_DOT]` — the same EKF-derived measured angular acceleration `AttitudeINDI` uses.

### Gains

Gains live per-drone in `configs/<Drone>/drone_config.cpp` (`AttitudePidPiGains`, see `configs/DroneConfig.hpp`). The outer angle-P and yaw loops are copied from `AttitudePID`'s gains; the rate loop ("SLC") is copied from `AttitudeINDI`'s rate loop (same role — its output is consumed as an acceleration-scale target, not a torque); the inner accel PI is ported verbatim from the ArduPilot source. **None of this has been bench/flight tuned for this specific three-stage combination** — `ControllersConfig::pid_pi_enabled` defaults to `false` until it has.

| Loop | Kp | Ki | Kd | T-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Roll attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Pitch attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Roll rate ("SLC") | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Pitch rate ("SLC") | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Roll accel (inner PI) | 0.70 | 0.80 | 0 | 50 Hz | off | 0.5 |
| Pitch accel (inner PI) | 0.70 | 0.80 | 0 | 50 Hz | off | 0.5 |
| Yaw rate | 0.18 | 0.018 | 0 | 20 Hz | 5 Hz | 0.5 |
| Yaw hold (heading-lock trim) | 0.60 | 0.05 | 0 | off | 30 Hz | 0.3 |

`yaw_stick_gain = 3.0` (`AttitudePidPiGains::yaw_stick_gain`), same value and role as `AttitudePidGains::yaw_stick_gain`.

There is no shadow-diagnostics log for this controller yet (unlike INDI's `get_diag()`/`INDI` log message) — its shadow-mode output (`out_cmds`) is only directly observable via `$TEL`/the `ATT` log when it's the active controller.

---

## AltControl (`AltControl.hpp/.cpp`)

Altitude and throttle controller. Used by all three flight modes.

### STABILIZE throttle passthrough

`compute_throttle()` applies an exponential curve and a tilt boost:

```
boost     = 1 / min(cos(roll), cos(pitch))
thrust    = constrain(thr_exp × boost, 0, 1)
```

The boost compensates for reduced vertical thrust when banked.

### Altitude hold — `alt_hold()` (ALT_HOLD and POS_HOLD)

One z-axis controller, called identically by both modes. POS_HOLD only adds
the N/E cascade in `PosControl` on top. `alt_hold()` is the stick's target
generator in front of `track(alt_tgt_D, climb_rate_ff, cur_D, vD)`, the
feedback loop itself, which `TrajectoryTracker` calls directly with a
commanded altitude.

```
pilot_thr [0,1]
    │
    ▼ stick_to_climb_rate()
    │   centered    = pilot_thr − 0.5            [-0.5, +0.5]
    │   deadband    = ±0.05 around centre
    │   climb_stick = ±MAX_CLIMB_RATE (3 m/s) outside deadband, 0 inside
    │
    ▼ alt_tgt += climb_stick × CONTROL_DT_S      (frozen while stick is in the deadband)
    │   alt_tgt leashed to cur_D ± ALT_LEASH_M (1 m)
    │
    ▼ rate_tgt = climb_stick + pos_pid(alt_tgt − cur_D)     clamp ±3 m/s
    │
    ▼ climb_rate_pid (rate_tgt − vD → delta_thr)
    │
    thrust_out = constrain(THR_MID − delta_thr, 0, 1)
```

`THR_MID = 0.4`. Same scheme as ArduPilot's
`set_pos_target_z_from_climb_rate_cm()`: the stick moves the altitude target
and is fed forward as a climb rate, so centring the stick holds the current
altitude. `cur_D` is the EKF D position and `vD` the NED D velocity (body
U/V/W rotated by roll/pitch), both positive down. The target is re-seeded
from the current altitude after every `reset_all()` (mode change, disarm,
ground idle).

The loop used to cascade through a second, inner acceleration PID fed by
the differentiated (noisy) body-frame accel estimate. That loop was removed
because the noise drove throttle changes fast enough to overheat the
motors — `climb_rate_pid` now commands throttle directly from rate error.

### Gains

Lives per-drone as `AltControlGains` in `configs/<Drone>/drone_config.cpp` (see `configs/DroneConfig.hpp`).

| Loop | Kp | Ki | Kd | E-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Altitude (`pos_D`) | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Climb rate | 0.15 | 0.05 | 0 | 5 Hz | 20 Hz | 0.3 |

Starting-point gains only — not yet re-tuned in flight since the accel loop was removed.

---

## PosControl (`PosControl.hpp/.cpp`)

N/E position controller producing lean angles for the attitude controller. The z axis is handled entirely by `AltControl`. One call per tick: `update(state, stick_fwd, stick_right, att_cmds)` for the sticks, or `track(state, pos_des, vel_des, accel_ff, att_cmds)` for a reference supplied by `TrajectoryTracker`. `track()` bypasses the target generator below and runs the same feedback; it also leaves the generator's state matching the reference, so the next `update()` carries on without a step.

It follows ArduPilot's Loiter (`AC_Loiter::calc_desired_velocity()`): the sticks command an **acceleration**, which is integrated into a velocity target and then into a position target. The position loop runs on that moving target all the time, so there is no hold point to latch — when the velocity target reaches zero the position target simply stops moving, and that is where the vehicle holds.

### Target generator (open loop)

```
sticks (fwd/right, deadband 0.10)
    │  stick → lean angle (MAX_STICK_LEAN_deg = 20° at full stick) → g·tan(angle)
    │  rotate by yaw into N/E (STICKS_BODY_FRAME)
    ▼
accel_pilot [m/s²]     jerk-limited at STICK_JERK_MAX (20 m/s³)
    │
    ▼  accel_cmd = accel_pilot − drag − brake      (both along −vel_des)
    │      drag  = STICK_ACCEL_MAX·|vel_des| / MAX_SPEED   (full stick settles at MAX_SPEED = 5 m/s)
    │      brake = min(BRAKE_GAIN·|vel_des|, BRAKE_ACCEL_MAX), ramped at BRAKE_JERK_MAX,
    │              only once the sticks have been centred for BRAKE_DELAY_S (0.3 s)
    ▼
accel_pred = accel_cmd through a first-order lag of ATT_LAG_S (0.15 s)
    │
    ▼  vel_des += accel_pred·dt
vel_des [m/s]
    │
    ▼  pos_des += vel_des·dt          leashed to within POS_LEASH_M (2 m) of the vehicle
pos_des [m]
```

`accel_pred` is the attitude-response predictor (ArduPilot's `_predicted_accel`). The vehicle's lean angle, and so its real acceleration, trails the commanded one by the attitude loop's lag. If the targets were integrated from the command itself they would get ahead of the vehicle by `ATT_LAG_S × (change in acceleration)` in velocity on every stick push and release, and the feedback loops would turn that into an overshoot (about 0.3 m in flight before this was added). Integrating the lagged value makes the targets move the way the vehicle will. `ATT_LAG_S` was measured from a flight log (roll 0.16 s, pitch 0.12 s with angle kp = 6); re-measure it if the attitude gains change.

On the first tick after a reset (mode change, disarm, ground idle) the targets are seeded from the measured position and velocity, so engaging POS_HOLD while moving is a smooth brake to a stop.

### Feedback with feed-forward

```
vel_tgt   = vel_des + clamp(_pos_N/E (P)(pos_des − pos), ±VEL_CORR_MAX = 2 m/s)
accel_fb  = _vel_N/E (PID)(vel_tgt − vel)  ──►  2nd-order lowpass @ ACCEL_FILT_HZ (10 Hz), in the N/E frame
accel_tgt = accel_cmd + accel_fb
                                │
                    rotate to body frame (yaw):
                      accel_N_body =  cos(ψ)·aN + sin(ψ)·aE
                      accel_E_body = −sin(ψ)·aN + cos(ψ)·aE
                                │
                    compute lean angles:
                      pitch_tgt = atan2(−accel_N_body, g)
                      roll_tgt  = atan2( accel_E_body·cos(pitch_tgt), g)
                                │
                          clamp: ±30° (MAX_LEAN_deg, feed-forward + feedback together)
```

The stick acceleration goes straight to the lean angle, so the response to the stick is immediate; the PIDs only correct tracking error, which stays small because the targets move no faster than the vehicle can. The feed-forward bypasses the 10 Hz filter so it doesn't pick up the filter's lag.

`vN`/`vE` are NED velocities: `FlightStateMachine::mode_pos_hold()` rotates the
EKF's body-frame U/V/W into NED before calling `PosControl`, which never sees
the raw EKF state layout (see the conventions comment in `PosControl.hpp`).

The feedback filter runs at the fixed `CONTROL_DT_S` (`src/threads.hpp`),
not a measured `dt`. `lowpass2p()` recomputes its coefficients from `dt` every
call, and tick-to-tick jitter in a measured `dt` shows up as noise on the
output proportional to the signal — that was the source of a noisy lean-angle
target with a clean velocity error.

### Stick and brake tuning

`static constexpr` members of `PosControl` (`PosControl.hpp`), not per-drone config:

| Constant | Value | Effect |
|---|---|---|
| `STICK_DEADBAND` | 0.10 | Stick travel around centre that commands nothing |
| `MAX_STICK_LEAN_deg` | 20° | Lean (and so acceleration, 3.57 m/s²) at full stick |
| `STICK_JERK_MAX` | 20 m/s³ | How fast the stick acceleration (and so the lean command) may change |
| `ATT_LAG_S` | 0.15 s | Attitude loop's lean-angle response lag used by the predictor |
| `MAX_SPEED` | 5 m/s | Speed a full stick settles at (ArduPilot `LOIT_SPEED`) |
| `BRAKE_DELAY_S` | 0.3 s | Sticks centred this long before braking starts (`LOIT_BRK_DELAY`, 1 s there) |
| `BRAKE_GAIN` | 2.0 1/s | Brake deceleration per m/s of target speed |
| `BRAKE_ACCEL_MAX` | 2.5 m/s² | Cap on brake deceleration (`LOIT_BRK_ACCEL`) |
| `BRAKE_JERK_MAX` | 10 m/s³ | How fast the brake deceleration ramps in (`LOIT_BRK_JERK`, 5 there) |
| `VEL_CORR_MAX` | 2 m/s | Cap on the position loop's velocity correction |
| `POS_LEASH_M` | 2 m | Max distance the position target may get from the vehicle |

### Gains

Lives per-drone as `PosControlGains` in `configs/<Drone>/drone_config.cpp` (see `configs/DroneConfig.hpp`).

| Loop | Kp | Ki | Kd | E-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Pos N | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Pos E | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Vel N | 3.0 | 1.0 | 0 | 20 Hz | 20 Hz | 0.8 |
| Vel E | 3.0 | 1.0 | 0 | 20 Hz | 20 Hz | 0.8 |

---

## HeadingControl (`HeadingControl.hpp/.cpp`)

POS_HOLD's heading loop: a heading reference in, a yaw-rate command out, which `FlightStateMachine` hands to every attitude controller through `AttitudeController::set_external_yaw_rate()`. While that is enabled a controller uses the supplied rate as its yaw-rate loop target and bypasses its own stick rate + heading-lock trim. STABILIZE and ALT_HOLD never enable it, so yaw there is unchanged.

```
shaped heading target:  rate_tgt slews to the demanded rate at max_accel;  target += rate_tgt·dt
yaw_rate_cmd = rate_tgt + constrain(kp · wrap(target − yaw), ±max_rate)
```

| Entry point | Used by | Demanded rate |
|---|---|---|
| `update_rate(yaw, rate)` | POS_HOLD yaw stick | `stick_rate` × stick (deadband 0.10, rescaled) |
| `update_heading(yaw, yaw_des, rate_ff)` | TrajectoryTracker | `rate_ff` + approach toward `yaw_des` (≤ `max_rate`, sqrt profile) |

The target is leashed to within 0.6 rad of the vehicle, and shifted by `yaw_frame_reset()` when the estimator's yaw steps. Gains are per-drone (`HeadingGains`): `kp` 1.75 /s, `stick_rate` 3.0 rad/s, `max_rate` 1.0 rad/s, `max_accel` 4.0 rad/s² — starting values, not flight-tuned.

---

## TrajectoryTracker (`TrajectoryTracker.hpp/.cpp`)

The layer above the position, altitude and heading controllers — the role `AC_Circle` and Guided play over `AC_PosControl` in ArduPilot. It turns a commanded point or circle into the moving reference (`TrajRef`: position, velocity, acceleration feed-forward, heading, yaw-rate feed-forward) that those controllers track through their `track()` / `update_heading()` entry points. It contains no feedback loops.

Commands arrive over MAVLink (`COMMAND_LONG`, see `src/coms/README.md`), cross to ControlThread through `g_traj_mailbox`, and are judged by `FlightStateMachine::traj_command()`. Everything is NED, metres; positions are offsets from an origin set by command. A NaN parameter means "not given".

| Command | Parameters | Behaviour |
|---|---|---|
| Set origin | — | Origin := current position. Refused while a trajectory is active. Survives disarm. |
| Point | N, E, D | Straight line at `V_MOVE`, then hold. Defaults: N/E 0, D = current altitude. Heading held. |
| Circle | radius, N, E, D, focus distance, direction, speed | Fly to the nearest point of the path, turn the nose along it, then follow it nose-first. Only the radius is required. |
| Stop | — | Back to plain position hold. |

**Ellipse** — focus distance `f` (0 = circle): foci `|f|` from the centre, along North if `f > 0`, East if `f < 0`; `radius` is focus-to-near-end, so semi-major = `r + |f|`, semi-minor = `sqrt(r·(r + 2|f|))`.
**Direction** — positive (default) is right-handed about D: yaw increasing, clockwise seen from above.
**Speed** — the commanded speed on the gentle parts, reduced where curvature would exceed `A_LAT_MAX` or `YAW_RATE_MAX`; the tracker looks ahead and brakes at `A_TAN_MAX` before a tight section.

Rules:

- Point and circle are only accepted while flying (phase ACTIVE) in POS_HOLD with a valid position and an origin set.
- Roll, pitch or yaw stick outside its deadband cancels the trajectory, and it stays cancelled. So do leaving POS_HOLD, ground idle, disarming and losing the position. The controllers carry on from the last reference, so the hand-back is smooth.
- The throttle stick moves the whole trajectory up and down (a height offset; the origin does not move). Each new command zeroes it.
- Floor: the D at take-off is recorded. A point or circle whose altitude is lower than `FLOOR_MARGIN_M` (0.15 m) above it is refused and changes nothing (a running trajectory carries on). The throttle stick cannot lower a running trajectory past that height either. Plain POS_HOLD is not limited, so landing is unaffected.
- If the vehicle is more than `ERR_SLOW_M` behind the reference, progress along the path slows, stopping at `ERR_PAUSE_M`.

Limits are constants at the top of `TrajectoryTracker.hpp` (starting values, to be flight-tuned). The `TRAJ` log message records the tracker state, reference, heading target and yaw-rate command. `tools/sim/` flies all of this on the PC — see `tools/sim/README.md`.

---

## POS_HOLD notes

Heading: `HeadingControl` — the yaw stick turns the heading target rather than commanding a raw rate.

D axis: `AltControl::alt_hold()`, exactly as in ALT_HOLD — the throttle stick commands climb rate outside its deadband and the current altitude is held inside it.

**Stick frame** — `PosControl::STICKS_BODY_FRAME` (`PosControl.hpp`, currently `true`): the pitch/roll sticks command forward/right acceleration relative to the nose and are rotated by yaw into N/E (ArduPilot Loiter behaviour). Set it to `false` for the sticks to command N/E directly.

**Shadow mode** — `FlightStateMachine::CTUN_POSHOLD_SHADOW` (`FlightStateMachine.hpp`, currently `false`): when `true`, POS_HOLD flies as hands-off STABILIZE while the whole cascade above still runs every tick and feeds the `CTUN` log message, for tuning without closing the loop. If you turn it on, also drop `TAKEOFF_THR_THRESHOLD_HOLD` back to 0.10 (see the comment next to it).

---

## Unmixer (`Unmixer.hpp/.cpp`)

Converts per-motor RPM (DShot GCR telemetry) to physical roll/pitch torques in N·m for INDI feedback. Uses real bench-fit constants (not placeholders) — supplied per-drone via `UnmixerConfig` (`configs/DroneConfig.hpp`) rather than hardcoded `static constexpr` members; `Unmixer`'s constructor takes the config by reference and stores it.

**RPM filtering** — each motor's RPM goes through a 2nd-order lowpass at `rpm_filt_hz` (20 Hz) and an optional 1st-order stage at `rpm_filt_extra_hz` (15 Hz, `<= 0` disables it) before the motor model, both at the fixed `CONTROL_DT_S`.

**Motor force model** — cubic fit against *normalized* mechanical RPM (values below are Drone1's; each drone's `UnmixerConfig` has its own):
```
rpm_norm = (rpm − rpm_norm_center) / rpm_norm_scale        // rpm_norm_center=2005, rpm_norm_scale=880.8
F_N      = C3·rpm_norm³ + C2·rpm_norm² + C1·rpm_norm + C0  // motor_c3=0.0134, motor_c2=0.5607, motor_c1=2.2831, motor_c0=2.4540
F_N      = clamp(F_N, 0, max_thrust_n)          // guard small negative thrust near zero RPM, cap at bench max
```

**X-frame geometry** (arm length `arm_length_m = 0.1275 m`, NED body frame):
```
roll_Nm  = (arm_length_m/√2) × (−F_FR + F_RL + F_FL − F_RR)
pitch_Nm = (arm_length_m/√2) × ( F_FR − F_RL + F_FL − F_RR)
```

Signs are consistent with `MotorMixer`: positive roll command spins RL/FL faster, producing positive `roll_Nm`.

**Normalization:** `max_thrust_n = 7.04` (single-motor max from the bench fit) gives `T_MAX_NM = 2·sin(45°)·arm_length_m·max_thrust_n ≈ 1.269 N·m`, computed once at construction (not a separate config field, so it can't drift out of sync with the fields it's derived from) and used to normalize torque to `[-1, 1]` via `normalize_torque()`.

The bench fit also gives motor reaction (drag) torque as a function of thrust (`torque_Nm = 0.03·((F_N − 2.991)/2.183) + 0.0423`), but it isn't wired in since yaw currently uses a rate PID rather than INDI torque feedback — see `Unmixer.hpp`'s doc comment if yaw moves to torque-based control.

---

## MotorMixer (`MotorMixer.hpp/.cpp`)

Converts `[roll_tq, pitch_tq, yaw_tq, thrust]` (all normalised) to four motor commands (0–1000) using an X-frame mixing matrix. Motor factor arrays, PWM min/idle/max, and the roll/pitch/yaw scale and disarm-angle constants come from `MotorMixerConfig` (`configs/DroneConfig.hpp`), passed to the constructor — no longer `static constexpr` class members, so per-drone geometry (different frame size, motor order, or PWM range) doesn't require touching this shared code.

All motors output 0 when disarmed or when |roll| or |pitch| exceeds the config's `max_angle_rad` (~80° for the drones today).

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

All internal math is in this logical `[FR, RL, FL, RR]` order — `MotorMixerConfig::motor_map[FR/RL/FL/RR]` (per-drone physical DShot lane assignment) is applied once, at the very end of `update()`, converting to physical lane order for `out[]`. See the root README's [MotorMixer](../../README.md#motormixer) section for the full rationale and why per-drone wiring quirks belong in `motor_map`, not in the factor tables above.

---

## PID base class (`PID.hpp/.cpp`)

All controllers use the same `PID` class. Filtering matches ArduPilot's `AC_PID` structure — target and error each get their own optional low-pass stage ahead of the derivative, not just a single D-term filter:

```cpp
PID(float kp, float ki, float kd, float imax,
    float filt_target_hz = 0.0f, float filt_error_hz = 0.0f, float filt_d_hz = 20.0f);
float update(float target, float measurement);
void  reset();
void  set_gains(float kp, float ki, float kd);
```

```
target  --[1st-order LPF @ filt_target_hz]--> target_filt
error   = target_filt - measurement
error   --[1st-order LPF @ filt_error_hz]--> error_filt   (feeds P and I)
derivative of error_filt --[1st-order LPF @ filt_d_hz]--> feeds D
```

`filt_target_hz`/`filt_error_hz` default to `0` (disabled → passthrough), matching ArduPilot's `FLTT`/`FLTE` default-off behaviour; each `AttitudePID` rate loop overrides these explicitly (see the AttitudePID gains table above). There is **no explicit `dt` parameter** — `update()` derives it internally from `bprl_micros()` (`src/uptime.hpp`, a monotonic clock that doesn't wrap with the 16-bit system tick on Drone3) between calls. Two edge cases are handled specially:

- **First sample** (`_deriv_valid == false`): returns `kp * error` only (no I or D term), and latches the timestamp — avoids a derivative spike from an undefined previous error.
- **Stale input** (gap since the last call `> 200 ms`, `STALE_TIMEOUT_US`): calls `reset()` (zeroes the integrator and derivative state) and returns `kp * error` only, same as the first-sample case — protects against a large derivative/integral kick if a caller stops calling `update()` for a while and then resumes.

The integrator is clamped to `±imax`.

---

## Notes

- **Yaw / heading hold** — in STABILIZE and ALT_HOLD the yaw stick commands yaw *rate*, and each attitude controller adds a heading-lock trim on top: while the yaw stick is inside its ±0.10 deadband the current heading is latched as the target and a PI on heading error (`yaw_hold` gains) adds a corrective rate, capped at `YAW_HOLD_MAX_RATE` (0.3 rad/s); moving the stick re-latches the target to wherever the nose is. POS_HOLD replaces this with `HeadingControl` (see above), which owns a heading target the stick turns or a trajectory sets. The heading itself comes from the EKF, which takes it from the IMX5 and/or the mocap yaw (see the root README's [State Estimation](../../README.md#3-state-estimation-ekf) section) — there is no magnetometer.

See the root README's [TODO](../../README.md#todo) list for planned feature work; this file documents the controllers as they exist today.
