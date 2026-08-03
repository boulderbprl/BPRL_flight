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
     │    AltControl::alt_hold_from_stick()                              │
     │      stick → climb_rate_pid → thrust_out                         │
     │                                                                    │
     └─ FLIGHT_MODE > +0.33  ───► POS_HOLD                              │
          PosControl::NED_update()   (pos error → vel targets)           │
          Per-axis state machine (PILOT/BRAKE/HOLD/RETURNING)            │
          PosControl::NE_rate_update() (vel error → lean angles)         │
          AltControl::alt_hold_from_rate() (D vel → thrust_out)          │
          Attitude (with lean angles overriding roll/pitch targets)  ─────┤
                                                                          │
     Attitude block (shared, dispatched by FlightStateMachine's           │
     config-driven controller list — see below):                         │
       list index 0 (default) → AttitudePID::update()               ◄────┘
       list index 1 (if enabled) → AttitudeINDI::update()
                    └─ Unmixer::compute() (RPM → torque N·m)
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

Transition to `DISARMED` (from any phase) resets all controller integrators and the position-hold latch.

### Flight mode (3-position RC switch on `input[InputIdx::FLIGHT_MODE]`)

| `input[4]` value | Mode | Attitude target | Throttle |
|---|---|---|---|
| < −0.33 | STABILIZE | Pilot stick (rad) | Expo passthrough with tilt boost |
| −0.33 to +0.33 | ALT_HOLD | Pilot stick (rad) | AltControl stick → climb rate |
| > +0.33 | POS_HOLD | PosControl lean angles (rad) | AltControl D-axis rate |

Mode changes reset all controllers.

### Attitude controller selection

The attitude controller is independent of the flight mode and is selected by `set_active_controller(int radio_switch_pos)` at runtime. `FlightStateMachine` holds a config-driven list of controllers (see `configs/DroneConfig.hpp`'s `ControllersConfig`) behind the common `AttitudeController` interface — every controller in the list runs every tick (shadow mode); only the one at the resolved list index drives the output:

| List index | Controller |
|---|---|
| 0 (always present, default) | `AttitudePID` — cascade P + PID |
| 1 (present only if `ControllersConfig::indi_enabled`) | `AttitudeINDI` — incremental NDI roll/pitch, PID yaw |

---

## AttitudePID (`Attitude_PID.hpp/.cpp`)

Standard two-loop cascade for roll, pitch, and yaw.

```
roll_tgt [rad]  ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[0]
pitch_tgt [rad] ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[1]
yaw_rate_tgt ───────────────────────────────────────► rate PID  ──► out_cmds[2]
```

Each loop's target/error/derivative filtering matches ArduPilot's `AC_PID` structure (target LPF → error = filtered target − measurement → error LPF → derivative of filtered error → D LPF); the rate loops' target-filter (`FLTT`) and yaw's error-filter (`FLTE`) are set to match `AC_AttitudeControl_Multi`'s actual defaults — see the D-filter/T-filter/E-filter columns below. Integrators are anti-windup clamped to `imax` (see table).

Gains now live per-drone in `configs/<Drone>/drone_config.cpp` (`AttitudePidGains`, see `configs/DroneConfig.hpp`) rather than hardcoded in `Attitude_PID.cpp` — the table below is Drone1/Drone2's current (identical) values.

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

`yaw_stick_gain = 3.0` (`AttitudePidGains::yaw_stick_gain`) scales the yaw rate target before the rate PID — not the same value as `AttitudeIndiGains::yaw_gain` (1.5); see the note in the AttitudeINDI section below.

---

## AttitudeINDI (`Attitude_INDI.hpp/.cpp`)

Incremental Nonlinear Dynamic Inversion for roll and pitch; yaw falls back to standard rate PID.

INDI uses the measured angular acceleration (`p_dot`, `q_dot` from `g_state[P_DOT/Q_DOT]`) to close feedback around the actual dynamics rather than a model.

### Control loop (roll shown, pitch symmetric)

```
1. Outer P:   angle_error [rad]    → rate_tgt [rad/s]
2. Inner PID: rate_error [rad/s]   → accel_cmd [rad/s²]
3. INDI step:
     delta_torque  = (accel_cmd − p_dot_measured) × INDI_GAIN     [N·m]
     total_torque  = current_torque_Nm + delta_torque              [N·m]
     out_cmds[0]   = clamp(total_torque / T_MAX_NM, −1, 1)
```

`current_torque_Nm` is estimated from live DShot RPM telemetry by the **Unmixer**, which now has real bench-fit constants (see the Unmixer section below) — `current_torque` is no longer always zero.

`indi_gain_roll = 0.0035` / `indi_gain_pitch = 0.0045` N·m·s²/rad (`AttitudeIndiGains`, see `configs/DroneConfig.hpp`) — effective moment of inertia (Ixx, Iyy), one value per axis (not a single shared gain). Like the other controllers, these now live per-drone in `configs/<Drone>/drone_config.cpp` rather than hardcoded in `Attitude_INDI.cpp`.

`accel_cmd` (the rate-PID's output, before the INDI increment) and `delta_torque` are no longer separate `update()` output parameters — that would break the common `AttitudeController` interface every controller in `FlightStateMachine`'s list now shares (see `src/controllers/AttitudeController.hpp`). `AttitudeINDI::update()` stores them internally instead; `get_diag(delta_torque[2], accel_cmd[2])` reads them back for the `INDI` log's `AccR`/`AccP` fields — see [SD Card Logging](../../README.md#5-sd-card-logging).

### Gains

The outer angle-P loops match `AttitudePID`'s attitude gains (4.00/0/0). The rate loops do **not** match `AttitudePID`'s rate gains — INDI's rate loop output is a commanded acceleration (rad/s²), not a torque, so it runs much higher gain and a much larger integrator clamp:

| Loop | Kp | Ki | Kd | T-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Roll attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Pitch attitude | 4.00 | 0 | 0 | off | 30 Hz | 0.5 |
| Roll rate | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Pitch rate | 6.50 | 0.20 | 0 | 30 Hz | 30 Hz | 10.0 |
| Yaw rate | 0.065 | 0.02 | 0 | off | 30 Hz | 0.5 |
| Yaw hold (heading-lock trim) | 0.60 | 0.05 | 0 | off | 30 Hz | 0.3 |

`yaw_gain = 1.5` (`AttitudeIndiGains::yaw_gain`) scales the yaw rate target before the rate PID — **not** the same value as `AttitudePidGains::yaw_stick_gain` (3.0); these are two distinct fields in two distinct gain structs, easy to conflate.

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

### ALT_HOLD loop — `alt_hold_from_stick()`

```
pilot_thr [0,1]
    │
    ▼ stick_to_climb_rate()
    │   centered = pilot_thr − 0.5               [-0.5, +0.5]
    │   deadband = 0.05 of half-range
    │   rate_tgt = ±MAX_CLIMB_RATE (3 m/s) outside deadband
    │
    ▼ climb_rate_pid (rate error → delta_thr)
    │
    thrust_out = constrain(THR_MID − delta_thr, 0, 1)
```

`THR_MID = 0.4`. Stick centred → hold current altitude (rate_tgt = 0).

The loop used to cascade through a second, inner acceleration PID fed by
the differentiated (noisy) body-frame accel estimate. That loop was removed
because the noise drove throttle changes fast enough to overheat the
motors — `climb_rate_pid` now commands throttle directly from rate error.

### POS_HOLD — `alt_hold_from_rate()`

Skips the outer stick-to-rate conversion. Called with the D-axis velocity target from `PosControl::NED_update()`.

### Gains

Lives per-drone as `AltControlGains` in `configs/<Drone>/drone_config.cpp` (see `configs/DroneConfig.hpp`).

| Loop | Kp | Ki | Kd | E-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Climb rate | 0.15 | 0.05 | 0 | 5 Hz | 20 Hz | 0.3 |

Starting-point gains only — not yet re-tuned in flight since the accel loop was removed.

---

## PosControl (`PosControl.hpp/.cpp`)

Two-stage NED position controller producing lean angles and a climb rate for the attitude and altitude controllers.

### Stage 1 — `NED_update()`: position → velocity targets

```
pos_tgt[N] − state[N]  ──► _pos_N (P) ──► vel_N_tgt
pos_tgt[E] − state[E]  ──► _pos_E (P) ──► vel_E_tgt
pos_tgt[D] − state[D]  ──► _pos_D (P) ──► vel_D_tgt
                                │
                          clamp: ±5 m/s (NE), ±3 m/s (D)
```

### Stage 2 — `NE_rate_update()`: velocity errors → lean angles

```
vel_N_tgt − vN  ──► _vel_N (PID) ──► accel_N_tgt [m/s²]
vel_E_tgt − vE  ──► _vel_E (PID) ──► accel_E_tgt [m/s²]
                                │
                    rotate to body frame (yaw):
                      accel_N_body =  cos(ψ)·aN + sin(ψ)·aE
                      accel_E_body = −sin(ψ)·aN + cos(ψ)·aE
                                │
                    compute lean angles:
                      roll_tgt  = atan2(−accel_E_body, g)
                      pitch_tgt = atan2( accel_N_body·cos(roll), g)
                                │
                          clamp: ±30°
```

### Gains

Lives per-drone as `PosControlGains` in `configs/<Drone>/drone_config.cpp` (see `configs/DroneConfig.hpp`).

| Loop | Kp | Ki | Kd | E-filter | D-filter | Imax |
|---|---|---|---|---|---|---|
| Pos N | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Pos E | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Pos D | 1.0 | 0 | 0 | off | 20 Hz | 0 |
| Vel N | 2.0 | 1.0 | 0 | 20 Hz | 20 Hz | 0.8 |
| Vel E | 2.0 | 1.0 | 0 | 20 Hz | 20 Hz | 0.8 |

---

## POS_HOLD per-axis state machine

N and E axes run the same state machine independently inside `FlightStateMachine::mode_pos_hold()`.

```
          stick active?
          ┌──────────────────────────────────────────────────────────┐
          │                                                          │
   ┌──► PILOT ──────────────────────► BRAKE ──── |vel| < 0.20 m/s ──► HOLD ──► RETURNING
   │      │  stick released                │                              │
   │      │                               │ stick pressed                │
   │      │◄──────────────────────────────┘                              │
   └──────────────────────────────────────────────────────────────────────┘
          (RETURNING blends from controller output → stick over ~0.5 s)
```

| State | N/E velocity source | Hold position latched? |
|---|---|---|
| PILOT | `stick × MAX_VEL_NE` | No |
| BRAKE | 0 m/s | No — latch on stop |
| HOLD | `PosControl::NED_update()` | Yes |
| RETURNING | linear blend (controller → stick, 200 ticks ≈ 0.5 s) | Releasing |

D axis: altitude position is latched once both N and E axes have settled to HOLD. After that it is held via `AltControl::alt_hold_from_rate()`.

---

## Unmixer (`Unmixer.hpp/.cpp`)

Converts per-motor RPM (DShot GCR telemetry) to physical roll/pitch torques in N·m for INDI feedback. Uses real bench-fit constants (not placeholders) — supplied per-drone via `UnmixerConfig` (`configs/DroneConfig.hpp`) rather than hardcoded `static constexpr` members; `Unmixer`'s constructor takes the config by reference and stores it.

**Motor force model** — cubic fit against *normalized angular velocity*, not raw RPM directly:
```
omega    = rpm × (π / 30)                       // RPM → rad/s
rpm_norm = (omega − rpm_norm_center) / rpm_norm_scale      // rpm_norm_center=2005, rpm_norm_scale=880.8 rad/s
F_N      = C3·rpm_norm³ + C2·rpm_norm² + C1·rpm_norm + C0  // motor_c3=0.0134, motor_c2=0.5607, motor_c1=2.2831, motor_c0=2.4540
F_N      = max(F_N, 0)                          // guard small negative thrust near zero RPM
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

All motors output 0 when disarmed or when |roll| or |pitch| exceeds the config's `max_angle_rad` (~80° for both drones today).

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

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

`filt_target_hz`/`filt_error_hz` default to `0` (disabled → passthrough), matching ArduPilot's `FLTT`/`FLTE` default-off behaviour; each `AttitudePID` rate loop overrides these explicitly (see the AttitudePID gains table above). There is **no explicit `dt` parameter** — `update()` derives it internally from `chVTGetSystemTimeX()` between calls. Two edge cases are handled specially:

- **First sample** (`_deriv_valid == false`): returns `kp * error` only (no I or D term), and latches the timestamp — avoids a derivative spike from an undefined previous error.
- **Stale input** (gap since the last call `> 200 ms`, `STALE_TIMEOUT_US`): calls `reset()` (zeroes the integrator and derivative state) and returns `kp * error` only, same as the first-sample case — protects against a large derivative/integral kick if a caller stops calling `update()` for a while and then resumes.

The integrator is clamped to `±imax`.

---

## TODOs

- **Gain tuning** — all gains are untested on H7 hardware.
- **Yaw hold** — POS_HOLD currently commands yaw *rate*, not yaw *angle*. An outer yaw loop requires a heading reference from the IMX5 or magnetometer.
