# Controllers

Flight control algorithms for the BPRL quadcopter firmware.
Called at 400 Hz from `ControlThread` in `src/threads.cpp`.

---

## Architecture

```
RC input[5]  [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
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
     Attitude block (shared, dispatched by _use_indi flag):              │
       false → AttitudePID::update()                                ◄────┘
       true  → AttitudeINDI::update()
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

The attitude controller is independent of the flight mode and is selected by `set_use_indi(bool)` at runtime (default: PID):

| Flag | Controller |
|---|---|
| `false` | `AttitudePID` — cascade P + PID |
| `true` | `AttitudeINDI` — incremental NDI roll/pitch, PID yaw |

---

## AttitudePID (`Attitude_PID.hpp/.cpp`)

Standard two-loop cascade for roll, pitch, and yaw.

```
roll_tgt [rad]  ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[0]
pitch_tgt [rad] ──► outer P ──► rate_tgt [rad/s] ──► inner PID ──► out_cmds[1]
yaw_rate_tgt ───────────────────────────────────────► rate PID  ──► out_cmds[2]
```

All derivative terms use a 30 Hz first-order low-pass filter. Integrators are anti-windup clamped to ±0.5.

### Gains

| Loop | Kp | Ki | Kd | D-filter | Imax |
|---|---|---|---|---|---|
| Roll attitude | 3.50 | 0 | 0 | 30 Hz | 0.5 |
| Pitch attitude | 3.50 | 0 | 0 | 30 Hz | 0.5 |
| Roll rate | 0.09 | 0.075 | 0.001 | 30 Hz | 0.5 |
| Pitch rate | 0.13 | 0.100 | 0.002 | 30 Hz | 0.5 |
| Yaw rate | 0.18 | 0.018 | 0 | 5 Hz | 0.5 |

`YAW_STICK_GAIN = 2.5` scales the yaw rate target before the rate PID — not the same constant as `AttitudeINDI::YAW_GAIN` (1.5); see the note in the AttitudeINDI section below.

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

`INDI_GAIN_ROLL = 0.0023f` / `INDI_GAIN_PITCH = 0.0033f` N·m·s²/rad — effective moment of inertia (Ixx, Iyy), one constant per axis (not a single shared `INDI_GAIN`).

`accel_cmd` (the rate-PID's output, before the INDI increment) is exposed as an extra `update()` output parameter and logged in the `INDI` log's `AccR`/`AccP` fields — see [SD Card Logging](../../README.md#5-sd-card-logging).

### Gains

The outer angle-P loops match `AttitudePID`'s attitude gains (3.50/0/0). The rate loops do **not** match `AttitudePID`'s rate gains — INDI's rate loop output is a commanded acceleration (rad/s²), not a torque, so it runs much higher gain with a real D-term:

| Loop | Kp | Ki | Kd | D-filter | Imax |
|---|---|---|---|---|---|
| Roll attitude | 3.50 | 0 | 0 | 30 Hz | 0.5 |
| Pitch attitude | 3.50 | 0 | 0 | 30 Hz | 0.5 |
| Roll rate | 6.50 | 0 | 0.65 | 30 Hz | 0.5 |
| Pitch rate | 6.50 | 0 | 0.65 | 30 Hz | 0.5 |
| Yaw rate | 0.065 | 0.02 | 0 | 30 Hz | 0.5 |

`YAW_GAIN = 1.5` scales the yaw rate target before the rate PID — **not** the same constant as `AttitudePID::YAW_STICK_GAIN` (2.5); these are two distinct constants in two distinct classes, easy to conflate.

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

| Loop | Kp | Ki | Kd | D-filter | Imax |
|---|---|---|---|---|---|
| Climb rate | 0.15 | 0.05 | 0 | 20 Hz | 0.3 |

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

| Loop | Kp | Ki | Kd | D-filter | Imax |
|---|---|---|---|---|---|
| Pos N | 1.0 | 0 | 0 | 20 Hz | 0 |
| Pos E | 1.0 | 0 | 0 | 20 Hz | 0 |
| Pos D | 1.0 | 0 | 0 | 20 Hz | 0 |
| Vel N | 2.0 | 1.0 | 0.5 | 20 Hz | 0 |
| Vel E | 2.0 | 1.0 | 0.5 | 20 Hz | 0 |

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

Converts per-motor RPM (DShot GCR telemetry) to physical roll/pitch torques in N·m for INDI feedback. Uses real bench-fit constants (not placeholders).

**Motor force model** — cubic fit against *normalized angular velocity*, not raw RPM directly:
```
omega    = rpm × (π / 30)                       // RPM → rad/s
rpm_norm = (omega − RPM_NORM_CENTER) / RPM_NORM_SCALE      // RPM_NORM_CENTER=2005, RPM_NORM_SCALE=880.8 rad/s
F_N      = C3·rpm_norm³ + C2·rpm_norm² + C1·rpm_norm + C0  // C3=0.0134, C2=0.5607, C1=2.2831, C0=2.4540
F_N      = max(F_N, 0)                          // guard small negative thrust near zero RPM
```

**X-frame geometry** (arm length `ARM_LENGTH_M = 0.1275 m`, NED body frame):
```
roll_Nm  = (ARM_LENGTH_M/√2) × (−F_FR + F_RL + F_FL − F_RR)
pitch_Nm = (ARM_LENGTH_M/√2) × ( F_FR − F_RL + F_FL − F_RR)
```

Signs are consistent with `MotorMixer`: positive roll command spins RL/FL faster, producing positive `roll_Nm`.

**Normalization:** `MAX_THRUST_N = 7.04` (single-motor max from the bench fit) gives `T_MAX_NM = 2·sin(45°)·ARM_LENGTH_M·MAX_THRUST_N ≈ 1.269 N·m`, used to normalize torque to `[-1, 1]` via `normalize_torque()`.

The bench fit also gives motor reaction (drag) torque as a function of thrust (`torque_Nm = 0.03·((F_N − 2.991)/2.183) + 0.0423`), but it isn't wired in since yaw currently uses a rate PID rather than INDI torque feedback — see `Unmixer.hpp`'s doc comment if yaw moves to torque-based control.

---

## MotorMixer (`MotorMixer.hpp/.cpp`)

Converts `[roll_tq, pitch_tq, yaw_tq, thrust]` (all normalised) to four motor commands (0–1000) using an X-frame mixing matrix.

All motors output 0 when disarmed or when |roll| or |pitch| exceeds ~80°.

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

---

## PID base class (`PID.hpp/.cpp`)

All controllers use the same `PID` class with derivative filtering and anti-windup.

```cpp
PID(float kp, float ki, float kd, float imax, float fcut_hz = 30.0f);
float update(float error);
void  reset();
void  set_gains(float kp, float ki, float kd);
```

There is **no explicit `dt` parameter** — `update()` derives it internally from `chVTGetSystemTimeX()` between calls. Two edge cases are handled specially:

- **First sample** (`_deriv_valid == false`): returns `kp * error` only (no I or D term), and latches the timestamp — avoids a derivative spike from an undefined previous error.
- **Stale input** (gap since the last call `> 200 ms`, `STALE_TIMEOUT_US`): calls `reset()` (zeroes the integrator and derivative state) and returns `kp * error` only, same as the first-sample case — protects against a large derivative/integral kick if a caller stops calling `update()` for a while and then resumes.

Derivative is filtered with a first-order IIR at `fcut_hz` (default 30 Hz). The integrator is clamped to `±imax`.

---

## TODOs

- **Gain tuning** — all gains are untested on H7 hardware.
- **Yaw hold** — POS_HOLD currently commands yaw *rate*, not yaw *angle*. An outer yaw loop requires a heading reference from the IMX5 or magnetometer.
