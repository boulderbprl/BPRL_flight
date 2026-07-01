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
     │      stick → climb_rate_pid → accel_pid → thrust_out              │
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
| ACTIVE | armed | Full controller stack; zero throttle idles props |

Transition ACTIVE → DISARMED resets all controller integrators and the position-hold latch.

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
| Yaw rate | 0.18 | 0.018 | 0 | 30 Hz | 0.5 |

`YAW_GAIN = 1.5` scales the yaw rate target before the rate PID.

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

`current_torque_Nm` is estimated from live DShot RPM telemetry by the **Unmixer**.
`INDI_GAIN = 0.01` N·m·s²/rad — effective moment of inertia (Ixx ≈ Iyy). Update once airframe is characterised.

> **Note:** Motor polynomial coefficients (`MOTOR_P1`–`MOTOR_P4`) in the Unmixer are currently zero (placeholders). In this state `current_torque` is always 0 N·m and INDI reduces to a pure increment from zero — still functional but without physical torque feedback.

### Gains

Same Kp/Ki/Kd values as `AttitudePID`.

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

### ALT_HOLD cascade — `alt_hold_from_stick()`

```
pilot_thr [0,1]
    │
    ▼ stick_to_climb_rate()
    │   centered = pilot_thr − 0.5               [-0.5, +0.5]
    │   deadband = 0.05 of half-range
    │   rate_tgt = ±MAX_CLIMB_RATE (3 m/s) outside deadband
    │
    ▼ climb_rate_pid (rate error → accel_tgt)
    ▼ accel_pid     (accel error → delta_thr)
    │
    thrust_out = constrain(THR_MID − delta_thr, 0, 1)
```

`THR_MID = 0.4`. Stick centred → hold current altitude (rate_tgt = 0).

### POS_HOLD — `alt_hold_from_rate()`

Skips the outer stick-to-rate conversion. Called with the D-axis velocity target from `PosControl::NED_update()`.

### Gains

| Loop | Kp | Ki | Kd | D-filter | Imax |
|---|---|---|---|---|---|
| Climb rate | 2.0 | 0.5 | 0 | 20 Hz | 2.0 |
| Accel | 0.05 | 0.01 | 0 | 20 Hz | 0.3 |

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

Converts per-motor RPM (DShot GCR telemetry) to physical roll/pitch torques in N·m for INDI feedback.

**Motor force model** (cubic polynomial — fill from bench test):
```
F_kg = P1·rpm³ + P2·rpm² + P3·rpm + P4
F_N  = F_kg × 9.81
```

**X-frame geometry** (arm length L = 0.225 m, NED body frame):
```
roll_Nm  = (L/√2) × (−F_FR + F_RL + F_FL − F_RR)
pitch_Nm = (L/√2) × (+F_FR − F_RL + F_FL − F_RR)
```

Signs are consistent with `MotorMixer`: positive roll command spins RL/FL faster, producing positive `roll_Nm`.

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
PID(float kp, float ki, float kd, float i_max, float d_lpf_hz);
float update(float error, float dt_s = 0.0025f);
void  reset();
```

Derivative is filtered with a first-order IIR at `d_lpf_hz`. The integrator is clamped to `±i_max`.

---

## TODOs

- **Unmixer calibration** — `MOTOR_P1`–`MOTOR_P4` must be filled from a bench thrust test before INDI torque feedback is meaningful.
- **Gain tuning** — all gains are untested on H7 hardware.
- **Yaw hold** — POS_HOLD currently commands yaw *rate*, not yaw *angle*. An outer yaw loop requires a heading reference from the IMX5 or magnetometer.
- **Arming logic** — `radio_armed()` returns `false` unconditionally; needs a dedicated switch channel.
