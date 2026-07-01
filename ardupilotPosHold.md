# ArduPilot Position Hold & Loiter: Controller Architecture

A deep-dive into how Loiter and PosHold work, how they talk to the attitude controller, how pilot input is handled, and how switching is managed.

---

## 1. Overview and File Map

| Component | File(s) |
|---|---|
| **Loiter flight mode** | `ArduCopter/mode_loiter.cpp` |
| **PosHold flight mode** | `ArduCopter/mode_poshold.cpp` |
| **Loiter navigation controller** | `libraries/AC_WPNav/AC_Loiter.cpp/.h` |
| **Position controller (XY + Z)** | `libraries/AC_AttitudeControl/AC_PosControl.cpp/.h` |
| **Attitude controller** | `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp/.h` |
| **Mode base class + switching** | `ArduCopter/mode.cpp`, `ArduCopter/mode.h` |

Both modes build on the same underlying stack:

```
Pilot RC Input
     │
     ▼
Flight Mode (Loiter or PosHold)
     │
     ▼
AC_Loiter  (loiter_nav)         ←── only when in LOITER sub-state
     │
     ▼
AC_PosControl  (pos_control)
     │   Position P → Velocity PID → Acceleration target
     │   → accel_to_lean_angles() → roll_target, pitch_target
     ▼
AC_AttitudeControl  (attitude_control)
     │   Quaternion error → body rate targets
     │   → rate PID (gyro feedback)
     ▼
Motor Mixer → ESCs
```

The vertical (Z) axis uses a parallel P→PID chain in `AC_PosControl` but feeds into throttle, not lean angles.

---

## 2. Loiter Mode (`mode_loiter.cpp`)

Loiter is the simpler of the two modes. Every call to `ModeLoiter::run()` (100 Hz+) does:

### 2.1 Pilot Input Processing

```cpp
// mode_loiter.cpp:99-102
get_pilot_desired_lean_angles(target_roll, target_pitch,
    loiter_nav->get_angle_max_cd(),
    attitude_control->get_althold_lean_angle_max_cd());
loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
```

`get_pilot_desired_lean_angles()` reads the RC channel stick positions (normalised ±1), runs them through `rc_input_to_roll_pitch()`, and scales them to a centidegree lean angle command bounded by `LOIT_ANG_MAX` (defaults to 2/3 of `ANGLE_MAX`).

`loiter_nav->set_pilot_desired_acceleration()` converts those Euler lean angles to a North-East frame acceleration using the current yaw, stores it as `_desired_accel`, and also runs a **predictive model** of what the attitude controller will actually achieve:

```cpp
// AC_Loiter.cpp:155-168
_attitude_control.input_shaping_rate_predictor(angle_error, _predicted_euler_rate, dt);
_predicted_euler_angle += _predicted_euler_rate * dt;
const Vector3f predicted_euler { ... };
const Vector3f predicted_accel = _pos_control.lean_angles_to_accel(predicted_euler);
_predicted_accel = predicted_accel.xy();
```

This predicted acceleration is used as **feedforward** in `calc_desired_velocity()`, compensating for the lag between commanding an angle and the vehicle actually reaching it.

### 2.2 Loiter Controller (`AC_Loiter::update()`)

Called each loop:

```cpp
void AC_Loiter::update(bool avoidance_on)
{
    calc_desired_velocity(avoidance_on);
    _pos_control.update_xy_controller();
}
```

**`calc_desired_velocity()`** (AC_Loiter.cpp:210):
1. Gets current loiter speed limits from EKF and `LOIT_SPEED` parameter.
2. Retrieves `_pos_control.get_vel_desired_cms().xy()` as the running desired velocity.
3. Integrates `_predicted_accel * dt` into it (feedforward from pilot input).
4. When sticks are released (`_desired_accel == 0`) and after `LOIT_BRK_DELAY`, applies a **braking deceleration** using a sqrt controller against current speed, jerk-limited by `LOIT_BRK_JERK`. A **drag** term proportional to speed is always active.
5. Applies EKF ground speed limit and fence/avoidance limit.
6. Calls `_pos_control.set_pos_vel_accel_xy(desired_pos, desired_vel, _desired_accel)` — provides all three kinematic quantities as feedforward to the position controller.

### 2.3 Position Controller (`AC_PosControl::update_xy_controller()`)

The XY position control chain (AC_PosControl.cpp:654):

```
_pos_target = _pos_desired + _pos_offset          (position target)
      │
      ▼
P controller on position error
      → vel_target  (correction velocity)
      │
      + _vel_desired + _vel_offset                  (feedforward from loiter)
      ▼
PID controller (_pid_vel_xy) on velocity error
      → accel_target  (correction acceleration)
      │
      + _accel_desired + _accel_offset              (feedforward from loiter)
      ▼
Clamp to max lean angle
      ▼
accel_to_lean_angles()
      → _roll_target, _pitch_target  (centidegrees)
```

`accel_to_lean_angles()` (AC_PosControl.cpp:1644) rotates the NE acceleration into body frame (using current yaw), then converts each axis via `accel_to_angle()` which is an arctan-based conversion accounting for gravity.

### 2.4 Attitude Controller Input

Loiter passes the result to the attitude controller as a **3D thrust vector**:

```cpp
// mode_loiter.cpp:189
attitude_control->input_thrust_vector_rate_heading(
    loiter_nav->get_thrust_vector(), target_yaw_rate, false);
```

`get_thrust_vector()` builds a 3D unit vector from `_roll_target` and `_pitch_target` computed in step 2.3. The thrust vector approach is geometrically cleaner than Euler angles for large lean angles — it avoids gimbal lock and allows the attitude controller to properly account for coupled roll/pitch dynamics.

Inside `input_thrust_vector_rate_heading()` (AC_AttitudeControl.cpp:674):
- Converts thrust vector to a quaternion attitude via `attitude_from_thrust_vector()`.
- Computes quaternion error between desired and current attitude.
- Runs through input shaping to get body-rate targets `_ang_vel_target`.
- Calls `attitude_controller_run_quat()` → runs roll/pitch/yaw rate PIDs → motor mixer output.

### 2.5 Vertical (Z) Axis

```cpp
// mode_loiter.cpp:200,205
pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);  // set desired Z from pilot climb rate
pos_control->update_z_controller();   // runs Z P→PID chain → throttle output
```

The Z controller is a separate P→PID chain whose output goes directly to the motor throttle command, not to lean angles.

---

## 3. PosHold Mode (`mode_poshold.cpp`)

PosHold tries to give the feel of manual control while providing position hold when the pilot releases the sticks. It does this with a **per-axis state machine** that runs independently for roll and pitch.

### 3.1 State Machine States (`RPMode`)

```
PILOT_OVERRIDE
      │ sticks return to center
      ▼
BRAKE
      │ velocity braked to near-zero (timeout)
      ▼
BRAKE_READY_TO_LOITER   (both axes must reach this simultaneously)
      │
      ▼
BRAKE_TO_LOITER         (blended transition ~1.5 s)
      │
      ▼
LOITER                  (full AC_Loiter position hold)
      │
      │ pilot moves stick → CONTROLLER_TO_PILOT_OVERRIDE
      ▼
CONTROLLER_TO_PILOT_OVERRIDE   (smooth blend back to pilot, ~0.5 s)
      │
      ▼
PILOT_OVERRIDE
```

The roll and pitch axes run their state machines independently. The **combined** states `BRAKE_TO_LOITER` and `LOITER` are shared (roll_mode drives both once both axes reach `BRAKE_READY_TO_LOITER`).

### 3.2 PILOT_OVERRIDE State

```cpp
// mode_poshold.cpp:191-204
update_pilot_lean_angle(pilot_roll, target_roll);
// exit to BRAKE if sticks centered and pilot_roll is small
if (is_zero(target_roll) && (fabsf(pilot_roll) < 2 * g.poshold_brake_rate)) {
    roll_mode = RPMode::BRAKE;
    ...
}
roll = pilot_roll + wind_comp_roll;
```

`update_pilot_lean_angle()` implements **stick release smoothing**: large or reversing inputs pass through immediately; near-zero stick input causes the filtered lean angle to decay toward zero at a rate set by `PHLD_BRAKE_RATE`. This prevents the vehicle from snapping to level when the pilot eases off the stick.

### 3.3 BRAKE State

```cpp
// mode_poshold.cpp:210-246
update_brake_angle_from_velocity(brake.roll, vel_right);
```

`update_brake_angle_from_velocity()` (mode_poshold.cpp:527):

```cpp
lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (fabsf(velocity) + 60.0f));
brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);
brake_angle = constrain_float(brake_angle, -PHLD_BRAKE_ANGLE_MAX, PHLD_BRAKE_ANGLE_MAX);
```

This is a nonlinear proportional controller on body-frame velocity that commands a lean angle opposing motion. The `500/(|vel|+60)` term gives higher gain at low velocities (more aggressive stopping as you slow down). The brake_rate slews the output to prevent jerk.

Braking timeout is estimated from the current lean angle and brake_rate; if speed drops below 10 cm/s it is cut to 0.5 s. When timeout expires → `BRAKE_READY_TO_LOITER`.

### 3.4 BRAKE_TO_LOITER State (Blend)

Once both axes are `BRAKE_READY_TO_LOITER`, the transition begins:

```cpp
// mode_poshold.cpp:404-415
float brake_to_loiter_mix = (float)brake.to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;
update_brake_angle_from_velocity(brake.roll, vel_right);
loiter_nav->update(false);
roll = mix_controls(brake_to_loiter_mix, brake.roll + wind_comp_roll, loiter_nav->get_roll());
pitch = mix_controls(brake_to_loiter_mix, brake.pitch + wind_comp_pitch, loiter_nav->get_pitch());
```

`mix_controls()` is a simple linear interpolation: at `brake_to_loiter_mix = 1.0` it is 100% brake output; at 0.0 it is 100% loiter output. The timer counts down over `~1.5 s` (150×4 ticks at 400 Hz). This prevents the jarring transition from a braking lean to a position-hold lean.

### 3.5 LOITER State

In full loiter:

```cpp
// mode_poshold.cpp:442-446
loiter_nav->update(false);
roll = loiter_nav->get_roll();
pitch = loiter_nav->get_pitch();
update_wind_comp_estimate();
```

`loiter_nav` here is the same `AC_Loiter` object used by Loiter mode. `loiter_nav->get_roll()` returns `_pos_control.get_roll_cd()` — the centidegree roll target computed by `update_xy_controller()`.

Wind compensation is updated only during full loiter (not during brake), because the position controller's acceleration demand reflects the steady-state bias needed to hold position.

### 3.6 CONTROLLER_TO_PILOT_OVERRIDE State

When the pilot moves a stick while in BRAKE_TO_LOITER or LOITER:

```cpp
// mode_poshold.cpp:615-622
void ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    pilot_roll = 0.0f;
    controller_final_roll = roll;   // snapshot of current loiter output
}
```

The timer counts down from ~50×4 ticks (~0.5 s). During this time:

```cpp
controller_to_pilot_roll_mix = (float)controller_to_pilot_timer_roll / POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
roll = mix_controls(controller_to_pilot_roll_mix, controller_final_roll, pilot_roll + wind_comp_roll);
```

At timer=max → 100% previous loiter output. At timer=0 → 100% pilot input. Prevents a sudden jerk when the pilot takes back control from loiter.

### 3.7 Attitude Controller Input

PosHold feeds the attitude controller via Euler angles (not thrust vector):

```cpp
// mode_poshold.cpp:488
attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);
```

Inside `input_euler_angle_roll_pitch_euler_rate_yaw()` (AC_AttitudeControl.cpp:345), with feedforward enabled (`_rate_bf_ff_enabled`):
- Uses a **sqrt controller** (`input_shaping_angle`) to compute target Euler rates that will smoothly drive the attitude to the commanded angles without overshoot.
- These Euler rates are converted to body-frame angular velocity targets.
- Rate PIDs run against gyro feedback.
- Output goes to the motor mixer.

### 3.8 Wind Compensation

```cpp
// mode_poshold.cpp:559-595
void ModePosHold::update_wind_comp_estimate()
```

- Only runs when horizontal speed < 10 cm/s and the timer has expired (no estimate in the first `POSHOLD_WIND_COMP_START_TIMER` ticks after entering loiter).
- Samples `pos_control->get_accel_target_cmss()` — the position controller's commanded acceleration. When hovering with zero velocity, this acceleration demand represents exactly the horizontal force needed to resist the wind.
- Applies a very slow IIR low-pass filter (`TC = 0.0025`, meaning ~time constant ~400 loops ≈ 1 second) to estimate the steady-state wind-induced acceleration.
- Limits to 2/3 of `ANGLE_MAX` to ensure the pilot can always override.
- `get_wind_comp_lean_angles()` rotates the NE frame estimate into body-frame roll/pitch at 10 Hz.

This estimate is added to lean angle output in PILOT_OVERRIDE and BRAKE states so the vehicle doesn't drift when the pilot tries to hold still.

---

## 4. Key Differences: Loiter vs. PosHold

| Aspect | Loiter | PosHold |
|---|---|---|
| **Pilot feel** | Position hold at all times; stick = velocity command | Manual control feel; position hold only on stick release |
| **Pilot input path** | `set_pilot_desired_acceleration()` → feedforward into AC_Loiter | State-dependent: lean angle (OVERRIDE), velocity brake (BRAKE), or AC_Loiter (LOITER) |
| **Attitude ctrl interface** | Thrust vector (`input_thrust_vector_rate_heading`) | Euler angles (`input_euler_angle_roll_pitch_euler_rate_yaw`) |
| **Braking** | Automatic via drag + jerk-limited deceleration in `calc_desired_velocity()` | Explicit velocity-proportional lean angle in BRAKE state |
| **Blend on transition** | None (position hold always active) | BRAKE_TO_LOITER blend (~1.5 s), CONTROLLER_TO_PILOT blend (~0.5 s) |
| **Wind comp** | Implicit — position controller I-term handles it | Explicit `wind_comp_ef` estimate carried into PILOT_OVERRIDE and BRAKE |
| **State complexity** | Single state, one code path | Per-axis state machine, 6 states |

---

## 5. Full Control Chain Summary

### Loiter

```
RC Channels (roll, pitch, throttle, yaw)
    │
    ▼ get_pilot_desired_lean_angles()
Lean angle commands (centidegrees)
    │
    ▼ AC_Loiter::set_pilot_desired_acceleration()
Desired NE accel (+ attitude predictor feedforward)
    │
    ▼ AC_Loiter::calc_desired_velocity()
Desired NE position, velocity, acceleration (with drag/brake/avoidance)
    │
    ▼ AC_PosControl::set_pos_vel_accel_xy()  [feedforward]
    + AC_PosControl::update_xy_controller()   [closed-loop P+PID]
Lean angle targets: _roll_target, _pitch_target (centidegrees)
    │
    ▼ get_thrust_vector() → attitude_control->input_thrust_vector_rate_heading()
Quaternion attitude target + body rate targets
    │
    ▼ attitude_controller_run_quat()
Roll/Pitch/Yaw rate PIDs (gyro feedback)
    │
    ▼ Motor Mixer → Motor PWM
```

Vertical axis in parallel:
```
Throttle RC → get_pilot_desired_climb_rate() → set_pos_target_z_from_climb_rate_cm()
    → update_z_controller() → throttle output to motors
```

### PosHold

```
RC Channels
    │
    ▼ get_pilot_desired_lean_angles()
Lean angle commands
    │
    ▼ RPMode State Machine (per axis: roll, pitch)
    │
    ├─ PILOT_OVERRIDE:  filtered lean angle + wind_comp
    │
    ├─ BRAKE:  velocity-proportional counter-lean + wind_comp
    │
    ├─ BRAKE_TO_LOITER:  blend(brake_lean, loiter_nav->get_roll/pitch())
    │
    ├─ LOITER:  loiter_nav->update() → loiter_nav->get_roll/pitch()
    │              (same AC_Loiter + AC_PosControl chain as Loiter mode)
    │
    └─ CONTROLLER_TO_PILOT_OVERRIDE:  blend(snapshot_loiter_lean, pilot_lean)
    │
    ▼ roll, pitch (centidegrees, constrained to ANGLE_MAX)
    │
    ▼ attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw()
Euler rate targets → body rate targets
    │
    ▼ attitude_controller_run_quat()
Roll/Pitch/Yaw rate PIDs
    │
    ▼ Motor Mixer → Motor PWM
```

---

## 6. Mode Switching

### How Switching is Triggered

Mode changes are requested by:
- **Pilot RC switch** (typically channel 5, mapped via `FLTMODE1`–`FLTMODE6` parameters)
- **GCS command** (MAVLink `SET_MODE` message)
- **Failsafes** (battery, radio, GPS, fence breach)

All calls funnel through `Copter::set_mode()` (`ArduCopter/mode.cpp:240`).

### Switching Logic (`Copter::set_mode()`)

```
set_mode(new_mode, reason)
    │
    ├─ Already in mode? → return true
    ├─ GCS mode change disabled? → return false
    ├─ GPS required but position not OK? → return false
    ├─ EKF altitude not available (switching from manual throttle)? → return false
    ├─ Throttle too high when switching to manual throttle mode? → return false
    ├─ Fence recovery in progress? → return false
    │
    ▼ new_flightmode->init(ignore_checks)
    │  (init() sets up controllers, resets targets — returns false if preconditions fail)
    │
    ▼ exit_mode(old_mode, new_mode)
    │  (cleanup for old mode)
    │
    ▼ flightmode = new_flightmode
    │
    ▼ logger, GCS notify, AP_Notify events
    │
    ▼ attitude_control->set_yaw_rate_tc()  (adjust yaw shaping for new mode)
```

### GPS Requirement

Both Loiter and PosHold require a valid GPS/EKF position estimate:
```cpp
// mode.h
bool requires_GPS() const override { return true; }  // both modes
```

If EKF position is lost in-flight while in either mode, a GPS failsafe will trigger a mode change (usually to AltHold or Land).

### Initialization on Mode Entry

`ModeLoiter::init()`:
- Processes current pilot input to seed `loiter_nav->set_pilot_desired_acceleration()`.
- Calls `loiter_nav->init_target()` → initialises `AC_PosControl` with current position and decaying velocity (smooth capture).
- Initialises the Z controller if not already active.

`ModePosHold::init()`:
- Sets initial sub-state based on landing status: landed → LOITER, flying → PILOT_OVERRIDE.
- Calls `loiter_nav->init_target()`.
- Resets wind compensation estimate.
- Computes `brake.gain` from `PHLD_BRAKE_RATE`.

---

## 7. Parameter Reference

| Parameter | Affects | Default |
|---|---|---|
| `LOIT_SPEED` | Loiter max horizontal speed | 1250 cm/s |
| `LOIT_ACC_MAX` | Loiter position correction acceleration | 500 cm/s² |
| `LOIT_BRK_ACCEL` | Loiter braking acceleration | 250 cm/s² |
| `LOIT_BRK_JERK` | Loiter braking jerk limit | 500 cm/s³ |
| `LOIT_BRK_DELAY` | Delay before braking starts after stick release | 1.0 s |
| `LOIT_ANG_MAX` | Loiter max pilot lean angle (0 = 2/3 ANGLE_MAX) | 0 |
| `PHLD_BRAKE_RATE` | PosHold braking rate (cd/s) | 8 cd/s |
| `PHLD_BRAKE_ANGLE` | PosHold max braking lean angle | 3000 cd |
| `PSC_POSXY_P` | Position controller XY P gain | — |
| `PSC_VELXY_P/I/D` | Velocity controller XY PID gains | — |
| `PSC_POSZ_P` | Position controller Z P gain | — |
| `PSC_VELZ_P` | Velocity controller Z P gain | — |
| `ATC_INPUT_TC` | Attitude controller input time constant | 0.15 s |
| `ATC_RATE_FF_ENAB` | Enable rate feedforward in attitude controller | 1 |

---

## 8. Notes for BPRL Modifications

- **PosHold's BRAKE lean angle** is computed purely from inertial velocity — it does not use the position controller at all. If you are injecting force disturbances via strain measurements, the BRAKE state will not compensate for them; only the LOITER state will.
- **The wind_comp estimate** is a very slow low-pass filter designed for quasi-static disturbances (wind). It would not track dynamic load changes fast enough to be useful for strain-based control.
- **The thrust vector interface** used by Loiter (`input_thrust_vector_rate_heading`) is the geometrically correct path for large lean angles and is preferred over the Euler interface for any autonomous path-following.
- **`AC_PosControl::update_xy_controller()`** is where the closed-loop position and velocity PIDs live. Strain-based feedforward could be injected at the `_accel_desired` level via `set_pos_vel_accel_xy()` or by overriding `input_accel_xy()`.
- The custom **`update_z_controller_strain()`** function already added to `AC_PosControl` mirrors this pattern for the Z axis.
