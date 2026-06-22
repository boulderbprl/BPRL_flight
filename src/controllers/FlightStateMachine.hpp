#pragma once
#include <cstdint>
#include "SLCPID.hpp"
#include "INDI.hpp"
#include "Unmixer.hpp"

enum class FlightMode  { STABILIZE, INDI };
enum class FlightPhase { DISARMED, IDLE, ACTIVE };

/*
 * FlightStateMachine — selects and drives the active attitude controller.
 *
 * Called at 400 Hz from ControlThread.  Manages three phases:
 *   DISARMED: arm switch low → motors off, all controllers reset.
 *   IDLE:     armed, throttle at floor → motors spin at PWM_IDLE, controllers
 *             reset each tick so integrators are clean for takeoff.
 *   ACTIVE:   throttle above threshold → full controller running.
 *
 * Flight mode is selected from input[InputIdx::FLIGHT_MODE]:
 *   < 0  → STABILIZE (SLC PID cascade)
 *   ≥ 0  → INDI (incremental NDI, roll/pitch only; yaw/thrust = SLC PID)
 *
 * Output (out_cmds[3], thrust_out) feeds the unchanged MotorMixer.
 */
class FlightStateMachine {
public:
    FlightStateMachine();

    // state_full[19]: full EKF state vector (StateIdx::*)
    // euler[3]:       [roll, pitch, yaw] rad
    // input[5]:       [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
    // armed:          arm switch state from radio
    // rpm[4]:         per-motor mechanical RPM [FR, RL, FL, RR] (for INDI unmixer)
    // out_cmds[3]:    normalised torque [roll, pitch, yaw] → MotorMixer
    // thrust_out:     throttle [0, 1] → MotorMixer
    void update(const float state_full[], const float euler[3],
                const float input[], bool armed,
                const uint32_t rpm[4],
                float out_cmds[3], float &thrust_out);

    FlightPhase phase() const { return _phase; }
    FlightMode  mode()  const { return _mode;  }

    void reset_all();

private:
    FlightPhase _phase = FlightPhase::DISARMED;
    FlightMode  _mode  = FlightMode::STABILIZE;

    SLCPID  _slc;
    INDI    _indi;
    Unmixer _unmixer;

    // Throttle must exceed this to transition IDLE → ACTIVE.
    static constexpr float ACTIVE_THR_THRESH = 0.05f;
};
