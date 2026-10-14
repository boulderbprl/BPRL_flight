#pragma once
#include <cstdint>

/*
 * Trajectory tracker — the layer above PosControl/AltControl/HeadingControl.
 * Same role as ArduPilot's AC_Circle / Guided over AC_PosControl: it turns a
 * commanded point or shape into the moving reference (position, velocity,
 * acceleration, heading) those controllers track. It never touches the
 * feedback loops themselves.
 *
 * Runs inside POS_HOLD (see FlightStateMachine::mode_pos_hold()). While it
 * is active the three controllers track its reference instead of the
 * sticks; the moment it goes idle they are back on the sticks, carrying on
 * from the last reference.
 *
 * Frame: everything is NED, metres, radians, D positive down. Commanded
 * positions are offsets from an origin set by command (set_origin()) to
 * wherever the vehicle is at that moment. Nothing starts until an origin
 * has been set.
 *
 * Commands (TrajCommand, sent over MAVLink — see src/coms/MAVLink.cpp for
 * the wire format). A NaN parameter means "not given"; zero is a value.
 *
 *   POINT   p[0..2] = N, E, D offset from the origin.
 *           Not given: N, E → 0 (the origin); D → the current altitude.
 *           Flies a straight line there and holds. Heading stays where it
 *           was when the command arrived.
 *
 *   CIRCLE  p[0]    = radius r [m]                               (required)
 *           p[1..3] = centre N, E, D offset from the origin      (as POINT)
 *           p[4]    = focus distance f [m]: 0 / not given → circle.
 *                     Otherwise an ellipse centred on the centre above with
 *                     its foci |f| from the centre — along North if f > 0,
 *                     along East if f < 0. r is then the distance from a
 *                     focus to the near end of the ellipse, so
 *                       semi-major a = r + |f|,  semi-minor b = sqrt(r·(r + 2|f|))
 *           p[5]    = direction: >= 0 / not given → positive rotation about
 *                     D (right-hand rule: yaw increasing, clockwise seen
 *                     from above); < 0 → the other way.
 *           p[6]    = speed [m/s] on the straight parts; not given or <= 0
 *                     → V_DEFAULT. Reduced automatically where the path
 *                     curves tightly (see A_LAT_MAX, YAW_RATE_MAX).
 *           Flies to the nearest point of the path, turns the nose along
 *           it, then follows it nose-first until told otherwise.
 *
 * Pilot interaction while a trajectory is active:
 *   roll / pitch / yaw stick out of its deadband → the trajectory is
 *       cancelled and stays cancelled; a new command is needed to restart.
 *   throttle stick → raises/lowers the whole trajectory (a height offset on
 *       top of the commanded D). The origin does not move. The offset is
 *       zeroed by each new command.
 *
 * Floor: FlightStateMachine records the D the vehicle took off from. A
 * point or circle whose altitude would be lower than FLOOR_MARGIN_M above
 * it is refused (BAD_PARAM) and changes nothing — a trajectory already
 * running carries on. The throttle stick cannot push a running trajectory
 * below that height either; there it simply stops descending.
 *
 * If the vehicle falls behind the reference (position error past
 * ERR_SLOW_M), progress along the path slows, and stops at ERR_PAUSE_M —
 * the path waits for the vehicle rather than being cut short by a leash.
 */

enum class TrajCmdType : uint8_t { SET_ORIGIN, POINT, CIRCLE, STOP };

struct TrajCommand {
    TrajCmdType type;
    float       p[7];
};

enum class TrajResult : uint8_t {
    ACCEPTED,
    NOT_READY,   // right command, wrong time: not flying in POS_HOLD, no valid position, or (set-origin) a trajectory is active
    NO_ORIGIN,   // point/circle before any set-origin
    BAD_PARAM,   // missing or out-of-range parameter, or an altitude below the floor limit
};

enum class TrajState : uint8_t {
    IDLE  = 0,   // not active — POS_HOLD is on the sticks
    MOVE  = 1,   // straight line to a point (a POINT command, or the approach to a circle)
    HOLD  = 2,   // POINT reached
    ORBIT = 3,   // following the circle/ellipse
};

// What the controllers are asked to track this tick.
struct TrajRef {
    float pos[3];     // N, E, D [m]
    float vel[3];     // N, E, D [m/s]
    float acc[2];     // N, E feed-forward [m/s²]
    float yaw;        // heading [rad]
    float yaw_rate;   // heading feed-forward [rad/s]
};

class TrajectoryTracker {
public:
    // Origin := pos. Refused while a trajectory is active (the path would jump).
    TrajResult set_origin(const float pos[3]);

    // Take-off D, recorded by FlightStateMachine as the vehicle leaves the
    // ground. clear_floor() when it took off without a valid position.
    void set_floor(float floor_D) { _floor_D = floor_D; _floor_valid = true; }
    void clear_floor()            { _floor_valid = false; }

    // Start a trajectory from p[] (see the command table above). hold is
    // the N, E, D the vehicle is holding now (POS_HOLD's current targets) —
    // where the reference starts from, and the "current altitude" a
    // not-given D means. yaw is the current heading.
    TrajResult start_point(const float p[7], const float hold[3], float yaw);
    TrajResult start_circle(const float p[7], const float hold[3], float yaw);

    void cancel() { _state = TrajState::IDLE; }

    // One control tick. Returns false (ref untouched) when idle, including
    // when this call's sticks have just cancelled the trajectory.
    //   pos, vel   N, E, D [m], [m/s]
    //   sticks     roll/pitch/yaw normalised [-1, 1], thr [0, 1]
    bool update(const float pos[3], const float vel[3], float yaw,
                float stick_roll, float stick_pitch, float stick_yaw, float stick_thr,
                TrajRef &ref);

    bool      active()     const { return _state != TrajState::IDLE; }
    bool      origin_set() const { return _origin_set; }
    TrajState state()      const { return _state; }

    // For logging.
    float height_offset() const { return _height_offset; }   // m, D (negative = raised)
    float path_speed()    const { return _state == TrajState::ORBIT ? _speed : _speed_pred; }   // m/s the reference is moving along the path

    // ── Limits — starting values, flight-tune here ──────────────────────────
    static constexpr float V_DEFAULT      = 0.5f;    // m/s, circle speed when none is given
    static constexpr float V_MAX          = 2.0f;    // m/s, cap on a commanded circle speed
    static constexpr float V_MOVE         = 0.5f;    // m/s, straight-line speed to a point / to the start of a circle
    static constexpr float V_VERT_MAX     = 0.5f;    // m/s, climb/descent speed toward a commanded D
    static constexpr float A_TAN_MAX      = 1.0f;    // m/s², speeding up / slowing down along the path
    static constexpr float A_VERT_MAX     = 1.0f;    // m/s², same, vertically
    static constexpr float A_LAT_MAX      = 2.0f;    // m/s², turning (PosControl's 30° lean limit allows ~5.7; ArduPilot's AC_Circle also keeps to under half)
    static constexpr float YAW_RATE_MAX   = 0.8f;    // rad/s, how fast the path may ask the nose to turn — keep below HeadingGains::max_rate
    static constexpr float RADIUS_MIN_M   = 0.3f;
    static constexpr float RADIUS_MAX_M   = 10.0f;   // also the cap on the semi-major axis, r + |f|
    static constexpr float OFFSET_MAX_M   = 20.0f;   // cap on any commanded N/E/D offset from the origin
    static constexpr float FLOOR_MARGIN_M = 0.15f;   // m, lowest a trajectory may go above the take-off height: commands below it are refused, the throttle stick stops at it
    static constexpr float ERR_SLOW_M     = 0.5f;    // m, position error at which progress along the path starts to slow
    static constexpr float ERR_PAUSE_M    = 1.0f;    // m, ... and at which it stops
    static constexpr float ARRIVE_POS_M   = 0.15f;   // m, |  vehicle must be this close to the start of the circle,
    static constexpr float ARRIVE_VEL_MS  = 0.2f;    // m/s, | this slow,
    static constexpr float ARRIVE_YAW_RAD = 0.26f;   // rad, | and pointing this close to along it, before the orbit starts (15°)

private:
    struct PathPoint {
        float pos[2];    // N, E
        float d1[2];     // dp/dθ
        float d2[2];     // d²p/dθ²
        float len;       // |dp/dθ| [m/rad]
        float kappa;     // curvature [1/m]
    };

    void       begin(const float hold[3]);
    bool       base_altitude(float d_offset, const float hold[3], float &goal_D) const;
    PathPoint  path_at(float theta) const;
    float      speed_limit(const PathPoint &pt) const;
    void       update_move(float scale, TrajRef &ref);
    void       update_orbit(float scale, TrajRef &ref);
    void       update_vertical(float stick_thr, TrajRef &ref);

    TrajState _state      = TrajState::IDLE;
    bool      _is_circle  = false;

    bool  _origin_set  = false;
    float _origin[3]   = {};
    bool  _floor_valid = false;
    float _floor_D     = 0.0f;

    // Reference state.
    float _ref_ne[2]     = {};     // horizontal reference position [m]
    float _speed         = 0.0f;   // commanded speed along the path [m/s]
    float _speed_pred    = 0.0f;   // MOVE: _speed lagged by the attitude loop's response — what the reference actually moves at
    float _ref_D         = 0.0f;   // altitude reference before the pilot's offset [m]
    float _vspeed        = 0.0f;   // its rate [m/s]
    float _height_offset = 0.0f;   // pilot's throttle-stick offset on top of _ref_D [m]

    // MOVE / HOLD.
    float _goal_ne[2] = {};
    float _goal_D     = 0.0f;
    float _hold_yaw   = 0.0f;      // heading while moving: as found (POINT) or along the path at its start (CIRCLE)

    // ORBIT: p(θ) = _centre + (_rN·cosθ, _rE·sinθ); θ increasing turns N → E.
    float _centre[2]       = {};
    float _rN              = 0.0f;
    float _rE              = 0.0f;
    float _dir             = 1.0f;   // ±1, sign of dθ/dt
    float _v_cmd           = 0.0f;
    float _theta           = 0.0f;
    float _theta_rate_prev = 0.0f;

    static constexpr float MOVE_KP        = 2.0f;   // 1/s, final approach to a point (linear end of the sqrt speed profile)
    static constexpr int   NEAREST_STEPS  = 90;     // samples of θ when looking for the nearest point of the path
    static constexpr int   LOOKAHEAD_STEPS = 10;    // samples of the path ahead when deciding how fast it is safe to go
};
