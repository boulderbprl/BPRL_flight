#include "src/coms/Radio.hpp"

#if RADIO_PROTOCOL == RADIO_PROTO_SBUS
  #include "src/coms/SBUS.hpp"
  #define PARSER g_sbus
#elif RADIO_PROTOCOL == RADIO_PROTO_CRSF
  #include "src/coms/CRSF.hpp"
  #define PARSER g_crsf
#else
  #error "Unknown RADIO_PROTOCOL value. Use RADIO_PROTO_SBUS or RADIO_PROTO_CRSF."
#endif

/*
 * Channel index assignment comes from DroneConfig::rc_map (see
 * configs/DroneConfig.hpp) — today's values for both drones (verified
 * against telemetry):
 *   thr         = 0  Throttle         → [0, 1]
 *   roll        = 1  Roll (aileron)   → [-1, 1]
 *   pitch       = 2  Pitch (elevator) → [-1, 1]
 *   yaw         = 3  Yaw (rudder)     → [-1, 1]
 *   arm         = 4  Arm switch       → >992 = armed
 *   flight_mode = 6  Flight mode switch (3-position) → [-1, 1]
 *   indi_switch = 7  Attitude-controller-select switch (3-position) → [-1, 1];
 *                    only the 3rd (highest) position selects the non-default
 *                    controller — see FlightStateMachine::set_active_controller()
 *
 * Both SBUS and CRSF use the same 11-bit value range: 172–1811, centre 992.
 */
static float norm_axis(uint16_t v) { return (float)(v - 992)  / 819.0f;  }
static float norm_thr (uint16_t v) { return (float)(v - 172)  / 1639.0f; }

void  radio_input_init()   { PARSER.init();   }
void  radio_input_update() { PARSER.update(); }

float radio_thr()         { return norm_thr (PARSER.channel(kDroneConfig.rc_map.thr)); }
float radio_roll()        { return norm_axis(PARSER.channel(kDroneConfig.rc_map.roll)); }
float radio_pitch()       { return -norm_axis(PARSER.channel(kDroneConfig.rc_map.pitch)); }
float radio_yaw()         { return norm_axis(PARSER.channel(kDroneConfig.rc_map.yaw)); }
bool  radio_armed()       { return PARSER.channel(kDroneConfig.rc_map.arm) > 992u; }
#if RADIO_PROTOCOL == RADIO_PROTO_CRSF
bool  radio_valid()       { return PARSER.data_valid(); }
#else // RADIO_PROTO_SBUS — no single _valid flag; frame_lost/failsafe stand in for it
bool  radio_valid()       { return !PARSER.frame_lost() && !PARSER.failsafe(); }
#endif
float radio_flight_mode() { return norm_axis(PARSER.channel(kDroneConfig.rc_map.flight_mode)); }
float radio_indi()        { return norm_axis(PARSER.channel(kDroneConfig.rc_map.indi_switch)); }

int radio_switch_position()
{
    const float v = radio_indi();
    if (v < -0.33f) return 0;
    if (v >  0.33f) return 2;
    return 1;
}
