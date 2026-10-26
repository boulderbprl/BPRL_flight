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
 * Channel index assignment comes from kRcMaps[kDroneConfig.rc_tx] (see
 * configs/DroneConfig.hpp) — one layout per physical transmitter, shared by
 * every drone that flies with it:
 *   thr, roll, pitch, yaw  sticks → [0, 1] / [-1, 1]
 *   arm                    arm switch → >992 = armed
 *   ctrl_select            3-position: which attitude controller flies —
 *                          ControllersConfig::switch_pos[low/mid/high]
 *   poshold_switch         high = POS_HOLD, low/mid = STABILIZE
 *   strain_cal_switch      momentary strain-bias-cal switch → >992 = held;
 *                          see JerkFit.hpp / ControlThread's strain calibration block
 *
 * The sysid switch is per drone instead (SysIdConfig::rc_channel, -1 = off).
 *
 * Both SBUS and CRSF use the same 11-bit value range: 172–1811, centre 992.
 */
static const RcChannelMap &rc() { return kRcMaps[static_cast<uint8_t>(kDroneConfig.rc_tx)]; }

static float norm_axis(uint16_t v) { return (float)(v - 992)  / 819.0f;  }
static float norm_thr (uint16_t v) { return (float)(v - 172)  / 1639.0f; }

void  radio_input_init()   { PARSER.init();   }
void  radio_input_update() { PARSER.update(); }

float radio_thr()         { return norm_thr (PARSER.channel(rc().thr)); }
float radio_roll()        { return norm_axis(PARSER.channel(rc().roll)); }
float radio_pitch()       { return -norm_axis(PARSER.channel(rc().pitch)); }
float radio_yaw()         { return norm_axis(PARSER.channel(rc().yaw)); }
bool  radio_armed()       { return PARSER.channel(rc().arm) > 992u; }
#if RADIO_PROTOCOL == RADIO_PROTO_CRSF
bool  radio_valid()       { return PARSER.data_valid(); }
#else // RADIO_PROTO_SBUS — no single _valid flag; frame_lost/failsafe stand in for it
bool  radio_valid()       { return !PARSER.frame_lost() && !PARSER.failsafe(); }
#endif
float radio_flight_mode() { return norm_axis(PARSER.channel(rc().poshold_switch)); }
float radio_indi()        { return norm_axis(PARSER.channel(rc().ctrl_select)); }
bool  radio_strain_cal()  { return PARSER.channel(rc().strain_cal_switch) > 992u; }
bool  radio_sysid()
{
    const int8_t ch = kDroneConfig.sysid.rc_channel;
    return ch >= 0 && PARSER.channel((uint8_t)ch) > 992u;
}

int radio_switch_position()
{
    const float v = radio_indi();
    if (v < -0.33f) return 0;
    if (v >  0.33f) return 2;
    return 1;
}
