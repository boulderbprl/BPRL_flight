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
 * Standard Mode 2 channel mapping:
 *   ch[0] = Roll (aileron)   → [-1, 1]
 *   ch[1] = Pitch (elevator) → [-1, 1]
 *   ch[2] = Throttle         → [0, 1]
 *   ch[3] = Yaw (rudder)     → [-1, 1]
 *   ch[4] = Arm switch       → >992 = armed
 *
 * Both SBUS and CRSF use the same 11-bit value range: 172–1811, centre 992.
 */
static float norm_axis(uint16_t v) { return (float)(v - 992)  / 819.0f;  }
static float norm_thr (uint16_t v) { return (float)(v - 172)  / 1639.0f; }

void  radio_input_init()   { PARSER.init();   }
void  radio_input_update() { PARSER.update(); }

float radio_thr()    { return norm_thr (PARSER.channel(2)); }
float radio_roll()   { return norm_axis(PARSER.channel(0)); }
float radio_pitch()  { return norm_axis(PARSER.channel(1)); }
float radio_yaw()    { return norm_axis(PARSER.channel(3)); }
bool  radio_armed()  { return PARSER.channel(4) > 992u;    }
