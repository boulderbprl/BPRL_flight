#pragma once
#include <cstdint>

/*
 * Three-level health shared by the INDI feedback sensors (EncoderRPM.hpp,
 * StrainGauge.hpp) and FlightStateMachine's INDI -> PID fallback.
 *
 *   GOOD     live data.
 *   HOLDING  a short hiccup (data late or frozen, < 50 ms): the last good
 *            value is held. INDI keeps flying — it tolerates a short hold —
 *            but its NLMS G1 adaptation pauses, since differences taken
 *            across a held stretch are wrong.
 *   BAD      outage longer than 50 ms, or data that is plainly wrong. INDI
 *            fades out to PID; it fades back in once health has been out of
 *            BAD long enough (see FlightStateMachine::set_indi_sensor_health()).
 *
 * The numeric order is the severity order, so the worst of several is the max.
 */
enum class SensorHealth : uint8_t { GOOD = 0, HOLDING = 1, BAD = 2 };

inline SensorHealth worst(SensorHealth a, SensorHealth b) { return (a > b) ? a : b; }
