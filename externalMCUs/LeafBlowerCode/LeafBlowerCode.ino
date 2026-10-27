/*
  leaf_blower.ino  —  Leaf blower burst controller, Arduino Nano (ATmega328P)

  Hardware (as reverse-engineered from the lab whiteboard; VERIFY before powering):
    A5  : push button input. Button connects A5 to +5V; external 2k pull-down to GND.
          -> active HIGH. (A5 is used as a plain digital input.)
    D11 : gate of low-side MOSFET #1 (30N06L). Drain -> blower connector Pin 1.
          ON = Pin 1 shorted to GND (Pin 2) = blower runs.  Also drives a 1k + LED indicator.
    D8  : gate of low-side MOSFET #2 (30N06L). Drain -> blower connector Pin 1_B.
          ON = Pin 1_B shorted to GND (Pin 2_B) = boost.   Also drives a 1k + LED indicator.

  Behaviour:
    A button press starts one burst. The main output (D11) is ON for LB_time ms.
    During the burst the boost output (D8) is pulsed Boost_num times, with a total
    boost on-time of Boost_time ms. Each pulse is Boost_time/Boost_num ms long.

    The burst window is split into Boost_num equal slots (each LB_time/Boost_num long),
    and one pulse is placed in each slot. Boost_phase_pct sets where in its slot the
    pulse sits: 0 = start of slot, 50 = centred, 100 = end of slot.
    If Boost_time == LB_time, the pulse fills each slot and boost is on continuously.

    All timing uses exact integer arithmetic (no rounding drift between pulses).
    Resolution is 1 ms (millis()).
*/

#include <Arduino.h>

// ============================== USER SETTINGS ==============================
const uint32_t LB_time         = 1000;  // [ms] how long the leaf blower burst is on
const uint16_t Boost_num       = 3;     // number of boost pulses per burst (0 = no boost)
const uint32_t Boost_time      = 300;   // [ms] TOTAL boost on-time per burst, must be <= LB_time
const uint8_t  Boost_phase_pct = 50;    // pulse position inside its slot: 0 = start, 50 = centre, 100 = end

const bool     BUTTON_ABORTS   = true;  // true: pressing the button during a burst stops it early
const uint32_t COOLDOWN_MS     = 500;   // [ms] dead time after a burst before a new press is accepted
const uint32_t DEBOUNCE_MS     = 30;    // [ms] button must be stable this long to count
const bool     DEBUG_SERIAL    = false;  // print schedule + edge timestamps at 115200 baud
// ===========================================================================

// ------------------------------- Pins --------------------------------------
const uint8_t PIN_BUTTON = A5;  // active HIGH (external pull-down)
const uint8_t PIN_MAIN   = 11;  // main blower MOSFET gate
const uint8_t PIN_BOOST  = 8;   // boost MOSFET gate

// --------------------------- Compile-time checks ----------------------------
static_assert(LB_time > 0, "LB_time must be > 0");
static_assert(Boost_time <= LB_time, "Boost_time must be <= LB_time");
static_assert(Boost_phase_pct <= 100, "Boost_phase_pct must be 0..100");
static_assert((uint64_t)LB_time * (Boost_num == 0 ? 1 : Boost_num) <= 0xFFFFFFFFULL,
              "LB_time * Boost_num overflows 32 bits");
static_assert(Boost_num == 0 || Boost_time == 0 || Boost_time >= Boost_num,
              "Each boost pulse would be shorter than 1 ms");

// ----------------------------- Boost schedule -------------------------------
// Work in units of (1/Boost_num) ms so every boundary is an exact integer:
//   slot length  = LB_time    units   (= LB_time/Boost_num ms)
//   pulse width  = Boost_time units   (= Boost_time/Boost_num ms)
//   free time    = LB_time - Boost_time units per slot
const uint32_t PULSE_START_IN_SLOT =
    (uint32_t)((uint64_t)(LB_time - Boost_time) * Boost_phase_pct / 100);

// Should boost be ON at elapsed time t [ms], 0 <= t < LB_time ?
bool boostActive(uint32_t t) {
  if (Boost_num == 0 || Boost_time == 0) return false;
  if (Boost_time >= LB_time) return true;               // continuous boost
  const uint32_t offset = (t * Boost_num) % LB_time;    // position within current slot
  return offset >= PULSE_START_IN_SLOT && offset < PULSE_START_IN_SLOT + Boost_time;
}

// ------------------------------ Button --------------------------------------
bool     btnStable     = LOW;
bool     btnLastRaw    = LOW;
uint32_t btnLastChange = 0;

// Returns true once per debounced LOW->HIGH transition.
bool buttonPressedEdge(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON);
  if (raw != btnLastRaw) {
    btnLastRaw = raw;
    btnLastChange = now;
  }
  if ((now - btnLastChange) >= DEBOUNCE_MS && raw != btnStable) {
    btnStable = raw;
    return btnStable == HIGH;
  }
  return false;
}

// ---------------------------- State machine ---------------------------------
enum State : uint8_t { IDLE, RUNNING, COOLDOWN };
State    state      = IDLE;
uint32_t stateStart = 0;
bool     boostOut   = false;

void setMain(bool on) { digitalWrite(PIN_MAIN, on ? HIGH : LOW); }

void setBoost(bool on, uint32_t t) {
  if (on == boostOut) return;
  boostOut = on;
  digitalWrite(PIN_BOOST, on ? HIGH : LOW);
  if (DEBUG_SERIAL) {
    Serial.print(F("  t="));
    Serial.print(t);
    Serial.println(on ? F(" ms  boost ON") : F(" ms  boost OFF"));
  }
}

void allOff() {
  digitalWrite(PIN_BOOST, LOW);
  digitalWrite(PIN_MAIN, LOW);
  boostOut = false;
}

void printSchedule() {
  Serial.println(F("Leaf blower controller"));
  Serial.print(F("LB_time=")); Serial.print(LB_time);
  Serial.print(F(" ms, Boost_num=")); Serial.print(Boost_num);
  Serial.print(F(", Boost_time=")); Serial.print(Boost_time);
  Serial.print(F(" ms, phase=")); Serial.print(Boost_phase_pct); Serial.println(F("%"));
  if (Boost_num == 0 || Boost_time == 0) { Serial.println(F("Boost: off")); return; }
  if (Boost_time >= LB_time)             { Serial.println(F("Boost: continuous")); return; }
  const float n = (Boost_num == 0) ? 1.0f : (float)Boost_num;  // (n==0 already returned; avoids a compiler warning)
  for (uint16_t k = 0; k < Boost_num; ++k) {
    // exact start/end in ms = (k*LB_time + offset) / Boost_num
    const float s = ((float)k * LB_time + PULSE_START_IN_SLOT) / n;
    const float e = s + (float)Boost_time / n;
    Serial.print(F("  pulse ")); Serial.print(k + 1);
    Serial.print(F(": ")); Serial.print(s, 1);
    Serial.print(F(" -> ")); Serial.print(e, 1); Serial.println(F(" ms"));
  }
}

void setup() {
  // Drive gates LOW before enabling outputs so the MOSFETs never see a floating gate.
  digitalWrite(PIN_MAIN, LOW);
  digitalWrite(PIN_BOOST, LOW);
  pinMode(PIN_MAIN, OUTPUT);
  pinMode(PIN_BOOST, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);  // external 2k pull-down, so no internal pull-up

  // Seed debouncer with the real button level, so a button held at power-up
  // (or a stuck-high input) does not fire a burst on boot.
  btnStable = btnLastRaw = digitalRead(PIN_BUTTON);
  btnLastChange = millis();

  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    printSchedule();
  }
}

void loop() {
  const uint32_t now = millis();
  const bool pressed = buttonPressedEdge(now);

  if (state == IDLE && pressed) {
    state = RUNNING;
    stateStart = now;
    if (DEBUG_SERIAL) Serial.println(F("Burst start"));
    setMain(true);
    // fall through to RUNNING so boost is evaluated in this same pass
  } else if (state == RUNNING && BUTTON_ABORTS && pressed) {
    allOff();
    state = COOLDOWN;
    stateStart = now;
    if (DEBUG_SERIAL) Serial.println(F("Burst aborted"));
    return;
  }

  if (state == RUNNING) {
    const uint32_t t = now - stateStart;
    if (t >= LB_time) {
      allOff();
      state = COOLDOWN;
      stateStart = now;
      if (DEBUG_SERIAL) { Serial.print(F("Burst end, t=")); Serial.println(t); }
      return;
    }
    setMain(true);
    setBoost(boostActive(t), t);
  } else if (state == COOLDOWN) {
    if (now - stateStart >= COOLDOWN_MS) state = IDLE;
  }
}