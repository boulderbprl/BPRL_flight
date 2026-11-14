/*
  leaf_blower.ino  —  Leaf blower burst controller, Arduino Nano (ATmega328P)

  Hardware (as reverse-engineered from the lab whiteboard; VERIFY before powering):
    A5  : push button input. Button connects A5 to +5V; external 2k pull-down to GND.
          -> active HIGH. (A5 is used as a plain digital input.)
    D2  : RC PWM input (standard servo timing, ~1000-2000 us pulse, ~50 Hz) from an
          RC receiver channel. Receiver signal -> D2, receiver GND -> Nano GND.
          D2 is INT0, so pulse widths are measured in a pin-change interrupt.
    D11 : gate of low-side MOSFET #1 (30N06L). Drain -> blower connector Pin 1.
          ON = Pin 1 shorted to GND (Pin 2) = blower runs.  Also drives a 1k + LED indicator.
    D8  : gate of low-side MOSFET #2 (30N06L). Drain -> blower connector Pin 1_B.
          ON = Pin 1_B shorted to GND (Pin 2_B) = boost.   Also drives a 1k + LED indicator.

  Behaviour:
    RC input (D2):
      While the RC channel is HIGH (pulse >= RC_ON_US) the blower runs in boost mode
      (main + boost ON). When it goes LOW (pulse <= RC_OFF_US) everything turns off.
      The gap between the two thresholds is hysteresis so a stick near centre does
      not chatter. RC overrides the button: going HIGH cancels any burst in progress,
      and button presses are ignored while RC is HIGH.

      Safety:
        - Signal loss (no valid pulse for RC_TIMEOUT_MS) is treated as LOW.
        - The RC input must be seen LOW at least once after power-up, and again after
          any signal loss, before a HIGH is honoured. So a switch left HIGH at boot,
          or a receiver reconnecting with the switch HIGH, does not start the blower.
        - Pulses outside RC_MIN_VALID_US..RC_MAX_VALID_US are ignored as glitches.
        - Set the receiver's own failsafe for this channel to LOW (or "no pulses").

    Button (A5):
      A press starts one burst. The main output (D11) is ON for LB_time ms.
      During the burst the boost output (D8) is pulsed Boost_num times, with a total
      boost on-time of Boost_time ms. Each pulse is Boost_time/Boost_num ms long.

      The burst window is split into Boost_num equal slots (each LB_time/Boost_num long),
      and one pulse is placed in each slot. Boost_phase_pct sets where in its slot the
      pulse sits: 0 = start of slot, 50 = centred, 100 = end of slot.
      If Boost_time == LB_time, the pulse fills each slot and boost is on continuously.

      All timing uses exact integer arithmetic (no rounding drift between pulses).
      Resolution is 1 ms (millis()).

    Debug (DEBUG_SERIAL = true, 115200 baud):
      Prints the schedule at boot, every state change (RC transitions include the
      pulse width), and a status line with the current RC pulse width every
      DEBUG_RC_PRINT_MS.
*/

#include <Arduino.h>

// ============================== USER SETTINGS ==============================
const uint32_t LB_time         = 1100;  // [ms] how long the leaf blower burst is on
const uint16_t Boost_num       = 1;     // number of boost pulses per burst (0 = no boost)
const uint32_t Boost_time      = LB_time;  // [ms] TOTAL boost on-time per burst, must be <= LB_time (= LB_time: boost on for the whole burst)
const uint8_t  Boost_phase_pct = 50;    // pulse position inside its slot: 0 = start, 50 = centre, 100 = end

const bool     BUTTON_ABORTS   = true;  // true: pressing the button during a burst stops it early
const uint32_t COOLDOWN_MS     = 500;   // [ms] dead time after a burst / RC-off before a new press is accepted
const uint32_t DEBOUNCE_MS     = 30;    // [ms] button must be stable this long to count

const uint16_t RC_ON_US        = 1600;  // [us] pulse width at/above this = RC HIGH (blower on, boost)
const uint16_t RC_OFF_US       = 1400;  // [us] pulse width at/below this = RC LOW  (blower off)
const uint16_t RC_MIN_VALID_US = 800;   // [us] pulses shorter than this are ignored as glitches
const uint16_t RC_MAX_VALID_US = 2200;  // [us] pulses longer than this are ignored as glitches
const uint32_t RC_TIMEOUT_MS   = 100;   // [ms] no valid pulse for this long = signal lost (treated as LOW)

const bool     DEBUG_SERIAL      = false;  // print schedule, state changes and RC width at 115200 baud
const uint32_t DEBUG_RC_PRINT_MS = 250;    // [ms] period of the RC status line (0 = transitions only)
// ===========================================================================

// ------------------------------- Pins --------------------------------------
const uint8_t PIN_BUTTON = A5;  // active HIGH (external pull-down)
const uint8_t PIN_RC     = 2;   // RC PWM input, INT0 (PD2)
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
static_assert(RC_MIN_VALID_US < RC_OFF_US && RC_OFF_US < RC_ON_US && RC_ON_US < RC_MAX_VALID_US,
              "Need RC_MIN_VALID_US < RC_OFF_US < RC_ON_US < RC_MAX_VALID_US");

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

// Returns +1 once per debounced press, -1 once per debounced release, otherwise 0.
int8_t buttonEdge(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON);
  if (raw != btnLastRaw) {
    btnLastRaw = raw;
    btnLastChange = now;
  }
  if ((now - btnLastChange) >= DEBOUNCE_MS && raw != btnStable) {
    btnStable = raw;
    return btnStable == HIGH ? 1 : -1;
  }
  return 0;
}

// ------------------------------ RC input ------------------------------------
// ISR measures the high time of each pulse on D2. Only in-range pulses are kept.
volatile uint32_t rcRiseUs   = 0;
volatile uint16_t rcWidthUs  = 0;  // last valid pulse width
volatile uint8_t  rcPulseSeq = 0;  // increments on every valid pulse

void rcISR() {
  const uint32_t t = micros();
  if (PIND & _BV(PD2)) {            // rising edge (D2 == PD2)
    rcRiseUs = t;
  } else {                          // falling edge
    const uint32_t w = t - rcRiseUs;
    if (w >= RC_MIN_VALID_US && w <= RC_MAX_VALID_US) {
      rcWidthUs = (uint16_t)w;
      rcPulseSeq++;
    }
  }
}

// Loop-side RC state. Freshness is tracked with millis() in the loop (not micros()
// in the ISR) so a long dropout can never look fresh again after a timer wrap.
uint8_t  rcSeenSeq    = 0;
uint32_t rcLastMs     = 0;
uint16_t rcLastWidth  = 0;      // last valid pulse width seen by the loop [us]
bool     rcHaveSignal = false;
bool     rcHigh       = false;  // channel state after hysteresis
bool     rcArmed      = false;  // true once a valid LOW has been seen since boot / signal loss

void printRcWidth() {
  Serial.print(F(" (pwm="));
  if (rcHaveSignal) { Serial.print(rcLastWidth); Serial.print(F(" us)")); }
  else              { Serial.print(F("none)")); }
}

// Returns true when the blower should be running from the RC channel.
bool rcUpdate(uint32_t now) {
  noInterrupts();
  const uint8_t  seq   = rcPulseSeq;
  const uint16_t width = rcWidthUs;
  interrupts();

  if (seq != rcSeenSeq) {           // new valid pulse since last pass
    rcSeenSeq = seq;
    rcLastMs = now;
    rcLastWidth = width;
    if (!rcHaveSignal && DEBUG_SERIAL) {
      Serial.print(F("RC signal acquired (pwm=")); Serial.print(width); Serial.println(F(" us)"));
    }
    rcHaveSignal = true;
    if (width >= RC_ON_US)       rcHigh = true;
    else if (width <= RC_OFF_US) rcHigh = false;
    if (!rcHigh && !rcArmed) {
      rcArmed = true;
      if (DEBUG_SERIAL) {
        Serial.print(F("RC armed (low seen, pwm=")); Serial.print(width); Serial.println(F(" us)"));
      }
    }
  } else if (rcHaveSignal && now - rcLastMs >= RC_TIMEOUT_MS) {
    rcHaveSignal = false;           // signal lost: force LOW and require LOW again
    rcHigh = false;
    rcArmed = false;
    if (DEBUG_SERIAL) {
      Serial.print(F("RC signal lost (last pwm=")); Serial.print(rcLastWidth); Serial.println(F(" us)"));
    }
  }

  return rcHaveSignal && rcArmed && rcHigh;
}

// ---------------------------- State machine ---------------------------------
enum State : uint8_t { IDLE, RUNNING, COOLDOWN, RC_ON };
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
  Serial.print(F("RC: on>=")); Serial.print(RC_ON_US);
  Serial.print(F(" us, off<=")); Serial.print(RC_OFF_US);
  Serial.print(F(" us, timeout ")); Serial.print(RC_TIMEOUT_MS); Serial.println(F(" ms"));
  if (Boost_num == 0 || Boost_time == 0) { Serial.println(F("Boost: off")); return; }
  if (Boost_time >= LB_time)             { Serial.println(F("Boost: continuous")); return; }
  for (uint16_t k = 0; k < Boost_num; ++k) {
    // exact start/end in ms = (k*LB_time + offset) / Boost_num
    const float s = ((float)k * LB_time + PULSE_START_IN_SLOT) / (float)Boost_num;
    const float e = s + (float)Boost_time / (float)Boost_num;
    Serial.print(F("  pulse ")); Serial.print(k + 1);
    Serial.print(F(": ")); Serial.print(s, 1);
    Serial.print(F(" -> ")); Serial.print(e, 1); Serial.println(F(" ms"));
  }
}

// Periodic status line: current RC pulse width and what the controller is doing.
void debugStatus(uint32_t now) {
  static uint32_t lastPrint = 0;
  if (!DEBUG_SERIAL || DEBUG_RC_PRINT_MS == 0 || now - lastPrint < DEBUG_RC_PRINT_MS) return;
  lastPrint = now;
  Serial.print(F("RC pwm="));
  if (rcHaveSignal) { Serial.print(rcLastWidth); Serial.print(F(" us")); }
  else              { Serial.print(F("none")); }
  Serial.print(rcHigh ? F(" HIGH") : F(" LOW"));
  if (!rcArmed) Serial.print(F(" (not armed)"));
  Serial.print(F("  state="));
  switch (state) {
    case IDLE:     Serial.println(F("IDLE"));     break;
    case RUNNING:  Serial.println(F("BURST"));    break;
    case COOLDOWN: Serial.println(F("COOLDOWN")); break;
    case RC_ON:    Serial.println(F("RC_ON"));    break;
  }
}

void setup() {
  // Drive gates LOW before enabling outputs so the MOSFETs never see a floating gate.
  digitalWrite(PIN_MAIN, LOW);
  digitalWrite(PIN_BOOST, LOW);
  pinMode(PIN_MAIN, OUTPUT);
  pinMode(PIN_BOOST, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);  // external 2k pull-down, so no internal pull-up

  // Pull-up on the RC pin: an unplugged receiver reads as a steady HIGH with no
  // edges, so no pulses are measured and the input stays in signal-lost (off).
  pinMode(PIN_RC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RC), rcISR, CHANGE);

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
  const bool pressed = buttonEdge(now) > 0;
  const bool rcOn    = rcUpdate(now);
  debugStatus(now);

  // ---- RC channel has priority over the button ----
  if (rcOn) {
    if (state != RC_ON) {
      state = RC_ON;
      setMain(true);
      setBoost(true, 0);
      if (DEBUG_SERIAL) { Serial.print(F("RC on (boost)")); printRcWidth(); Serial.println(); }
    }
    return;                         // button ignored while RC is HIGH
  }
  if (state == RC_ON) {             // RC just went LOW or signal was lost
    allOff();
    state = COOLDOWN;
    stateStart = now;
    if (DEBUG_SERIAL) { Serial.print(F("RC off")); printRcWidth(); Serial.println(); }
    return;
  }

  // ---- Button burst ----
  if (pressed) {
    if (state == IDLE) {
      state = RUNNING;
      stateStart = now;
      if (DEBUG_SERIAL) Serial.println(F("Burst start"));
      setMain(true);
      // fall through to RUNNING so boost is evaluated in this same pass
    } else if (state == RUNNING && BUTTON_ABORTS) {
      allOff();
      state = COOLDOWN;
      stateStart = now;
      if (DEBUG_SERIAL) Serial.println(F("Burst aborted"));
      return;
    }
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
