#include <Wire.h>
#include <math.h>
#include <string.h>

#define I2C_SLAVE_ADDR  0x11
#define SAMPLE_INTERVAL_US  250   // 4 kHz sampling (filter design rate)

#define FILTER_FS_HZ  4000.0
#define FILTER_FC_HZ  10.0   // lowpass cutoff -- matches the earlier i2c 4th-order LPF build that gave
                              // good z-jerk correlation; adjust if that build used a different cutoff

#define BLADE_COUNT          3     // propeller blades per motor — sets blade-pass notch = BLADE_COUNT * RPM/60
#define NOTCH_HALF_WIDTH_HZ  20.0  // each notch covers center Hz +/- this many Hz
#define MIN_MOTOR_RPM        6000  // ~100 Hz rotation freq; below this (e.g. bench testing, motors off), notches are bypassed
#define RPM_RETUNE_DEADBAND  25    // RPM must move by more than this before coefficients are recomputed —
                                    // avoids redundant retuning once the smoothed RPM below is already stable
#define RPM_SMOOTH_TAU_S     0.03  // one-pole low-pass on RPM telemetry before it drives retuning — keeps
                                    // each retune's frequency step small (tau is far faster than real motor
                                    // spool-up dynamics) so filter state can safely carry across retunes

#define PIN_CS     10
#define PIN_RST    11
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

elapsedMicros sampleTimer;

volatile int16_t ch_safe[4] = {0, 0, 0, 0};
volatile uint16_t rpm_avg_safe = 0;  // avg motor RPM, written by CubeOrangePlus over I2C

// Three RPM-tracking notch filters (Direct Form II Transposed biquads):
//   [0] = 1x motor rotation frequency (RPM/60)
//   [1] = 2x motor rotation frequency (2*RPM/60)
//   [2] = 1x blade-pass frequency (BLADE_COUNT*RPM/60)
// Coefficients depend only on RPM, so they're shared across channels;
// filter state (z1/z2) is per-channel, per-notch.
#define NOTCH_COUNT 3

struct BiquadCoef {
  double b0, b1, b2, a1, a2;
};
struct BiquadState {
  double z1, z2;
};

BiquadCoef  notchCoef[NOTCH_COUNT];
BiquadState notchState[NOTCH_COUNT][4];

// State (z1/z2) is clamped to a wide margin beyond the int16 output range.
// Without this, a transient (coefficient retune, ADC glitch, brief real
// overload) can leave the state holding a huge value that a near-unity-pole
// notch only bleeds off over many samples — producing sustained rail-to-rail
// clipped output long after whatever caused the transient is gone. The
// clamp bounds how long that ringdown can possibly take without changing
// behavior for any normal in-range signal.
#define BIQUAD_STATE_LIMIT 200000.0

double biquadProcess(const BiquadCoef &c, BiquadState &s, double x) {
  double y = c.b0 * x + s.z1;
  double z1 = c.b1 * x - c.a1 * y + s.z2;
  double z2 = c.b2 * x - c.a2 * y;
  if (z1 > BIQUAD_STATE_LIMIT) z1 = BIQUAD_STATE_LIMIT;
  else if (z1 < -BIQUAD_STATE_LIMIT) z1 = -BIQUAD_STATE_LIMIT;
  if (z2 > BIQUAD_STATE_LIMIT) z2 = BIQUAD_STATE_LIMIT;
  else if (z2 < -BIQUAD_STATE_LIMIT) z2 = -BIQUAD_STATE_LIMIT;
  s.z1 = z1;
  s.z2 = z2;
  return y;
}

// Designs one RBJ-cookbook notch (band-reject) biquad section centered at
// f0, sample rate fs, with sharpness Q.
void setBiquadNotch(BiquadCoef &f, double f0, double fs, double Q) {
  double w0 = 2.0 * PI * f0 / fs;
  double cosw0 = cos(w0);
  double sinw0 = sin(w0);
  double alpha = sinw0 / (2.0 * Q);

  double b0 = 1.0;
  double b1 = -2.0 * cosw0;
  double b2 = 1.0;
  double a0 = 1.0 + alpha;
  double a1 = -2.0 * cosw0;
  double a2 = 1.0 - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;
}

// 4th-order Butterworth lowpass, applied after the notches on every channel
// regardless of motor state. The notches only remove narrow tones at the
// current RPM harmonics; everything else in the broadband noise floor
// passes through them at unity gain. This stage is what actually rejects
// that broadband noise and limits output bandwidth to the range relevant
// to airframe/structural dynamics -- fixed coefficients (fc doesn't track
// RPM), computed once in setup(), so it carries none of the notch cascade's
// retuning/stability concerns.
BiquadCoef  lpCoef[2];
BiquadState lpState[2][4];

void setBiquadLowpass(BiquadCoef &f, double fc, double fs, double Q) {
  double w0 = 2.0 * PI * fc / fs;
  double cosw0 = cos(w0);
  double sinw0 = sin(w0);
  double alpha = sinw0 / (2.0 * Q);

  double b0 = (1.0 - cosw0) / 2.0;
  double b1 = 1.0 - cosw0;
  double b2 = (1.0 - cosw0) / 2.0;
  double a0 = 1.0 + alpha;
  double a1 = -2.0 * cosw0;
  double a2 = 1.0 - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;
}

// Keeps each notch center strictly inside (0, Nyquist) so setBiquadNotch
// never sees an invalid f0 — e.g. RPM=0 (no telemetry yet) or a harmonic
// aliasing past FILTER_FS_HZ/2 at very high RPM.
double clampNotchFreq(double f) {
  const double fmin = 1.0;
  const double fmax = FILTER_FS_HZ / 2.0 - 1.0;
  if (f < fmin) return fmin;
  if (f > fmax) return fmax;
  return f;
}

// RBJ-cookbook Q is defined w.r.t. a -3 dB bandwidth in Hz: Q = f0/BW.
// Using the average RPM means any single motor can sit off-center from
// f0, so the notch is sized to a fixed +/-NOTCH_HALF_WIDTH_HZ window
// around each center rather than a fixed Q (which would narrow as RPM drops).
double notchQForWidth(double f0) {
  const double bw = 2.0 * NOTCH_HALF_WIDTH_HZ;
  return f0 / bw;
}

// Recomputes the 3 shared notch coefficient sets from the latest avg RPM.
void updateNotchCoeffs(uint16_t rpm) {
  double f_rot = rpm / 60.0;
  double f1 = clampNotchFreq(f_rot * 1.0);
  double f2 = clampNotchFreq(f_rot * 2.0);
  double f3 = clampNotchFreq(f_rot * BLADE_COUNT);
  setBiquadNotch(notchCoef[0], f1, FILTER_FS_HZ, notchQForWidth(f1));
  setBiquadNotch(notchCoef[1], f2, FILTER_FS_HZ, notchQForWidth(f2));
  setBiquadNotch(notchCoef[2], f3, FILTER_FS_HZ, notchQForWidth(f3));
}

void onRequest() {
  Wire.write((uint8_t*)ch_safe, 8);
}

// CubeOrangePlus writes 2 bytes (uint16_t, little-endian): avg motor RPM,
// ahead of the strain-data read, in one combined transaction (repeated start).
void onReceive(int numBytes) {
  if (numBytes < 2) {
    while (Wire.available()) Wire.read();
    return;
  }
  uint16_t lo = Wire.read();
  uint16_t hi = Wire.read();
  while (Wire.available()) Wire.read();  // discard anything unexpected
  rpm_avg_safe = (uint16_t)(lo | (hi << 8));
}

int16_t readWord() {
  int16_t result = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(1);
    result = (result << 1) | digitalRead(PIN_MISO);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(1);
  }
  return result;
}

void setup() {
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_CONVST, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_BUSY, INPUT);

  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CONVST, LOW);
  digitalWrite(PIN_SCK, LOW);
  digitalWrite(PIN_RST, LOW);
  delay(10);

  digitalWrite(PIN_RST, HIGH);
  delay(1);
  digitalWrite(PIN_RST, LOW);
  delay(100);

  Wire.setClock(400000);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);

  // rpm_avg_safe is 0 until the first I2C write arrives; updateNotchCoeffs
  // clamps that to a harmless near-DC notch rather than an invalid f0=0.
  updateNotchCoeffs(rpm_avg_safe);

  // 4th-order Butterworth = two biquad sections with the standard
  // Butterworth pole-pair Q values: Q_k = 1 / (2*cos((2k-1)*pi/8))
  double lpQ1 = 1.0 / (2.0 * cos(PI / 8.0));       // 0.541196
  double lpQ2 = 1.0 / (2.0 * cos(3.0 * PI / 8.0)); // 1.306563
  setBiquadLowpass(lpCoef[0], FILTER_FC_HZ, FILTER_FS_HZ, lpQ1);
  setBiquadLowpass(lpCoef[1], FILTER_FC_HZ, FILTER_FS_HZ, lpQ2);
}

void loop() {
  if (sampleTimer >= SAMPLE_INTERVAL_US) {
    sampleTimer -= SAMPLE_INTERVAL_US;

    // Start conversion
    digitalWrite(PIN_CONVST, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_CONVST, LOW);

    // Wait for BUSY
    uint32_t t = millis();
    while (digitalRead(PIN_BUSY) == HIGH) {
      if (millis() - t > 100) return;
    }

    // Read only CH1-CH4, then deassert CS early (CH5-CH8 are never clocked out)
    int16_t tmp[4];
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 4; i++) {
      tmp[i] = readWord();
    }

    digitalWrite(PIN_CS, HIGH);

    // Low-pass the raw RPM telemetry before it ever reaches the notch
    // coefficient calculation. Feeding raw, jittery/fast-moving telemetry
    // straight into updateNotchCoeffs made each retune a potentially large,
    // abrupt frequency jump -- resetting filter state on every retune
    // bounded that (no more compounding across retunes), but a single
    // retune's zero-state transient could still clip on its own during a
    // fast RPM sweep, since jumping a resonant notch's center frequency by
    // a large step is inherently transient-producing. Smoothing the target
    // keeps each retune step small, which means state can safely carry
    // across retunes again instead of needing a full reset every time.
    static const double s_rpm_smooth_alpha =
        (SAMPLE_INTERVAL_US * 1e-6) / (RPM_SMOOTH_TAU_S + SAMPLE_INTERVAL_US * 1e-6);
    static double   s_rpm_smooth = 0.0;
    static uint16_t s_last_rpm = 0xFFFF;
    static bool     s_was_spinning = false;

    s_rpm_smooth += s_rpm_smooth_alpha * ((double)rpm_avg_safe - s_rpm_smooth);
    uint16_t rpm_now = (uint16_t)lround(s_rpm_smooth);
    bool motors_spinning = (rpm_now >= MIN_MOTOR_RPM);
    bool spin_state_changed = (motors_spinning != s_was_spinning);
    int32_t rpm_delta = (int32_t)rpm_now - (int32_t)s_last_rpm;
    if (rpm_delta < 0) rpm_delta = -rpm_delta;
    if (spin_state_changed || (motors_spinning && rpm_delta > RPM_RETUNE_DEADBAND)) {
      s_last_rpm = rpm_now;
      s_was_spinning = motors_spinning;
      if (motors_spinning) {
        updateNotchCoeffs(rpm_now);
      } else {
        memset(notchState, 0, sizeof(notchState));
      }
    }

    // Filter each channel: cascade the 3 RPM-tracking notches (bypassed
    // below MIN_MOTOR_RPM) followed by the fixed 4th-order lowpass (always
    // active, motors on or off).
    int16_t filtered[4];
    for (int i = 0; i < 4; i++) {
      double y = (double)tmp[i];
      if (motors_spinning) {
        for (int n = 0; n < NOTCH_COUNT; n++) {
          y = biquadProcess(notchCoef[n], notchState[n][i], y);
        }
      }
      y = biquadProcess(lpCoef[0], lpState[0][i], y);
      y = biquadProcess(lpCoef[1], lpState[1][i], y);
      if (y > 32767.0) y = 32767.0;
      if (y < -32768.0) y = -32768.0;
      filtered[i] = (int16_t)lround(y);
    }

    // Commit atomically
    noInterrupts();
    ch_safe[0] = filtered[0];
    ch_safe[1] = filtered[1];
    ch_safe[2] = filtered[2];
    ch_safe[3] = filtered[3];
    interrupts();
  }
}