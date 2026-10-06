#include <FlexCAN_T4.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

const bool FILTER_ENABLED = true;  // false bypasses the whole notch + LPF chain

#define CAN_BAUD_RATE  1000000
#define CAN_TX_ID      0x69      // 8-byte frame: ch1,ch2,ch3,ch4 as int16, little-endian

// Hardware SPI -- validated via the hwspi_diag sketch at SPI_MODE2.
// PIN_RST is on 9, not 11 -- 11 is SPI0's default MOSI pin and SPI.begin()
// claims it.
#define SPI_CLOCK_HZ  10000000
#define SPI_MODE_SEL  SPI_MODE2

#define SAMPLE_INTERVAL_US  100    // 10 kHz sampling (filter design rate)
#define CAN_TX_INTERVAL_US  667    // ~1.5 kHz CAN broadcast, decoupled from the sample rate
#define CAN_DIAG_INTERVAL_US 500000 // 2 Hz CAN tx ok/fail diagnostic print (temporary)

// ---- Filter design ---------------------------------------------------------
// Stage 1: notch on the structural mode.
// Stage 2: 2nd-order Butterworth LPF for motor noise.
// Approx DC group delay: notch 1/(Q*w0), LPF 1.414/(2*pi*fc).
#define NOTCH_FC_HZ   39.5    // structural mode frequency
#define NOTCH_Q       2.3     // lower Q = wider notch, more delay
#define LPF_FC_HZ     110.0    // motor-noise lowpass cutoff
#define LPF_Q         0.7071  // Butterworth, 2nd order
#define FILTER_FS_HZ  10000.0 // must equal 1e6 / SAMPLE_INTERVAL_US

#define PIN_CS     10   // SPI0 CS0 (default)
#define PIN_RST    9
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13   // SPI0 SCK -- owned by SPI.begin()
#define PIN_MISO   12   // SPI0 MISO -- owned by SPI.begin()

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

SPISettings adcSpiSettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE_SEL);

elapsedMicros sampleTimer;
elapsedMicros canTimer;
elapsedMicros diagTimer;

// Temporary CAN tx diagnostic counters -- remove once 0x69 is confirmed on the bus.
static uint32_t s_can_tx_ok = 0;
static uint32_t s_can_tx_fail = 0;

// Biquad (Direct Form II Transposed), one notch + one LPF per channel.
struct Biquad {
  double b0, b1, b2, a1, a2;
  double z1, z2;
};

Biquad notchStage[4];
Biquad lpfStage[4];

double biquadProcess(Biquad &f, double x) {
  double y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x - f.a1 * y + f.z2;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}

// RBJ-cookbook lowpass biquad.
void setBiquadLowpass(Biquad &f, double fc, double fs, double Q) {
  double w0 = 2.0 * PI * fc / fs;
  double cosw0 = cos(w0);
  double alpha = sin(w0) / (2.0 * Q);

  double a0 = 1.0 + alpha;
  f.b0 = ((1.0 - cosw0) / 2.0) / a0;
  f.b1 = (1.0 - cosw0) / a0;
  f.b2 = ((1.0 - cosw0) / 2.0) / a0;
  f.a1 = (-2.0 * cosw0) / a0;
  f.a2 = (1.0 - alpha) / a0;
  f.z1 = 0.0;
  f.z2 = 0.0;
}

// RBJ-cookbook notch biquad. Unity gain at DC, zero at fc.
void setBiquadNotch(Biquad &f, double fc, double fs, double Q) {
  double w0 = 2.0 * PI * fc / fs;
  double cosw0 = cos(w0);
  double alpha = sin(w0) / (2.0 * Q);

  double a0 = 1.0 + alpha;
  f.b0 = 1.0 / a0;
  f.b1 = (-2.0 * cosw0) / a0;
  f.b2 = 1.0 / a0;
  f.a1 = (-2.0 * cosw0) / a0;
  f.a2 = (1.0 - alpha) / a0;
  f.z1 = 0.0;
  f.z2 = 0.0;
}

int16_t readWordHW() {
  // transfer16() clocks 16 bits MSB-first and returns what came back on MISO.
  return (int16_t)SPI.transfer16(0x0000);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_CONVST, OUTPUT);
  pinMode(PIN_BUSY, INPUT);

  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CONVST, LOW);
  digitalWrite(PIN_RST, LOW);
  delay(10);

  digitalWrite(PIN_RST, HIGH);
  delay(1);
  digitalWrite(PIN_RST, LOW);
  delay(100);

  // Reset sequence done before SPI touches pin muxing -- deliberate ordering.
  SPI.begin();

  can2.begin();
  can2.setBaudRate(CAN_BAUD_RATE);

  for (int i = 0; i < 4; i++) {
    setBiquadNotch(notchStage[i], NOTCH_FC_HZ, FILTER_FS_HZ, NOTCH_Q);
    setBiquadLowpass(lpfStage[i], LPF_FC_HZ, FILTER_FS_HZ, LPF_Q);
  }
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
    SPI.beginTransaction(adcSpiSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 4; i++) {
      tmp[i] = readWordHW();
    }

    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();

    // Per channel: notch (structural mode) -> LPF (motor noise)
    int16_t filtered[4];
    for (int i = 0; i < 4; i++) {
      if (FILTER_ENABLED) {
        double y = biquadProcess(notchStage[i], (double)tmp[i]);
        y = biquadProcess(lpfStage[i], y);
        if (y > 32767.0) y = 32767.0;
        if (y < -32768.0) y = -32768.0;
        filtered[i] = (int16_t)lround(y);
      } else {
        filtered[i] = tmp[i];
      }
    }

    // Broadcast all 4 channels in one 8-byte CAN frame, throttled to
    // CAN_TX_INTERVAL_US independent of the 10 kHz sample/filter rate.
    if (canTimer >= CAN_TX_INTERVAL_US) {
      canTimer -= CAN_TX_INTERVAL_US;
      CAN_message_t msg;
      msg.id = CAN_TX_ID;
      msg.len = 8;
      memcpy(msg.buf, filtered, 8);
      if (can2.write(msg)) s_can_tx_ok++; else s_can_tx_fail++;
    }

    // Temporary diagnostic: whether can2.write() itself thinks it's succeeding.
    if (diagTimer >= CAN_DIAG_INTERVAL_US) {
      diagTimer -= CAN_DIAG_INTERVAL_US;
      Serial.print("CAN tx ok=");
      Serial.print(s_can_tx_ok);
      Serial.print(" fail=");
      Serial.println(s_can_tx_fail);
    }
  }
}
