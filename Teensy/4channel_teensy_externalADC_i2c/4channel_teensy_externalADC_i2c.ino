#include <Wire.h>
#include <math.h>

#define I2C_SLAVE_ADDR  0x11
#define SAMPLE_INTERVAL_US  200   // 5 kHz sampling (filter design rate)
#define PRINT_INTERVAL_US  25000  // 40 Hz serial print rate

#define FILTER_FC_HZ  8.0
#define FILTER_FS_HZ  5000.0

#define PIN_CS     10
#define PIN_RST    11
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

elapsedMicros sampleTimer;
elapsedMicros printTimer;

volatile int16_t ch_safe[4] = {0, 0, 0, 0};

// 4th-order Butterworth LPF, implemented as two cascaded biquads
// (Direct Form II Transposed) per channel.
struct Biquad {
  double b0, b1, b2, a1, a2;
  double z1, z2;
};

Biquad stage1[4];
Biquad stage2[4];

double biquadProcess(Biquad &f, double x) {
  double y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x - f.a1 * y + f.z2;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}

// Designs one RBJ-cookbook lowpass biquad section for cutoff fc at
// sample rate fs, with the given pole Q (Butterworth Q for this section).
void setBiquadLowpass(Biquad &f, double fc, double fs, double Q) {
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
  f.z1 = 0.0;
  f.z2 = 0.0;
}

void onRequest() {
  Wire.write((uint8_t*)ch_safe, 8);
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
  Serial.begin(115200);

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

  // 4th-order Butterworth = two biquad sections with the standard
  // Butterworth pole-pair Q values: Q_k = 1 / (2*cos((2k-1)*pi/8))
  double Q1 = 1.0 / (2.0 * cos(PI / 8.0));       // 0.541196
  double Q2 = 1.0 / (2.0 * cos(3.0 * PI / 8.0)); // 1.306563
  for (int i = 0; i < 4; i++) {
    setBiquadLowpass(stage1[i], FILTER_FC_HZ, FILTER_FS_HZ, Q1);
    setBiquadLowpass(stage2[i], FILTER_FC_HZ, FILTER_FS_HZ, Q2);
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
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 4; i++) {
      tmp[i] = readWord();
    }

    digitalWrite(PIN_CS, HIGH);

    // Filter each channel: 4th-order Butterworth LPF, fc = 10 Hz, fs = 5 kHz
    int16_t filtered[4];
    for (int i = 0; i < 4; i++) {
      double y = biquadProcess(stage1[i], (double)tmp[i]);
      y = biquadProcess(stage2[i], y);
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

    // Filtered channel data to Serial Plotter, throttled to ~40 Hz
    // if (printTimer >= PRINT_INTERVAL_US) {
    //   printTimer -= PRINT_INTERVAL_US;
    //   Serial.print(filtered[0]);
    //   Serial.print('\t');
    //   Serial.print(filtered[1]);
    //   Serial.print('\t');
    //   Serial.print(filtered[2]);
    //   Serial.print('\t');
    //   Serial.println(filtered[3]);
    // }
  }
}