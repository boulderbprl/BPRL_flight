#include <Wire.h>

#define I2C_SLAVE_ADDR  0x11
#define SAMPLE_INTERVAL_US  2000
#define OFFSET 400

elapsedMicros sampleTimer;

volatile int16_t ch_safe[4] = {0, 0, 0, 0};  // only touched atomically

void onRequest() {
  Wire.write((uint8_t*)ch_safe, 8);
}

void setup() {
  Serial.begin(1000000);
  analogReadResolution(12);

  // Wire.setSDA(17);
  // Wire.setSCL(16);
  Wire.setClock(400000);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onRequest);
}

void loop() {
  if (sampleTimer >= SAMPLE_INTERVAL_US) {
    sampleTimer -= SAMPLE_INTERVAL_US;

    // Sample into local variables first
    int16_t tmp[4];
    tmp[0] = (int16_t)analogRead(A0) - OFFSET;
    tmp[1] = (int16_t)analogRead(A1) - OFFSET;
    tmp[2] = (int16_t)analogRead(A2) - OFFSET;
    tmp[3] = (int16_t)analogRead(A3) - OFFSET;

    // Commit atomically
    noInterrupts();
    ch_safe[0] = tmp[0];
    ch_safe[1] = tmp[1];
    ch_safe[2] = tmp[2];
    ch_safe[3] = tmp[3];
    interrupts();
  }
}