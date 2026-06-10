#include <Wire.h>

#define I2C_SLAVE_ADDR  0x11

#define SAMPLE_INTERVAL_US  2000  // 500 Hz

#define OFFSET 1494

elapsedMicros sampleTimer;

volatile int16_t ch[4] = {0, 0, 0, 0};

void onRequest() {
  Wire.write((uint8_t*)ch, 8);
}

void setup() {
  Serial.begin(1000000);
  Serial.println("Serial is started");
  analogReadResolution(12);

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.setClock(400000);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onRequest);
}

void loop() {
  if (sampleTimer >= SAMPLE_INTERVAL_US) {
    sampleTimer -= SAMPLE_INTERVAL_US;

    ch[0] = (int16_t)analogRead(A0) - OFFSET;
    ch[1] = (int16_t)analogRead(A1) - OFFSET;
    ch[2] = (int16_t)analogRead(A2) - OFFSET;
    ch[3] = (int16_t)analogRead(A3) - OFFSET;
  }
}
