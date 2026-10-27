#include <Wire.h>

#define I2C_SLAVE_ADDR  0x11
#define SAMPLE_INTERVAL_US  2000

#define PIN_CS     10
#define PIN_RST    11
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

elapsedMicros sampleTimer;

volatile int16_t ch_safe[4] = {0, 0, 0, 0};

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

    // Read all 8 channels but only keep CH1-CH4
    int16_t tmp[4];
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 4; i++) {
      tmp[i] = readWord();
    }
    // Read and discard CH5-CH8
    for (int i = 0; i < 4; i++) {
      readWord();
    }

    digitalWrite(PIN_CS, HIGH);

    // Commit atomically
    noInterrupts();
    ch_safe[0] = tmp[0];
    ch_safe[1] = tmp[1];
    ch_safe[2] = tmp[2];
    ch_safe[3] = tmp[3];
    interrupts();
  }
}