#define PIN_CS     10
#define PIN_RST    11
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

void setup() {
  Serial.begin(115200);
  while (!Serial);

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

  Serial.println("CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8");
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

void loop() {
  digitalWrite(PIN_CONVST, HIGH);
  delay(1);
  digitalWrite(PIN_CONVST, LOW);

  uint32_t t = millis();
  while (digitalRead(PIN_BUSY) == HIGH) {
    if (millis() - t > 100) {
      delay(500);
      return;
    }
  }

  int16_t channels[8];
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 8; i++) {
    channels[i] = readWord();
  }

  digitalWrite(PIN_CS, HIGH);

  for (int i = 0; i < 8; i++) {
    Serial.print(channels[i]);
    if (i < 7) Serial.print(",");
  }
  Serial.println();

  delay(10);
}