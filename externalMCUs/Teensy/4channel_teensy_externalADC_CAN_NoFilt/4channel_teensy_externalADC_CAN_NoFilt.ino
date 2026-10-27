#include <FlexCAN_T4.h>
#include <string.h>

// No filtering on this build -- the strain-rate consumer (BPRL flight
// controller) does its own filtering downstream. There's nothing here to
// tune, so there's no filter framework: the ADC is polled exactly once per
// CAN transmission, right before building the frame, rather than sampling
// on an independent schedule and holding results for a separate TX timer.

#define CAN_BAUD_RATE   1000000
#define CAN_TX_ID       0x69      // 8-byte frame: ch1,ch2,ch3,ch4 as int16, little-endian
#define TX_INTERVAL_US  667       // 1.5 kHz -- one ADC read + one CAN frame per tick
#define CAN_DIAG_INTERVAL_US 500000 // 2 Hz CAN tx ok/fail diagnostic print (temporary)

#define PIN_CS     10
#define PIN_RST    11
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

elapsedMicros txTimer;
elapsedMicros diagTimer;

// Temporary CAN tx diagnostic counters -- remove once 0x69 is confirmed on the bus.
static uint32_t s_can_tx_ok = 0;
static uint32_t s_can_tx_fail = 0;

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

  can2.begin();
  can2.setBaudRate(CAN_BAUD_RATE);
}

void loop() {
  if (txTimer < TX_INTERVAL_US) return;
  txTimer -= TX_INTERVAL_US;

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
  int16_t ch[4];
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 4; i++) {
    ch[i] = readWord();
  }

  digitalWrite(PIN_CS, HIGH);

  // Raw channel data straight into the frame -- no filtering, no
  // intermediate buffering between the read and the send.
  CAN_message_t msg;
  msg.id = CAN_TX_ID;
  msg.len = 8;
  memcpy(msg.buf, ch, 8);
  if (can2.write(msg)) s_can_tx_ok++; else s_can_tx_fail++;

  // Temporary diagnostic: confirms whether can2.write() itself thinks it's
  // succeeding, independent of whether the cube ever sees the frame.
  if (diagTimer >= CAN_DIAG_INTERVAL_US) {
    diagTimer -= CAN_DIAG_INTERVAL_US;
    Serial.print("CAN tx ok=");
    Serial.print(s_can_tx_ok);
    Serial.print(" fail=");
    Serial.println(s_can_tx_fail);
  }
}
