#include <ACAN_T4.h>

#define CAN_ID_ADC  0x69

#define SAMPLE_INTERVAL_US  2000  // 500 Hz

#define OFFSET 1494

elapsedMicros sampleTimer;

void setup() {
  Serial.begin(1000000);
  Serial.println("Serial is started");
  analogReadResolution(12);

  ACAN_T4_Settings settings(1000000);
  settings.mLoopBackMode = true;
  settings.mSelfReceptionMode = true;
  const uint32_t errorCode = ACAN_T4::can2.begin(settings);

  if (errorCode != 0) {
    Serial.print("CAN2 init error: 0x");
    Serial.println(errorCode, HEX);
    while (1);
  }
}

void loop() {
  if (sampleTimer >= SAMPLE_INTERVAL_US) {
    sampleTimer -= SAMPLE_INTERVAL_US;

    int16_t ch0 = (int16_t)analogRead(A0)-OFFSET;
    int16_t ch1 = (int16_t)analogRead(A1)-OFFSET;
    int16_t ch2 = (int16_t)analogRead(A2)-OFFSET;
    int16_t ch3 = (int16_t)analogRead(A3)-OFFSET;

    CANMessage msg;
    msg.id  = CAN_ID_ADC;
    msg.len = 8;
    msg.ext = false;  // force standard 11-bit frame
    msg.rtr = false;

    memcpy(msg.data + 0, &ch0, 2);
    memcpy(msg.data + 2, &ch1, 2);
    memcpy(msg.data + 4, &ch2, 2);
    memcpy(msg.data + 6, &ch3, 2);

    ACAN_T4::can2.tryToSend(msg);
    if (!ACAN_T4::can2.tryToSend(msg)) {
      Serial.println("TX failed - buffer full");
      Serial.print("Receive error counter: ");
      Serial.println(ACAN_T4::can2.receiveErrorCounter());
      Serial.print("Transmit error counter: ");
      Serial.println(ACAN_T4::can2.transmitErrorCounter());
    }else{
      Serial.println("Frame sent");
    }
  }

  ACAN_T4::can2.dispatchReceivedMessage();
}