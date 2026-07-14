#include <FlexCAN_T4.h>

// Standalone CAN diagnostic sketch. Isolates "is the FlexCAN peripheral +
// code path working" (internal loopback, no transceiver involved) from
// "is the transceiver/wiring/bus actually working" (normal mode), and
// surfaces the FlexCAN error/fault-confinement state so a dead transceiver,
// swapped CANH/CANL, missing termination, or a baud mismatch all show up
// as distinct, readable symptoms instead of just "nothing happens."

#define CAN_BAUD_RATE       1000000
#define TEST_ID             0x69
#define TX_INTERVAL_US      200000   // 5 Hz test frame
#define STATUS_INTERVAL_US  500000   // 2 Hz status line

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

elapsedMicros txTimer;
elapsedMicros statusTimer;

bool     loopbackMode = true;   // start in loopback: proves the MCU side works
uint32_t txOkCount    = 0;
uint32_t txFailCount  = 0;
uint32_t rxCount      = 0;
uint32_t seq          = 0;

void printErrorState() {
  CAN_error_t err;
  if (!can2.error(err, false)) {
    Serial.println("  (no error/state events captured yet)");
    return;
  }
  Serial.print("  state=");     Serial.print(err.state);
  Serial.print("  FLT_CONF=");  Serial.print(err.FLT_CONF);
  Serial.print("  TEC=");       Serial.print(err.TX_ERR_COUNTER);
  Serial.print("  REC=");       Serial.print(err.RX_ERR_COUNTER);
  if (err.ACK_ERR) Serial.print("  ACK_ERR<-- no node acked (dead/unpowered transceiver, open wiring, or no other node+terminator on the bus)");
  if (err.BIT0_ERR || err.BIT1_ERR) Serial.print("  BIT_ERR<-- bus contention / wrong levels (check CANH/CANL wiring, termination)");
  if (err.CRC_ERR) Serial.print("  CRC_ERR");
  if (err.STF_ERR) Serial.print("  STF_ERR");
  if (err.FRM_ERR) Serial.print("  FRM_ERR");
  if (strstr((const char*)err.FLT_CONF, "Bus off")) {
    Serial.print("  <-- CONTROLLER IS BUS-OFF: it has stopped driving TX entirely. "
                 "This alone explains \"nothing transmits.\" Needs 128 x 11 recessive "
                 "bits of bus-idle to auto-recover; if it never recovers, the bus is "
                 "stuck dominant (short, wrong termination, or a jammed node).");
  }
  Serial.println();
}

void onFrameReceived(const CAN_message_t &msg) {
  rxCount++;
  Serial.print("  RX id=0x"); Serial.print(msg.id, HEX);
  Serial.print(" len=");      Serial.print(msg.len);
  Serial.print(" data=");
  for (int i = 0; i < msg.len; i++) { Serial.print(msg.buf[i], HEX); Serial.print(' '); }
  Serial.println();
}

void printMenu() {
  Serial.println();
  Serial.println("=== CAN2 diagnostic ===");
  Serial.println("  s = self-test mode (internal loopback, transceiver bypassed)");
  Serial.println("  n = normal mode (drives the real CAN_TX pin / transceiver / bus)");
  Serial.println("  e = print error/fault-confinement state now");
  Serial.println("  m = dump mailbox status");
  Serial.println("  g = raw GPIO toggle test on pin 1 (CAN2_TX), bypassing FlexCAN entirely");
  Serial.println();
}

// Temporarily steals pin 1 (CAN2_TX on Teensy 4.x) away from the FlexCAN
// peripheral and drives it as a plain GPIO square wave. This isolates
// "can the MCU pin itself toggle" from "does the transceiver/wiring pass
// that signal on to the bus" -- probe pin 1 directly first, then walk
// outward to the transceiver's TXD/D input, then CANH/CANL, to find
// exactly where the signal disappears.
void gpioToggleTest() {
  Serial.println("-> GPIO test: driving pin 1 directly (bypassing FlexCAN) as a slow "
                  "2 Hz square wave for 5 s. Probe pin 1 with a meter/scope now -- "
                  "if it's not toggling here, the fault is the MCU pin/board itself, "
                  "not the transceiver. If it IS toggling here, walk outward: check "
                  "the transceiver's TXD/D input next, then CANH/CANL.");
  pinMode(1, OUTPUT);
  for (int i = 0; i < 20; i++) {
    digitalWrite(1, i % 2);
    delay(250);
  }
  can2.setTX(DEF);   // restore pin 1 to CAN2 TX alternate function
  Serial.println("<- GPIO test done, pin 1 restored to CAN2_TX.");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  can2.begin();
  can2.setBaudRate(CAN_BAUD_RATE);
  can2.setMBFilter(ACCEPT_ALL);   // without this, mailboxes still latch frames in
                                  // hardware (visible via mailboxStatus()) but the
                                  // software dispatch path silently drops them, so
                                  // onReceive()/rxCount never fires
  can2.enableLoopBack(loopbackMode);
  can2.onReceive(onFrameReceived);

  printMenu();
  Serial.println(loopbackMode ? "Starting in LOOPBACK (self-test) mode" : "Starting in NORMAL mode");
}

void loop() {
  can2.events();

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's':
        loopbackMode = true;
        can2.enableLoopBack(true);
        Serial.println("-> LOOPBACK mode: TX is looped straight back to RX inside the chip. "
                        "If txOk keeps climbing 1:1 with rxCount and the bytes match, the "
                        "FlexCAN peripheral, baud config, and message-building code are all fine "
                        "-- so a failure in normal mode points at the transceiver or wiring, not this sketch.");
        break;
      case 'n':
        loopbackMode = false;
        can2.enableLoopBack(false);
        Serial.println("-> NORMAL mode: now driving the real CAN_TX pin into the transceiver.");
        break;
      case 'e':
        printErrorState();
        break;
      case 'm':
        can2.mailboxStatus();
        break;
      case 'g':
        gpioToggleTest();
        break;
      default:
        break;
    }
  }

  if (txTimer >= TX_INTERVAL_US) {
    txTimer -= TX_INTERVAL_US;

    CAN_message_t msg;
    msg.id = TEST_ID;
    msg.len = 8;
    seq++;
    memcpy(msg.buf, &seq, 4);        // bytes 0-3: incrementing counter
    msg.buf[4] = 0xDE; msg.buf[5] = 0xAD;
    msg.buf[6] = 0xBE; msg.buf[7] = 0xEF;

    bool ok = can2.write(msg);
    if (ok) txOkCount++; else txFailCount++;
  }

  if (statusTimer >= STATUS_INTERVAL_US) {
    statusTimer -= STATUS_INTERVAL_US;
    Serial.print("mode="); Serial.print(loopbackMode ? "LOOPBACK" : "NORMAL");
    Serial.print("  txOk=");   Serial.print(txOkCount);
    Serial.print("  txFail="); Serial.print(txFailCount);
    Serial.print("  rx=");     Serial.print(rxCount);
    Serial.println();
    printErrorState();
  }
}
