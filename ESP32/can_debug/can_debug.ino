// Standalone CAN diagnostic sketch for ESP32 (esp-wroom-32), ported from the
// Teensy/FlexCAN_T4 version. Uses the ESP-IDF TWAI driver (Arduino-ESP32's
// "driver/twai.h") instead of FlexCAN_T4. Isolates "is the TWAI peripheral +
// code path working" (internal loopback, no transceiver involved) from
// "is the transceiver/wiring/bus actually working" (normal mode), and
// surfaces the TWAI error/fault-confinement state so a dead transceiver,
// swapped CANH/CANL, missing termination, or a baud mismatch all show up
// as distinct, readable symptoms instead of just "nothing happens."
//
// Porting notes (why this differs from the Teensy version):
//   - The ESP32 TWAI driver has no live "onReceive" callback; loop() polls
//     twai_receive() with a 0 timeout instead.
//   - There's no elapsedMicros helper outside Teensyduino, so periodic timers
//     use micros() with a fixed-rate accumulator (same drift-free behavior).
//   - FlexCAN_T4's enableLoopBack(true) loops TX to RX inside the peripheral
//     without touching any pin. The TWAI controller has no such internal
//     loopback; instead this sketch uses the officially documented ESP-IDF
//     self-test idiom: TWAI_MODE_NO_ACK with TX and RX mapped to the SAME
//     GPIO, so the controller's own transmission is read back on its own
//     receiver via that one pin, with the transceiver and any real bus
//     completely out of the picture. Switching modes therefore means
//     stopping/uninstalling and reinstalling the driver with a different
//     GPIO/mode config -- FlexCAN_T4 can reconfigure this live, TWAI cannot.
//   - The TWAI driver doesn't decode individual ACK_ERR/BIT_ERR/CRC_ERR/
//     STF_ERR/FRM_ERR flags the way FlexCAN_T4's error() (from the ESR1
//     register) does. It only exposes an aggregate bus_error_count plus
//     arb_lost_count/tx_failed_count/rx_missed_count/rx_overrun_count.
//   - Bus-off recovery is not automatic like FlexCAN_T4 hardware; you must
//     call twai_initiate_recovery() and then, once recovered, twai_start()
//     again (done here by pressing 's' or 'n' after the state is STOPPED).

#include <driver/twai.h>

// Adjust these to match your transceiver wiring.
#define CAN_TX_GPIO         GPIO_NUM_27
#define CAN_RX_GPIO         GPIO_NUM_32

#define TEST_ID             0x69
#define TX_INTERVAL_US      200000   // 5 Hz test frame
#define STATUS_INTERVAL_US  500000   // 2 Hz status line

bool     loopbackMode = true;   // start in loopback: proves the MCU side works
uint32_t txOkCount    = 0;
uint32_t txFailCount  = 0;
uint32_t rxCount      = 0;
uint32_t seq          = 0;

unsigned long lastTx     = 0;
unsigned long lastStatus = 0;

// Installs and starts the TWAI driver in either loopback (self-test) or
// normal mode. Any previously installed driver must already be
// stopped/uninstalled by the caller before calling this.
void startTwai(bool loopback) {
  twai_general_config_t g_config = loopback
      ? (twai_general_config_t)TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_TX_GPIO, TWAI_MODE_NO_ACK)
      : (twai_general_config_t)TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("!! twai_driver_install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("!! twai_start failed");
    return;
  }
}

// Stops and uninstalls the driver so it can be reinstalled with a different
// mode/GPIO config. Safe to call even if the driver is already stopped.
void stopTwai() {
  twai_stop();
  twai_driver_uninstall();
}

void printErrorState() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("  (unable to read status -- driver not installed?)");
    return;
  }

  Serial.print("  state=");
  switch (status.state) {
    case TWAI_STATE_STOPPED:    Serial.print("STOPPED");    break;
    case TWAI_STATE_RUNNING:    Serial.print("RUNNING");    break;
    case TWAI_STATE_BUS_OFF:    Serial.print("BUS_OFF");    break;
    case TWAI_STATE_RECOVERING: Serial.print("RECOVERING"); break;
    default:                    Serial.print("UNKNOWN");    break;
  }
  Serial.print("  TEC=");        Serial.print(status.tx_error_counter);
  Serial.print("  REC=");        Serial.print(status.rx_error_counter);
  Serial.print("  tx_failed=");  Serial.print(status.tx_failed_count);
  Serial.print("  arb_lost=");   Serial.print(status.arb_lost_count);
  Serial.print("  bus_err=");    Serial.print(status.bus_error_count);
  Serial.print("  rx_missed=");  Serial.print(status.rx_missed_count);
  Serial.print("  rx_overrun="); Serial.print(status.rx_overrun_count);

  if (status.bus_error_count > 0) {
    Serial.print("  BUS_ERR<-- ACK/bit/CRC/stuff/form errors on the bus (check "
                 "transceiver power/wiring, CANH/CANL, and termination -- the "
                 "TWAI driver doesn't break this count down further)");
  }
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.print("  <-- CONTROLLER IS BUS-OFF: it has stopped driving TX entirely. "
                 "This alone explains \"nothing transmits.\" Press 'r' to call "
                 "twai_initiate_recovery(); needs 128 x 11 recessive bits of "
                 "bus-idle to recover. If it never recovers, the bus is stuck "
                 "dominant (short, wrong termination, or a jammed node).");
  }
  if (status.state == TWAI_STATE_STOPPED) {
    Serial.print("  <-- driver stopped (e.g. recovery just completed) -- press "
                 "'s' or 'n' to reinitialize and resume.");
  }
  Serial.println();
}

// Polls for received frames every loop() iteration since TWAI has no
// onReceive-style callback. Bounded to a handful of frames per call --
// on a live, busy bus, printing every frame can take longer than frames
// keep arriving, and an unbounded drain loop would then never return,
// starving the TX/status-print timers below in loop(). Any backlog beyond
// this cap just waits in the driver's RX queue (or shows up as
// rx_missed/rx_overrun in printErrorState() if that queue fills).
#define RX_DRAIN_MAX_PER_LOOP 8
void pollReceive() {
  twai_message_t msg;
  int drained = 0;
  while (drained < RX_DRAIN_MAX_PER_LOOP && twai_receive(&msg, 0) == ESP_OK) {
    drained++;
    rxCount++;
    // Per-frame print disabled while isolating the status-line starvation
    // issue -- on a busy bus this alone was slow enough to compete with the
    // TX/status timers below. Re-enable once confirmed the status line prints
    // reliably.
    // Serial.print("  RX id=0x"); Serial.print(msg.identifier, HEX);
    // Serial.print(" len=");      Serial.print(msg.data_length_code);
    // Serial.print(" data=");
    // for (int i = 0; i < msg.data_length_code; i++) { Serial.print(msg.data[i], HEX); Serial.print(' '); }
    // Serial.println();
  }
}

void printMenu() {
  Serial.println();
  Serial.println("=== ESP32 TWAI (CAN) diagnostic ===");
  Serial.println("  s = self-test mode (TX looped to RX on the same GPIO, transceiver bypassed)");
  Serial.println("  n = normal mode (drives the real CAN_TX_GPIO / transceiver / bus)");
  Serial.println("  e = print error/fault-confinement state now");
  Serial.println("  m = dump queued TX/RX message counts");
  Serial.println("  g = raw GPIO toggle test on CAN_TX_GPIO, bypassing TWAI entirely");
  Serial.println("  r = initiate bus-off recovery (then press s/n again once state=STOPPED)");
  Serial.println();
}

// Temporarily tears down the TWAI driver so CAN_TX_GPIO can be driven as a
// plain GPIO square wave. This isolates "can the MCU pin itself toggle" from
// "does the transceiver/wiring pass that signal on to the bus" -- probe
// CAN_TX_GPIO directly first, then walk outward to the transceiver's TXD/D
// input, then CANH/CANL, to find exactly where the signal disappears.
void gpioToggleTest() {
  Serial.println("-> GPIO test: stopping the TWAI driver and driving CAN_TX_GPIO "
                  "directly as a slow 2 Hz square wave for 5 s. Probe CAN_TX_GPIO "
                  "with a meter/scope now -- if it's not toggling here, the fault "
                  "is the MCU pin/board itself, not the transceiver. If it IS "
                  "toggling here, walk outward: check the transceiver's TXD/D "
                  "input next, then CANH/CANL.");
  stopTwai();

  pinMode(CAN_TX_GPIO, OUTPUT);
  for (int i = 0; i < 20; i++) {
    digitalWrite(CAN_TX_GPIO, i % 2);
    delay(250);
  }

  startTwai(loopbackMode);   // reinstall + restart in whichever mode was active
  Serial.println("<- GPIO test done, TWAI driver reinstalled.");
}

void setup() {
  Serial.begin(921600);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  startTwai(loopbackMode);

  printMenu();
  Serial.println(loopbackMode ? "Starting in LOOPBACK (self-test) mode" : "Starting in NORMAL mode");
}

void loop() {
  pollReceive();

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's':
        loopbackMode = true;
        stopTwai();
        startTwai(true);
        Serial.println("-> LOOPBACK mode: TX is looped straight back to RX via a shared "
                        "GPIO inside the chip. If txOk keeps climbing 1:1 with rxCount and "
                        "the bytes match, the TWAI peripheral, baud config, and message-"
                        "building code are all fine -- so a failure in normal mode points "
                        "at the transceiver or wiring, not this sketch.");
        break;
      case 'n':
        loopbackMode = false;
        stopTwai();
        startTwai(false);
        Serial.println("-> NORMAL mode: now driving the real CAN_TX_GPIO into the transceiver.");
        break;
      case 'e':
        printErrorState();
        break;
      case 'm': {
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
          Serial.print("  msgs_to_tx="); Serial.print(status.msgs_to_tx);
          Serial.print("  msgs_to_rx="); Serial.print(status.msgs_to_rx);
          Serial.println();
        } else {
          Serial.println("  (unable to read status -- driver not installed?)");
        }
        break;
      }
      case 'g':
        gpioToggleTest();
        break;
      case 'r':
        if (twai_initiate_recovery() == ESP_OK) {
          Serial.println("-> Bus-off recovery initiated; waiting for 128 x 11 recessive bits...");
        } else {
          Serial.println("-> Recovery not initiated (controller might not be in BUS_OFF).");
        }
        break;
      default:
        break;
    }
  }

  if (micros() - lastTx >= TX_INTERVAL_US) {
    lastTx += TX_INTERVAL_US;

    twai_message_t msg = {};
    msg.identifier = TEST_ID;
    msg.data_length_code = 8;
    seq++;
    memcpy(msg.data, &seq, 4);        // bytes 0-3: incrementing counter
    msg.data[4] = 0xDE; msg.data[5] = 0xAD;
    msg.data[6] = 0xBE; msg.data[7] = 0xEF;

    bool ok = (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
    if (ok) txOkCount++; else txFailCount++;
  }

  if (micros() - lastStatus >= STATUS_INTERVAL_US) {
    lastStatus += STATUS_INTERVAL_US;
    Serial.print("mode="); Serial.print(loopbackMode ? "LOOPBACK" : "NORMAL");
    Serial.print("  txOk=");   Serial.print(txOkCount);
    Serial.print("  txFail="); Serial.print(txFailCount);
    Serial.print("  rx=");     Serial.print(rxCount);
    Serial.println();
    printErrorState();
  }
}
