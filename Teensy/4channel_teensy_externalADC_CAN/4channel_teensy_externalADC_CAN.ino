#include <FlexCAN_T4.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

const bool LPF_ENABLED = true;  // set false to bypass the Butterworth LPF

#define CAN_BAUD_RATE  1000000
#define CAN_TX_ID      0x69      // 8-byte frame: ch1,ch2,ch3,ch4 as int16, little-endian

// Hardware SPI -- validated working via the hwspi_diag sketch at
// SPI_MODE2/1 MHz before porting here. SPI_CLOCK_HZ pushed up from that
// sketch's conservative 1 MHz default; if reads start looking wrong (not
// just noisier, actually wrong/garbage), that's the first thing to dial
// back down. PIN_RST is on 9, not 11 -- 11 is SPI0's default MOSI pin and
// SPI.begin() silently claims it; needs the same physical rewire the diag
// sketch called for if this board hasn't already had that done.
#define SPI_CLOCK_HZ  10000000
#define SPI_MODE_SEL  SPI_MODE2

#define SAMPLE_INTERVAL_US  100    // 20 kHz sampling (filter design rate) -- was 4 kHz under
                                    // bit-banged SPI; hardware SPI's ~6.4us 4-channel read
                                    // (vs ~128us bit-banged) is what buys this. Comfortable
                                    // margin above the ~15-20us hard floor (SPI read + CONVST
                                    // + CS overhead), not pushed to that floor on a first try.
#define CAN_TX_INTERVAL_US  667   // 1.5 kHz CAN broadcast rate -- deliberately slower than
                                    // sampling now; decoupled from SAMPLE_INTERVAL_US rather
                                    // than matching it 1:1, so the filter gets a faster/more
                                    // accurate design rate without flooding the bus (0x69 is
                                    // the lowest-priority ID sharing it with the IMX5).
#define PRINT_INTERVAL_US  25000  // 40 Hz serial print rate
#define CAN_DIAG_INTERVAL_US 500000 // 2 Hz CAN tx ok/fail diagnostic print (temporary)

#define FILTER_FC_HZ  20.0
#define FILTER_FS_HZ  10000.0  // must track SAMPLE_INTERVAL_US -- biquad coefficients are
                                 // computed for this rate in setup()

#define PIN_CS     10   // SPI0 CS0 (default)
#define PIN_RST    9    // moved from 11 -- see SPI note above
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13   // SPI0 SCK (default) -- no pinMode() needed, SPI.begin() owns this pin now
#define PIN_MISO   12   // SPI0 MISO (default) -- no pinMode() needed, SPI.begin() owns this pin now

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

SPISettings adcSpiSettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE_SEL);

elapsedMicros sampleTimer;
elapsedMicros canTimer;
elapsedMicros printTimer;
elapsedMicros diagTimer;

// Temporary CAN tx diagnostic counters -- remove once 0x69 is confirmed on the bus.
static uint32_t s_can_tx_ok = 0;
static uint32_t s_can_tx_fail = 0;

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

int16_t readWordHW() {
  // transfer16() clocks 16 bits MSB-first and returns what came back on
  // MISO -- same bit order/semantics the bit-banged readWord() had.
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

  // Reset sequence fully done before SPI ever touches pin muxing --
  // deliberate ordering.
  SPI.begin();

  can2.begin();
  can2.setBaudRate(CAN_BAUD_RATE);

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
    SPI.beginTransaction(adcSpiSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 4; i++) {
      tmp[i] = readWordHW();
    }

    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();

    // Filter each channel: 4th-order Butterworth LPF, FILTER_FC_HZ / FILTER_FS_HZ above
    int16_t filtered[4];
    for (int i = 0; i < 4; i++) {
      if (LPF_ENABLED) {
        double y = biquadProcess(stage1[i], (double)tmp[i]);
        y = biquadProcess(stage2[i], y);
        if (y > 32767.0) y = 32767.0;
        if (y < -32768.0) y = -32768.0;
        filtered[i] = (int16_t)lround(y);
      } else {
        filtered[i] = tmp[i];
      }
    }

    // Broadcast all 4 channels in one 8-byte CAN frame, throttled to
    // CAN_TX_INTERVAL_US independent of the 1.5 kHz sample/filter rate.
    if (canTimer >= CAN_TX_INTERVAL_US) {
      canTimer -= CAN_TX_INTERVAL_US;
      CAN_message_t msg;
      msg.id = CAN_TX_ID;
      msg.len = 8;
      memcpy(msg.buf, filtered, 8);
      if (can2.write(msg)) s_can_tx_ok++; else s_can_tx_fail++;
    }

    // Temporary diagnostic: confirms whether can2.write() itself thinks it's
    // succeeding, independent of whether the cube ever sees the frame.
    if (diagTimer >= CAN_DIAG_INTERVAL_US) {
      diagTimer -= CAN_DIAG_INTERVAL_US;
      Serial.print("CAN tx ok=");
      Serial.print(s_can_tx_ok);
      Serial.print(" fail=");
      Serial.println(s_can_tx_fail);
    }

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