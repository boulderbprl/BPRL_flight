#include <SPI.h>

/*
 * Hardware-SPI bring-up/diagnostic for the 4-channel external ADC board.
 * Prints raw channel values to Serial for the Arduino Serial Plotter --
 * same spirit as the other diagnostic .ino's in this repo, no CAN, no
 * filtering, just "is the read correct."
 *
 * Replaces the bit-banged readWord() (digitalWrite'd SCK/MISO, 1us/half-bit)
 * with real hardware SPI. CONVST/BUSY handshake and manual CS control around
 * the burst are otherwise unchanged from the known-working bit-banged
 * version, so hardware SPI is the only variable that's different here.
 *
 * PIN_CS=10 / PIN_SCK=13 / PIN_MISO=12 already match Teensy 4.0's default
 * hardware SPI0 pins (CS0/SCK0/MISO0) -- no rewiring needed for those three.
 *
 * PIN_RST HAS moved: it was on pin 11 in the bit-banged sketches, which is
 * SPI0's default MOSI pin. The instant SPI.begin() runs, it claims pin 11
 * for the SPI peripheral's alternate function and digitalWrite(11, ...)
 * stops doing what you expect -- silently, no error. Bit-banging never
 * touches the SPI peripheral at all, so this never bit anyone until now.
 * This is the most likely reason a straight swap to hardware SPI "didn't
 * work" before. PIN_RST is now on pin 9 -- move that wire on the board
 * before flashing this.
 *
 * If channel data still doesn't look sane after that: sweep SPI_MODE_SEL
 * through 0-3 below. CPOL/CPHA is the other classic reason a chip that
 * worked bit-banged goes silent/garbage on hardware SPI -- the bit-banged
 * loop drives SCK low-at-idle and samples MISO right after driving SCK
 * high, which doesn't necessarily map cleanly onto one of the 4 standard
 * SPI_MODEn's without checking against the actual ADC datasheet (part
 * number isn't named anywhere in this repo's .ino's). SPI_MODE2 is set as
 * the starting guess (common for AD7606-family serial readback) -- treat it
 * as a guess, not a confirmed value.
 */

#define PIN_CS     10
#define PIN_RST    9        // moved from 11 -- see note above
#define PIN_CONVST 14
#define PIN_BUSY   15
#define PIN_SCK    13
#define PIN_MISO   12

#define SPI_CLOCK_HZ  1000000   // conservative starting point -- ramp up once reads look sane
#define SPI_MODE_SEL  SPI_MODE2  // guess -- try SPI_MODE0/1/3 if channel data looks wrong

#define SAMPLE_INTERVAL_US  1000   // 1 kHz -- plenty for a correctness check, not a speed test
#define PRINT_INTERVAL_US   25000 // 40 Hz serial plotter rate

SPISettings adcSpiSettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE_SEL);

elapsedMicros sampleTimer;
elapsedMicros printTimer;

int16_t g_ch[4] = {0, 0, 0, 0};

int16_t readWordHW() {
  // transfer16() clocks 16 bits MSB-first and returns what came back on
  // MISO -- same bit order/semantics as the bit-banged readWord(), just
  // done by the SPI peripheral instead of a digitalWrite() loop.
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

  // Reset sequence is fully done before SPI ever touches pin muxing --
  // deliberate ordering, not incidental.
  SPI.begin();
}

void loop() {
  if (sampleTimer < SAMPLE_INTERVAL_US) return;
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

  // Read CH1-CH4 over hardware SPI, same manual CS bracketing as the
  // bit-banged version -- SPI.begin()/SPISettings don't drive CS
  // themselves on Teensy, so this part didn't need to change.
  SPI.beginTransaction(adcSpiSettings);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 4; i++) {
    g_ch[i] = readWordHW();
  }

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  if (printTimer >= PRINT_INTERVAL_US) {
    printTimer -= PRINT_INTERVAL_US;
    Serial.print(g_ch[0]);
    Serial.print('\t');
    Serial.print(g_ch[1]);
    Serial.print('\t');
    Serial.print(g_ch[2]);
    Serial.print('\t');
    Serial.println(g_ch[3]);
  }
}
