# ConBoard I2C Controller Reference

**Target address:** `0x09`  
**Bus speed:** 400 kHz  
**Source:** `V2_ADC_ConBoard.cpp`

---

## Data Layout

36 channels of ADC data are packed as 36 × `int16_t` = **72 bytes total**, split into two 36-byte chunks due to I2C buffer limits.

| Chunk | `chunkIndex` byte | Channels | Bytes |
|-------|-------------------|----------|-------|
| 0     | `0x00`            | 0 – 17   | 36    |
| 1     | `0x01`            | 18 – 35  | 36    |

Each `int16_t` value = `filtered_voltage × 10,000,000`.  
To recover volts: `float volts = val / 10000000.0f;`

---

## Full Read Sequence

Each chunk **must be preceded by a write** to select it — the board does not auto-advance.

```cpp
// --- Chunk 0: channels 0–17 ---
Wire.beginTransmission(0x09);
Wire.write(0x00);              // select chunk 0
Wire.endTransmission();

Wire.requestFrom(0x09, 36);   // read 36 bytes (18 × int16_t)
// parse 36 bytes → channels 0–17

// --- Chunk 1: channels 18–35 ---
Wire.beginTransmission(0x09);
Wire.write(0x01);              // select chunk 1
Wire.endTransmission();

Wire.requestFrom(0x09, 36);   // read 36 bytes (18 × int16_t)
// parse 36 bytes → channels 18–35
```

---

## Reassembling int16_t from Raw Bytes

Bytes arrive **little-endian** (nRF52840 native). Read low byte first:

```cpp
uint8_t lo = Wire.read();
uint8_t hi = Wire.read();
int16_t val = (int16_t)(hi << 8 | lo);
float volts = val / 10000000.0f;
```

---

## Command Bytes

Send a single byte ≥ `0x02` to issue a command. No read is needed after.

| Byte   | Char | Effect                                      |
|--------|------|---------------------------------------------|
| `0x5A` | `Z`  | Zero all channels (`performZeroing()`)      |
| `0x52` | `R`  | Reset all ADCs (`reset()`)                  |
| `0x44` | `D`  | Toggle debug serial plotter output          |
| `0x50` | `P`  | No-op poll marker (data returned on next request) |

```cpp
Wire.beginTransmission(0x09);
Wire.write('Z');               // zero all channels
Wire.endTransmission();
```

---

## Timing Notes

- Board updates ADC data every **2000 µs** (500 Hz).
- Data is filtered with a **2nd-order Butterworth low-pass** (`fc = 100 Hz, fs = 500 Hz`) before transmission.
- Reads always return the **last complete snapshot** — the buffer is protected atomically with `noInterrupts()`.
- After sending `Z` or `R`, allow **~1 ms** before reading data.
