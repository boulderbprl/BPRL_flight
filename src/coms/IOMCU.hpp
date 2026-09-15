#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Driver for the Cube-family carrier board's IO co-processor ("IOMCU") —
 * a separate STM32F1-class MCU, factory-present on every Cube/Pixhawk-family
 * board, that drives the MAIN 1-8 servo/PWM outputs. It is electrically
 * independent of the FMU's own AUX outputs (src/coms/PWM.cpp's
 * MOTOR_PROTO_PWM path, TIM1/TIM4) — on CubeBlueH7, AUX was bench-confirmed
 * unreliable (only one channel ever connects, un-throttleable), reproduced
 * even under stock ArduPilot firmware on the same hardware, ruling out this
 * project's firmware as the cause. MAIN, on the same board under ArduPilot,
 * worked correctly — hence this driver.
 *
 * Protocol summary (register-based request/response over UART), transcribed
 * from ArduPilot's libraries/AP_IOMCU/{AP_IOMCU.cpp,iofirmware/ioprotocol.h}
 * — NOT copied from that source, reimplemented from the algorithm/format
 * description below. The IO co-processor's own firmware is untouched by
 * this project: whatever ArduPilot (or the factory) last flashed onto it is
 * what this driver talks to. This driver deliberately does NOT implement
 * the firmware CRC-check/reflash path (REBOOT_BL_MAGIC etc.) — touching
 * that risks trying to reflash a chip we have no firmware image for.
 *
 *   UART: 1,500,000 baud, 8N1, no flow control. Half-duplex request/
 *     response — one request in, exactly one reply out, never unsolicited.
 *   Packet: byte0 = count(6 bits) | code(2 bits)<<6, byte1 = crc,
 *     byte2 = page, byte3 = offset, then `count` x uint16 registers
 *     (little-endian). Size = count*2 + 4, max 48 bytes (count <= 22).
 *   Opcodes: request CODE_READ=0 / CODE_WRITE=1.
 *            reply   CODE_SUCCESS=0 / CODE_CORRUPT=1 / CODE_ERROR=2.
 *   CRC: CRC-8, poly 0x07, init 0, no reflection, no final XOR
 *     (crc = table[crc ^ byte]), computed over the whole packet with the
 *     crc byte itself zeroed.
 *   A write's reply is always a fixed 4-byte ACK (count=0, code=
 *     CODE_SUCCESS, page=0, offset=0, its own CRC over those 4 bytes).
 *   A read request may use a compact 4-byte form (CRC over just those 4
 *     bytes, count/regs unused) — the ChibiOS IO firmware accepts this and
 *     it's what this driver always sends.
 *   No reset pin, no boot handshake — the IO firmware is already running
 *     from its own power-on; just start sending register packets.
 *
 * Pages/offsets this driver actually uses:
 *   PAGE_CONFIG (0), read-only: regs[0]=protocol_version, regs[1]=
 *     protocol_version2. Bring-up sanity check only (expect 4 / 10) —
 *     never gates normal operation.
 *   PAGE_SETUP (50):
 *     offset 27 CHANNEL_MASK   — bitmask of enabled channels. Defaults to 0
 *                                 on the IO's own boot — REQUIRED, or every
 *                                 channel stays silent forever with no
 *                                 error at all.
 *     offset 20 IGNORE_SAFETY  — bitmask of channels that ignore the
 *                                 hardware safety switch.
 *     offset 12 FORCE_SAFETY_OFF — write magic value 22027 to force safety
 *                                 off in software (belt-and-suspenders with
 *                                 IGNORE_SAFETY above, since the physical
 *                                 safety button can still re-arm otherwise).
 *     offset 3  DEFAULTRATE    — output refresh rate in Hz (25-400).
 *     offset 1  ARMING         — bit0 IO_ARM_OK, bit6 RC_HANDLING_DISABLED
 *                                 (this project handles RC on the FMU, not
 *                                 through the IO co-processor).
 *   PAGE_DIRECT_PWM (54): write `count` channel widths in microseconds
 *     starting at offset 0. 0xFFFF in a slot means "leave that channel
 *     unchanged" (unused here — this driver always writes all 4).
 *
 * IMPORTANT — the IO co-processor's own 200 ms failsafe watchdog: if no
 * PAGE_DIRECT_PWM write arrives for 200 ms, the IO zeroes every output
 * itself and lights its failsafe LED. IOMCUThread (src/threads.cpp) must
 * keep sending well above 5 Hz for as long as motor output is wanted.
 *
 * Architecture — why writes are split into "publish" (this header's
 * iomcu_set_pwm()) and "actually send" (IOMCUThread): a write+ACK
 * transaction is a blocking UART round-trip (bounded by a 10 ms timeout in
 * the worst case). ControlThread runs the 400 Hz control loop and must
 * never block on that. So motor_output_write() (src/coms/PWM.cpp's
 * MOTOR_PROTO_IOMCU branch) only calls iomcu_set_pwm() — an O(1) mutex
 * lock + memcpy + unlock, no UART access at all — and a separate,
 * dedicated IOMCUThread owns the UART, reads the published values, and
 * performs the actual send/ACK cycle at its own pace. This mirrors how
 * EncoderRPM.hpp/StrainGauge.hpp publish sensor data into a *Raw struct
 * for other threads to read, just in the write direction.
 *
 * Output-channel mapping: g_iomcu_cmd.pwm_us[i] drives MAIN (i+1) — i.e.
 * index 0 -> MAIN1, ... index 3 -> MAIN4. This is a fresh assignment (MAIN
 * has no pre-existing physical-lane convention the way AUX's DShot.cpp
 * pin table does) — if a motor ends up on the wrong physical corner,
 * that's a MotorMixerConfig::motor_map fix in the drone config, not a
 * change here, exactly like the existing AUX/DShot output paths.
 */

#define IOMCU_NUM_CHANNELS 4  // MAIN 1-4

// PAGE_CONFIG (read-only): regs[0]=protocol_version, regs[1]=
// protocol_version2. Expected 4/10 on a modern ChibiOS-based IO firmware.
// A bring-up sanity check only — exposed here for a USB diagnostic command
// (see threads.cpp), never gates iomcu_init()/normal operation.
#define IOMCU_PAGE_CONFIG 0

struct IOMCUCmd {
    uint16_t pwm_us[IOMCU_NUM_CHANNELS];  // commanded pulse width, microseconds
};

extern mutex_t  iomcu_cmd_mtx;
extern IOMCUCmd g_iomcu_cmd;

// Call once from motor_output_init() (PWM.cpp's MOTOR_PROTO_IOMCU branch).
// Starts SD6 (USART6, PC6=TX/PC7=RX, 1.5 Mbaud) and runs the one-time IO
// setup sequence (channel mask, ignore-safety, force-safety-off, rate,
// arming). Does NOT start sending PWM — see the architecture note above.
void iomcu_init(void);

// Publish new commanded PWM values. O(1), no UART access — safe to call
// from ControlThread's 400 Hz tick. Values are clamped to 1000-2000 by the
// caller (PWM.cpp), same convention as the AUX PWM path.
void iomcu_set_pwm(const uint16_t pwm_us[IOMCU_NUM_CHANNELS]);

// Send whatever's currently published in g_iomcu_cmd as one PAGE_DIRECT_PWM
// write. Called in a loop by IOMCUThread (src/threads.cpp) — the actual
// UART transaction, blocking up to 10 ms for the IO's ACK. Never call from
// ControlThread. Returns false on timeout/CRC/protocol error (IOMCUThread
// just retries next tick; a single missed write is harmless as long as the
// next one lands within the IO's 200 ms failsafe window).
bool iomcu_send_now(void);

// Low-level protocol primitives — for iomcu_send_now()/iomcu_init() above
// and bench diagnostics (e.g. a PAGE_CONFIG read) only. Each blocks up to
// 10 ms waiting for the IO's reply. Never call these from ControlThread.
// count is in 16-bit registers (not bytes); count <= 22 (PKT_MAX_REGS).
bool iomcu_write_registers(uint8_t page, uint8_t offset, uint8_t count, const uint16_t *regs);
bool iomcu_read_registers(uint8_t page, uint8_t offset, uint8_t count, uint16_t *regs);
