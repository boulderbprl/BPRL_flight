#pragma once
#include "hal.h"

/*
 * ── RC radio input protocol selection ──────────────────────────────────────
 *
 * SBUS  — SBUSo port (USART6, PC7 RX, AF8)
 *         100000 baud, 8E2, hardware-inverted on Cube PCB.
 *         25-byte fixed frame, 16 × 11-bit channels.
 *
 * CRSF  — TELEM1 port (USART3, PD9 RX, AF7)
 *         420000 baud, 8N1, uninverted.
 *         Variable-length frame, type 0x16 carries 16 × 11-bit channels.
 *
 * Change the define below, or pass -DRADIO_PROTOCOL=RADIO_PROTO_CRSF
 * via UDEFS_EXTRA in the Makefile — no other source changes required.
 */
#define RADIO_PROTO_SBUS  0
#define RADIO_PROTO_CRSF  1

#ifndef RADIO_PROTOCOL
#define RADIO_PROTOCOL    RADIO_PROTO_CRSF   /* SBUS disabled — use CRSF on TELEM1 */
#endif

void  radio_input_init(void);
void  radio_input_update(void);

float radio_thr(void);          /* throttle       [0, 1]  */
float radio_roll(void);         /* roll           [-1, 1] */
float radio_pitch(void);        /* pitch          [-1, 1] */
float radio_yaw(void);          /* yaw rate       [-1, 1] */
float radio_flight_mode(void);  /* flight mode sw [-1, 1] from channel 5 */
bool  radio_armed(void);
