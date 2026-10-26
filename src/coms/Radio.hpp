#pragma once
#include "hal.h"
#include "configs/DroneConfig.hpp"

/*
 * ── RC radio input protocol selection ──────────────────────────────────────
 *
 * SBUS  — SBUSo port (USART6, PC7 RX, AF8)
 *         100000 baud, 8E2, hardware-inverted on Cube PCB.
 *         25-byte fixed frame, 16 × 11-bit channels.
 *
 * CRSF  — TELEM1 port (USART2, PD6 RX, AF7)
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
float radio_flight_mode(void);  /* poshold_switch [-1, 1]: high = POS_HOLD, else STABILIZE (RcChannelMap::poshold_switch) */
bool  radio_armed(void);
bool  radio_valid(void);        /* true while the RC link is delivering fresh frames */
float radio_indi(void);         /* raw ctrl_select switch position [-1, 1], RcChannelMap::ctrl_select (3-pos) */
int   radio_switch_position(void);  /* radio_indi() bucketed to 0/1/2 (low/mid/high) — see FlightStateMachine::set_active_controller() */
bool  radio_strain_cal(void);   /* momentary strain-bias-cal switch, RcChannelMap::strain_cal_switch — see JerkFit.hpp */
bool  radio_sysid(void);        /* system-ID sweep switch, SysIdConfig::rc_channel (always false when -1) — see SysId.hpp */
