# Drone2 — CubeBlueH7 flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=blue branch used to set.

BOARD_FULL  = CubeBlueH7
# Drone2's ESCs are standard PWM (1000-2000 us pulses on FMU CH1-4), not
# DShot — override the project-wide DShot default here. See src/coms/PWM.hpp.
#
# -DSTM32H743xx: this is the physically-correct chip for Cube Blue (confirmed
# by the user against real hardware and against ArduPilot's own "CubeOrange"
# hwdef, which targets this exact board/silicon and declares
# `MCU STM32H7xx STM32H743xx`) — earlier comments here calling this a
# mislabel/"escape hatch" mixed up which Cube uses which chip; that
# confusion is resolved now, this define is simply correct. (CubeOrangePlus
# is the H753/H757-class part; see configs/Drone1/config.mk.)
#
# -DSTM32_ENFORCE_H7_REV_XY: ArduPilot's own H743-class mcuconf template
# (libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h) unconditionally
# defines this for any target clocked <=400 MHz, which is what we run
# (PLL1: 24 MHz HSE / DIVM 3 -> 8 MHz refclk * DIVN 100 = 800 MHz VCO,
# /DIVP 2 = 400 MHz). It changes the PWR/VOS overdrive sequencing in
# ChibiOS's stm32_clock_init() (hal_lld.c) to match this silicon family —
# without it, that sequencing assumes newer Rev-V-style behavior that may
# not match the actual chip. This had never actually been exercised on this
# board: stm32_clock_init() was never called at all until the __early_init()
# fix in board.c (added alongside this), since no board.c in this project
# defined that hook — cfg/mcuconf.h's whole PLL/VOS tree was dead code, and
# the MCU was booting on the ~64 MHz HSI reset default the entire time. If
# Cube Blue still doesn't boot with both fixes in place, this define — not
# the H743 chip macro — is the next thing to suspect/revert.
BOARD_UDEFS = -DSTM32H743xx -DSTM32_ENFORCE_H7_REV_XY -DBPRL_BOARD_CUBEBLUE -DMOTOR_PROTOCOL=MOTOR_PROTO_PWM
