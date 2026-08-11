# Drone3 — Orqa QuadCore H7 flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
#
# STM32H743xx (same MCU variant as Drone1/CubeOrangePlus's escape-hatch
# labeling, and the true part on this board — no H757-vs-H743 ambiguity here).
# BPRL_BOARD_ORQA gates: cfg/mcuconf.h's 8 MHz HSE PLL branch, src/coms/SPI.hpp's
# dual-ICM42688 IMU set, src/coms/DShot.cpp's 2-timer/4-motor (MOT1-4 ESC
# connector, 2 motors time-multiplexed per timer) motor layout,
# src/coms/CRSF.cpp's full-duplex USART6 RC input (this board's hwdef-default
# RC pad is half-duplex USART3/GHST — CRSF.cpp deliberately moved off it, see
# CRSF.hpp), and threads.cpp's IMU rotation block.

# DShot claims TIM2/TIM4 outright (two motors per timer) — these, plus the
# now-unused TIM3/TIM5, are the only 32-bit-capable general-purpose timers on
# this MCU — the ones the kernel's system tick would otherwise want (see
# cfg/mcuconf.h's STM32_ST_USE_TIMER branch). TIM12 (16-bit) is used for the tick instead,
# which requires dropping CH_CFG_ST_RESOLUTION from the project default of
# 32 to 16 — exactly what ArduPilot's own OrqaH7QuadCore hwdef.dat does
# ("STM32_ST_USE_TIMER 12" + "define CH_CFG_ST_RESOLUTION 16"), so this
# mirrors a tested upstream choice rather than an untested guess. At
# CH_CFG_ST_FREQUENCY=10000 (cfg/chconf.h) a 16-bit counter wraps every
# ~6.5s and safely represents single delays up to ~3.3s — comfortably above
# this firmware's longest actual sleep (main()'s 1.5s USB-enumeration wait).
BOARD_FULL  = OrqaH7QuadCore
BOARD_UDEFS = -DSTM32H743xx -DBPRL_BOARD_ORQA -DCH_CFG_ST_RESOLUTION=16
