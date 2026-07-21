##############################################################################
# BPRL Flight Controller — ChibiOS Makefile
#
# Usage:
#   make                            (default: BOARD=orange, plain flight build)
#   make BOARD=orange               (CubeOrangePlus, default)
#   make BOARD=blue                 (CubeBlueH7)
#   make flash  BOARD=blue   PORT=/dev/ttyACM0   (Cube bootloader)
#   make flash-stlink BOARD=blue                  (ST-Link / OpenOCD)
#
# Debug UART (USART3 @ 115200, Telem1 connector):
#   make BOARD=blue UDEFS_EXTRA=-DBPRL_DEBUG
#   (Disable before flight — adds a 10 Hz print thread)
#
# Thread timing / CPU utilization instrumentation (schedulability testing):
#   make BOARD=blue UDEFS_EXTRA=-DBPRL_TIMING
#   (Testing/bench only — query over USB with "TIM,status" / "TIM,reset".
#   Zero-cost when the flag is absent; see src/diagnostics/ThreadTiming.hpp)
#

##############################################################################
# Board selection
#
# BOARD is the short user-facing name (blue/orange); BOARD_FULL maps it to
# the boards/<dir> and internal board name used everywhere else (linker
# scripts, board.h BOARD_NAME, etc.) — that internal naming is unchanged.

BOARD ?= orange

ifeq ($(BOARD), blue)
  BOARD_FULL  = CubeBlueH7
  BOARD_UDEFS = -DSTM32H753xx -DSTM32_ENFORCE_H7_REV_XY -DBPRL_BOARD_CUBEBLUE
else ifeq ($(BOARD), orange)
  BOARD_FULL  = CubeOrangePlus
  BOARD_UDEFS = -DSTM32H757xx -DCORE_CM7 -DBPRL_BOARD_CUBEORANGEPLUS
else
  $(error Unknown BOARD="$(BOARD)". Valid values: blue, orange)
endif

BOARDDIR := boards/$(BOARD_FULL)

##############################################################################
# Flash / upload targets  (defined after 'all' so bare 'make' builds only)
#

PORT            ?= /dev/ttyACM0
UPLOAD_SCRIPT   := tools/flash_upload.py
OPENOCD_CFG     := -f interface/stlink.cfg -f target/stm32h7x.cfg

##############################################################################
# Build global options
#

ifeq ($(USE_OPT),)
  USE_OPT = -O3 -ggdb -fomit-frame-pointer -falign-functions=16
endif

ifeq ($(USE_COPT),)
  USE_COPT =
endif

ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

##############################################################################
# Architecture / project specific options
#

ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x800
endif

ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x800
endif

# STM32H7 has a double-precision FPU — always use hard float
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

##############################################################################
# Project, target, sources and paths
#

PROJECT = BPRL

MCU = cortex-m7

CHIBIOS  := third_party/ChibiOS
CONFDIR  := cfg
BUILDDIR := build
DEPDIR   := .dep

# ChibiOS includes
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
include $(BOARDDIR)/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk

# CubeOrangePlus: application starts at 0x08020000 (after 128 KB BL).
# CubeBlueH7: same flash layout as the generic ChibiOS script (org=0x08000000,
# no bootloader offset), but with its own .nocache placement fixed to match
# STM32_NOCACHE_RBAR/RASR in cfg/mcuconf.h — see boards/CubeBlueH7/STM32H743xI.ld.
ifeq ($(BOARD_FULL),CubeOrangePlus)
    LDSCRIPT = $(BOARDDIR)/STM32H743xI_app.ld
else ifeq ($(BOARD_FULL),CubeBlueH7)
    LDSCRIPT = $(BOARDDIR)/STM32H743xI.ld
else
    LDSCRIPT = $(STARTUPLD)/STM32H743xI.ld
endif

# C sources
CSRC = $(ALLCSRC) \
       $(BOARDDIR)/board.c \
       $(CHIBIOS)/os/various/syscalls.c

# C++ sources
CPPSRC = $(ALLCPPSRC) \
         main.cpp \
         src/threads.cpp \
         src/math/math.cpp \
         src/state_estimator/EKF.cpp \
         src/state_estimator/StateManager.cpp \
         src/controllers/PID.cpp \
         src/controllers/Attitude_PID.cpp \
         src/controllers/Attitude_INDI.cpp \
         src/controllers/AltControl.cpp \
         src/controllers/PosControl.cpp \
         src/controllers/Unmixer.cpp \
         src/controllers/FlightStateMachine.cpp \
         src/controllers/MotorMixer.cpp \
         src/coms/IMUs/ICM42688.cpp \
         src/coms/IMUs/ICM45686.cpp \
         src/coms/IMUs/ICM20602.cpp \
         src/coms/IMUs/ICM20948.cpp \
         src/coms/Baro/MS5611.cpp \
         src/coms/SPI.cpp \
         src/coms/CAN.cpp \
         src/coms/CalFlash.cpp \
         src/coms/I2C.cpp \
         src/sensors/StrainRate.cpp \
         src/coms/PWM.cpp \
         src/coms/DShot.cpp \
         src/coms/Radio.cpp \
         src/coms/SBUS.cpp \
         src/coms/CRSF.cpp \
         src/coms/MAVLink.cpp \
         src/usb_serial.cpp \
         src/logging/Logger.cpp \
         src/diagnostics/ThreadTiming.cpp

ASMSRC  = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include paths: cfg/ provides chconf.h, halconf.h, mcuconf.h
# BOARDDIR provides board.h
INCDIR = $(CONFDIR) $(BOARDDIR) $(ALLINC)

CWARN   = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User defines
#

# Board-specific MCU variant + optional debug flag
UDEFS   = $(BOARD_UDEFS) -DCHPRINTF_USE_FLOAT=1 $(UDEFS_EXTRA)
UADEFS  =
UINCDIR = $(CURDIR)/third_party/mavlink
ULIBDIR =
ULIBS   = -lm

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

##############################################################################
# Upload targets (after ChibiOS rules so 'all' is the default target)
#

flash: all
	python3 $(UPLOAD_SCRIPT) --port $(PORT) build/$(PROJECT).bin

flash-stlink: all
	openocd $(OPENOCD_CFG) \
	    -c "program build/$(PROJECT).hex verify reset exit"

.PHONY: flash flash-stlink
