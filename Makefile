##############################################################################
# BPRL Flight Controller — ChibiOS Makefile
#
# Usage:
#   make                            (default: DRONE=Drone1, plain flight build)
#   make DRONE=Drone1               (CubeOrangePlus, default)
#   make DRONE=Drone2               (CubeBlueH7)
#   make flash  DRONE=Drone2 PORT=/dev/ttyACM0   (Cube bootloader)
#   make flash-stlink DRONE=Drone2                (ST-Link / OpenOCD)
#
# Build output is segregated per-drone AND per-UDEFS_EXTRA: build/<DRONE>/
# BPRL.bin, or build/<DRONE>-<UDEFS_EXTRA>/BPRL.bin when UDEFS_EXTRA is set
# (e.g. build/Drone2-BPRL_DEBUG) — not a single shared build/BPRL.bin.
# Switching DRONE= or UDEFS_EXTRA= is always safe, no clean needed either way.
#   make clean                      (wipes ALL drones'/flags' build/.dep)
#   make clean-all                  (identical — kept as an explicit synonym)
#   'clean' ignores DRONE=/UDEFS_EXTRA= and always wipes everything; there is
#   no per-drone-only clean target.
#
# IMPORTANT: UDEFS_EXTRA is not remembered between separate `make` calls —
# it must be repeated on the flash command too, or `make flash` will build
# and flash the DEFAULT (non-debug) binary instead of the one you just built:
#   make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_DEBUG flash PORT=/dev/ttyACM0   (correct, one line)
#   make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_DEBUG               (builds build/Drone2-BPRL_DEBUG/)
#   make flash DRONE=Drone2 UDEFS_EXTRA=-DBPRL_DEBUG PORT=/dev/ttyACM0   (must repeat the flag)
#   make flash DRONE=Drone2 PORT=/dev/ttyACM0                 (WRONG: flashes build/Drone2/, not the debug build)
#
# Debug UART (USART3 @ 115200, Telem1 connector):
#   make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_DEBUG
#   (Disable before flight — adds a 10 Hz print thread)
#
# Thread timing / CPU utilization instrumentation (schedulability testing):
#   make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_TIMING
#   (Testing/bench only — query over USB with "TIM,status" / "TIM,reset".
#   Zero-cost when the flag is absent; see src/diagnostics/ThreadTiming.hpp)
#

##############################################################################
# Drone selection
#
# DRONE picks a physical airframe's config — configs/<DRONE>/config.mk sets
# BOARD_FULL/BOARD_UDEFS (which flight-controller board this drone uses; same
# mechanism/internal naming as the old BOARD=blue/orange selector) and
# configs/<DRONE>/drone_config.cpp supplies its DroneConfig (gains, RC
# mapping, motor geometry, sensors, logging — see configs/DroneConfig.hpp).

ifdef BOARD
  $(error BOARD= is retired — use DRONE=Drone1 or DRONE=Drone2 instead. \
    (make BOARD=blue would otherwise be silently ignored and quietly build \
    whatever DRONE defaults to, which on a flight controller is worse than \
    a build error.))
endif

DRONE ?= Drone1

include configs/$(DRONE)/config.mk

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
# Per-DRONE *and* per-UDEFS_EXTRA build/dep dirs — NOT just "build"/".dep".
# make's incremental build only compares .o mtimes against source/header
# mtimes; it has no way to know a command-line define changed between
# invocations (neither BOARD_UDEFS, e.g. -DBPRL_BOARD_CUBEBLUE vs _ORQA, nor
# UDEFS_EXTRA, e.g. -DBPRL_DEBUG). A shared build dir meant "make DRONE=
# Drone2" right after a Drone3 build would print "Nothing to be done for
# 'all'" and silently leave the *previous* board's .bin sitting there under
# the Drone2 name — flashing it onto different hardware than it was built
# for. Segregating by $(DRONE) alone fixed that specific case, but the same
# staleness bug still applied to UDEFS_EXTRA: "make DRONE=Drone2
# UDEFS_EXTRA=-DBPRL_DEBUG" right after a plain "make DRONE=Drone2" also
# printed "Nothing to be done" and silently flashed the *non*-debug binary
# — confirmed by inspecting the .elf's own strings (it had "CAL,ERR,
# debug_build_required", the non-debug fallback, not "$TEL"/"CAL,SET").
# Folding a sanitized UDEFS_EXTRA into the path too closes that the same
# way: switching UDEFS_EXTRA always lands in its own directory, so there is
# always a genuine build for exactly what was requested.
empty :=
space := $(empty) $(empty)
UDEFS_EXTRA_CLEAN := $(strip $(UDEFS_EXTRA))
ifneq ($(UDEFS_EXTRA_CLEAN),)
UDEFS_EXTRA_SAFE  := $(subst =,_,$(subst $(space),-,$(subst -D,,$(UDEFS_EXTRA_CLEAN))))
BUILD_SUFFIX      := -$(UDEFS_EXTRA_SAFE)
else
BUILD_SUFFIX      :=
endif
BUILDDIR := build/$(DRONE)$(BUILD_SUFFIX)
DEPDIR   := .dep/$(DRONE)$(BUILD_SUFFIX)

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
# OrqaH7QuadCore: application starts at 0x08060000 (after 384 KB BL) — see
# boards/OrqaH7QuadCore/STM32H743xI_app.ld.
ifeq ($(BOARD_FULL),CubeOrangePlus)
    LDSCRIPT = $(BOARDDIR)/STM32H743xI_app.ld
else ifeq ($(BOARD_FULL),CubeBlueH7)
    LDSCRIPT = $(BOARDDIR)/STM32H743xI.ld
else ifeq ($(BOARD_FULL),OrqaH7QuadCore)
    LDSCRIPT = $(BOARDDIR)/STM32H743xI_app.ld
else
    LDSCRIPT = $(STARTUPLD)/STM32H743xI.ld
endif

# C sources
CSRC = $(ALLCSRC) \
       $(BOARDDIR)/board.c \
       $(CHIBIOS)/os/various/syscalls.c

# C++ sources
CPPSRC = $(ALLCPPSRC) \
         configs/$(DRONE)/drone_config.cpp \
         main.cpp \
         src/threads.cpp \
         src/math/math.cpp \
         src/state_estimator/EKF.cpp \
         src/state_estimator/StateManager.cpp \
         src/controllers/PID.cpp \
         src/controllers/Attitude_PID.cpp \
         src/controllers/Attitude_INDI.cpp \
         src/controllers/Attitude_PID_PI.cpp \
         src/controllers/AltControl.cpp \
         src/controllers/PosControl.cpp \
         src/controllers/Unmixer.cpp \
         src/controllers/FlightStateMachine.cpp \
         src/controllers/MotorMixer.cpp \
         src/coms/IMUs/ICM42688.cpp \
         src/coms/IMUs/ICM45686.cpp \
         src/coms/IMUs/ICM20602.cpp \
         src/coms/IMUs/ICM20948.cpp \
         src/coms/IMUs/ICM20649.cpp \
         src/coms/Baro/MS5611.cpp \
         src/coms/Baro/DPS310.cpp \
         src/coms/SPI.cpp \
         src/coms/CAN.cpp \
         src/coms/CalFlash.cpp \
         src/coms/I2C.cpp \
         src/sensors/StrainRate.cpp \
         src/sensors/StrainGauge.cpp \
         src/sensors/EncoderRPM.cpp \
         src/coms/PWM.cpp \
         src/coms/DShot.cpp \
         src/coms/IOMCU.cpp \
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

# Overrides ChibiOS's own 'clean' target (defined by the rules.mk included
# above, which only removes $(BUILDDIR)/$(DEPDIR) for whichever single
# DRONE is currently selected — build/<DRONE>, .dep/<DRONE>; see the
# per-DRONE BUILDDIR/DEPDIR comment near the top of this file). A rule
# redefinition like this is intentional here, not an accident — GNU Make
# prints an "overriding recipe for target 'clean'" warning for it, which is
# expected and fine to ignore. Bare 'make clean' cleaning only whichever
# DRONE defaults to (Drone1) surprised users expecting every board's cache
# gone; 'clean' now always means "start completely fresh," full stop.
# 'clean-all' is kept as an explicit synonym for anyone who typed the old name.
clean clean-all:
	@echo Cleaning all drones
	@rm -rf build .dep
	@echo Done

.PHONY: clean clean-all

flash: all
	python3 $(UPLOAD_SCRIPT) --port $(PORT) $(BUILDDIR)/$(PROJECT).bin

flash-stlink: all
	openocd $(OPENOCD_CFG) \
	    -c "program $(BUILDDIR)/$(PROJECT).hex verify reset exit"

.PHONY: flash flash-stlink
