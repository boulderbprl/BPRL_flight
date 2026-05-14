##############################################################################
# BPRL Flight Controller — ChibiOS Makefile
#
# Usage:
#   make BOARD=CubeBlueH7          (default)
#   make BOARD=CubeOrangePlus
#   make flash  BOARD=CubeBlueH7   PORT=/dev/ttyACM0   (Cube bootloader)
#   make flash-stlink BOARD=CubeBlueH7                  (ST-Link / OpenOCD)
#
# Debug UART (USART3 @ 115200, Telem1 connector):
#   make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG
#   (Disable before flight — adds a 10 Hz print thread)
#

##############################################################################
# Board selection
#

BOARD ?= CubeBlueH7

BOARDDIR := boards/$(BOARD)

ifeq ($(BOARD), CubeBlueH7)
  BOARD_UDEFS = -DSTM32H753xx -DSTM32_ENFORCE_H7_REV_XY
else ifeq ($(BOARD), CubeOrangePlus)
  BOARD_UDEFS = -DSTM32H743xx
else
  $(error Unknown BOARD="$(BOARD)". Valid values: CubeBlueH7, CubeOrangePlus)
endif

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
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
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

# H743 and H753 have identical flash/RAM layout
LDSCRIPT = $(STARTUPLD)/STM32H743xI.ld

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
         src/controllers/AttitudeController.cpp \
         src/controllers/MotorMixer.cpp \
         src/coms/IMUs/ICM20948.cpp \
         src/coms/IMUs/ICM20602.cpp \
         src/coms/SPI.cpp \
         src/coms/CAN.cpp \
         src/coms/I2C.cpp \
         src/coms/PWM.cpp \
         src/coms/DShot.cpp \
         src/coms/Radio.cpp \
         src/coms/SBUS.cpp \
         src/coms/CRSF.cpp \
         src/usb_serial.cpp \
         src/logging/Logger.cpp

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
UDEFS   = $(BOARD_UDEFS) $(UDEFS_EXTRA)
UADEFS  =
UINCDIR =
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
