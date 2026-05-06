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
# Architecture or project specific options
#

# Main thread process stack size
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x800
endif

# IRQ/exceptions stack size
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x800
endif

# STM32H753 has a double-precision FPU - always use hard float
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

##############################################################################
# Project, target, sources and paths
#

PROJECT = ch

# Cortex-M7 core
MCU = cortex-m7

# Path to pinned ChibiOS submodule
CHIBIOS  := third_party/ChibiOS

# Config headers live in cfg/
CONFDIR  := cfg

# Board files live in board/
BOARDDIR := board

BUILDDIR := build
DEPDIR   := .dep

# Licensing files
include $(CHIBIOS)/os/license/license.mk
# Startup files for STM32H7xx
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
# HAL
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
# Board files - use our own instead of the Nucleo board
include $(BOARDDIR)/board.mk
# OSAL
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build sources in ./src recursively (create this dir for your code)
# include $(CHIBIOS)/tools/mk/autobuild.mk

# Linker script - H743 and H753 have identical flash/RAM layout
LDSCRIPT = $(STARTUPLD)/STM32H743xI.ld

# C sources
CSRC = $(ALLCSRC) \
       $(BOARDDIR)/board.c \
       main.c

# C++ sources
CPPSRC = $(ALLCPPSRC)

# ASM sources
ASMSRC  = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include paths - cfg/ provides chconf.h, halconf.h, mcuconf.h
INCDIR = $(CONFDIR) $(BOARDDIR) $(ALLINC)

# Warnings
CWARN   = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User defines
#

# STM32H753 is a Rev-XY silicon variant of the H7 family.
# STM32_ENFORCE_H7_REV_XY enables workarounds for this revision.
# Keep this unless you are sure you have a newer silicon rev.
UDEFS = -DSTM32H753xx \
        -DSTM32_ENFORCE_H7_REV_XY

UADEFS  =
UINCDIR =
ULIBDIR =
ULIBS   =

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
