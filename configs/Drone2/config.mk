# Drone2 — CubeBlueH7 flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=blue branch used to set.

BOARD_FULL  = CubeBlueH7
# Drone2's ESCs are standard PWM (1000-2000 us pulses on FMU CH1-4), not
# DShot — override the project-wide DShot default here. See src/coms/PWM.hpp.
#
# Deliberately H743, not the physically-correct H753: mirrors Drone1's
# escape hatch in configs/Drone1/config.mk for the exact same symptom
# (FC totally unresponsive over USB, ~32s IWDG reset signature) on the same
# H74x/H75x/H757 chip family — see that file's comment for the full
# investigation history. Root cause not confirmed for Drone2 specifically;
# this mirrors a working, verified fix from a sibling board, not a
# from-scratch diagnosis. board.h notes CubeBlueH7 is pin-identical to
# CubeOrange+/H743, "only the MCU variant differs: H753 vs H743", and
# already reuses CubeOrange's STM32H743xI.ld linker script. Revert to
# -DSTM32H753xx if this doesn't fix it or causes a new regression.
BOARD_UDEFS = -DSTM32H743xx -DBPRL_BOARD_CUBEBLUE -DMOTOR_PROTOCOL=MOTOR_PROTO_PWM
