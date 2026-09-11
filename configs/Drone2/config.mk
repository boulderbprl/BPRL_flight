# Drone2 — CubeBlueH7 flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=blue branch used to set.

BOARD_FULL  = CubeBlueH7
# Drone2's ESCs are standard PWM (1000-2000 us pulses on FMU CH1-4), not
# DShot — override the project-wide DShot default here. See src/coms/PWM.hpp.
BOARD_UDEFS = -DSTM32H753xx -DSTM32_ENFORCE_H7_REV_XY -DBPRL_BOARD_CUBEBLUE -DMOTOR_PROTOCOL=MOTOR_PROTO_PWM
