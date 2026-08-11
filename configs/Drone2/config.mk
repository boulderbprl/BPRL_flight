# Drone2 — CubeBlueH7 flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=blue branch used to set.

BOARD_FULL  = CubeBlueH7
BOARD_UDEFS = -DSTM32H753xx -DSTM32_ENFORCE_H7_REV_XY -DBPRL_BOARD_CUBEBLUE
