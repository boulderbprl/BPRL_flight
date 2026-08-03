# Drone1 — CubeOrangePlus flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=orange branch used to set.

BOARD_FULL  = CubeOrangePlus
BOARD_UDEFS = -DSTM32H757xx -DCORE_CM7 -DBPRL_BOARD_CUBEORANGEPLUS
