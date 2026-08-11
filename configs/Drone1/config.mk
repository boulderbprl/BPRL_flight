# Drone1 — CubeOrangePlus flight controller.
# Included by the top-level Makefile via: include configs/$(DRONE)/config.mk
# Sets exactly what the old BOARD=orange branch used to set.

BOARD_FULL  = CubeOrangePlus
# Deliberately H743, not the physically-correct H757: building with the
# correct -DSTM32H757xx -DCORE_CM7 makes i2cStart(&I2CD2, ...) hang forever
# (no assert, no CPU fault — only the IWDG recovers it) as soon as anything
# calls it, reproducible on two separate physical CubeOrangePlus units.
# Investigated at length (see git history/session around 2026-08-04): ruled
# out DMA enable bits, our bus-recovery bit-banging, I2C123SEL clock source,
# I2C2 pin mapping (verified against real ArduPilot CubeOrange hwdef.inc —
# correct), CM4 auto-boot interference (explicitly held in reset in main.cpp,
# no change), and every H743-vs-H757 CMSIS/RCC register difference found by
# inspection (all identical). ArduPilot's own CubeOrangePlus build flies
# fine on this same hardware, so it's not a chip defect either — something
# about our H757-path bring-up specifically. Root cause not found; this is a
# working, verified escape hatch, not a real fix. Flash/RAM base addresses
# and sizes are identical between the two headers on this part, so nothing
# else is known to regress from the mislabeling. Revisit with a debugger.
BOARD_UDEFS = -DSTM32H743xx -DBPRL_BOARD_CUBEORANGEPLUS
