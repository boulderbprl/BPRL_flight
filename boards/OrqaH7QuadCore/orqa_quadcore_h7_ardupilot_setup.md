# Orqa QuadCore H7 — ArduPilot DFU Setup (Ubuntu)

Initial ArduPilot flash for a factory-fresh Orqa FC 3030 H7 QuadCore. QGroundControl/Mission Planner **cannot** be used for this first flash — the board ships without an ArduPilot bootloader, so it must be bootstrapped via true DFU first.

## Prerequisites
```bash
sudo apt install dfu-util binutils   # binutils provides objcopy
```

## 1. Enter DFU mode
- Unplug USB
- Hold the **BOOT/DFU** button on the board
- Plug in USB-C while holding it
- Release after ~1 second

Confirm the board is visible:
```bash
lsusb | grep -i "0483:df11"
```
Expected: `ID 0483:df11 STMicroelectronics STM Device in DFU Mode`

## 2. Download firmware
Pick Copter or Plane and the desired release from the [ArduPilot firmware server](https://firmware.ardupilot.org/), then:
```bash
cd ~/Downloads
wget https://firmware.ardupilot.org/Copter/stable/OrqaH7QuadCore/arducopter_with_bl.hex
```

## 3. Convert hex → bin
dfu-util does not read `.hex` directly — convert first:
```bash
objcopy -I ihex -O binary arducopter_with_bl.hex arducopter_with_bl.bin
ls -la arducopter_with_bl.bin
```
Sanity check: resulting `.bin` should be roughly the same size as the source `.hex` (~5–6 MB). A dramatically larger file suggests an address-gap padding issue — stop and investigate before flashing.

## 4. Flash via DFU
```bash
dfu-util -a 0 --dfuse-address 0x08000000 -D arducopter_with_bl.bin
```
Let the progress bar finish completely. Do not unplug mid-write.

## 5. Reboot normally
Unplug USB, then plug back in **without** holding the boot button.

## 6. Verify
```bash
dmesg | tail -30
ls /dev/ttyACM*
```
A new `/dev/ttyACM*` device should appear, identifying as ArduPilot/STM32 CDC ACM.

## 7. Ongoing updates
Once the ArduPilot bootloader is in place, QGroundControl or Mission Planner can flash future firmware updates normally over that serial port — no more manual DFU needed unless the bootloader gets corrupted.

## Reference specs (for context)
- MCU: STM32H743, up to 480 MHz
- IMU: dual ICM-42688 (orthogonal mounting per Orqa's sibling WingCore board; not independently confirmed for QuadCore)
- Source: [ArduPilot hwdef — OrqaH7QuadCore](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/OrqaH7QuadCore)
