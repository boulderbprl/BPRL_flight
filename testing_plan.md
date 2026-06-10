# BPRL Flight Controller — Testing Plan

Each phase builds on the previous one. Do not advance to the next phase until all
pass criteria in the current phase are met. Every phase assumes a fresh flash of
the firmware and a USB connection to the host running `bprl.py`.

---

## Phase 1 — All 4 Motors Working (DShot600 Bidirectional)

**Goal:** Confirm all four motors arm, spin, and return valid RPM telemetry with the
per-channel TX DMA fix (change #31) in place.

**Current state:** Motor 1 (AUX 4 / PE9) was confirmed working. Motors 0, 2, 3 had
the root cause fixed in code but have not yet been verified on hardware.

**Build:**
```bash
make clean && make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0
```

**Active threads:** ControlThread, USBCmdThread, HeartbeatThread, DebugThread.
SPIThread, CANThread, StateEstThread, RadioThread all remain disabled.

---

### 1.1 Power-up check

1. Power the ESC rail (5 V via servo rail or BEC). Do **not** attach propellers.
2. Connect USB. Watch the LED: 3 fast blinks → 5 slow blinks → steady 0.5 Hz heartbeat.
3. Open `bprl.py motor-test` or a plain serial terminal on `/dev/ttyACM0`.
4. Confirm the ESC connection chime plays on **all four ESCs** within a few seconds of
   power-up. If any ESC does not chime, it is not receiving a valid DShot signal.

**Pass:** All 4 ESCs produce an arming chime within 5 s of power.
**Fail:** Any ESC silent → check wiring, verify `$DSHOT tc` is non-zero.

---

### 1.2 DShot diagnostic baseline

Send `DSHOT,diag` over USB and confirm:

```
DSHOT,DIAG,dma_tc=<N>/<M>,cc_isr=<N>/<M>,
           edges=0/0/0/0,   ← expected when motors are idle
           e0=0,0,0,0,0,
           ...
```

| Field | Expected when idle |
|-------|--------------------|
| `dma_tc` | Growing at ~400/s per timer |
| `cc_isr` | Growing at ~400/s (IC cycling) |
| `edges` | 0/0/0/0 (no GCR response at idle throttle) |

If `dma_tc` is 0/0, DShot init failed — check the LED for the 20/30-flash error code.

---

### 1.3 Motor test — spin each motor individually

Hold the drone **firmly on the bench, no propellers**. Send each command and wait 3 s:

```
MT,0,5     ← Motor 0 (FR, AUX 3, PE11)
MT,stop
MT,1,5     ← Motor 1 (RL, AUX 4, PE9) — previously confirmed
MT,stop
MT,2,5     ← Motor 2 (FL, AUX 5, PD13)
MT,stop
MT,3,5     ← Motor 3 (RR, AUX 2, PE13)
MT,stop
```

For each motor:
- Listen for the motor spinning up. The bell tone of the motor should be audible.
- Run `DSHOT,diag` while spinning. Confirm `edges` is non-zero on the correct motor slot:
  - Motor 0 → `edges[0]`
  - Motor 1 → `edges[1]`
  - Motor 2 → `edges[2]`
  - Motor 3 → `edges[3]`
- Check the `$TEL` output from `bprl.py motor-test` and confirm RPM increases from 0
  when `MT,N,5` is sent and drops back to 0 after `MT,stop`.

```
MT,0,10    ← raise to 10% to get a clear RPM reading
MT,0,5     ← drop back
MT,stop
```

**Pass criteria:**
- All 4 motors spin on command.
- All 4 `edges` counters are non-zero while the corresponding motor is spinning.
- Mechanical RPM displayed in `$TEL` is plausible (hundreds to low thousands at 5–10%).
- RPM increases as `pct` increases, decreases when load is applied manually.

**Fail — motor does not spin:**
1. Check `dma_tc` is still incrementing (DShot TX is alive).
2. Check `edges[N]` — if still 0, the ESC is not receiving a valid frame on that channel.
3. Power-cycle the ESC rail and retry. If the ESC chimed at power-up but MT fails,
   suspect the DShot output is valid but throttle value is below the ESC's arming minimum.
   Try `MT,N,15`.

**Fail — RPM telemetry reads 0 while motor is spinning:**
- ESC may not be in bidirectional mode. Re-enable bidirectional DShot in BLHeli32/AM32
  configurator with the motor running at a fixed throttle via an RC transmitter.
- Alternatively, inspect raw edge timestamps: `DSHOT,diag` → `eN=...` should show
  non-zero timestamps. If edges arrive but RPM is 0, there is a GCR decode issue.

---

### 1.4 All-motors simultaneous test

```
MT,0,5
```
*(Motor test only commands one motor at a time — for simultaneous spin, modify `g_motor_test_cmd[all]` temporarily or implement a `MT,all,5` command.)*

As an alternative, temporarily set all four `g_motor_test_cmd` slots in a debug build
and verify all four spin simultaneously without one motor dropping frames.

**Phase 1 complete when:** All 4 motors spin individually with valid RPM telemetry on demand.

---

## Phase 2 — EKF + State Estimator Alongside Motors

**Goal:** Re-enable SPIThread and StateEstThread. Verify the EKF produces correct
attitude output and that bidirectional DShot (edge capture via DMA) is not disrupted
by SPI DMA ISRs.

**Risk:** SPIThread runs SPI DMA at 1 kHz, ISR priority ~12. DShot IC now uses DMA-based
capture at priority 7. The old ISR-based IC was disrupted by SPI at priority 15; the new
DMA-based IC should be immune since the DMA engine captures edges without CPU intervention.
This phase confirms that empirically.

---

### 2.1 Code changes in `src/threads.cpp` `threads_start()`

Re-enable SPIThread and StateEstThread:

```cpp
void threads_start(const ThreadRates &rates)
{
    chThdCreateStatic(waSPI,      sizeof(waSPI),      NORMALPRIO + 30, SPIThread,      (void *)&rates.spi);
    chThdCreateStatic(waStateEst, sizeof(waStateEst), NORMALPRIO + 25, StateEstThread, (void *)&rates.est);
    chThdCreateStatic(waControl,  sizeof(waControl),  NORMALPRIO + 20, ControlThread,  (void *)&rates.control);
    chThdCreateStatic(waUSBCmd,   sizeof(waUSBCmd),   NORMALPRIO - 20, USBCmdThread,   nullptr);
    chThdCreateStatic(waHeartbeat,sizeof(waHeartbeat),NORMALPRIO -  5, HeartbeatThread,(void *)&rates.heartbeat);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,    sizeof(waDebug),    NORMALPRIO - 10, DebugThread,    (void *)&rates.debug);
#endif
}
```

Re-enable IMU init in `main.cpp` (already called via `motor_output_init()` → `dshot_init()`
but SPIThread calls `spi_drv_init()` — verify the call is in `SPIThread` body, not in `main()`).

**Build and flash** with `BPRL_DEBUG` enabled.

---

### 2.2 Verify IMU data arriving

Open `bprl.py telemetry`. In the `$TEL` output, check:

```
imu0_v=1, imu1_v=1, imu2_v=1
```

All three IMU valid flags should become 1 within a few seconds of boot (SPIThread
initialises IMUs during its first tick — ICM-45686 init takes ~100 ms each).

If any IMU shows `imu_v=0`:
- Run `DSHOT,diag` — if `tc` has dropped to 0, SPIThread is interfering with DShot init.
- Check `spi_drv_init()` is called from SPIThread (not main), after `dshot_init()` runs.

---

### 2.3 Verify EKF attitude output

Open `bprl.py ekf-status`. This shows the `$EKFL` line: per-lane roll/pitch/yaw in
degrees and p/q/r in rad/s.

Place the drone flat on the bench (level). Expected:
```
Primary lane: 0 (or whichever lane has best data)
Lane 0: roll≈0°, pitch≈0°, yaw≈anything (unobservable without magnetometer)
Lane 1: roll≈0°, pitch≈0°
Lane 2: roll≈0°, pitch≈0°
All lanes agree within ±2°
```

Tilt the drone ~45° roll. Verify roll reports ~45° and pitch remains near 0°.
Return to level. Verify roll returns to ~0°.

Rotate around yaw. Verify yaw changes (EKF integrates gyro for yaw; drift is expected).

---

### 2.4 Verify DShot still works with SPI running

While SPIThread is running, re-run the motor test from Phase 1:

```
MT,0,5
```

Monitor `DSHOT,diag` for 10 s. `edges[0]` should remain non-zero continuously.
`dma_tc` and `cc_isr` should both increment at ~400/s.

If `edges` drops to 0 intermittently or stays 0 while the motor spins:
- SPIThread ISRs are disrupting DShot IC even with DMA-based capture.
- Option A: Lower SPIThread to NORMALPRIO+22 so its DMA ISRs run at a lower priority.
- Option B: Increase `STM32_IRQ_TIM1_CC_PRIORITY` (currently 7) above SPI DMA ISR priority.
  Check mcuconf.h for the SPI DMA priorities and adjust.
- Option C: Switch SPIThread to polling mode (no DMA) at the cost of CPU time.

**Phase 2 complete when:** All 3 IMUs valid, EKF attitude correct to ±2°, and all 4 motors
can still spin with non-zero `edges` with SPIThread running.

---

## Phase 3 — CAN IMU (IMX5) Fused with EKF

**Goal:** Re-enable FDCAN1 and CANThread. Verify the IMX5 quaternion and rate data
arrive at ~200 Hz. Verify StateEstThread fuses the CAN data and the EKF state improves.

**Hardware required:** IMX5 INS unit connected to CAN1 (PH13=TX, PH14=RX) with
appropriate termination resistors (120 Ω at each end of the bus).

---

### 3.1 Code changes in `main.cpp`

Re-enable CAN driver init and CANThread:

```cpp
// In main(), after usb_serial_init():
can_drv_init();                    // start FDCAN1, register IMX5 callbacks

// In threads_start():
chThdCreateStatic(waCAN, sizeof(waCAN), NORMALPRIO + 28, CANThread, nullptr);
```

Ensure `can_drv_init()` is called **before** `threads_start()` so CANThread has an
initialised peripheral to receive from.

**Build and flash.**

---

### 3.2 Verify CAN bus is up

Before connecting the IMX5, check the CAN peripheral is not in bus-off state:

```
CAN,status
```

Expected output (no bus, nothing connected):
```
CAN,STATUS,psr=0x00000000,ecr=0x00000000,rxf0s=0x00000000,cccr=0x00000000
```

- `PSR.BO` (bit 7) = 0: not bus-off.
- `ECR.TEC` and `ECR.REC` should be 0 with no bus connected.
- If `PSR.BO` = 1, the peripheral entered bus-off due to missing ACK (no terminator or no other node). Power-cycle and retry after connecting at least one CAN node with a terminator.

---

### 3.3 Connect IMX5 and verify frame arrival

Power the IMX5. The CAN bitrate is configured for **1 Mbit/s** (`NBTP=0x06001104`,
HSE=24 MHz). Confirm the IMX5 is also configured for 1 Mbit/s.

Open `bprl.py telemetry`. Watch the `can_v` and `can_quat_hz` fields in `$TEL`:

```
can_v=1, can_quat_hz≈200, can_rate_hz≈200
```

- `can_quat_hz` should stabilise at ~200 Hz (IMX5 publish rate).
- `can_v` = 1 means at least one quaternion frame arrived within the last 200 ms.
- If `can_v=0` or `can_quat_hz=0`, check `CAN,status` for error counters and verify
  the IMX5 is transmitting on IDs 0x01–0x04.

---

### 3.4 Verify quaternion format and EKF fusion

Place the drone level. After a few seconds of CAN data arriving, open `bprl.py ekf-status`.

The IMX5 sends `q_NED→body` (conjugated in StateManager before passing to EKF). Verify:
- Lane attitudes still agree with each other (CAN fusion should not destabilise the EKF).
- Roll/pitch from `$TEL` still read ~0° when the drone is level.
- Rotate the drone slowly in roll. All three EKF lanes should track the motion closely,
  with the CAN quaternion accelerating convergence vs. IMU-only.

Check `$EKFL` for `can_valid=1` and that the primary lane is consistent.

**Verify IMX5 conjugation:** The IMX5 outputs `q_NED→body`. BPRL needs `q_body→NED`.
StateManager applies conjugation before calling `EKF::update_quaternion()`. Verify by
tilting the drone: if the EKF roll/pitch sign is correct (tilt right = positive roll),
conjugation is correct. If sign is inverted, the conjugation was not applied.

---

### 3.5 CAN stale timeout

Disconnect the IMX5 CAN cable mid-flight simulation. Verify:
- `can_v` drops to 0 within ~200 ms (100-tick timeout at 500 Hz StateEstThread).
- EKF continues running on IMU-only data without crashing.
- `can_quat_hz` drops to 0.
- Reconnect — `can_v` returns to 1 within ~200 ms.

**Phase 3 complete when:** `can_quat_hz ≈ 200`, EKF attitude correct with CAN data fused,
CAN stale-out handled gracefully.

---

## Phase 4 — CRSF Radio + Telemetry Tool

**Goal:** Re-enable RadioThread and USART2 (CRSF receiver on TELEM1). Verify all RC
channels parse correctly. Exercise the full telemetry dashboard with all data sources live.

**Hardware required:** ExpressLRS (ELRS) receiver connected to TELEM1:
- PD5 = USART2 TX (to receiver RX)
- PD6 = USART2 RX (to receiver TX, **3.3 V logic only**)
- CRSF baud: 420000 (ELRS standard). TBS CRSF spec is 416666 — ELRS receivers also accept
  420000; verify against your receiver's documentation.

---

### 4.1 Code changes in `main.cpp`

Re-enable radio init and RadioThread:

```cpp
// In main():
radio_input_init();              // starts USART2 at 420000 baud

// In threads_start():
chThdCreateStatic(waRadio, sizeof(waRadio), NORMALPRIO + 10, RadioThread, (void *)&rates.radio);
```

Ensure `radio_input_init()` is called before `threads_start()`.

**Verify `RADIO_PROTOCOL` is set to `RADIO_PROTO_CRSF`** (default in `Radio.hpp`):
```cpp
#define RADIO_PROTOCOL RADIO_PROTO_CRSF
```

**Build and flash.**

---

### 4.2 Verify CRSF frames parsing

Power on the RC transmitter. Bind the ELRS receiver to the transmitter if not already done.

Open `bprl.py telemetry`. Watch the RC channel fields in `$TEL`:
```
thr, rc_roll, rc_pitch, rc_yaw
```

Move each stick through its range and verify:
- Throttle stick at minimum → `thr ≈ 0.0`
- Throttle stick at maximum → `thr ≈ 1.0`
- Roll stick left → `rc_roll ≈ -1.0`; right → `rc_roll ≈ +1.0`
- Pitch stick forward → `rc_pitch ≈ -1.0` (or +1.0 depending on TX mode); back → opposite
- Yaw stick left → `rc_yaw ≈ -1.0`; right → `rc_yaw ≈ +1.0`
- Arm switch channel (Ch5, mapped in `Radio.cpp` channel(4)) → `armed=0` when switch low,
  `armed=1` when switch high

**Channel mapping** (Mode 2, standard):
```
Radio.cpp: ch[0]=Roll, ch[1]=Pitch, ch[2]=Throttle, ch[3]=Yaw, ch[4]=Arm
CRSF raw:  172 (min) ... 992 (centre) ... 1811 (max)
Normalised: norm_axis() = (raw - 992) / 819 → [-1, 1]
            norm_thr()  = (raw - 172) / 1639 → [0, 1]
```

If `thr`, `rc_roll`, etc. are stuck at 0 despite stick movement:
- Verify USART2 RX (PD6) is receiving data: use a logic analyser or toggle a debug GPIO
  from CrsfParser::update() on first byte receipt.
- Verify the receiver is outputting CRSF (not SBUS) — check receiver firmware settings.
- Verify baud rate: some ELRS receivers use 416666 baud. If parsing fails, try 416666.

---

### 4.3 Arm switch verification

Toggle the arm switch on the transmitter. Verify `armed` field in `$TEL` changes between
0 and 1. With `g_armed=true`, the `MT,` command should be **blocked**:

```
MT,0,10
→ MT,ERR,armed
```

This confirms RadioThread is writing `g_armed` and ControlThread + USBCmdThread are
reading it correctly.

---

### 4.4 Full telemetry dashboard test

With all threads running (SPI, StateEst, CAN, Radio, Control, USB, Heartbeat, Debug),
run:

```bash
python3 tools/bprl.py telemetry
```

Verify all fields are live and plausible:
- Attitude: roll/pitch/yaw track drone orientation
- Rates: p/q/r show angular rates when drone is rotated by hand
- RC channels: respond to stick input
- IMU valid: all 3 show `●`
- CAN valid: shows `●` with ~200 Hz rate
- Armed: follows arm switch

```bash
python3 tools/bprl.py ekf-status
```

Verify all 3 EKF lanes agree. Identify primary lane (usually lane 0, ICM-45686 on SPI1).

**Phase 4 complete when:** All RC channels parse correctly, arm switch works, full
telemetry dashboard shows all data sources live and consistent.

---

## Phase 5 — Controller Test (Props Off, Bench)

**Goal:** Restore the full PID → MotorMixer → DShot path in ControlThread. Verify
the attitude controller drives the motors correctly in response to attitude errors and
RC stick input. Do **not** attach propellers.

**This phase requires code changes to ControlThread and the MotorMixer output.**

---

### 5.1 Required code changes

#### 5.1.1 PWM-to-DShot conversion

MotorMixer outputs PWM microseconds (1000–1950). DShot uses 0 (disarm) or 48–2047
(throttle). Add a conversion helper to ControlThread or MotorMixer:

```cpp
// In threads.cpp or a shared utility:
static uint16_t pwm_to_dshot(int32_t pwm_us)
{
    if (pwm_us <= MotorMixer::PWM_IDLE) return 0U;   // disarm
    const int32_t clamped = (pwm_us < MotorMixer::PWM_MIN) ? MotorMixer::PWM_MIN :
                            (pwm_us > MotorMixer::PWM_MAX) ? MotorMixer::PWM_MAX : pwm_us;
    // Map [PWM_MIN=1100 .. PWM_MAX=1950] → [48 .. 2047]
    return (uint16_t)(48U + (uint32_t)(clamped - MotorMixer::PWM_MIN) * 1999U
                                     / (MotorMixer::PWM_MAX - MotorMixer::PWM_MIN));
}
```

#### 5.1.2 Restore ControlThread full path

Replace the current idle stub in ControlThread with the full cascade:

```cpp
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    systime_t next = chVTGetSystemTime();

    while (true) {
        /* Motor test bypass — unchanged */
        { /* ... existing MT bypass code ... */ }

        /* Build state vector for controller */
        float ctrl_state[6];
        bool  armed;
        float input[4];
        chMtxLock(&state_mtx);
        ctrl_state[0] = g_euler[0];          // roll  (rad)
        ctrl_state[1] = g_euler[1];          // pitch (rad)
        ctrl_state[2] = g_euler[2];          // yaw   (rad)
        ctrl_state[3] = g_state[StateIdx::P]; // p (rad/s)
        ctrl_state[4] = g_state[StateIdx::Q]; // q (rad/s)
        ctrl_state[5] = g_state[StateIdx::R]; // r (rad/s)
        armed  = g_armed;
        memcpy(input, g_input, sizeof(input));
        if (!armed) att_ctrl.reset_all();
        chMtxUnlock(&state_mtx);

        float torque_cmds[3];
        att_ctrl.update(ctrl_state, input, torque_cmds);
        const float thrust = att_ctrl.compute_throttle(ctrl_state, input);

        int32_t pwm_out[4];
        mixer.update(torque_cmds, thrust, armed, ctrl_state, pwm_out);

        uint16_t dshot_cmd[4];
        for (int i = 0; i < 4; i++) dshot_cmd[i] = pwm_to_dshot(pwm_out[i]);
        dshot_write(dshot_cmd);

        chMtxLock(&state_mtx);
        memcpy(g_output, pwm_out, sizeof(g_output));
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}
```

**Build and flash with BPRL_DEBUG.**

---

### 5.2 Disarmed motor output verification

With RC transmitter connected but arm switch in **disarmed** position:
- Run `bprl.py telemetry`. Confirm `armed=0`.
- Confirm all motor outputs are 0 (DShot 0 = disarm command).
- Move sticks in all axes. Motor outputs should remain 0 (disarmed).
- ESCs should produce no spin. Connection chime still active, no motor rotation.

---

### 5.3 Arm and verify zero-throttle idle

Set arm switch to **armed**, throttle at minimum:
- `armed=1` appears in telemetry.
- Motors should receive `DShot = 48` (PWM_MIN = 1100 µs → DShot 48).
  Depending on ESC configuration, motors may or may not spin at this level.
  If motors do not spin at 48, the ESC arming minimum is above 48 — this is normal.
- Do **not** raise throttle yet.

Safety check: confirm disarming (arm switch to off) immediately sends DShot 0 and
motors (if spinning) stop within ~100 ms.

---

### 5.4 Attitude response check (level)

Place the drone **flat and level** on the bench. Arm it. Raise throttle to ~20%:
- All 4 motors should spin at equal speed (attitude error ≈ 0).
- Roll/pitch telemetry should read ≈ 0°. Rate p/q/r should read ≈ 0.
- Verify RPM is roughly equal across all 4 motors in `$TEL`.

Tilt the drone manually ~20° in roll while holding throttle at 20%:
- Roll error = 20°. The controller should increase output on the low side and decrease
  on the high side to correct.
- In telemetry, `p` (roll rate) should show a non-zero response.
- Return the drone to level — motors should equalise.

**This verifies that the cascade PID is flowing from EKF attitude error → rate command
→ torque → mixer → DShot, end to end.**

---

### 5.5 Gain sanity checks

With the drone tilted by hand, observe the $TEL output for oscillation or divergence:

| Symptom | Likely cause |
|---------|-------------|
| Motors spin up smoothly, drone wants to level | Gains approximately correct |
| Rapid oscillation (jitter) at low tilt | Rate gains too high — reduce `_roll_rate.kp` (0.11 → ~0.06) |
| Sluggish or no response | Gains too low, or state/input wiring inverted |
| One motor always at max | Sign error in mixer mapping (check FR/RL/FL/RR convention) |
| Motors diverge on arm | Integrator winding up — ensure `att_ctrl.reset_all()` runs on every disarm tick |

**Current gains** (from AttitudeController.cpp):
```
Roll/pitch attitude: Kp=4.50
Roll/pitch rate:     Kp=0.11, Ki=0.09, Kd=0.003, Imax=0.5
Yaw rate:            Kp=0.10, Ki=0.02
```
These were tuned for a prior Tiva-based frame and will almost certainly need adjustment for
the CubeOrange+ with these specific motors/props. Treat them as a starting point only.

---

### 5.6 Safety disarm test

Tilt the drone past 80° (MotorMixer::MAX_ANGLE = 1.396 rad ≈ 80°):
- `MotorMixer::should_disarm()` returns true.
- All motors should drop to PWM_IDLE → DShot 0.
- Verify in telemetry: motor outputs go to 0 when tilt exceeds threshold.

**Phase 5 complete when:** Full PID path active, attitude response correct in direction,
safety disarm triggers at ~80°, no instability at bench tilt tests.

---

## Phase 6 — First Flight (Stabilize Mode)

**Goal:** Fly the drone in a stable hover. The ControlThread is running the full
cascade PID → mixer → DShot pipeline with all sensors active.

**Prerequisites — all must be met before flight:**
- [ ] Phase 5 complete: controller verified on bench, correct response direction, no oscillation
- [ ] All 4 motors spin with valid RPM telemetry (Phase 1)
- [ ] EKF attitude correct to ±2° on bench (Phase 2)
- [ ] CAN IMU fused and providing ~200 Hz data (Phase 3)
- [ ] CRSF radio arm switch confirmed working (Phase 4)
- [ ] PID gains tuned on bench — no oscillation
- [ ] Propellers installed and balanced
- [ ] Frame mechanically sound; no loose motor mounts or wiring
- [ ] Battery voltage appropriate for ESC/motor rating
- [ ] Clear, open outdoor area with no overhead obstacles

---

### 6.1 Pre-flight checks

**On the bench, props on, tied down:**
1. Connect battery. All 4 ESCs chime.
2. USB telemetry dashboard: all IMUs valid, CAN valid at ~200 Hz, RC channels active.
3. Arm with arm switch. Raise throttle to ~10%. All 4 motors spin.
4. Wave hand near each motor: RPM telemetry should dip as prop load increases.
5. Disarm. Confirm all motors stop.
6. Verify battery voltage is adequate. Do not fly below manufacturer minimum.

**LogThread** (if re-enabled): Confirm SD card mounted and log file opening before flight.
Send `LOG,list` and verify a new file is created after arming.

---

### 6.2 Initial hover attempt

**Location:** Open outdoor area, calm wind, clear ground.

1. Place drone on flat ground. Check attitude in telemetry: roll/pitch within ±2° of level.
2. Arm. Raise throttle slowly from 0 to approximately 50–60% while watching the drone.
3. **Expected:** Drone lifts off and hovers with small corrections. Small drift in position
   is expected (no GPS / position hold).
4. If the drone hovers stably, hold for 5–10 s then land by gradually reducing throttle.
5. Disarm immediately on touchdown.

**If the drone tips or rolls on takeoff:**
- **Identify the direction** of the uncontrolled rotation (roll left/right or pitch forward/back).
- A consistent tip in one direction indicates a sign error in either:
  a. The EKF attitude convention (roll sign wrong → controller applies wrong correction).
  b. The motor mixer sign (e.g., FR/RL/FL/RR frame numbering wrong).
  c. IMU rotation correction wrong.
- Reduce throttle below hover threshold immediately. Do **not** attempt to catch the drone.
- Debug on bench: tilt drone manually left in roll while watching motor outputs in `$TEL`.
  Motor FR[0] and RR[3] should increase (they are on the right side of an X-frame and should
  push the left side up). If they decrease instead, the roll mixer sign is inverted.

---

### 6.3 Common first-flight issues and fixes

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| Drone flips immediately on arm | Motor rotation direction wrong | Swap two motor wires or reverse in ESC config |
| Drone tilts left on takeoff | Roll mixer sign inverted | Negate roll term in MotorMixer::update() |
| Drone pitches forward | Pitch mixer or EKF pitch sign | Check mixer and att_ctrl pitch convention |
| Oscillating in roll/pitch | Rate Kp too high | Reduce `_roll_rate.kp` and `_pitch_rate.kp` by 30% |
| Slow response, under-damps | Gains too low | Increase `_roll_att.kp` or `_roll_rate.kp` |
| Yaw drifts | Yaw rate Ki too low | Increase `_yaw_rate.ki` from 0.02 → 0.05 |
| Unexpected disarm mid-air | MotorMixer::MAX_ANGLE triggered | Reduce aggression, or temporarily raise MAX_ANGLE |
| EKF attitude diverges in flight | Gyro bias estimate drifting | Run IMU calibration before flight (`python3 tools/bprl.py calibrate`) |

---

### 6.4 Gain tuning procedure

Perform tuning in a large open space with safety line attached if available. Adjust one
parameter at a time:

**Step 1 — Roll/pitch rate Kp:**
With drone hovering, apply a sharp roll stick input and release. Observe:
- Underdamped (oscillates after release): reduce `_roll_rate.kp`.
- Overdamped (slow return): increase `_roll_rate.kp`.
Target: 2–3 oscillation cycles that decay within 0.5 s.

**Step 2 — Roll/pitch rate Kd:**
Increase `_roll_rate.kd` gradually from 0.003. Stop when oscillation at high-frequency
noise appears. Derivative filters at 30 Hz cutoff (`fcut_hz=30.0f` in PID constructor).

**Step 3 — Roll/pitch attitude Kp:**
With rate loop stable, apply a roll angle setpoint and observe tracking speed.
- Too slow: increase `_roll_att.kp` from 4.50.
- Oscillation: decrease it.

**Step 4 — Yaw rate:**
Yaw is slower. Test by applying full yaw stick input. Increase `_yaw_rate.kp` if
yaw response is sluggish; add `_yaw_rate.kd` if yaw oscillates.

**Step 5 — Integrators:**
Enable `_roll_rate.ki` and `_pitch_rate.ki` (currently 0.09) after rate Kp/Kd are set.
They help correct steady-state errors from wind or CG offset. If integrator wind-up causes
slow oscillations, ensure `Imax=0.5` is appropriate.

---

### 6.5 Telemetry monitoring during flight

Run `bprl.py telemetry` on a laptop with USB connected during bench-side tests. For
outdoor flight, use the SD logger (LogThread, currently disabled — re-enable before flying):

```cpp
// In threads_start():
chThdCreateStatic(waLog, sizeof(waLog), NORMALPRIO - 15, LogThread, (void *)&rates.log);
```

After landing, download the log:
```bash
python3 tools/bprl.py logs download
python3 tools/bprl.py logs decode <FILE>.bin
```

Inspect `roll`, `pitch`, `yaw`, `p`, `q`, `r`, and motor `pwm` columns in the CSV to
verify the control loop was well-behaved throughout the flight.

---

### 6.6 Phase 6 completion criteria

- [ ] Drone lifts off and hovers without pilot correction for ≥ 5 s
- [ ] Roll and pitch respond correctly to stick input in direction and magnitude
- [ ] Yaw responds to yaw stick; drift < 10°/s in calm conditions
- [ ] Disarm on touchdown: all motors stop within 200 ms of arm switch
- [ ] No unexpected disarms during hover (EKF stable, no ISR faults)
- [ ] Log file downloaded and roll/pitch error within ±5° during hover

---

## Summary Checklist

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | All 4 motors: DShot600 bidirectional, RPM telemetry | Pending hardware test |
| 2 | EKF + SPIThread running alongside motors | Not started |
| 3 | CAN IMU fused with EKF | Not started |
| 4 | CRSF radio + full telemetry dashboard | Not started |
| 5 | Full PID controller bench test, props off | Not started |
| 6 | First flight, stabilize mode | Not started |

---

## Code Changes Required Per Phase

| Phase | File | Change |
|-------|------|--------|
| 2 | `src/threads.cpp` `threads_start()` | Re-enable SPIThread, StateEstThread |
| 3 | `main.cpp` | Call `can_drv_init()` before `threads_start()` |
| 3 | `src/threads.cpp` `threads_start()` | Re-enable CANThread |
| 4 | `main.cpp` | Call `radio_input_init()` before `threads_start()` |
| 4 | `src/threads.cpp` `threads_start()` | Re-enable RadioThread |
| 5 | `src/threads.cpp` `ControlThread` | Restore full PID/mixer path; add `pwm_to_dshot()` |
| 6 | `src/threads.cpp` `threads_start()` | Re-enable LogThread |
