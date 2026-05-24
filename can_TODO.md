# CAN INS (IMX5) — Debug Notes & TODO

## Current Status

FC-side CAN bus is healthy. IMX5 is not transmitting.

```
$ python3 tools/bprl.py can-status
FDCAN1 Status
  PSR  0x0000070f  →  ACT=Idle  LEC=NoChange  OK
  ECR  0x00000000  →  TEC=0  REC=0  CEL=0
  RXF0S 0x00000000  →  fill=0
  CCCR 0x00000000  →  INIT=0  MON=0  TEST=0
```

**Interpretation:** FDCAN1 synchronized to bus (ACT=Idle, not Synchronizing), zero
errors (TEC=REC=0), zero frames received. Baud rate match and termination are both
fine on the FC side. The IMX5 is simply not outputting CAN data.

---

## Root Cause

The IMX-5 defaults to UART output. CAN output must be explicitly enabled via
InertialSense EvalTool — requires the IMX5-to-USB cable.

---

## TODO

### 1. Enable CAN output on the IMX-5 (needs USB cable)

1. Connect IMX-5 to PC via the USB config cable.
2. Open InertialSense EvalTool.
3. Navigate to **Communications → CAN** and set baud rate to **1000 kbps**.
4. Enable the following data sets for CAN output:
   - **INS quaternion** (NED→Body) → must map to CAN ID **0x01**
   - **Angular rates** (p, q, r) → must map to CAN IDs **0x02–0x04**
   - **Accelerometer** (ax, ay, az) → same frames as rates
5. Save configuration and power-cycle the IMX-5.

### 2. Verify CAN ID and scaling match firmware expectations

The firmware (`src/coms/CAN.cpp`) parses this protocol:

| CAN ID | Contents | Scaling |
|--------|----------|---------|
| 0x01 | Quaternion NED→Body [W, X, Y, Z] | int16 ÷ 10000 (each component) |
| 0x02 | p rate (bytes 0–1), ax (bytes 2–3) | int16 ÷ 1000 rad/s, int16 ÷ 100 m/s² |
| 0x03 | q rate (bytes 0–1), ay (bytes 2–3) | same scaling |
| 0x04 | r rate (bytes 0–1), az (bytes 2–3) | same scaling |

If the IMX-5 firmware uses different IDs or different scaling, update
`imx5_can_cb()` in `src/coms/CAN.cpp` to match.

### 3. Verify on hardware

After IMX-5 is configured:
```bash
python3 tools/bprl.py can-status
```
Expected (frames arriving):
- `ACT=Receiver` or `ACT=Idle`
- `LEC=NoError`
- `RXF0S fill > 0` (transiently)

Then check telemetry:
```bash
python3 tools/bprl.py telemetry
```
Expected: `CAN INS (IMX5): ● valid`, `can_quat_hz ≈ 200`, `can_rate_hz ≈ 100`.

### 4. Validate fusion

Once frames are arriving:
1. Hold drone still — verify blended body rates stay near zero.
2. Rotate slowly about each axis — verify EKF innovation norm (per-lane) stays low
   and doesn't diverge when IMX5 quaternion updates are fused.
3. If innovation norm spikes on rotation, check the quaternion conjugation sign in
   `StateManager::update()` (line ~69): `{q0, -q1, -q2, -q3}` assumes IMX5 outputs
   q_NED→Body in Hamilton convention. Confirm with IMX-5 datasheet.
