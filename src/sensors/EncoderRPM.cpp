#include "src/sensors/EncoderRPM.hpp"
#include "src/coms/CAN.hpp"
#include "src/uptime.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

static void encoder_rpm_can_cb(const CANRxFrame &f, void *ctx)
{
    const int node = (int)(intptr_t)ctx;

    float rpm;
    memcpy(&rpm, &f.data8[0], sizeof(rpm));
    uint16_t angle_raw  = (uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8);
    uint8_t  error_flag = f.data8[6];
    uint8_t  status     = f.data8[7];

    chMtxLock(&encoderRpm_mtx);
    g_encoder_rpm[node].rpm        = rpm;
    g_encoder_rpm[node].angle_raw  = angle_raw;
    g_encoder_rpm[node].error_flag = error_flag;
    g_encoder_rpm[node].status     = status;
    g_encoder_rpm[node].valid      = true;
    g_encoder_rpm[node].last_rx_us = bprl_micros();
    g_encoder_rpm[node].rx_count++;
    chMtxUnlock(&encoderRpm_mtx);
}

void EncoderRpmMonitor::update(const EncoderRPMRaw snap[ENCODER_RPM_NUM_NODES], uint32_t now_us,
                               bool require_spin, const float motor_cmd[ENCODER_RPM_NUM_NODES],
                               const int8_t expected_sign[ENCODER_RPM_NUM_NODES], float dt_s,
                               EncoderNodeHealth out[ENCODER_RPM_NUM_NODES])
{
    const float model_alpha = dt_s / (ENCODER_MODEL_TAU_S + dt_s);

    for (int node = 0; node < ENCODER_RPM_NUM_NODES; node++) {
        const EncoderRPMRaw &e = snap[node];
        Node &n = _node[node];

        const float model_ss = ENCODER_MODEL_RPM_AT_FULL * motor_cmd[node];
        n.model_rpm += model_alpha * (model_ss - n.model_rpm);

        // Frames lost between the ones this tick saw: the counter should
        // have advanced by exactly the number of frames received.
        const uint32_t new_frames = e.rx_count - n.last_rx_count;
        const uint8_t  seq        = e.status >> ENCODER_STATUS_SEQ_SHIFT;
        if (new_frames > 0 && new_frames < 8 && n.last_rx_count != 0) {
            n.seq_gaps += (uint8_t)(seq - n.last_seq - new_frames) & 0x07;
        }
        n.last_rx_count = e.rx_count;
        n.last_seq      = seq;

        const float rpm_abs = fabsf(e.rpm);
        n.slow_ticks = (require_spin && rpm_abs < ENCODER_RPM_MIN_SPIN) ? n.slow_ticks + 1 : 0;
        const bool wrong_sign = require_spin && expected_sign[node] != 0
                             && e.rpm * (float)expected_sign[node] < 0.0f;
        n.sign_ticks = wrong_sign ? n.sign_ticks + 1 : 0;

        // Signed: a frame that lands after the caller read now_us gives a
        // small negative age, which is fresh, not 71 minutes stale.
        const int32_t age_us = (int32_t)(now_us - e.last_rx_us);

        uint8_t fault = 0;
        if (!e.valid) {
            fault |= ENCODER_FAULT_NO_DATA;
        } else {
            if (age_us > (int32_t)ENCODER_RPM_STALE_US)          fault |= ENCODER_FAULT_STALE;
            if (!(e.status & ENCODER_STATUS_HEALTHY))            fault |= ENCODER_FAULT_NODE;
            if (!(rpm_abs <= ENCODER_RPM_MAX_ABS))               fault |= ENCODER_FAULT_RANGE;  // also NaN
            if (n.slow_ticks >= ENCODER_RPM_FAULT_TICKS)         fault |= ENCODER_FAULT_STALL;
            if (n.sign_ticks >= ENCODER_RPM_FAULT_TICKS)         fault |= ENCODER_FAULT_SIGN;
        }

        EncoderNodeHealth &h = out[node];
        h.fault     = fault;
        h.model_rpm = n.model_rpm;
        if (fault) {
            h.state   = SensorHealth::BAD;
            h.rpm_abs = 0.0f;
        } else if (age_us > (int32_t)ENCODER_RPM_FRESH_US) {
            // Carry the last good RPM forward by however much the model
            // says the motor has changed since — better than freezing it,
            // even with the placeholder model, since only the change is used.
            h.state   = SensorHealth::HOLDING;
            h.rpm_abs = fmaxf(0.0f, n.last_good_abs + (n.model_rpm - n.model_at_good));
        } else {
            h.state         = SensorHealth::GOOD;
            h.rpm_abs       = rpm_abs;
            n.last_good_abs = rpm_abs;
            n.model_at_good = n.model_rpm;
        }
    }
}

void encoder_rpm_init(void)
{
    for (int node = 0; node < ENCODER_RPM_NUM_NODES; node++) {
        bprl_can_register(CAN_BUS_1, ENCODER_RPM_CAN_ID_BASE + node, encoder_rpm_can_cb,
                          (void *)(intptr_t)node);
    }
}
