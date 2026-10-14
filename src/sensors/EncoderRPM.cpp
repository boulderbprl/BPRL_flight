#include "src/sensors/EncoderRPM.hpp"
#include "src/coms/CAN.hpp"
#include <cstdint>
#include <cstring>

static void encoder_rpm_can_cb(const CANRxFrame &f, void *ctx)
{
    const int node = (int)(intptr_t)ctx;

    float rpm;
    memcpy(&rpm, &f.data8[0], sizeof(rpm));
    uint16_t angle_raw  = (uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8);
    uint8_t  error_flag = f.data8[6];

    chMtxLock(&encoderRpm_mtx);
    g_encoder_rpm[node].rpm        = rpm;
    g_encoder_rpm[node].angle_raw  = angle_raw;
    g_encoder_rpm[node].error_flag = error_flag;
    g_encoder_rpm[node].valid      = true;
    chMtxUnlock(&encoderRpm_mtx);
}

void encoder_rpm_init(void)
{
    for (int node = 0; node < ENCODER_RPM_NUM_NODES; node++) {
        bprl_can_register(ENCODER_RPM_CAN_ID_BASE + node, encoder_rpm_can_cb,
                           (void *)(intptr_t)node);
    }
}
