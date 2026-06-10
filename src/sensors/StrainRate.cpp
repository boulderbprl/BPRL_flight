#include "src/sensors/StrainRate.hpp"
#include "src/coms/I2C.hpp"
#include "src/coms/CAN.hpp"

// ── CAN implementation ────────────────────────────────────────────────────────

#if STRAIN_RATE_INTERFACE == STRAIN_RATE_CAN

static void strain_rate_can_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    chMtxLock(&strainRate_mtx);
    g_strain_rate.val[0] = (int16_t)((uint16_t)f.data8[0] | ((uint16_t)f.data8[1] << 8));
    g_strain_rate.val[1] = (int16_t)((uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8));
    g_strain_rate.val[2] = (int16_t)((uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8));
    g_strain_rate.val[3] = (int16_t)((uint16_t)f.data8[6] | ((uint16_t)f.data8[7] << 8));
    g_strain_rate.valid  = true;
    chMtxUnlock(&strainRate_mtx);
}

void strain_rate_init(void)
{
    bprl_can_register(0x69, strain_rate_can_cb, nullptr);
}

// ── I2C implementation ────────────────────────────────────────────────────────

#elif STRAIN_RATE_INTERFACE == STRAIN_RATE_I2C

static void strain_rate_i2c_poll(void *ctx)
{
    (void)ctx;
    uint8_t buf[8];
    msg_t status = i2cMasterReceiveTimeout(&I2CD2, 0x11, buf, sizeof(buf), TIME_MS2I(5));

    if (status != MSG_OK) {
        if (status == MSG_TIMEOUT) i2c_drv_reset();
        return;
    }

    chMtxLock(&strainRate_mtx);
    g_strain_rate.val[0] = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    g_strain_rate.val[1] = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    g_strain_rate.val[2] = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    g_strain_rate.val[3] = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
    g_strain_rate.valid  = true;
    chMtxUnlock(&strainRate_mtx);
}

void strain_rate_init(void)
{
    bprl_i2c_register(0x11, strain_rate_i2c_poll, nullptr);
}

#endif
