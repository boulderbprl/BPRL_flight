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

/*
 * DMA RX buffer for the strain rate sensor.
 *
 * The ChibiOS I2Cv2 driver does no D-cache maintenance around DMA transfers.
 * Thread stacks are in cacheable AXI SRAM, so a stack-allocated buf[] would
 * be stale in cache after the DMA writes it — producing corrupt readings,
 * especially on the last bytes of the frame.  Placing this in .nocache puts
 * it in the MPU-marked non-cacheable AXI window (0x2407C000) so the CPU
 * always reads what DMA wrote.
 */
static __attribute__((section(".nocache"))) uint8_t s_i2c_rx[8];

static void strain_rate_i2c_poll(void *ctx)
{
    (void)ctx;

    i2cAcquireBus(&I2CD2);
    msg_t status = i2cMasterReceiveTimeout(&I2CD2, 0x11, s_i2c_rx, sizeof(s_i2c_rx), TIME_US2I(1500));
    i2cReleaseBus(&I2CD2);

    if (status != MSG_OK) {
        // MSG_RESET means the driver hit an error and is now I2C_LOCKED.
        // Without resetting here it stays locked forever (MSG_TIMEOUT never fires).
        if (status == MSG_TIMEOUT || status == MSG_RESET) i2c_drv_reset();
        return;
    }

    chMtxLock(&strainRate_mtx);
    g_strain_rate.val[0] = (int16_t)((uint16_t)s_i2c_rx[0] | ((uint16_t)s_i2c_rx[1] << 8));
    g_strain_rate.val[1] = (int16_t)((uint16_t)s_i2c_rx[2] | ((uint16_t)s_i2c_rx[3] << 8));
    g_strain_rate.val[2] = (int16_t)((uint16_t)s_i2c_rx[4] | ((uint16_t)s_i2c_rx[5] << 8));
    g_strain_rate.val[3] = (int16_t)((uint16_t)s_i2c_rx[6] | ((uint16_t)s_i2c_rx[7] << 8));
    g_strain_rate.valid  = true;
    chMtxUnlock(&strainRate_mtx);
}

void strain_rate_init(void)
{
    bprl_i2c_register(0x11, strain_rate_i2c_poll, nullptr);
}

#endif
