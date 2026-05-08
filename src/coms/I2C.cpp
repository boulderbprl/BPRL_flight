#include "src/coms/I2C.hpp"

struct I2CDevice {
    uint8_t     addr;
    I2CCallback poll;
    void       *ctx;
};
static I2CDevice i2c_table[MAX_I2C_DEVICES];
static int       num_i2c_devices = 0;

void bprl_i2c_register(uint8_t addr, I2CCallback poll_fn, void *ctx)
{
    if (num_i2c_devices < MAX_I2C_DEVICES) {
        i2c_table[num_i2c_devices++] = {addr, poll_fn, ctx};
    }
}

void i2c_poll_all(void)
{
    for (int i = 0; i < num_i2c_devices; i++) {
        i2c_table[i].poll(i2c_table[i].ctx);
    }
}

void i2c_drv_init(void)
{
    // TODO: i2cStart(&I2CD1, &i2c_cfg);
    // TODO: bprl_i2c_register(0x1E, compass_poll, nullptr);  // HMC5883
    // TODO: bprl_i2c_register(0x76, baro_poll,    nullptr);  // MS5611
}
