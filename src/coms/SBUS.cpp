#include "src/coms/SBUS.hpp"

SbusParser g_sbus;

/* SBUS disabled — PC7/USART6 is the FMU↔IOMCU bridge.
 * RC input is CRSF on TELEM1 (USART2). All methods are stubs. */

void SbusParser::init() {}

void SbusParser::update() {}

uint16_t SbusParser::channel(uint8_t n) const
{
    return (n < 16) ? _ch[n] : 992u;
}

void SbusParser::unpack(const uint8_t *data, uint16_t *ch)
{
    for (int n = 0; n < 16; n++) {
        const int bit  = n * 11;
        const int bi   = bit / 8;
        const int sh   = bit % 8;
        const uint32_t w = (uint32_t)data[bi]
                         | ((uint32_t)data[bi + 1] << 8)
                         | ((uint32_t)data[bi + 2] << 16);
        ch[n] = (w >> sh) & 0x7FFu;
    }
}
