#include "src/coms/SBUS.hpp"

SbusParser g_sbus;

static const SerialConfig sbus_cfg = {
    100000,                                   // 100 kbaud
    USART_CR1_M | USART_CR1_PCE,             // 9-bit frame: 8 data + even parity (PS=0)
    USART_CR2_STOP2_BITS,                     // 2 stop bits; no RXINV — Cube HW inverter on board
    0,
    nullptr, nullptr
};

void SbusParser::init()
{
    sdStart(&SD6, &sbus_cfg);
}

void SbusParser::update()
{
    uint8_t byte;
    while (chnReadTimeout(&SD6, &byte, 1, TIME_IMMEDIATE) == 1) {
        switch (_state) {
        case State::WAIT_START:
            if (byte == 0x0F) {
                _buf[0] = byte;
                _count  = 1;
                _state  = State::IN_FRAME;
            }
            break;

        case State::IN_FRAME:
            _buf[_count++] = byte;
            if (_count == 25) {
                _state = State::WAIT_START;
                if (_buf[24] == 0x00) {
                    unpack(&_buf[1], _ch);
                    _frame_lost = (_buf[23] >> 2) & 0x01;
                    _failsafe   = (_buf[23] >> 3) & 0x01;
                }
                _count = 0;
            }
            break;
        }
    }
}

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
