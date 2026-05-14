#include "src/coms/CRSF.hpp"

CrsfParser g_crsf;

static const SerialConfig crsf_cfg = {
    420000,                   // 420 kbaud (ELRS standard; TBS spec is 416666)
    0,                        // 8 data bits, no parity
    USART_CR2_STOP1_BITS,     // 1 stop bit; uninverted
    0,
    nullptr, nullptr
};

/* DVB-S2 CRC-8, polynomial 0xD5 */
uint8_t CrsfParser::crc8_dvb_s2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0xD5u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* 16 × 11-bit little-endian channel extraction from 22-byte payload */
void CrsfParser::unpack(const uint8_t *data, uint16_t *ch)
{
    for (int n = 0; n < 16; n++) {
        const int bit = n * 11;
        const int bi  = bit / 8;
        const int sh  = bit % 8;
        const uint32_t w = (uint32_t)data[bi]
                         | ((uint32_t)data[bi + 1] << 8)
                         | ((uint32_t)data[bi + 2] << 16);
        ch[n] = (w >> sh) & 0x7FFu;
    }
}

void CrsfParser::init()
{
    sdStart(&SD3, &crsf_cfg);
}

void CrsfParser::update()
{
    uint8_t byte;
    while (chnReadTimeout(&SD3, &byte, 1, TIME_IMMEDIATE) == 1) {
        switch (_state) {
        case State::WAIT_SYNC:
            if (byte == 0xC8u) {
                _buf[0] = byte;
                _count  = 1;
                _state  = State::WAIT_LEN;
            }
            break;

        case State::WAIT_LEN: {
            /* len = bytes after the length field (type + payload + crc)
             * frame_len = sync(1) + len_byte(1) + (type + payload + crc)(len) = len + 2 */
            const uint8_t frame_len = byte + 2u;
            if (frame_len < 4u || frame_len > 64u) {
                _state = State::WAIT_SYNC; // reject implausible lengths
                break;
            }
            _buf[1]     = byte;
            _count      = 2;
            _frame_len  = frame_len;
            _state      = State::IN_FRAME;
            break;
        }

        case State::IN_FRAME:
            _buf[_count++] = byte;
            if (_count == _frame_len) {
                _state = State::WAIT_SYNC;
                /* CRC covers buf[2..frame_len-2] = type + payload (excludes crc byte itself) */
                const uint8_t crc_len = _frame_len - 3u; // skip sync, len, crc
                const uint8_t crc_calc = crc8_dvb_s2(&_buf[2], crc_len);
                const uint8_t crc_recv = _buf[_frame_len - 1u];
                if (crc_calc == crc_recv && _buf[2] == 0x16u) {
                    /* type 0x16 = RC_CHANNELS_PACKED; payload starts at buf[3] */
                    unpack(&_buf[3], _ch);
                    _valid = true;
                }
                _count = 0;
            }
            break;
        }
    }
}

uint16_t CrsfParser::channel(uint8_t n) const
{
    return (n < 16) ? _ch[n] : 992u;
}
