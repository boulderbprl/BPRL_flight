#pragma once
#include "hal.h"
#include <cstdint>

/*
 * CRSF (Crossfire / ELRS) RC input parser — USART2 (TELEM1 port), PD6 RX / PD5 TX, AF7
 *
 * Frame: variable length ≤ 64 bytes
 *   [sync=0xC8][len][type][payload...][CRC8]
 *   len = bytes following len field (type + payload + CRC)
 *   total frame bytes = len + 2  (sync + len + payload)
 *
 * Serial: 420000 baud, 8N1, uninverted.
 * CRC: DVB-S2 CRC-8 (poly 0xD5) over type + payload (not sync/len/crc).
 *
 * RC frame type 0x16: 22-byte payload, 16 channels × 11-bit packed (172–1811, centre 992).
 */
class CrsfParser {
public:
    void     init();
    void     update();                  // drain SD3, run state machine; call at ~100 Hz
    uint16_t channel(uint8_t n) const; // raw 11-bit value for channel n (0–15)
    bool     data_valid() const { return _valid; }

private:
    static uint8_t crc8_dvb_s2(const uint8_t *data, uint8_t len);
    static void    unpack(const uint8_t *payload, uint16_t *ch);

    enum class State : uint8_t { WAIT_SYNC, WAIT_LEN, IN_FRAME } _state{State::WAIT_SYNC};
    uint8_t  _buf[64]{};
    uint8_t  _frame_len{0}; // total expected bytes in frame (sync + len_byte + payload + crc)
    uint8_t  _count{0};
    uint16_t _ch[16]{};
    bool     _valid{false};
};

extern CrsfParser g_crsf;
