#pragma once
#include "hal.h"
#include <cstdint>

/*
 * CRSF (Crossfire / ELRS) RC input parser.
 * Cube boards: USART2 (TELEM1 port), PD6 RX / PD5 TX, AF7, full-duplex.
 * BPRL_BOARD_ORQA: USART6, PC6 TX6 / PC7 RX6, AF7, full-duplex (see
 * boards/OrqaH7QuadCore/board.h) — not this board's hwdef-default RC pad
 * (that's half-duplex USART3/T3, default protocol GHST); moved to TX6/RX6
 * for simpler two-wire wiring and to leave USART3 free for MAVLink
 * (src/coms/MAVLink.cpp is unconditionally SD3 on every board). Note the
 * receiver must actually be set to CRSF output — this parser doesn't speak
 * GHST, which is a different framing despite similar wiring.
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
    void     update();                  // drain the RC UART (SD2 Cube boards / SD6 BPRL_BOARD_ORQA), run state machine; call at ~100 Hz
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
