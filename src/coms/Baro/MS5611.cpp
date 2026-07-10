#include "src/coms/Baro/MS5611.hpp"
#include <cmath>
#include <cstring>

// MS5611 commands
static constexpr uint8_t CMD_RESET      = 0x1E;
static constexpr uint8_t CMD_CONVERT_D1 = 0x44;  // pressure,    OSR=1024
static constexpr uint8_t CMD_CONVERT_D2 = 0x54;  // temperature, OSR=1024
static constexpr uint8_t CMD_ADC_READ   = 0x00;
static constexpr uint8_t CMD_PROM_BASE  = 0xA0;  // + 2*addr, addr=1..6

// Datasheet: reset needs >=2.8ms, OSR=1024 conversion needs ~2.28ms — both
// rounded up with margin.
static constexpr systime_t RESET_WAIT_MS    = 3;
static constexpr systime_t CONVERT_WAIT_MS  = 3;

// Reject compensated pressure outside a plausible range (~30-110 kPa covers
// sea level to well above any altitude this vehicle will fly, plus margin
// for a corrupted ADC read) before it ever reaches read()'s caller.
static constexpr float PRESSURE_MIN_PA = 30000.0f;
static constexpr float PRESSURE_MAX_PA = 110000.0f;

bool MS5611::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _ready    = false;

    _cmd(CMD_RESET);
    chThdSleepMilliseconds(RESET_WAIT_MS);

    for (int i = 0; i < 6; ++i) {
        uint8_t buf[2];
        _read_bytes(static_cast<uint8_t>(CMD_PROM_BASE + 2 * (i + 1)), buf, 2);
        _prom[i] = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }

    // Sanity check: all-zero or all-ones PROM means nothing answered on the bus.
    uint16_t prom_or = 0, prom_and = 0xFFFF;
    for (int i = 0; i < 6; ++i) { prom_or |= _prom[i]; prom_and &= _prom[i]; }
    if (prom_or == 0 || prom_and == 0xFFFF) { return false; }

    _state       = State::CONVERT_D1;
    _ref_captured = false;
    _warmup_count = 0;
    _warmup_sum_m = 0.0f;
    _ready        = true;
    return true;
}

bool MS5611::read(float &pressure_pa, float &temperature_c, float &alt_m)
{
    if (!_ready) { return false; }

    const systime_t now = chVTGetSystemTime();

    switch (_state) {
    case State::RESET:
    case State::PROM_READ:
        // Handled synchronously in init(); not reachable from read().
        _state = State::CONVERT_D1;
        return false;

    case State::CONVERT_D1:
        _cmd(CMD_CONVERT_D1);
        _wait_start = now;
        _state      = State::WAIT_D1;
        return false;

    case State::WAIT_D1:
        if (chVTTimeElapsedSinceX(_wait_start) < TIME_MS2I(CONVERT_WAIT_MS)) { return false; }
        _state = State::READ_D1;
        [[fallthrough]];

    case State::READ_D1: {
        uint8_t buf[3];
        _read_bytes(CMD_ADC_READ, buf, 3);
        _d1_raw = (static_cast<uint32_t>(buf[0]) << 16)
                | (static_cast<uint32_t>(buf[1]) << 8)
                |  static_cast<uint32_t>(buf[2]);
        _state = State::CONVERT_D2;
        return false;
    }

    case State::CONVERT_D2:
        _cmd(CMD_CONVERT_D2);
        _wait_start = now;
        _state      = State::WAIT_D2;
        return false;

    case State::WAIT_D2:
        if (chVTTimeElapsedSinceX(_wait_start) < TIME_MS2I(CONVERT_WAIT_MS)) { return false; }
        _state = State::READ_D2;
        [[fallthrough]];

    case State::READ_D2: {
        uint8_t buf[3];
        _read_bytes(CMD_ADC_READ, buf, 3);
        _d2_raw = (static_cast<uint32_t>(buf[0]) << 16)
                | (static_cast<uint32_t>(buf[1]) << 8)
                |  static_cast<uint32_t>(buf[2]);
        _state = State::COMPUTE;
        [[fallthrough]];
    }

    case State::COMPUTE: {
        _state = State::CONVERT_D1;   // always advance, whether or not this sample is used

        float p_pa, t_c;
        _compensate(p_pa, t_c);
        if (p_pa < PRESSURE_MIN_PA || p_pa > PRESSURE_MAX_PA) { return false; }  // corrupted ADC read

        const float alt_raw_m = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1903f));

        if (!_ref_captured) {
            _warmup_sum_m += alt_raw_m;
            if (++_warmup_count >= WARMUP_SAMPLES) {
                _alt_ref_m     = _warmup_sum_m / static_cast<float>(WARMUP_SAMPLES);
                _ref_captured  = true;
            }
            return false;   // don't report samples until the zero-reference is set
        }

        pressure_pa   = p_pa;
        temperature_c = t_c;
        alt_m         = alt_raw_m - _alt_ref_m;
        return true;
    }
    }

    return false;
}

void MS5611::_compensate(float &pressure_pa, float &temperature_c)
{
    // MS5611 datasheet second-order compensation, int64 intermediate math.
    const int64_t C1 = _prom[0], C2 = _prom[1], C3 = _prom[2];
    const int64_t C4 = _prom[3], C5 = _prom[4], C6 = _prom[5];
    const int64_t D1 = _d1_raw,  D2 = _d2_raw;

    const int64_t dT   = D2 - (C5 << 8);
    int64_t temp        = 2000 + ((dT * C6) >> 23);
    int64_t off          = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t sens         = (C1 << 15) + ((C3 * dT) >> 8);

    // Second-order compensation for low temperature (per datasheet).
    if (temp < 2000) {
        const int64_t t2      = (dT * dT) >> 31;
        int64_t       off2    = 5 * (temp - 2000) * (temp - 2000) / 2;
        int64_t       sens2   = 5 * (temp - 2000) * (temp - 2000) / 4;
        if (temp < -1500) {
            const int64_t adj = (temp + 1500) * (temp + 1500);
            off2  += 7 * adj;
            sens2 += 11 * adj / 2;
        }
        temp -= t2;
        off  -= off2;
        sens -= sens2;
    }

    const int64_t p = ((D1 * sens) >> 21) - off;

    // P is in units of 0.01 mbar == 1 Pa numerically (0.01 mbar * 100 Pa/mbar).
    pressure_pa   = static_cast<float>(p >> 15);
    temperature_c = static_cast<float>(temp) * 0.01f;
}

void MS5611::_cmd(uint8_t cmd)
{
    _txbuf[0] = cmd;
    cacheBufferFlush(_txbuf, 8);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiSend(_spid, 1, _txbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
}

void MS5611::_read_bytes(uint8_t cmd, uint8_t *buf, size_t n)
{
    _txbuf[0] = cmd;
    memset(_txbuf + 1, 0xFF, n);
    cacheBufferFlush(_txbuf, 8);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_fast);
    spiSelect(_spid);
    spiExchange(_spid, n + 1, _txbuf, _rxbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    cacheBufferInvalidate(_rxbuf, 8);
    memcpy(buf, _rxbuf + 1, n);
}
