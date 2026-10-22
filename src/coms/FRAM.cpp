#include "src/coms/FRAM.hpp"

#if defined(BPRL_HAS_FRAM)
#include <cstring>

// ── Op-codes (AP_RAMTRON.cpp) ──────────────────────────────────────────────
static constexpr uint8_t OP_RDID  = 0x9F;
static constexpr uint8_t OP_WREN  = 0x06;
static constexpr uint8_t OP_READ  = 0x03;
static constexpr uint8_t OP_WRITE = 0x02;

// AP_RAMTRON's RAMTRON_RETRIES / RAMTRON_DELAY_MS for the double-read check.
static constexpr int      READ_RETRIES  = 10;
static constexpr uint32_t READ_RETRY_MS = 10;

// SPI2 kernel clock = PLL1_Q = 50 MHz on the Cubes (cfg/mcuconf.h, same mux
// as SPI1). DIV8 -> 6.25 MHz, at or under ArduPilot's 8 MHz for this part.
static const SPIConfig fram_cfg = {
    false, nullptr, GPIOD, 10U,
    SPI_CFG1_MBR_DIV8 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,   // MODE3
    nullptr, nullptr
};

enum class RdidType : uint8_t { Cypress, Fujitsu, Petabytes };

struct RamtronId {
    uint8_t     id1;
    uint8_t     id2;
    uint16_t    size_kbyte;
    uint8_t     addrlen;   // 2 or 3 address bytes
    RdidType    type;
    const char *name;
};

// AP_RAMTRON::ramtron_ids, complete.
static const RamtronId kRamtronIds[] = {
    { 0x21, 0x00,  16, 2, RdidType::Cypress,   "FM25V01"     },
    { 0x21, 0x08,  16, 2, RdidType::Cypress,   "FM25V01A"    },
    { 0x22, 0x00,  32, 2, RdidType::Cypress,   "FM25V02"     },
    { 0x22, 0x08,  32, 2, RdidType::Cypress,   "FM25V02A"    },
    { 0x22, 0x48,  32, 2, RdidType::Cypress,   "FM25V02A-X"  },
    { 0x22, 0x01,  32, 2, RdidType::Cypress,   "FM25VN02"    },
    { 0x23, 0x00,  64, 2, RdidType::Cypress,   "FM25V05"     },
    { 0x23, 0x01,  64, 2, RdidType::Cypress,   "FM25VN05"    },
    { 0x24, 0x00, 128, 3, RdidType::Cypress,   "FM25V10"     },
    { 0x24, 0x01, 128, 3, RdidType::Cypress,   "FM25VN10"    },
    { 0x25, 0x08, 256, 3, RdidType::Cypress,   "FM25V20A"    },
    { 0x26, 0x08, 512, 3, RdidType::Cypress,   "CY15B104Q"   },
    { 0x27, 0x03, 128, 3, RdidType::Fujitsu,   "MB85RS1MT"   },
    { 0x05, 0x09,  32, 2, RdidType::Fujitsu,   "MB85RS256B"  },
    { 0x24, 0x03,  16, 2, RdidType::Fujitsu,   "MB85RS128TY" },
    { 0x25, 0x03,  32, 2, RdidType::Fujitsu,   "MB85RS256TY" },
    { 0x22, 0x00,  16, 2, RdidType::Petabytes, "PB85RS128C"  },
    { 0x24, 0x00, 256, 3, RdidType::Petabytes, "PB85RS2MC"   },
};
static constexpr uint8_t kNumIds = sizeof(kRamtronIds) / sizeof(kRamtronIds[0]);

static uint8_t s_id    = 0xFF;   // index into kRamtronIds, 0xFF = not identified
static bool    s_ready = false;

// DMA-reachable (.bss is AXI SRAM on the Cubes), 32-byte-aligned scratch —
// same H7 D-cache handling as the IMU drivers' _txbuf/_rxbuf. Every
// transfer buffer lives here, never on a stack (main()'s stack is DTCM,
// which DMA1/DMA2 can't reach). Only touched while holding the SPI2 bus.
static constexpr size_t kMaxPayload = 128;
static constexpr size_t kBufSize    = 4 + kMaxPayload;   // opcode + 3-byte addr + payload
static __attribute__((aligned(32))) uint8_t s_txbuf[kBufSize];
static __attribute__((aligned(32))) uint8_t s_rxbuf[kBufSize];

static uint8_t header_len()
{
    return kRamtronIds[s_id].addrlen + 1U;   // + 1 opcode byte
}

static void build_header(uint8_t opcode, uint32_t offset)
{
    s_txbuf[0] = opcode;
    if (kRamtronIds[s_id].addrlen == 3) {
        s_txbuf[1] = (uint8_t)((offset >> 16) & 0xFF);
        s_txbuf[2] = (uint8_t)((offset >> 8)  & 0xFF);
        s_txbuf[3] = (uint8_t)(offset & 0xFF);
    } else {
        s_txbuf[1] = (uint8_t)((offset >> 8) & 0xFF);
        s_txbuf[2] = (uint8_t)(offset & 0xFF);
    }
}

// One full-duplex transaction of n bytes from s_txbuf into s_rxbuf. Caller
// holds the bus.
static void exchange(size_t n)
{
    cacheBufferFlush(s_txbuf, kBufSize);
    spiSelect(&SPID2);
    spiExchange(&SPID2, n, s_txbuf, s_rxbuf);
    spiUnselect(&SPID2);
    cacheBufferInvalidate(s_rxbuf, kBufSize);
}

bool fram_drv_init(void)
{
    s_ready = false;
    s_id    = 0xFF;

    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &fram_cfg);
    memset(s_txbuf, 0xFF, kBufSize);
    s_txbuf[0] = OP_RDID;
    exchange(10);   // opcode + 9 reply bytes (Cypress RDID length)
    spiReleaseBus(&SPID2);

    // s_rxbuf[0] clocks in during the opcode (don't-care); the reply starts
    // at [1]. Cypress: 6 manufacturer + 1 memory + id1 + id2. Fujitsu /
    // Petabytes: 2 manufacturer + id1 + id2 — same decode as AP_RAMTRON::init().
    const uint8_t *r = s_rxbuf + 1;
    for (uint8_t i = 0; i < kNumIds; i++) {
        const RamtronId &c = kRamtronIds[i];
        bool match = false;
        switch (c.type) {
        case RdidType::Cypress:   match = r[7] == c.id1 && r[8] == c.id2; break;
        case RdidType::Fujitsu:   match = r[2] == c.id1 && r[3] == c.id2; break;
        case RdidType::Petabytes: match = r[0] == 0x62 && r[1] == 0x8C
                                       && r[2] == c.id1 && r[3] == c.id2; break;
        }
        if (match) { s_id = i; break; }
    }

    s_ready = (s_id != 0xFF);
    return s_ready;
}

bool fram_ready(void) { return s_ready; }

uint32_t fram_size(void)
{
    return s_ready ? (uint32_t)kRamtronIds[s_id].size_kbyte * 1024UL : 0U;
}

const char *fram_part_name(void)
{
    return s_ready ? kRamtronIds[s_id].name : "none";
}

// Read twice and only accept matching data, retrying with a delay — as
// AP_RAMTRON::read() does, "to prevent transient errors from causing
// parameter corruption".
bool fram_read(uint32_t offset, uint8_t *buf, uint32_t size)
{
    if (!s_ready || size == 0 || size > kMaxPayload) { return false; }
    if (offset > fram_size() - size) { return false; }   // part wraps past its end

    const uint8_t hlen = header_len();
    for (int attempt = 0; attempt < READ_RETRIES; attempt++) {
        if (attempt != 0) { chThdSleepMilliseconds(READ_RETRY_MS); }

        spiAcquireBus(&SPID2);
        spiStart(&SPID2, &fram_cfg);
        build_header(OP_READ, offset);
        memset(s_txbuf + hlen, 0xFF, size);
        exchange(hlen + size);
        memcpy(buf, s_rxbuf + hlen, size);
        exchange(hlen + size);   // same header/payload still in s_txbuf
        const bool same = memcmp(buf, s_rxbuf + hlen, size) == 0;
        spiReleaseBus(&SPID2);

        if (same) { return true; }
    }
    return false;
}

bool fram_write(uint32_t offset, const uint8_t *buf, uint32_t size)
{
    if (!s_ready || size == 0 || size > kMaxPayload) { return false; }
    if (offset > fram_size() - size) { return false; }

    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &fram_cfg);

    // WREN before every WRITE — the latch auto-clears after each write.
    s_txbuf[0] = OP_WREN;
    exchange(1);

    const uint8_t hlen = header_len();
    build_header(OP_WRITE, offset);
    memcpy(s_txbuf + hlen, buf, size);
    exchange(hlen + size);

    spiReleaseBus(&SPID2);
    return true;   // callers verify with fram_read()
}

#endif // BPRL_HAS_FRAM
