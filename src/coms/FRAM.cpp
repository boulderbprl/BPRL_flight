#include "src/coms/FRAM.hpp"
#include <cstring>

// ── Op-codes (same across the whole Cypress/Fujitsu FM25 family) ───────────
static constexpr uint8_t OP_RDID  = 0x9F;
static constexpr uint8_t OP_WREN  = 0x06;
static constexpr uint8_t OP_READ  = 0x03;
static constexpr uint8_t OP_WRITE = 0x02;

// SPI2 = PLL1_Q = 50 MHz, same clock mux as SPI1 (STM32_SPI123SEL). DIV8 ->
// 6.25 MHz, under the 8 MHz ArduPilot runs this part at (hwdef.inc: "SPIDEV
// ramtron SPI2 DEVID10 FRAM_CS MODE3 8*MHZ 8*MHZ") and comfortably under
// every FM25 part's real maximum (typically 20-40 MHz).
static const SPIConfig fram_cfg = {
    false, nullptr, GPIOD, 10U,
    SPI_CFG1_MBR_DIV8 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,   // MODE3
    nullptr, nullptr
};

enum class RdidType : uint8_t { Cypress, Fujitsu };

struct RamtronId {
    uint8_t  id1;
    uint8_t  id2;
    uint16_t size_kbyte;
    uint8_t  addrlen;   // 2 or 3 address bytes
    RdidType type;
};

// Subset of ArduPilot's AP_RAMTRON::ramtron_ids covering the parts actually
// seen on Cube-family carrier boards, plus the wider Cypress family for
// margin. Extend from AP_RAMTRON.cpp's full table if a board turns up with
// something not listed here — RDID identifies the exact part, so an
// unlisted one just fails init() safely rather than mis-sizing itself.
static const RamtronId kRamtronIds[] = {
    { 0x21, 0x00,  16, 2, RdidType::Cypress }, // FM25V01
    { 0x21, 0x08,  16, 2, RdidType::Cypress }, // FM25V01A
    { 0x22, 0x00,  32, 2, RdidType::Cypress }, // FM25V02
    { 0x22, 0x08,  32, 2, RdidType::Cypress }, // FM25V02A
    { 0x22, 0x48,  32, 2, RdidType::Cypress }, // FM25V02A (extended temp)
    { 0x22, 0x01,  32, 2, RdidType::Cypress }, // FM25VN02
    { 0x23, 0x00,  64, 2, RdidType::Cypress }, // FM25V05
    { 0x23, 0x01,  64, 2, RdidType::Cypress }, // FM25VN05
    { 0x24, 0x00, 128, 3, RdidType::Cypress }, // FM25V10
    { 0x24, 0x01, 128, 3, RdidType::Cypress }, // FM25VN10
    { 0x25, 0x08, 256, 3, RdidType::Cypress }, // FM25V20A
    { 0x26, 0x08, 512, 3, RdidType::Cypress }, // CY15B104Q
    { 0x27, 0x03, 128, 3, RdidType::Fujitsu }, // MB85RS1MT
    { 0x05, 0x09,  32, 2, RdidType::Fujitsu }, // MB85RS256B
    { 0x24, 0x03,  16, 2, RdidType::Fujitsu }, // MB85RS128TY
    { 0x25, 0x03,  32, 2, RdidType::Fujitsu }, // MB85RS256TY
};

static uint8_t s_id     = 0xFF;   // index into kRamtronIds, 0xFF = not identified
static bool    s_ready  = false;

// DMA-safe, 32-byte-aligned scratch buffer — same H7 D-cache-coherency
// rationale as ICM20602.cpp's _txbuf/_rxbuf (cacheBufferFlush/Invalidate
// around every SPI DMA transfer). Sized for one opcode+3-byte-address
// header (4 B) plus up to a 128-byte payload (CalFlash's CalibData is
// exactly 128 B, the only real caller today).
static constexpr size_t kMaxPayload = 128;
static constexpr size_t kBufSize    = 4 + kMaxPayload;
static __attribute__((aligned(32))) uint8_t s_txbuf[kBufSize];
static __attribute__((aligned(32))) uint8_t s_rxbuf[kBufSize];

static uint8_t header_len()
{
    return kRamtronIds[s_id].addrlen + 1U;   // + 1 opcode byte
}

static void build_header(uint8_t *hdr, uint8_t opcode, uint32_t offset)
{
    hdr[0] = opcode;
    if (kRamtronIds[s_id].addrlen == 3) {
        hdr[1] = (uint8_t)((offset >> 16) & 0xFF);
        hdr[2] = (uint8_t)((offset >> 8)  & 0xFF);
        hdr[3] = (uint8_t)(offset & 0xFF);
    } else {
        hdr[1] = (uint8_t)((offset >> 8) & 0xFF);
        hdr[2] = (uint8_t)(offset & 0xFF);
    }
}

bool fram_drv_init(void)
{
    s_ready = false;
    s_id    = 0xFF;

    s_txbuf[0] = OP_RDID;
    memset(s_txbuf + 1, 0xFF, 9);
    cacheBufferFlush(s_txbuf, kBufSize);
    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &fram_cfg);
    spiSelect(&SPID2);
    spiExchange(&SPID2, 10, s_txbuf, s_rxbuf);
    spiUnselect(&SPID2);
    spiReleaseBus(&SPID2);
    cacheBufferInvalidate(s_rxbuf, kBufSize);

    // s_rxbuf[0] is the echoed opcode byte (don't-care); the real reply
    // starts at s_rxbuf[1]. Cypress: 6 manufacturer + 1 memory-type + id1 +
    // id2 = 9 bytes. Fujitsu: 2 manufacturer + id1 + id2 = 4 bytes — we
    // only compare the bytes each type actually defines.
    const uint8_t *reply = s_rxbuf + 1;
    for (uint8_t i = 0; i < (uint8_t)(sizeof(kRamtronIds) / sizeof(kRamtronIds[0])); i++) {
        const RamtronId &cand = kRamtronIds[i];
        if (cand.type == RdidType::Cypress) {
            if (reply[7] == cand.id1 && reply[8] == cand.id2) { s_id = i; break; }
        } else { // Fujitsu
            if (reply[2] == cand.id1 && reply[3] == cand.id2) { s_id = i; break; }
        }
    }

    if (s_id == 0xFF) { return false; }

    s_ready = true;
    return true;
}

bool fram_ready(void) { return s_ready; }

uint32_t fram_size(void)
{
    return s_ready ? (uint32_t)kRamtronIds[s_id].size_kbyte * 1024UL : 0U;
}

bool fram_read(uint32_t offset, uint8_t *buf, uint32_t size)
{
    if (!s_ready || size == 0 || size > kMaxPayload) { return false; }
    if (offset > fram_size() - size) { return false; }

    const uint8_t hlen = header_len();
    build_header(s_txbuf, OP_READ, offset);
    memset(s_txbuf + hlen, 0xFF, size);
    cacheBufferFlush(s_txbuf, kBufSize);

    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &fram_cfg);
    spiSelect(&SPID2);
    spiExchange(&SPID2, hlen + size, s_txbuf, s_rxbuf);
    spiUnselect(&SPID2);
    spiReleaseBus(&SPID2);
    cacheBufferInvalidate(s_rxbuf, kBufSize);

    memcpy(buf, s_rxbuf + hlen, size);
    return true;
}

bool fram_write(uint32_t offset, const uint8_t *buf, uint32_t size)
{
    if (!s_ready || size == 0 || size > kMaxPayload) { return false; }
    if (offset > fram_size() - size) { return false; }

    spiAcquireBus(&SPID2);
    spiStart(&SPID2, &fram_cfg);

    // WREN must precede every WRITE — the write-enable latch auto-clears
    // once the write completes, so this can't be done once at init().
    uint8_t wren = OP_WREN;
    cacheBufferFlush(&wren, sizeof(wren));
    spiSelect(&SPID2);
    spiSend(&SPID2, 1, &wren);
    spiUnselect(&SPID2);

    const uint8_t hlen = header_len();
    build_header(s_txbuf, OP_WRITE, offset);
    memcpy(s_txbuf + hlen, buf, size);
    cacheBufferFlush(s_txbuf, kBufSize);

    spiSelect(&SPID2);
    spiSend(&SPID2, hlen + size, s_txbuf);
    spiUnselect(&SPID2);
    spiReleaseBus(&SPID2);

    return true;
}
