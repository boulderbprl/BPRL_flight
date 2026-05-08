#pragma once
#include "ff.h"
#include <cstdint>
#include <cstddef>

/*
 * Logger — ring-buffered binary SD card logger.
 *
 * All public methods are called exclusively from LogThread (NORMALPRIO-15),
 * so no mutex protection is needed.
 *
 * Log file format:
 *   • Header: N×FMT records (see LOG_MSG_FMT in LogMessages.hpp)
 *   • Body:   sequential  [0xA3][0x95][msg_id][packed struct body]  records
 *
 * Usage:
 *   logger.init();           // mount SD, open LOGxxxx.BIN, write FMT header
 *   logger.write(id, msg);   // append to ring buffer (non-blocking)
 *   logger.flush();          // drain ring buffer → SD (call periodically)
 *   logger.close();          // sync and unmount
 */
class Logger {
public:
    bool init();
    bool is_ready() const;

    template<typename T>
    bool write(uint8_t msg_id, const T &msg)
    {
        if (!_open) return false;
        const uint8_t hdr[3] = { 0xA3U, 0x95U, msg_id };
        if (!ring_write(hdr, 3)) return false;
        return ring_write(&msg, sizeof(T));
    }

    void flush();
    void close();

private:
    static constexpr size_t   BUF_SIZE      = 32768U;  // ring buffer (AXI SRAM)
    static constexpr uint32_t SYNC_INTERVAL = 100U;    // f_sync every N flushes

    /* Ring buffer — in normal cached AXI SRAM, never DMA'd directly. */
    uint8_t  _buf[BUF_SIZE];
    size_t   _head        = 0;
    size_t   _tail        = 0;
    bool     _open        = false;
    uint32_t _sync_count  = 0;

    bool   ring_write(const void *data, size_t n);
    size_t ring_read(void *out, size_t max_n);
    bool   write_schema_header();
    void   find_next_log_index(char *path, size_t path_size);
};

extern Logger logger;
