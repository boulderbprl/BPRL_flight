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
    const char* current_path() const { return _open ? _current_path : nullptr; }
    uint8_t last_init_err() const { return _last_init_err; }
    uint8_t last_ff_err()   const { return _last_ff_err; }
    uint8_t expand_err()    const { return _expand_err; }
    // Error codes: 0=not tried, 1=sdcStart, 2=sdcConnect, 3=f_mount, 4=f_open, 5=ok
    // last_ff_err: FatFS FRESULT (FR_DISK_ERR=1, FR_NO_FILESYSTEM=13, FR_TIMEOUT=15, ...)
    // expand_err: FatFS FRESULT from the f_expand() pre-allocation call in
    // init() (see PRE_ALLOC_SIZE below). 0=FR_OK, i.e. pre-allocation actually
    // succeeded this boot. Nonzero (commonly FR_DENIED=7, no contiguous run of
    // that size was free) means logging still works, but has silently fallen
    // back to FatFS's normal incremental-growth writes — the exact stall
    // pattern the pre-allocation exists to avoid. 0xFF = f_expand not called
    // yet (init() hasn't reached that point, e.g. mount/open itself failed).

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
    static constexpr uint32_t SYNC_INTERVAL = 5U;      // f_sync every N flushes (~100 ms at 50 Hz)
    // Contiguous pre-allocation size for a new log file (~7 min at the current
    // ~23.7 KB/s logging rate). Reserving this up front in one f_expand() call
    // at init() means f_write() never needs to grow the FAT chain mid-flight —
    // that incremental growth (plus f_sync()'s directory/FAT write) is what was
    // causing LogThread's periodic multi-ms-to-hundreds-of-ms stalls. Trimmed
    // back to the actual bytes written by f_truncate() in close().
    static constexpr FSIZE_t PRE_ALLOC_SIZE = 10UL * 1024UL * 1024UL;

    /* Ring buffer — in normal cached AXI SRAM, never DMA'd directly. */
    uint8_t  _buf[BUF_SIZE];
    size_t   _head        = 0;
    size_t   _tail        = 0;
    bool     _open          = false;
    uint8_t  _last_init_err = 0;
    uint8_t  _last_ff_err   = 0;
    uint8_t  _expand_err    = 0xFFU;
    uint32_t _sync_count    = 0;
    char     _current_path[32] = {};

    bool   ring_write(const void *data, size_t n);
    size_t ring_read(void *out, size_t max_n);
    bool   write_schema_header();
    void   find_next_log_index(char *path, size_t path_size);
};

extern Logger logger;
