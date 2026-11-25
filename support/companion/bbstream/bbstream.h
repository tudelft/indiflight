/*
 * bbstream: near-live decoding of the packetised blackbox stream (blackbox_serial_packets = ON).
 *
 * Bytes from the serial port go into bbs_feed(). Packets are CRC-checked, and the payload (the plain
 * log) is decoded by the unmodified blackbox-tools parser: every so often the header plus all data since
 * the last decoded I-frame is placed in an in-memory file and parsed, and the rows that are new are
 * appended to a growing table. This is NOT per-sample streaming: rows appear in batches, and the newest
 * frame appears once the next one has started (the parser needs that lookahead).
 *
 * The code is not tied to the packet boundaries: window starts are the frame offsets reported by the
 * parser itself. After lost or corrupt data the next frame boundary is not known, so the decoder tries
 * candidate I-frame bytes until one decodes to rows that are plausible with the last good row.
 *
 * Rows hold the raw integers of the main (I/P) frame fields, in file order, followed by the fields of the
 * most recent slow frame, like blackbox_decode's CSV. The only change to a value is the time column
 * (index 1): it is unwrapped to 64 bit so it keeps growing across the 32-bit rollover and across windows.
 *
 * Not thread safe; use one instance from one thread.
 */
#ifndef BBSTREAM_H_
#define BBSTREAM_H_

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

typedef struct bbs bbs_t;

typedef struct {
    uint32_t decode_every_bytes;  // decode when this many undecoded bytes are buffered, 0 = 16384
    int32_t max_iter_jump;        // plausibility gate for rows after a gap, 0 = 50000 iterations
    int64_t max_dt_us;            // same, in microseconds, 0 = 30 s
    int library_stderr;           // nonzero: let the parser print its diagnostics (they are silenced by default)
    // optional: called after a decode with the rows that were just appended (n_cols values each)
    void (*on_rows)(void *user, const int64_t *rows, size_t n_rows, size_t n_cols);
    void *user;
} bbs_config_t;

typedef struct {
    uint64_t bytes_in;
    uint64_t packets_ok, packets_bad_crc, packets_lost; // lost = seq gaps (number of missing packets)
    uint64_t bytes_skipped;                             // bytes that were not part of a good packet
    uint64_t header_copies, header_installs;            // repeated headers received complete / used because the log had no header
    uint64_t decodes, rows, rows_duplicate, rows_rejected, resyncs, logs;
    double decode_ms_total, decode_ms_max;
} bbs_stats_t;

bbs_t *bbs_create(const bbs_config_t *cfg);
void bbs_destroy(bbs_t *b);

// Feed raw bytes from the serial port, in chunks of any size.
void bbs_feed(bbs_t *b, const uint8_t *data, size_t len);
// Decode what is buffered now (also done automatically by bbs_feed).
void bbs_decode(bbs_t *b);

const int64_t *bbs_rows(const bbs_t *b);   // n_rows * n_cols values, row major
size_t bbs_n_rows(const bbs_t *b);
size_t bbs_n_cols(const bbs_t *b);
const char *bbs_col_name(const bbs_t *b, size_t col);
const bbs_stats_t *bbs_stats(const bbs_t *b);
// Row index at which each log in the stream starts (a new header = a new log, e.g. after re-arming).
const size_t *bbs_log_starts(const bbs_t *b, size_t *n_logs);

void bbs_write_csv(const bbs_t *b, FILE *f);

#endif
