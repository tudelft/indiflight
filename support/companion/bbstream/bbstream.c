#define _GNU_SOURCE
#include "bbstream.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <setjmp.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

// blackbox-tools, used unmodified
#include "parser.h"
#include "tools.h"

#define PKT_SYNC 0xA5
#define PKT_PAYLOAD_MAX 240
#define PKT_OVERHEAD 6
#define PKT_LEN_MASK 0x1FFF
#define PKT_FLAG_HEADER 0x8000  // header channel: repeated copies of the log header
#define PKT_FLAG_FIRST 0x4000
#define PKT_FLAG_LAST 0x2000
#define HDR_MAX (256 * 1024)
#define DAT_MAX (4 * 1024 * 1024)
#define RESYNC_TRIES 256
#define RESYNC_MIN_ROWS 3  // a candidate frame start must yield this many consecutive rows

#define LOG_MARKER "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
#define LOG_START "H Product:Blackbox"  // seen in the data = a new log has begun

struct bbs {
    bbs_config_t cfg;
    bbs_stats_t st;

    // packet layer
    uint8_t pbuf[4 * (PKT_PAYLOAD_MAX + PKT_OVERHEAD)];
    size_t plen;
    bool have_seq, disc_pending;
    uint8_t next_seq;

    double last_header_ms;  // when a header (or complete copy of it) was last seen

    // header channel: the copy being assembled
    uint8_t *hc; size_t hc_len; bool hc_active; uint8_t hc_next_seq;

    // header / data of the log being received
    uint8_t *hdr; size_t hdr_len; bool hdr_done, at_line_start;
    uint8_t *dat; size_t dat_len, dat_cap, dat_decoded;
    size_t marker_match;
    bool resync;

    // decode window bookkeeping (valid during bbs_decode)
    int64_t win_i_off;   // offset in dat of the newest I-frame the parser completed, -1 if none
    size_t win_seen;     // valid main frames in this window (new or already known)
    size_t win_accepted; // rows appended in this window
    size_t win_invalid;  // frames the parser flagged as invalid or that failed the plausibility gate
    bool win_have_i, win_i_ok; // two accepted I-frames exactly one I-frame interval apart
    uint32_t win_i_it;

    // last accepted row
    bool have_last; uint32_t last_it, last_raw_time; int64_t last_time64;

    // output
    int64_t *rows; size_t n_rows, cap_rows, n_cols, n_main, n_slow;
    int64_t slow[FLIGHT_LOG_MAX_FIELDS]; size_t slow_n;
    char **names; size_t names_n; size_t names_log;
    size_t *log_starts; size_t n_logs, cap_logs;

    int memfd;
};

static __thread bbs_t *cur;  // the instance being decoded: the parser callbacks have no user pointer

static double now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e3 + t.tv_nsec * 1e-6;
}

static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---------------------------------------------------------------- rows */

static void free_names(bbs_t *b)
{
    for (size_t i = 0; i < b->names_n; i++) free(b->names[i]);
    free(b->names);
    b->names = NULL;
    b->names_n = 0;
}

static void capture_names(bbs_t *b, flightLog_t *log)
{
    free_names(b);
    size_t ni = log->frameDefs['I'].fieldCount, ns = log->frameDefs['S'].fieldCount;
    b->names = calloc(ni + ns, sizeof(char *));
    if (!b->names) return;
    for (size_t i = 0; i < ni; i++) b->names[i] = strdup(log->frameDefs['I'].fieldName[i]);
    for (size_t i = 0; i < ns; i++) b->names[ni + i] = strdup(log->frameDefs['S'].fieldName[i]);
    b->names_n = ni + ns;
    b->n_main = ni;
    b->n_slow = ns;
    b->n_cols = ni + ns;
    b->names_log = b->n_logs;
}

static bool append_row(bbs_t *b, const int64_t *frame, int64_t time64)
{
    if (b->n_rows == b->cap_rows) {
        size_t cap = b->cap_rows ? b->cap_rows * 2 : 4096;
        int64_t *r = realloc(b->rows, cap * b->n_cols * sizeof(int64_t));
        if (!r) return false;
        b->rows = r;
        b->cap_rows = cap;
    }
    int64_t *row = b->rows + b->n_rows * b->n_cols;
    memcpy(row, frame, b->n_main * sizeof(int64_t));
    row[FLIGHT_LOG_FIELD_INDEX_TIME] = time64;
    for (size_t i = 0; i < b->n_slow; i++) row[b->n_main + i] = i < b->slow_n ? b->slow[i] : 0;
    b->n_rows++;
    return true;
}

static void on_frame(flightLog_t *log, bool valid, int64_t *frame, uint8_t type, int field_count, int frame_offset, int frame_size)
{
    bbs_t *b = cur;

    if (!valid || !frame) {
        b->win_invalid++;
        return;
    }

    /*
     * The parser accepts a frame that is cut off by the end of the file (it reads zeros) as if it was
     * complete. So only trust frames that are followed by at least one more byte: that byte is the
     * marker of the next frame. A frame that is wrongly dropped here is decoded again in the next window.
     */
    if ((size_t) frame_offset + (size_t) frame_size >= b->hdr_len + b->dat_len) return;

    if (type == 'S') {
        // an S frame before the first plausible main frame of the window is just garbage after a gap
        if (b->win_seen == 0 || (b->resync && b->win_accepted == 0)) return;
        b->slow_n = (size_t) field_count;
        memcpy(b->slow, frame, b->slow_n * sizeof(int64_t));
        return;
    }
    if (type != 'I' && type != 'P') return;

    if (b->names_log != b->n_logs || !b->names) capture_names(b, log);
    if (!b->names || (size_t) field_count != b->n_main) return;

    const uint32_t it = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION];
    const uint32_t raw_time = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_TIME]; // drops the parser's rollover offset
    int64_t time64 = (int64_t) raw_time;

    if (b->have_last) {
        const int32_t d_it = (int32_t)(it - b->last_it);
        const int32_t d_t = (int32_t)(raw_time - b->last_raw_time);
        if (d_it <= 0 && !b->resync) {  // already have it: windows overlap by design
            b->st.rows_duplicate++;
            b->win_seen++;
            goto i_frame;
        }
        if (d_it <= 0 || d_it > b->cfg.max_iter_jump || d_t < 0 || d_t > b->cfg.max_dt_us) {
            b->st.rows_rejected++;
            b->win_invalid++;
            return;
        }
        time64 = b->last_time64 + d_t;
    }

    if (!append_row(b, frame, time64)) return;
    b->have_last = true;
    b->last_it = it;
    b->last_raw_time = raw_time;
    b->last_time64 = time64;
    b->st.rows++;
    b->win_seen++;
    b->win_accepted++;
    if (type == 'I') {
        if (!b->win_have_i) {
            b->win_have_i = true;
            b->win_i_it = it;
        } else if (it == b->win_i_it + log->frameIntervalI) {
            b->win_i_ok = true;
        }
    }

i_frame:
    if (type == 'I') {
        // frame_offset is the first byte after the 'I' marker, relative to the start of the window file
        b->win_i_off = (int64_t) frame_offset - 1 - (int64_t) b->hdr_len;
    }
}

/* ---------------------------------------------------------------- decoding */

static bool dat_reserve(bbs_t *b, size_t extra)
{
    if (b->dat_len + extra <= b->dat_cap) return true;
    size_t cap = b->dat_cap ? b->dat_cap : 65536;
    while (cap < b->dat_len + extra) cap *= 2;
    uint8_t *d = realloc(b->dat, cap);
    if (!d) return false;
    b->dat = d;
    b->dat_cap = cap;
    return true;
}

static void dat_drop_front(bbs_t *b, size_t n)
{
    if (n > b->dat_len) n = b->dat_len;
    memmove(b->dat, b->dat + n, b->dat_len - n);
    b->dat_len -= n;
    b->dat_decoded = b->dat_decoded > n ? b->dat_decoded - n : 0;
}

// Parse header + dat with the library. Returns false if the parser could not run at all.
static bool run_window(bbs_t *b)
{
    if (b->memfd < 0) b->memfd = memfd_create("bbstream", 0);
    if (b->memfd < 0) return false;

    const size_t total = b->hdr_len + b->dat_len;
    if (ftruncate(b->memfd, 0) || ftruncate(b->memfd, (off_t) total)) return false;
    if (pwrite(b->memfd, b->hdr, b->hdr_len, 0) != (ssize_t) b->hdr_len) return false;
    if (b->dat_len && pwrite(b->memfd, b->dat, b->dat_len, (off_t) b->hdr_len) != (ssize_t) b->dat_len) return false;

    flightLog_t *volatile log = flightLogCreate(b->memfd);
    if (!log) return false;

    jmp_buf jb;
    setFatalErrorHandler(&jb);
    cur = b;
    b->win_i_off = -1;
    b->win_seen = b->win_accepted = b->win_invalid = 0;
    b->win_have_i = b->win_i_ok = false;
    // The parser prints diagnostics for normal conditions (e.g. "no events" at the end of the window)
    int saved_stderr = -1, devnull = -1;
    if (!b->cfg.library_stderr) {
        fflush(stderr);
        saved_stderr = dup(STDERR_FILENO);
        devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) dup2(devnull, STDERR_FILENO);
    }
    if (setjmp(jb) == 0) {
        flightLogParse(log, 0, NULL, on_frame, NULL, false);
    } else {
        b->win_i_off = -1;  // the parser gave up on garbage; do not trust this window
    }
    if (saved_stderr >= 0) { dup2(saved_stderr, STDERR_FILENO); close(saved_stderr); }
    if (devnull >= 0) close(devnull);
    setFatalErrorHandler(NULL);
    flightLogDestroy(log);
    return true;
}

void bbs_decode(bbs_t *b)
{
    if (!b->hdr_done || b->dat_len == 0) return;

    const double t0 = now_ms();
    const size_t first_new_row = b->n_rows;

    for (int tries = 0; tries < RESYNC_TRIES; tries++) {
        if (b->resync) {
            // the next frame boundary is unknown: the next candidate is any 'I' byte
            uint8_t *p = memchr(b->dat, 'I', b->dat_len);
            if (!p) { b->dat_len = 0; b->dat_decoded = 0; break; }
            dat_drop_front(b, (size_t)(p - b->dat));
        }

        // a candidate is tentative: remember what to restore if it turns out to be garbage
        const bool tentative = b->resync;
        const size_t saved_rows = b->n_rows, saved_slow_n = b->slow_n;
        const bool saved_have_last = b->have_last;
        const uint32_t saved_it = b->last_it, saved_raw = b->last_raw_time;
        const int64_t saved_t64 = b->last_time64;
        const bbs_stats_t saved_stats = b->st;
        int64_t *saved_slow = NULL;
        if (tentative && (saved_slow = malloc(sizeof(b->slow))) != NULL) memcpy(saved_slow, b->slow, sizeof(b->slow));

        const bool ran = run_window(b);
        if (ran) b->st.decodes++;

        bool done = !tentative || !ran;
        if (tentative && ran) {
            /*
             * Without a previous row to compare with (the start of the log was lost) there is no plausibility
             * gate, so also require a second I-frame at exactly one I-interval distance.
             */
            if (b->win_accepted >= RESYNC_MIN_ROWS && (saved_have_last || b->win_i_ok)) {
                b->resync = false;
                done = true;
            } else {
                // not (yet) convincing: undo what this candidate produced
                b->n_rows = saved_rows;
                b->slow_n = saved_slow_n;
                b->have_last = saved_have_last;
                b->last_it = saved_it; b->last_raw_time = saved_raw; b->last_time64 = saved_t64;
                if (saved_slow) memcpy(b->slow, saved_slow, sizeof(b->slow));
                const uint64_t decodes = b->st.decodes;
                b->st = saved_stats;
                b->st.decodes = decodes;
                if (b->win_accepted > 0 && b->win_invalid == 0) {
                    done = true;  // looks right but the window ends too early to tell: wait for more data
                } else {
                    dat_drop_front(b, 1);  // garbage: try the next 'I'
                    if (b->dat_len == 0) done = true;
                }
            }
        }
        free(saved_slow);
        if (done) break;
    }

    // continue next time from the newest I-frame the parser completed
    if (!b->resync && b->win_i_off > 0) dat_drop_front(b, (size_t) b->win_i_off);
    if (b->dat_len > DAT_MAX) { dat_drop_front(b, b->dat_len); b->resync = true; b->st.resyncs++; }
    b->dat_decoded = b->dat_len;

    const double ms = now_ms() - t0;
    b->st.decode_ms_total += ms;
    if (ms > b->st.decode_ms_max) b->st.decode_ms_max = ms;

    if (b->cfg.on_rows && b->n_rows > first_new_row) {
        b->cfg.on_rows(b->cfg.user, b->rows + first_new_row * b->n_cols, b->n_rows - first_new_row, b->n_cols);
    }
}

/* ---------------------------------------------------------------- stream layer */

static void discontinuity(bbs_t *b)
{
    if (!b->hdr_done) {  // the header is damaged: wait for the next log
        b->hdr_len = 0;
        b->at_line_start = true;
        return;
    }
    bbs_decode(b);       // salvage what came before the gap
    b->dat_len = 0;
    b->dat_decoded = 0;
    b->marker_match = 0;
    if (!b->resync) b->st.resyncs++;
    b->resync = true;
}

static void register_log(bbs_t *b)
{
    b->last_header_ms = now_ms();
    b->st.logs++;
    if (b->n_logs == b->cap_logs) {
        size_t cap = b->cap_logs ? b->cap_logs * 2 : 8;
        size_t *l = realloc(b->log_starts, cap * sizeof(size_t));
        if (!l) return;
        b->log_starts = l;
        b->cap_logs = cap;
    }
    b->log_starts[b->n_logs++] = b->n_rows;
}

static void new_log(bbs_t *b)
{
    bbs_decode(b);
    memcpy(b->hdr, LOG_START, strlen(LOG_START));
    b->hdr_len = strlen(LOG_START);
    b->hdr_done = false;
    b->at_line_start = false;
    b->dat_len = b->dat_decoded = 0;
    b->marker_match = 0;
    b->resync = false;
    b->have_last = false;
    b->slow_n = 0;
}

static void payload_byte(bbs_t *b, uint8_t c)
{
    if (!b->hdr_done) {
        if (b->hdr_len == 0 && c != 'H') return;  // wait for the start of a header
        if (b->at_line_start && c != 'H') {       // first byte after the last header line
            const size_t ml = strlen(LOG_MARKER);
            if (b->hdr_len < ml || memcmp(b->hdr, LOG_MARKER, ml) != 0) {  // not a real header
                b->hdr_len = 0;
                b->at_line_start = true;
                return;
            }
            b->hdr_done = true;
            b->marker_match = 0;
            register_log(b);
        } else {
            if (b->hdr_len >= HDR_MAX) { b->hdr_len = 0; b->at_line_start = true; return; }
            b->hdr[b->hdr_len++] = c;
            b->at_line_start = (c == '\n');
            return;
        }
    }

    if (!dat_reserve(b, 1)) return;
    b->dat[b->dat_len++] = c;

    // a new header in the data means that logging was restarted
    static const char start[] = LOG_START;
    if (c == (uint8_t) start[b->marker_match]) {
        if (++b->marker_match == sizeof(start) - 1) {
            b->dat_len -= sizeof(start) - 2;  // the marker bytes before this one
            b->dat_len--;                      // and this one
            new_log(b);
        }
    } else {
        b->marker_match = c == (uint8_t) start[0];
    }
}

static void handle_packet(bbs_t *b, uint8_t seq, const uint8_t *p, size_t n)
{
    if (b->have_seq && seq != b->next_seq) {
        b->st.packets_lost += (uint8_t)(seq - b->next_seq);
        b->disc_pending = true;
    }
    if (b->disc_pending) {
        discontinuity(b);
        b->disc_pending = false;
    }
    b->have_seq = true;
    b->next_seq = (uint8_t)(seq + 1);
    b->st.packets_ok++;
    for (size_t i = 0; i < n; i++) payload_byte(b, p[i]);
}

/*
 * Header channel: the firmware repeats the log header while logging, so that a receiver that started late, or
 * lost part of the header, can still decode. A copy is a run of packets with consecutive seq, from one marked
 * FIRST to one marked LAST; a run with a gap is dropped. Nothing is guessed: if the log already has this
 * header the copy is ignored, otherwise it becomes the header of the log whose data follows.
 */
static void header_copy_complete(bbs_t *b)
{
    const size_t ml = strlen(LOG_MARKER);
    if (b->hc_len < ml || memcmp(b->hc, LOG_MARKER, ml) != 0 || b->hc[b->hc_len - 1] != '\n') return;
    b->st.header_copies++;

    /*
     * The same header means the same log, unless no header has been seen for a while: the copies come every
     * few seconds, so then logging stopped in between and this is another log (whose own header was lost).
     */
    const double now = now_ms();
    const bool stale = now - b->last_header_ms > b->cfg.header_gap_ms;
    b->last_header_ms = now;
    if (b->hdr_done && !stale && b->hdr_len == b->hc_len && memcmp(b->hdr, b->hc, b->hc_len) == 0) return;

    // no header for the data that is coming in (late start, or the header was damaged), or a different log
    if (b->hdr_done) bbs_decode(b);
    memcpy(b->hdr, b->hc, b->hc_len);
    b->hdr_len = b->hc_len;
    b->hdr_done = true;
    b->at_line_start = true;
    b->dat_len = b->dat_decoded = 0;
    b->marker_match = 0;
    b->resync = true;  // where in a frame the data continues is unknown
    b->have_last = false;
    b->slow_n = 0;
    b->st.header_installs++;
    register_log(b);
}

static void handle_header_packet(bbs_t *b, uint8_t seq, unsigned flags, const uint8_t *p, size_t n)
{
    if (flags & PKT_FLAG_FIRST) {
        b->hc_len = 0;
        b->hc_active = true;
    } else if (!b->hc_active || seq != b->hc_next_seq) {
        b->hc_active = false;
        b->hc_next_seq = (uint8_t)(seq + 1);
        return;
    }
    b->hc_next_seq = (uint8_t)(seq + 1);
    if (b->hc_len + n > HDR_MAX) { b->hc_active = false; return; }
    memcpy(b->hc + b->hc_len, p, n);
    b->hc_len += n;
    if (flags & PKT_FLAG_LAST) {
        b->hc_active = false;
        header_copy_complete(b);
    }
}

static size_t process_packets(bbs_t *b)
{
    size_t o = 0;
    for (;;) {
        uint8_t *s = memchr(b->pbuf + o, PKT_SYNC, b->plen - o);
        const size_t skip = (s ? (size_t)(s - b->pbuf) : b->plen) - o;
        if (skip) {
            b->st.bytes_skipped += skip;
            if (b->have_seq) b->disc_pending = true;
            o += skip;
        }
        if (!s || b->plen - o < 4) break;

        const unsigned lenflags = b->pbuf[o + 2] | (unsigned) b->pbuf[o + 3] << 8;
        const size_t n = lenflags & PKT_LEN_MASK;
        if (n < 1 || n > PKT_PAYLOAD_MAX) {  // false sync
            o++;
            b->st.bytes_skipped++;
            if (b->have_seq) b->disc_pending = true;
            continue;
        }
        if (b->plen - o < PKT_OVERHEAD + n) break;  // wait for the rest

        const uint16_t crc = b->pbuf[o + 4 + n] | (uint16_t) b->pbuf[o + 5 + n] << 8;
        if (crc16(b->pbuf + o + 1, 3 + n) != crc) {
            b->st.packets_bad_crc++;
            b->st.bytes_skipped++;
            b->disc_pending = true;
            o++;  // maybe the real packet starts inside this one
            continue;
        }
        if (lenflags & PKT_FLAG_HEADER) {
            handle_header_packet(b, b->pbuf[o + 1], lenflags, b->pbuf + o + 4, n);
        } else {
            handle_packet(b, b->pbuf[o + 1], b->pbuf + o + 4, n);
        }
        o += PKT_OVERHEAD + n;
    }
    return o;
}

void bbs_feed(bbs_t *b, const uint8_t *data, size_t len)
{
    b->st.bytes_in += len;
    while (len) {
        size_t k = sizeof(b->pbuf) - b->plen;
        if (k > len) k = len;
        memcpy(b->pbuf + b->plen, data, k);
        b->plen += k;
        data += k;
        len -= k;

        const size_t used = process_packets(b);
        memmove(b->pbuf, b->pbuf + used, b->plen - used);
        b->plen -= used;

        if (b->hdr_done && b->dat_len - b->dat_decoded >= b->cfg.decode_every_bytes) bbs_decode(b);
    }
}

/* ---------------------------------------------------------------- API */

bbs_t *bbs_create(const bbs_config_t *cfg)
{
    bbs_t *b = calloc(1, sizeof(*b));
    if (!b) return NULL;
    if (cfg) b->cfg = *cfg;
    if (!b->cfg.decode_every_bytes) b->cfg.decode_every_bytes = 16384;
    if (!b->cfg.max_iter_jump) b->cfg.max_iter_jump = 50000;
    if (!b->cfg.header_gap_ms) b->cfg.header_gap_ms = 6000;
    if (!b->cfg.max_dt_us) b->cfg.max_dt_us = 30 * 1000000;
    b->hdr = malloc(HDR_MAX);
    b->hc = malloc(HDR_MAX);
    b->memfd = -1;
    b->at_line_start = true;
    b->win_i_off = -1;
    if (!b->hdr || !b->hc) { free(b->hdr); free(b->hc); free(b); return NULL; }
    return b;
}

void bbs_destroy(bbs_t *b)
{
    if (!b) return;
    free_names(b);
    if (b->memfd >= 0) close(b->memfd);
    free(b->hdr); free(b->hc); free(b->dat); free(b->rows); free(b->log_starts);
    free(b);
}

const int64_t *bbs_rows(const bbs_t *b) { return b->rows; }
size_t bbs_n_rows(const bbs_t *b) { return b->n_rows; }
size_t bbs_n_cols(const bbs_t *b) { return b->n_cols; }
const char *bbs_col_name(const bbs_t *b, size_t c) { return c < b->names_n ? b->names[c] : ""; }
const bbs_stats_t *bbs_stats(const bbs_t *b) { return &b->st; }
const size_t *bbs_log_starts(const bbs_t *b, size_t *n) { *n = b->n_logs; return b->log_starts; }

void bbs_write_csv(const bbs_t *b, FILE *f)
{
    for (size_t c = 0; c < b->n_cols; c++) fprintf(f, "%s%s", c ? ", " : "", b->names[c]);
    fputc('\n', f);
    for (size_t r = 0; r < b->n_rows; r++) {
        for (size_t c = 0; c < b->n_cols; c++) {
            fprintf(f, "%s%lld", c ? ", " : "", (long long) b->rows[r * b->n_cols + c]);
        }
        fputc('\n', f);
    }
}
