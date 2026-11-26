/*
 * bbstream_cli: read the packetised blackbox stream from a serial port (or a file / stdin), decode it
 * with bbstream and write the rows as CSV while they arrive. Rows are not kept in memory, so it can run for
 * as long as the disk lasts.
 *
 *   bbstream_cli -d /dev/serial0 -b 2000000 -o out.csv
 *   bbstream_cli -f capture.bin -o out.csv        (-f - is stdin)
 *
 * Two threads, so that decoding can never make the UART wait:
 *   reader  : only read() into a lock-free ring. Runs SCHED_FIFO if permitted; the ring is locked in RAM.
 *   decoder : drains the ring into bbs_feed(), decodes and writes the CSV.
 * The 16-byte PL011 FIFO must be served by the kernel interrupt handler; this program can only make
 * sure that user space reads promptly. Whatever is lost anyway is caught by the packet CRCs.
 */
#define _GNU_SOURCE
#include "bbstream.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/serial.h>

#define RING_SIZE (1u << 20)  // power of two, ~5 s of data at 2 Mbaud

static uint8_t *ring;
static _Atomic size_t ring_head, ring_tail;  // head written by reader, tail by decoder
static _Atomic uint64_t ring_dropped;        // bytes the decoder was too slow for
static _Atomic int reader_done;
static volatile sig_atomic_t stop;

static int fd = -1;
static int rt_prio = 50, reader_cpu = -1;
static int low_latency = 1;  // -L clears it
static int backpressure;  // files and pipes can wait for the decoder, a serial port cannot

static void on_sigint(int s) { (void) s; stop = 1; }

/* ---------------------------------------------------------------- CSV output, written as rows arrive */

typedef struct {
    FILE *f;
    FILE *bfl;  // -r: the log as received
    bbs_t *b;
    char **names;  // columns of the header line last written
    size_t n_names;
} csv_t;

static bool csv_same_columns(const csv_t *c, size_t n_cols)
{
    if (!c->names || c->n_names != n_cols) return false;
    for (size_t i = 0; i < n_cols; i++) {
        if (strcmp(c->names[i], bbs_col_name(c->b, i))) return false;
    }
    return true;
}

static void bfl_on_log_bytes(void *user, const uint8_t *data, size_t len)
{
    csv_t *c = user;
    if (c->bfl) fwrite(data, 1, len, c->bfl);
}

static void csv_on_rows(void *user, const int64_t *rows, size_t n_rows, size_t n_cols)
{
    csv_t *c = user;
    if (!c->f) return;
    if (!csv_same_columns(c, n_cols)) {  // first rows, or a log with other fields: (re)write the header line
        for (size_t i = 0; i < c->n_names; i++) free(c->names[i]);
        free(c->names);
        c->n_names = 0;
        c->names = calloc(n_cols, sizeof(char *));
        if (c->names) {
            for (size_t i = 0; i < n_cols; i++) c->names[i] = strdup(bbs_col_name(c->b, i));
            c->n_names = n_cols;
        }
        for (size_t i = 0; i < n_cols; i++) fprintf(c->f, "%s%s", i ? ", " : "", bbs_col_name(c->b, i));
        fputc('\n', c->f);
    }
    for (size_t r = 0; r < n_rows; r++) {
        const int64_t *row = rows + r * n_cols;
        for (size_t i = 0; i < n_cols; i++) fprintf(c->f, "%s%lld", i ? ", " : "", (long long) row[i]);
        fputc('\n', c->f);
    }
}

static speed_t baud_const(long baud)
{
    switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    default: return 0;
    }
}

/*
 * Any other baud rate (e.g. 2470000): termios2 with BOTHER. <asm/termbits.h> clashes with <termios.h>, so the
 * kernel struct is declared here (asm-generic layout, as on arm64 and x86). The driver picks the nearest rate
 * it can make; the PL011 with its 48 MHz clock makes 2461538 for 2470000 (0.3% off, UARTs tolerate ~2%).
 */
struct termios2_ {
    tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed, c_ospeed;
};
#ifndef BOTHER
#define BOTHER 0010000
#endif
#define TCGETS2_ _IOR('T', 0x2A, struct termios2_)
#define TCSETS2_ _IOW('T', 0x2B, struct termios2_)

static int set_custom_baud(int f, long baud)
{
    struct termios2_ t2;
    if (ioctl(f, TCGETS2_, &t2)) { perror("TCGETS2"); return -1; }
    t2.c_cflag &= ~(tcflag_t) (CBAUD | (CBAUD << 16));  // output and input speed fields
    t2.c_cflag |= BOTHER | (BOTHER << 16);
    t2.c_ispeed = t2.c_ospeed = (speed_t) baud;
    if (ioctl(f, TCSETS2_, &t2)) { perror("TCSETS2"); return -1; }
    if (ioctl(f, TCGETS2_, &t2) == 0 && t2.c_ospeed && (speed_t) baud != t2.c_ospeed) {
        fprintf(stderr, "note: requested %ld baud, the port runs at %u (%+.2f%%)\n", baud, (unsigned) t2.c_ospeed,
                100.0 * ((double) t2.c_ospeed - (double) baud) / (double) baud);
    }
    return 0;
}

static int open_port(const char *dev, long baud)
{
    int f = open(dev, O_RDONLY | O_NOCTTY);
    if (f < 0) { perror(dev); return -1; }
    if (!isatty(f)) return f;

    struct termios t;
    if (tcgetattr(f, &t)) { perror("tcgetattr"); return -1; }
    cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
    t.c_cc[VMIN] = 1;
    t.c_cc[VTIME] = 0;
    speed_t sp = baud_const(baud);
    cfsetispeed(&t, sp ? sp : B38400);  // nonstandard rates are set below
    cfsetospeed(&t, sp ? sp : B38400);
    if (tcsetattr(f, TCSANOW, &t)) { perror("tcsetattr"); return -1; }
    if (!sp && (baud <= 0 || set_custom_baud(f, baud))) { fprintf(stderr, "unsupported baud rate %ld\n", baud); return -1; }

    // hand received bytes to user space without waiting for the tty layer's timer
    // (-L: leave it off, to compare the uart overrun count with and without)
    struct serial_struct ss;
    if (ioctl(f, TIOCGSERIAL, &ss) == 0) {
        if (low_latency) ss.flags |= ASYNC_LOW_LATENCY;
        else ss.flags &= ~ASYNC_LOW_LATENCY;
        ioctl(f, TIOCSSERIAL, &ss);
    }
    tcflush(f, TCIFLUSH);
    return f;
}

static void *reader_thread(void *arg)
{
    (void) arg;
    struct sched_param sp = { .sched_priority = rt_prio };
    if (rt_prio > 0 && pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
        fprintf(stderr, "note: no SCHED_FIFO for the reader (run as root or grant CAP_SYS_NICE)\n");
    }
    if (reader_cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(reader_cpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    }

    uint8_t tmp[8192];
    while (!stop) {
        struct pollfd p = { .fd = fd, .events = POLLIN };
        int r = poll(&p, 1, 100);
        if (r < 0 && errno != EINTR) break;
        if (r <= 0) continue;
        ssize_t n = read(fd, tmp, sizeof(tmp));
        if (n == 0 || (n < 0 && errno != EINTR && errno != EAGAIN)) break;
        if (n <= 0) continue;

        size_t head = atomic_load_explicit(&ring_head, memory_order_relaxed);
        size_t tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
        while (backpressure && !stop && RING_SIZE - (head - tail) < (size_t) n) {
            nanosleep(&(struct timespec){ 0, 200000 }, NULL);
            tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
        }
        if (RING_SIZE - (head - tail) < (size_t) n) {  // full: drop; the CRC layer sees it as a gap
            atomic_fetch_add(&ring_dropped, (uint64_t) n);
            continue;
        }
        for (ssize_t i = 0; i < n; i++) ring[(head + i) & (RING_SIZE - 1)] = tmp[i];
        atomic_store_explicit(&ring_head, head + n, memory_order_release);
    }
    atomic_store(&reader_done, 1);
    return NULL;
}

static void print_status(const bbs_t *b, const char *dev_is_tty)
{
    const bbs_stats_t *s = bbs_stats(b);
    fprintf(stderr,
        "rows %zu | pkts ok %llu crc %llu lost %llu (fc dropped %llu) skipped %lluB | logs %llu | hdr copies %llu/%llu used | resyncs %llu | decodes %llu (max %.1f ms) | ring drops %lluB%s\n",
        (size_t) s->rows, (unsigned long long) s->packets_ok, (unsigned long long) s->packets_bad_crc,
        (unsigned long long) s->packets_lost, (unsigned long long) s->packets_dropped_tx,
        (unsigned long long) s->bytes_skipped,
        (unsigned long long) s->logs, (unsigned long long) s->header_copies, (unsigned long long) s->header_installs,
        (unsigned long long) s->resyncs, (unsigned long long) s->decodes, s->decode_ms_max,
        (unsigned long long) atomic_load(&ring_dropped), dev_is_tty);
}

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s (-d tty | -f file|-) [-b baud] [-o out.csv] [-r out.BFL] [-n decode_bytes] [-g header_gap_ms]\n"
        "       [-p rt_prio] [-c reader_cpu] [-L] [-v]\n"
        "  -r  also write the log as received (a .BFL for blackbox_decode / the log tools)\n"
        "  -L  do not set ASYNC_LOW_LATENCY on the serial port\n", a0);
}

int main(int argc, char **argv)
{
    const char *dev = NULL, *file = NULL, *out = NULL, *bfl = NULL;
    long baud = 2000000;
    unsigned decode_bytes = 0;
    int header_gap_ms = 0;
    int verbose = 0, opt;

    while ((opt = getopt(argc, argv, "d:f:b:o:r:n:g:p:c:Lvh")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'f': file = optarg; break;
        case 'b': baud = atol(optarg); break;
        case 'o': out = optarg; break;
        case 'r': bfl = optarg; break;
        case 'n': decode_bytes = (unsigned) atoi(optarg); break;
        case 'g': header_gap_ms = atoi(optarg); break;
        case 'p': rt_prio = atoi(optarg); break;
        case 'c': reader_cpu = atoi(optarg); break;
        case 'L': low_latency = 0; break;
        case 'v': verbose = 1; break;
        default: usage(argv[0]); return 2;
        }
    }
    if (!dev == !file) { usage(argv[0]); return 2; }

    fd = file ? (strcmp(file, "-") ? open(file, O_RDONLY) : STDIN_FILENO) : open_port(dev, baud);
    if (fd < 0) return 1;
    int is_tty = isatty(fd);
    backpressure = !is_tty;

    ring = malloc(RING_SIZE);
    if (!ring) return 1;
    memset(ring, 0, RING_SIZE);  // fault the pages in now
    // Lock only the ring. Not mlockall(MCL_FUTURE): everything allocated later (decoder buffers, stdio)
    // would be pinned too, and with no swap that starves the rest of the system.
    if (mlock(ring, RING_SIZE)) fprintf(stderr, "note: mlock failed: %s\n", strerror(errno));

    csv_t csv = { 0 };
    if (out) {
        csv.f = fopen(out, "w");
        if (!csv.f) { perror(out); return 1; }
        setvbuf(csv.f, NULL, _IOFBF, 1 << 16);
    }
    if (bfl) {
        csv.bfl = fopen(bfl, "wb");
        if (!csv.bfl) { perror(bfl); return 1; }
        setvbuf(csv.bfl, NULL, _IOFBF, 1 << 16);
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    bbs_config_t cfg = { .decode_every_bytes = decode_bytes, .header_gap_ms = header_gap_ms,
                         .on_rows = csv_on_rows, .user = &csv, .discard_rows = 1,
                         .on_log_bytes = bfl_on_log_bytes };
    bbs_t *b = bbs_create(&cfg);
    if (!b) return 1;
    csv.b = b;

    pthread_t th;
    if (pthread_create(&th, NULL, reader_thread, NULL)) { perror("pthread_create"); return 1; }

    struct serial_icounter_struct ic0 = {0}, ic;
    if (is_tty) ioctl(fd, TIOCGICOUNT, &ic0);

    time_t last_status = time(NULL);
    // link utilisation: bytes received per second against what the baud rate can carry (8N1 = 10 bits/byte)
    uint64_t last_bytes = 0;
    double util_max = 0;
    for (;;) {
        size_t tail = atomic_load_explicit(&ring_tail, memory_order_relaxed);
        size_t head = atomic_load_explicit(&ring_head, memory_order_acquire);
        if (head != tail) {
            // two contiguous pieces at most
            while (tail != head) {
                size_t i = tail & (RING_SIZE - 1);
                size_t n = RING_SIZE - i;
                if (n > head - tail) n = head - tail;
                bbs_feed(b, ring + i, n);
                tail += n;
            }
            atomic_store_explicit(&ring_tail, tail, memory_order_release);
        } else if (atomic_load(&reader_done)) {
            break;
        } else {
            nanosleep(&(struct timespec){ 0, 1000000 }, NULL);
        }
        if (verbose && time(NULL) != last_status) {
            last_status = time(NULL);
            char extra[160] = "";
            if (is_tty && ioctl(fd, TIOCGICOUNT, &ic) == 0) {
                const uint64_t bytes = bbs_stats(b)->bytes_in;
                const double util = 100.0 * (double) (bytes - last_bytes) / (baud / 10.0);
                last_bytes = bytes;
                if (util > util_max) util_max = util;
                snprintf(extra, sizeof extra, " | uart overrun %d frame %d parity %d | link %.0f%% (max %.0f%%)",
                         ic.overrun - ic0.overrun, ic.frame - ic0.frame, ic.parity - ic0.parity, util, util_max);
            }
            print_status(b, extra);
        }
    }
    pthread_join(th, NULL);
    bbs_decode(b);

    char extra[160] = "";
    if (is_tty && ioctl(fd, TIOCGICOUNT, &ic) == 0) {
        snprintf(extra, sizeof extra, " | uart overrun %d frame %d parity %d | link max %.0f%%",
                 ic.overrun - ic0.overrun, ic.frame - ic0.frame, ic.parity - ic0.parity, util_max);
    }
    print_status(b, extra);

    int ret = 0;
    if (csv.f && fclose(csv.f)) { perror(out); ret = 1; }
    if (csv.bfl && fclose(csv.bfl)) { perror(bfl); ret = 1; }
    for (size_t i = 0; i < csv.n_names; i++) free(csv.names[i]);
    free(csv.names);
    bbs_destroy(b);
    return ret;
}
