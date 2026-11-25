/*
 * bbstream_cli: read the packetised blackbox stream from a serial port (or a file / stdin), decode it
 * with bbstream and write the rows as CSV on exit.
 *
 *   bbstream_cli -d /dev/serial0 -b 2000000 -o out.csv
 *   bbstream_cli -f capture.bin -o out.csv        (-f - is stdin)
 *
 * Two threads, so that decoding can never make the UART wait:
 *   reader  : only read() into a lock-free ring. Runs SCHED_FIFO if permitted, memory locked.
 *   decoder : drains the ring into bbs_feed() and decodes.
 * The 16-byte PL011 FIFO must be served by the kernel interrupt handler; this program can only make
 * sure that user space reads promptly. Whatever is lost anyway is caught by the packet CRCs.
 */
#define _GNU_SOURCE
#include "bbstream.h"

#include <stdatomic.h>
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
static int backpressure;  // files and pipes can wait for the decoder, a serial port cannot

static void on_sigint(int s) { (void) s; stop = 1; }

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
    if (!sp) { fprintf(stderr, "unsupported baud rate %ld\n", baud); return -1; }
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    if (tcsetattr(f, TCSANOW, &t)) { perror("tcsetattr"); return -1; }

    // hand received bytes to user space without waiting for the tty layer's timer
    struct serial_struct ss;
    if (ioctl(f, TIOCGSERIAL, &ss) == 0) {
        ss.flags |= ASYNC_LOW_LATENCY;
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
        "rows %zu | pkts ok %llu crc %llu lost %llu skipped %lluB | resyncs %llu | decodes %llu (max %.1f ms) | ring drops %lluB%s\n",
        bbs_n_rows(b), (unsigned long long) s->packets_ok, (unsigned long long) s->packets_bad_crc,
        (unsigned long long) s->packets_lost, (unsigned long long) s->bytes_skipped,
        (unsigned long long) s->resyncs, (unsigned long long) s->decodes, s->decode_ms_max,
        (unsigned long long) atomic_load(&ring_dropped), dev_is_tty);
}

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s (-d tty | -f file|-) [-b baud] [-o out.csv] [-n decode_bytes] [-p rt_prio] [-c reader_cpu] [-v]\n", a0);
}

int main(int argc, char **argv)
{
    const char *dev = NULL, *file = NULL, *out = NULL;
    long baud = 2000000;
    unsigned decode_bytes = 0;
    int verbose = 0, opt;

    while ((opt = getopt(argc, argv, "d:f:b:o:n:p:c:vh")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'f': file = optarg; break;
        case 'b': baud = atol(optarg); break;
        case 'o': out = optarg; break;
        case 'n': decode_bytes = (unsigned) atoi(optarg); break;
        case 'p': rt_prio = atoi(optarg); break;
        case 'c': reader_cpu = atoi(optarg); break;
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
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) fprintf(stderr, "note: mlockall failed: %s\n", strerror(errno));

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    bbs_config_t cfg = { .decode_every_bytes = decode_bytes };
    bbs_t *b = bbs_create(&cfg);
    if (!b) return 1;

    pthread_t th;
    if (pthread_create(&th, NULL, reader_thread, NULL)) { perror("pthread_create"); return 1; }

    struct serial_icounter_struct ic0 = {0}, ic;
    if (is_tty) ioctl(fd, TIOCGICOUNT, &ic0);

    time_t last_status = time(NULL);
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
            char extra[96] = "";
            if (is_tty && ioctl(fd, TIOCGICOUNT, &ic) == 0) {
                snprintf(extra, sizeof extra, " | uart overrun %d frame %d parity %d",
                         ic.overrun - ic0.overrun, ic.frame - ic0.frame, ic.parity - ic0.parity);
            }
            print_status(b, extra);
        }
    }
    pthread_join(th, NULL);
    bbs_decode(b);

    char extra[96] = "";
    if (is_tty && ioctl(fd, TIOCGICOUNT, &ic) == 0) {
        snprintf(extra, sizeof extra, " | uart overrun %d frame %d parity %d",
                 ic.overrun - ic0.overrun, ic.frame - ic0.frame, ic.parity - ic0.parity);
    }
    print_status(b, extra);

    if (out) {
        FILE *f = fopen(out, "w");
        if (!f) { perror(out); return 1; }
        bbs_write_csv(b, f);
        fclose(f);
    }
    bbs_destroy(b);
    return 0;
}
