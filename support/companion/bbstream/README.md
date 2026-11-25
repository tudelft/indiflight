# bbstream: near-live decoding of the blackbox stream on a companion computer

C library (`bbstream.c/.h`) plus a command line program (`bbstream_cli`) that decode the packetised blackbox
stream (`blackbox_serial_packets = ON`, see `../README.md`) into a growing table of rows in memory.

The blackbox-tools parser in `../../LogAnalysis/.../blackbox-tools/src` is used **unmodified**.

```
make              # obj/bbstream_cli and obj/blackbox_decode (reference decoder, for the tests)
make test         # python3 test_bbstream.py (uses logs-mockup/LOG00043.BFL)
./obj/bbstream_cli -d /dev/serial0 -b 2000000 -o out.csv -v      # Ctrl-C writes the CSV
./obj/bbstream_cli -f capture.bin -o out.csv                      # replay a capture, or "-f -" for stdin
```

## How it works

1. `bbs_feed()` takes raw bytes in any chunking. A small state machine finds packets, checks the CRC16 and the
   sequence number. Bad or missing packets are a *discontinuity*.
2. Payload bytes are the plain log. The header (the leading `H ...` lines) is kept; the data after it is kept in
   a buffer.
3. Every `decode_every_bytes` (default 16 kB, about 80 ms at 2 Mbaud) the header and the buffered data are written
   to an in-memory file (`memfd`, nothing touches the SD card) and parsed by `flightLogParse()`.
4. The parser reports the offset of every frame. The newest I-frame it completed becomes the start of the next
   window, so a window only holds the data since the last I-frame, and the overlap is small. No knowledge about
   packet boundaries is needed.
5. Rows whose loop iteration is not newer than the last row are duplicates from the overlap and are skipped.
   New rows are appended to the table (and handed to the optional `on_rows` callback).

Values are the raw integers of the I/P frame fields in file order, followed by the fields of the latest slow
frame, i.e. the same columns as `blackbox_decode`'s CSV (compared with it in the tests). Nothing is converted
to SI units. The one exception: the time column is unwrapped to 64 bit.

## Loss and corruption

- After a gap the next frame boundary is unknown. The decoder takes the next `I` byte as a candidate frame start,
  parses from there and only believes it if it yields 3 consecutive rows that pass the parser's own validation
  and a plausibility check against the last good row; otherwise the candidate is rolled back and the next
  `I` is tried. Rows after a loss therefore resume at the next real I-frame (up to one I-frame interval lost per gap).
- The tests compare every decoded row with the reference decoder: no row may differ, under dropped packets,
  flipped bits, injected junk (hundreds of random patterns over three real logs). What happens is that rows are
  lost, never that wrong rows appear. After a gap the table skips ahead, so look at the loop iteration column for gaps.
- Slow-frame columns (the S frame: flight mode flags, state, ...) carry the last S frame that was received. If an
  S frame is lost they stay stale until the next one, and they are 0 until the first one was seen. This is the one
  place where a row can be wrong after packet loss; the tests do not check these columns on damaged streams.
- Without a previous row to compare with (the start of the log was lost) a candidate is only accepted if a second
  I-frame follows at exactly one I-interval distance.
- The parser accepts a frame that is cut off by the end of its input as if it was complete. bbstream only trusts
  a frame that is followed by at least one more byte, so the newest row appears one frame late.
- **The header comes from the data channel, or the header channel.** The header is about 14 kB, i.e. 60
  packets, and nothing can be decoded without it. With `blackbox_serial_header_interval` > 0 the firmware repeats it
  in the header channel (flags in `len`, see `../README.md`). bbstream assembles a copy from packets with
  consecutive seq between `FIRST` and `LAST`. If the current log already has that header the copy is ignored.
  Otherwise (the receiver started late, or the header was damaged) the copy becomes the header, and decoding
  starts at the next I-frame. Data that arrived before is lost, about one interval at most.
  Without repeats a damaged header or a late start means no rows for that log.
- A header copy that differs from the current header starts a new log. A new log with a *lost* initial header
  and the same settings as the previous log looks like a continuation, and its rows are skipped until the
  iteration counter passes the old one. A new header in the data channel (logging restarted) is detected
  and starts a new log (`bbs_log_starts()`).

## Pi notes (what the code does, and what it cannot do)

- The Pi UART has a 16-byte FIFO, at 2 Mbaud a byte arrives every 5 us. Whether it overruns depends on how fast the
  kernel interrupt handler runs; user space cannot influence that, except by keeping the CPU free and not
  running work at higher priority. Enable DMA for the PL011 if your kernel allows. Overruns show up in
  `uart overrun` in the `-v` status line (from `TIOCGICOUNT`).
- Whatever is lost anyway is detected by the CRC and sequence number and costs rows, not correctness.
- `bbstream_cli` has two threads. The reader thread only does `poll`+`read` into a 1 MiB lock-free ring (5 s of data);
  it runs `SCHED_FIFO` (`-p`, needs root or `CAP_SYS_NICE`), with locked memory and optionally pinned to a core (`-c`).
  The decoder thread drains the ring, so a slow decode (about 10 ms worst case on a PC) cannot stall the UART read. If it ever
  falls more than 5 s behind, data is dropped and counted (`ring drops`).
- `low_latency` is set on the tty and it is put in raw mode, 8N1, no flow control.
- The parser prints diagnostics for ordinary conditions; they are silenced while it runs (`library_stderr` to keep them).
- For files and pipes the reader waits for the decoder (no drops); a serial port cannot be stopped, so there the
  ring drops instead.
- Memory: the row table grows without a limit (350 columns x 8 bytes per row, about 2.8 kB). Consume rows with `on_rows`
  and drop them if the flight is long.
- One `bbs_t` per thread; the parser callbacks use a thread-local pointer.
