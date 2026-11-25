# Companion computer: live blackbox over serial

The flight controller can stream its blackbox log to a companion computer (e.g. Raspberry Pi Zero 2 W)
for in-flight analysis. No MSP is involved, the FC just uses the serial blackbox device.

## Flight controller setup

```
serial <port> 128 115200 57600 0 2000000   # FUNCTION_BLACKBOX (bit 7) on the chosen UART; last field = blackbox baud
set blackbox_device = SERIAL
set blackbox_serial_packets = ON
set blackbox_sample_rate = 1/2             # start at 250 Hz, see the budget below
```

Build with a larger UART TX buffer, e.g. `EXTRA_FLAGS=-DUART_TX_BUFFER_SIZE=8192` (default is 256 bytes).
A packet is up to 246 bytes, so the default buffer can hold one packet at best, and every burst (I-frames, the
header) would be dropped.

## Wire format

The plain blackbox stream has no integrity protection: a lost or flipped byte corrupts every following P-frame
(they are deltas). With `blackbox_serial_packets = ON` the stream is cut into packets, and the concatenated
payloads are exactly the plain `.bbl` log.

```
0xA5 | seq u8 | len u16 LE (1..240) | payload | crc16 u16 LE      CRC16 CCITT-FALSE over seq, len, payload
```

- The FC ends a packet once per `blackboxUpdate()`, so a packet normally holds whole frames, and it also ends
  when 240 bytes are staged. A frame can therefore span two packets.
- If the UART TX buffer cannot take a whole packet, that packet is dropped (never partially sent) and `seq`
  still increments. The receiver sees a gap. With `DEBUG_BB_OUTPUT` the drop counter is `debug[2]`.
- Firmware: `blackbox_io.c` (`serialPacketFlush()`), format in `blackbox_io.h`.

## Pi side

`blackbox_packets.py` is a reference receiver: `PacketParser.feed(bytes)` yields `(seq, payload, lost)`.
It resyncs on the sync byte after a bad CRC or a false sync. Ideas for the rest:

1. **Hardware.** Use the PL011 UART (`dtoverlay=disable-bt`, or `miniuart-bt` on the Zero 2 W), not the mini-UART,
   whose clock follows the core clock. Disable the serial console. 2 Mbaud works on PL011. Common wiring is
   FC TX -> Pi RX, plus GND. Keep the cable short.
2. **Reader thread.** Read in a dedicated thread (or asyncio) with large reads (4 kB) into a queue. Never let
   analysis code sit between the UART and the kernel buffer: the Pi's UART FIFO is only 16 bytes, and
   overrun there looks like a lost packet. Raising the tty buffer or using `os.read` on the raw fd helps.
3. **Stage 1: unwrap.** Write payloads to disk unchanged (so you always get a normal log, decodable by the
   existing Python tools and Blackbox Explorer) and pass them to stage 2.
4. **Stage 2: incremental frame decoder.** Parse the header lines (`H ...`) once, then decode frames in
   a state machine. The field table gives the encodings and predictors, and P-frames need only the
   previous frame, so it is O(1) state. Use the same logic as `indiflight_log_importer.py` but fed bytes
   instead of a file. Prefer a minimal decoder for just the few fields the analysis needs: Python at
   500 Hz with 400 fields is likely too slow on a Zero 2 W, while a C extension or `numpy`
   vectorised decode of a whole packet is not.
5. **Loss handling.** On `lost > 0` (or a CRC failure), throw away the decoder state, skip until the next
   `I` frame (at most 32 ms of loop time away, `blackboxIInterval`), and mark the gap in the output. A frame
   cut in two by a lost packet is dropped, which is what the CRC buys you.
6. **Timing.** Use the frame's own time field, not arrival time. The serial link adds jitter and buffering.
   The analysis can still run on the latest decoded frame, and should look at the age
   of the newest frame as a health signal for the link.
7. **Late start.** If the Pi starts while the FC is already logging, it has no header. Either wait for the
   next log, or have the Pi disarm-safe restart logging, or (future firmware work) resend the header
   every few seconds.
8. **Bandwidth check.** Compute from a real log: `mean P-frame bytes * rate`, plus 6/240 = 2.5 % packet
   overhead, against baud/10 bytes/s. Count in the I-frames too.

Nothing here talks back to the FC. If the Pi must act on its analysis, use a separate, authenticated path
(e.g. MSP set commands) and treat everything from the stream as advisory.
