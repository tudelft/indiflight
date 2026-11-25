#!/usr/bin/env python3
"""Receive the packetised blackbox stream (blackbox_serial_packets = ON) on a companion computer.

Packet layout (see src/main/blackbox/blackbox_io.h):
    0xA5 | seq u8 | len u16 LE | payload[len] | crc16 u16 LE   (CRC16 CCITT-FALSE over seq, len, payload)

The concatenated payloads are the plain blackbox log. `PacketParser` yields (seq, payload, lost) where
`lost` is the number of packets missing before this one (seq gap, or bytes skipped while resyncing).
When lost > 0 the downstream frame decoder must discard its P-frame state until the next I-frame.

Usage: blackbox_packets.py /dev/serial0 2000000 out.bbl
"""
import struct
import sys

SYNC = 0xA5
PAYLOAD_MAX = 240
OVERHEAD = 6


def crc16_ccitt_false(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_packet(seq, payload):
    """Same as the firmware; used by tests."""
    body = bytes([seq & 0xFF]) + struct.pack("<H", len(payload)) + bytes(payload)
    return bytes([SYNC]) + body + struct.pack("<H", crc16_ccitt_false(body))


class PacketParser:
    def __init__(self):
        self.buf = bytearray()
        self.next_seq = None
        self.bad_crc = 0
        self.skipped_bytes = 0

    def feed(self, data):
        self.buf += data
        out = []
        buf = self.buf
        while True:
            i = buf.find(bytes([SYNC]))
            if i < 0:
                self.skipped_bytes += len(buf)
                buf.clear()
                break
            if i:
                self.skipped_bytes += i
                del buf[:i]
            if len(buf) < 4:
                break
            seq = buf[1]
            n = buf[2] | buf[3] << 8
            if not 1 <= n <= PAYLOAD_MAX:
                del buf[0]  # false sync
                self.skipped_bytes += 1
                continue
            if len(buf) < OVERHEAD + n:
                break
            crc = buf[4 + n] | buf[5 + n] << 8
            if crc16_ccitt_false(bytes(buf[1:4 + n])) != crc:
                self.bad_crc += 1
                del buf[0]  # resync at the next sync byte inside/after this one
                self.skipped_bytes += 1
                self.next_seq = None
                continue
            lost = 0 if self.next_seq is None else (seq - self.next_seq) & 0xFF
            if self.next_seq is None and self.skipped_bytes:
                lost = 1
            self.next_seq = (seq + 1) & 0xFF
            out.append((seq, bytes(buf[4:4 + n]), lost))
            del buf[:OVERHEAD + n]
        return out


def main():
    import serial  # pyserial
    port, baud, path = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    parser = PacketParser()
    with serial.Serial(port, baud, timeout=0.05) as ser, open(path, "wb") as f:
        while True:
            for seq, payload, lost in parser.feed(ser.read(4096)):
                if lost:
                    print(f"lost {lost} packet(s) before seq {seq}", file=sys.stderr)
                f.write(payload)  # gaps are visible to a decoder as a seq break; see README
                f.flush()


if __name__ == "__main__":
    main()
