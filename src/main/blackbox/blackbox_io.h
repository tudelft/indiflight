/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

typedef enum {
    BLACKBOX_RESERVE_SUCCESS,
    BLACKBOX_RESERVE_TEMPORARY_FAILURE,
    BLACKBOX_RESERVE_PERMANENT_FAILURE
} blackboxBufferReserveStatus_e;

/*
 * We want to limit how bursty our writes to the device are. Note that this will also restrict the maximum size of a
 * header write we can make:
 */
#define BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET 256

/*
 * Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
 * regular logging iteration. This way we won't hog the CPU by making a gigantic write:
 */
#define BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION 64

/*
 * Optional packet framing for BLACKBOX_DEVICE_SERIAL (blackbox_serial_packets = ON).
 *
 * The plain blackbox byte stream has no integrity protection, so a lost or flipped byte silently corrupts
 * every following P-frame. With packets enabled the stream is cut into packets, at the latest once per
 * blackboxUpdate() so that a packet holds whole frames in the normal case. The concatenated payloads of the
 * data packets are exactly the plain log file.
 *
 *   offset  size  field
 *   0       1     sync, 0xA5
 *   1       1     seq, increments per packet of its channel (see below)
 *   2       2     len, little-endian: bits 0..12 payload length 1..BLACKBOX_PACKET_PAYLOAD_MAX, bits 13..15 flags
 *   4       len   payload
 *   4+len   2     crc16 CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) over bytes 1..3+len,
 *                 little-endian
 *
 * Two channels, each with its own seq counter, are told apart by the flags:
 *
 *   data channel   (flags 0)       the log. A packet that does not fit in the UART TX buffer is dropped as a
 *                                  whole (counted, seq still advances) so the receiver sees a seq gap and can
 *                                  resynchronise at the next I-frame.
 *   header channel (BLACKBOX_PACKET_FLAG_HEADER set)
 *                                  repeated copies of the log header (everything the log starts with, up to
 *                                  the first data frame), for receivers that started late or lost part of the
 *                                  header. Only sent while logging, every blackbox_serial_header_interval
 *                                  seconds, at about a tenth of the line rate and only if the TX buffer has
 *                                  room, so it never replaces data. A copy is a run of packets with
 *                                  consecutive seq: the first has BLACKBOX_PACKET_FLAG_FIRST, the last
 *                                  BLACKBOX_PACKET_FLAG_LAST (a copy of one packet has both). A receiver
 *                                  discards a copy with a seq gap. The initial header is sent in the data
 *                                  channel as usual.
 *
 * The packet size is 6 + len bytes, the UART TX buffer must be larger than that to ever send one.
 */
#define BLACKBOX_PACKET_SYNC 0xA5
#define BLACKBOX_PACKET_PAYLOAD_MAX 240
#define BLACKBOX_PACKET_OVERHEAD 6
#define BLACKBOX_PACKET_LEN_MASK 0x1FFF
#define BLACKBOX_PACKET_FLAG_HEADER 0x8000
#define BLACKBOX_PACKET_FLAG_FIRST 0x4000
#define BLACKBOX_PACKET_FLAG_LAST 0x2000

/*
 * RAM for the copy of the header that is repeated, 0 disables the repeat. The header is about 14 kB.
 * If the header does not fit, it is not repeated.
 */
#ifndef BLACKBOX_HEADER_COPY_SIZE
#if defined(STM32H7) || defined(SITL) || defined(MOCKUP)
#define BLACKBOX_HEADER_COPY_SIZE 24576
#else
#define BLACKBOX_HEADER_COPY_SIZE 0
#endif
#endif

extern int32_t blackboxHeaderBudget;

void blackboxOpen(void);
void blackboxWrite(uint8_t value);
int blackboxWriteString(const char *s);

void blackboxDeviceFlush(void);
// Record the header being written, so that it can be repeated (serial packets only). End when logging starts.
void blackboxSerialHeaderCaptureBegin(void);
void blackboxSerialHeaderCaptureEnd(void);
bool blackboxDeviceFlushForce(void);
bool blackboxDeviceFlushForceComplete(void);

bool blackboxDeviceOpen(void);
void blackboxDeviceClose(void);

void blackboxEraseAll(void);
bool isBlackboxErased(void);

bool blackboxDeviceBeginLog(void);
bool blackboxDeviceEndLog(bool retainLog);

bool isBlackboxDeviceFull(void);
bool isBlackboxDeviceWorking(void);
int32_t blackboxGetLogNumber(void);

void blackboxReplenishHeaderBudget(void);
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes);
int8_t blackboxGetLogFileNo(void);

#ifdef USE_SDCARD
bool blackboxSDCardIsIdle(void);
void blackboxSDCardResetWorkingDirectory(void);
#endif
