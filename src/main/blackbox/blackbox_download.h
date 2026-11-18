/*
 * This file is part of Indiflight.
 *
 * Indiflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Indiflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Listing, downloading and erasing blackbox logs over MSP, as an alternative to USB mass storage mode.
 *
 * Logs are read from the device blackbox is configured to log to (SD card or onboard flash). If that is neither,
 * the SD card is used if one is inserted, otherwise the flash.
 *  - SD card: the regular files in /logs.
 *  - Flash: logs are stored back to back; they are found by scanning for the blackbox header and are named
 *    LOG00001.BFL, LOG00002.BFL, ... in the order they are stored. They have no date.
 *
 * All integers are little-endian. Filenames are 8.3 names (e.g. "LOG00012.BFL") NUL-padded to 12 bytes.
 *
 * MSP2_INDI_BLACKBOX_LOG_LIST
 *   request: u16 startIndex
 *   reply:   u8 status, u16 startIndex, u8 count, u8 flags, u8 progress,
 *            count x { char name[12], u32 size, u16 fatDate, u16 fatTime }   (last write date/time, 0 if unknown)
 *   flags: bit0 more entries follow, bit1 logs are on onboard flash.
 *   status IN_PROGRESS: the flash is still being scanned, progress is the percentage done; ask again.
 *
 * MSP2_INDI_BLACKBOX_LOG_READ
 *   request: char name[12], u32 offset, u16 size, u8 allowCompression
 *   reply:   u8 status, then if status is OK the same layout as the MSP_DATAFLASH_READ reply:
 *            u32 offset, u16 dataSize, u8 compressionType (0: none, 1: huffman),
 *            dataSize bytes of data (huffman: u16 uncompressedByteCount followed by the compressed bytes).
 *            dataSize 0 means end of file.
 *   The file stays open between requests, so sequential reads are cheap. Reading another file closes it.
 *
 * MSP2_INDI_BLACKBOX_LOG_CLOSE
 *   request: none
 *   reply:   u8 status
 *   Releases the SD card. Also happens automatically after a period of inactivity and before blackbox starts logging.
 *
 * MSP2_INDI_BLACKBOX_LOG_ERASE_ALL
 *   request: u8 start (1: start erasing all logs, 0: only report progress)
 *   reply:   u8 status, u8 progress (percent, 255 if unknown)
 *   status IN_PROGRESS while erasing; keep sending requests with start = 0 (every 100-500 ms) until it is something
 *   else. OK means the last erase finished (or none was started). Deleting SD card files only progresses while
 *   requests come in; if they stop, the erase is abandoned with the remaining files kept.
 *   SD card: deletes the files in /logs, then makes their space usable for new logs again. Flash: erases the flash.
 */

#pragma once

#include <stdbool.h>

#include "common/streambuf.h"
#include "common/time.h"

typedef enum {
    LOG_DOWNLOAD_OK = 0,
    LOG_DOWNLOAD_NOT_READY = 1,          // no storage, or not ready (e.g. no card, filesystem not ready)
    LOG_DOWNLOAD_BUSY = 2,               // armed, blackbox is logging, or an erase is running
    LOG_DOWNLOAD_IO_ERROR = 3,           // I/O error or timeout, retrying is reasonable
    LOG_DOWNLOAD_NOT_FOUND = 4,
    LOG_DOWNLOAD_BAD_REQUEST = 5,
    LOG_DOWNLOAD_IN_PROGRESS = 6,        // flash scan or erase in progress, ask again
} logDownloadStatus_e;

void blackboxDownloadUpdate(timeMs_t currentTimeMs);
bool blackboxDownloadReleaseForLogging(void);

void blackboxDownloadMspList(sbuf_t *src, sbuf_t *dst);
void blackboxDownloadMspRead(sbuf_t *src, sbuf_t *dst);
void blackboxDownloadMspClose(sbuf_t *dst);
void blackboxDownloadMspEraseAll(sbuf_t *src, sbuf_t *dst);
