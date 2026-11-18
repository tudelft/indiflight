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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MSP_LOG_DOWNLOAD

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "common/huffman.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#ifdef USE_SDCARD
#include "drivers/sdcard.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "pg/sdcard.h"
#endif

#ifdef USE_FLASHFS
#include "io/flashfs.h"
#endif

#include "blackbox_download.h"

/*
 * The MSP handlers only run while disarmed and while blackbox is not logging. MSP replies are synchronous, so the
 * handlers wait for the storage themselves (for the SD card by polling the asyncfatfs filesystem, which is otherwise
 * driven from the main task) until their operation completes or times out.
 *
 * SD card: blackbox keeps the working directory in /logs once it has created a log. While a download session is
 * active the working directory is /logs too, and blackbox is told to forget its directory state. When the session is
 * released, the working directory goes back to the root directory, so blackbox re-opens /logs on its next log.
 */

#define DOWNLOAD_NAME_LENGTH        12      // "LOG00012.BFL", NUL-padded on the wire
#define DOWNLOAD_LIST_ENTRY_SIZE    (DOWNLOAD_NAME_LENGTH + 4 + 2 + 2)
#define DOWNLOAD_TIMEOUT_MS         250     // per MSP request
#define DOWNLOAD_IDLE_TIMEOUT_MS    2000    // release the card after this long without requests
#define DOWNLOAD_LOCKOUT_MS         500     // refuse requests for this long after blackbox asked for the storage
#define DOWNLOAD_READ_HEADER_SIZE   12      // offset, size, compression, huffman info, huffman encoder overshoot
#define DOWNLOAD_HUFFMAN_CHUNK_SIZE 256
#define DOWNLOAD_HUFFMAN_MIN_BUFFER 1024    // below this a single compressed chunk might not fit

#define LIST_FLAG_MORE              (1 << 0)
#define LIST_FLAG_FLASH             (1 << 1)

#define PROGRESS_UNKNOWN            255

#define SDCARD_LOG_DIRECTORY        "logs"

// First line of every blackbox log header, see blackboxHeader in blackbox.c
#define FLASH_LOG_MARKER            "H Product:Blackbox flight data recorder"
#define FLASH_LOG_MARKER_LENGTH     (sizeof(FLASH_LOG_MARKER) - 1)
#define FLASH_MAX_LOGS              128     // further logs are counted as part of the last one
#define FLASH_SCAN_CHUNK_SIZE       512

enum {
    COMPRESSION_NONE = 0,
    COMPRESSION_HUFFMAN = 1,
};

typedef enum {
    STORAGE_NONE,
    STORAGE_SDCARD,
    STORAGE_FLASH,
} logStorage_e;

typedef enum {
    ERASE_IDLE,
    ERASE_SDCARD_DELETE,        // deleting the files, progresses only while MSP requests come in
    ERASE_SDCARD_REBUILD,       // making the space of the deleted files available for logging again
    ERASE_SDCARD_WAIT_READY,
    ERASE_FLASH,
} erasePhase_e;

#ifdef USE_SDCARD
typedef enum {
    OPEN_NONE,
    OPEN_WAITING,           // afatfs_fopen/mkdir started, callback pending
    OPEN_COMPLETE,          // callback fired, result in openResult
    OPEN_ABANDONED,         // timed out while waiting; the handle must be closed when the callback fires
} openState_e;
#endif

static struct {
    logStorage_e storage;

    bool logOpen;                       // fileName is open for reading
    char fileName[DOWNLOAD_NAME_LENGTH];

    // SD card: bytes read from the file but not sent yet. The file cursor is at carryOffset + carryLen.
    uint8_t carry[DOWNLOAD_HUFFMAN_CHUNK_SIZE];
    uint32_t carryOffset;
    uint16_t carryLen;

#ifdef USE_SDCARD
    struct {
        bool active;                    // working directory may not be the root directory
        bool inLogDirectory;            // logDirectory is open and is the working directory
        afatfsFilePtr_t logDirectory;
        afatfsFilePtr_t file;

        openState_e openState;
        afatfsFilePtr_t openResult;
        afatfsFilePtr_t orphan;         // handle from an abandoned open, to be closed

        bool unlinkPending;
    } sd;
#endif

#ifdef USE_FLASHFS
    struct {
        uint32_t start;                 // absolute flash addresses of the open log
        uint32_t end;
        uint32_t cursor;
    } flash;
#endif

    struct {
        erasePhase_e phase;
        logDownloadStatus_e result;     // of the last erase
        uint16_t total;
        uint16_t deleted;
    } erase;

    timeMs_t lastAccessMs;
    timeMs_t lockoutUntilMs;
} dl;

#ifdef USE_FLASHFS
static struct {
    bool valid;
    uint16_t count;
    uint32_t start[FLASH_MAX_LOGS];     // absolute address of each log, start[0] is always 0
    uint32_t scanned;                   // markers starting below this address have been found
    uint32_t end;                       // used size of the flash covered by the index
} flashIndex;

static uint8_t flashScanBuffer[FLASH_SCAN_CHUNK_SIZE];
#endif

static bool timedOut(timeMs_t deadline)
{
    return cmp32(millis(), deadline) >= 0;
}

/* ---------------------------------------------------------------------------------------------------- SD card */

#ifdef USE_SDCARD

// Advances the filesystem by one step. Returns false once the deadline has passed.
static bool pollUntil(timeMs_t deadline)
{
    if (timedOut(deadline)) {
        return false;
    }
    afatfs_poll();
    return true;
}

static bool sdFilesystemReady(void)
{
    return sdcardConfig()->mode != SDCARD_MODE_NONE
        && sdcard_isInserted()
        && sdcard_isFunctional()
        && afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY;
}

static void sdOpenCallback(afatfsFilePtr_t file)
{
    if (dl.sd.openState == OPEN_ABANDONED) {
        dl.sd.orphan = file;
        dl.sd.openState = OPEN_NONE;
    } else {
        dl.sd.openResult = file;
        dl.sd.openState = OPEN_COMPLETE;
    }
}

static void sdUnlinkCallback(void)
{
    dl.sd.unlinkPending = false;
}

/*
 * Opens a file (or the log directory, if mode is NULL) in the working directory.
 * Returns NULL and sets *status if it failed.
 */
static afatfsFilePtr_t sdOpenBlocking(const char *name, const char *mode, timeMs_t deadline, logDownloadStatus_e *status)
{
    // Let an earlier, abandoned open finish first and close its handle
    while (dl.sd.openState != OPEN_NONE || dl.sd.orphan) {
        if (dl.sd.orphan && afatfs_fclose(dl.sd.orphan, NULL)) {
            dl.sd.orphan = NULL;
        } else if (!pollUntil(deadline)) {
            *status = LOG_DOWNLOAD_IO_ERROR;
            return NULL;
        }
    }

    dl.sd.openState = OPEN_WAITING;
    dl.sd.openResult = NULL;

    // mkdir opens the directory if it already exists (that is how blackbox opens it too)
    const bool started = mode ? afatfs_fopen(name, mode, sdOpenCallback) : afatfs_mkdir(name, sdOpenCallback);
    if (!started) {
        // Out of file handles
        dl.sd.openState = OPEN_NONE;
        *status = LOG_DOWNLOAD_IO_ERROR;
        return NULL;
    }

    while (dl.sd.openState == OPEN_WAITING) {
        if (!pollUntil(deadline)) {
            dl.sd.openState = OPEN_ABANDONED;
            *status = LOG_DOWNLOAD_IO_ERROR;
            return NULL;
        }
    }

    dl.sd.openState = OPEN_NONE;
    if (!dl.sd.openResult) {
        *status = mode ? LOG_DOWNLOAD_NOT_FOUND : LOG_DOWNLOAD_IO_ERROR;
    }
    return dl.sd.openResult;
}

static bool sdIsHoldingCard(void)
{
    return dl.sd.active || dl.sd.file || dl.sd.logDirectory || dl.sd.orphan || dl.sd.openState != OPEN_NONE
        || dl.sd.unlinkPending;
}

/*
 * Makes one step towards releasing the SD card: closes our handles and changes back to the root directory.
 * Never blocks. Returns true once everything is released.
 */
static bool sdReleaseStep(void)
{
    dl.sd.inLogDirectory = false;

    if (afatfs_getFilesystemState() != AFATFS_FILESYSTEM_STATE_READY) {
        // The filesystem is only re-initialised at runtime after an erase, when we hold nothing. Otherwise the handles
        // are useless now.
        memset(&dl.sd, 0, sizeof(dl.sd));
        return true;
    }

    if (dl.sd.orphan) {
        if (!afatfs_fclose(dl.sd.orphan, NULL)) {
            return false;
        }
        dl.sd.orphan = NULL;
    }

    if (dl.sd.openState != OPEN_NONE || dl.sd.unlinkPending) {
        // Wait for the abandoned open to call back so its handle can be closed, or for the delete to finish
        return false;
    }

    if (dl.sd.file) {
        if (!afatfs_fclose(dl.sd.file, NULL)) {
            return false;
        }
        dl.sd.file = NULL;
    }

    if (dl.sd.logDirectory) {
        if (!afatfs_fclose(dl.sd.logDirectory, NULL)) {
            return false;
        }
        dl.sd.logDirectory = NULL;
    }

    if (dl.sd.active) {
        if (!afatfs_chdir(NULL)) {
            return false;
        }
        dl.sd.active = false;
    }

    return true;
}

// Opens /logs and makes it the working directory.
static logDownloadStatus_e sdAcquire(timeMs_t deadline)
{
    if (dl.sd.inLogDirectory) {
        return LOG_DOWNLOAD_OK;
    }

    // Clean up first, e.g. after an earlier sdAcquire() timed out half way
    while (!sdReleaseStep()) {
        if (!pollUntil(deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
    }

    blackboxSDCardResetWorkingDirectory();

    // From here on, sdReleaseStep() restores the root directory
    dl.sd.active = true;

    while (!afatfs_chdir(NULL)) {
        if (!pollUntil(deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
    }

    logDownloadStatus_e status = LOG_DOWNLOAD_OK;
    dl.sd.logDirectory = sdOpenBlocking(SDCARD_LOG_DIRECTORY, NULL, deadline, &status);
    if (!dl.sd.logDirectory) {
        return status;
    }

    while (!afatfs_chdir(dl.sd.logDirectory)) {
        if (!pollUntil(deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
    }

    dl.sd.inLogDirectory = true;
    return LOG_DOWNLOAD_OK;
}

// Converts an 11 character FAT directory entry name ("LOG00012BFL") to "LOG00012.BFL", NUL-padded.
static void formatFatFileName(const char *fatName, char *out)
{
    memset(out, 0, DOWNLOAD_NAME_LENGTH);

    int len = 0;
    for (int i = 0; i < 8 && fatName[i] != ' '; i++) {
        out[len++] = fatName[i];
    }

    if (fatName[8] != ' ') {
        out[len++] = '.';
        for (int i = 8; i < FAT_FILENAME_LENGTH && fatName[i] != ' '; i++) {
            out[len++] = fatName[i];
        }
    }
}

static bool sdIsListedEntry(fatDirectoryEntry_t *entry)
{
    // Long filename entries have the volume ID attribute set, so they are skipped as well
    return !fat_isDirectoryEntryEmpty(entry)
        && (entry->attrib & (FAT_FILE_ATTRIBUTE_VOLUME_ID | FAT_FILE_ATTRIBUTE_DIRECTORY)) == 0;
}

// Return false to stop the iteration
typedef bool sdFileVisitor_t(int index, const fatDirectoryEntry_t *entry, void *context);

// Calls visit() for each file in /logs, in directory order. Requires sdAcquire().
static logDownloadStatus_e sdForEachFile(sdFileVisitor_t *visit, void *context, timeMs_t deadline)
{
    logDownloadStatus_e status = LOG_DOWNLOAD_OK;
    afatfsFinder_t finder;
    int index = 0;

    afatfs_findFirst(dl.sd.logDirectory, &finder);

    while (true) {
        fatDirectoryEntry_t *entry;
        const afatfsOperationStatus_e findStatus = afatfs_findNext(dl.sd.logDirectory, &finder, &entry);

        if (findStatus == AFATFS_OPERATION_IN_PROGRESS) {
            if (!pollUntil(deadline)) {
                status = LOG_DOWNLOAD_IO_ERROR;
                break;
            }
            continue;
        }

        if (findStatus == AFATFS_OPERATION_FAILURE) {
            status = LOG_DOWNLOAD_IO_ERROR;
            break;
        }

        if (entry == NULL || fat_isDirectoryEntryTerminator(entry)) {
            break;
        }

        if (sdIsListedEntry(entry)) {
            if (!visit(index, entry, context)) {
                break;
            }
            index++;
        }
    }

    afatfs_findLast(dl.sd.logDirectory);
    return status;
}

typedef struct {
    sbuf_t *dst;
    int startIndex;
    int maxCount;
    int count;
    bool more;
} sdListContext_t;

static bool sdListVisitor(int index, const fatDirectoryEntry_t *entry, void *context)
{
    sdListContext_t *list = context;

    if (index < list->startIndex) {
        return true;
    }
    if (list->count >= list->maxCount) {
        list->more = true;
        return false;
    }

    char name[DOWNLOAD_NAME_LENGTH];
    formatFatFileName(entry->filename, name);
    sbufWriteData(list->dst, name, DOWNLOAD_NAME_LENGTH);
    sbufWriteU32(list->dst, entry->fileSize);
    sbufWriteU16(list->dst, entry->lastWriteDate);
    sbufWriteU16(list->dst, entry->lastWriteTime);
    list->count++;
    return true;
}

static bool sdCountVisitor(int index, const fatDirectoryEntry_t *entry, void *context)
{
    UNUSED(index);
    UNUSED(entry);
    (*(int *)context)++;
    return true;
}

static bool sdFirstFileVisitor(int index, const fatDirectoryEntry_t *entry, void *context)
{
    UNUSED(index);
    formatFatFileName(entry->filename, context);
    return false;
}

// Deletes files from /logs until there are none left or the deadline passes.
static void sdEraseDeleteStep(timeMs_t deadline)
{
    if (sdAcquire(deadline) != LOG_DOWNLOAD_OK) {
        return;
    }

    while (!timedOut(deadline)) {
        if (dl.sd.unlinkPending) {
            pollUntil(deadline);
            continue;
        }

        char name[DOWNLOAD_NAME_LENGTH + 1] = { 0 };
        if (sdForEachFile(sdFirstFileVisitor, name, deadline) != LOG_DOWNLOAD_OK) {
            return;
        }

        if (name[0] == '\0') {
            dl.erase.phase = ERASE_SDCARD_REBUILD;
            return;
        }

        logDownloadStatus_e status;
        dl.sd.file = sdOpenBlocking(name, "r+", deadline, &status);
        if (!dl.sd.file) {
            return;
        }

        while (!afatfs_funlink(dl.sd.file, sdUnlinkCallback)) {
            if (!pollUntil(deadline)) {
                // Still held in dl.sd.file, closed by the next release
                return;
            }
        }

        // The handle closes itself once the delete is done
        dl.sd.file = NULL;
        dl.sd.unlinkPending = true;
        dl.erase.deleted++;
    }
}

#endif // USE_SDCARD

/* ---------------------------------------------------------------------------------------------------- flash */

#ifdef USE_FLASHFS

// Flash logs have no names, they are named by position. Writes "LOG00012.BFL" for number 12, NUL-padded
static void formatLogName(uint32_t number, char *out)
{
    memcpy(out, "LOG00000.BFL", DOWNLOAD_NAME_LENGTH);
    for (int i = 7; i >= 3; i--) {
        out[i] = '0' + number % 10;
        number /= 10;
    }
}

static void flashIndexReset(void)
{
    memset(&flashIndex, 0, sizeof(flashIndex));
}

/*
 * Brings the index of logs on the flash up to date by scanning for log headers, until the deadline passes.
 * Returns true once the index covers all data on the flash.
 */
static bool flashIndexUpdate(timeMs_t deadline)
{
    const uint32_t used = flashfsGetOffset();

    if (!flashIndex.valid || used < flashIndex.end) {
        // First scan, or the flash has been erased since
        flashIndexReset();
        flashIndex.valid = true;
    }
    flashIndex.end = used;

    if (used == 0) {
        return true;
    }
    if (flashIndex.count == 0) {
        // Anything before the first header (or data without any header) still counts as a log
        flashIndex.start[0] = 0;
        flashIndex.count = 1;
    }

    while (flashIndex.scanned + FLASH_LOG_MARKER_LENGTH <= used) {
        if (timedOut(deadline)) {
            return false;
        }

        const uint32_t chunkStart = flashIndex.scanned;
        const int bytesRead = flashfsReadAbs(chunkStart, flashScanBuffer, MIN(sizeof(flashScanBuffer), used - chunkStart));
        if (bytesRead < (int)FLASH_LOG_MARKER_LENGTH) {
            return false;
        }

        for (int i = 0; i + (int)FLASH_LOG_MARKER_LENGTH <= bytesRead; i++) {
            if (flashScanBuffer[i] == 'H' && memcmp(flashScanBuffer + i, FLASH_LOG_MARKER, FLASH_LOG_MARKER_LENGTH) == 0) {
                const uint32_t address = chunkStart + i;
                if (address > flashIndex.start[flashIndex.count - 1] && flashIndex.count < FLASH_MAX_LOGS) {
                    flashIndex.start[flashIndex.count++] = address;
                }
            }
        }

        // The next chunk overlaps this one, so markers crossing the boundary are found (exactly once)
        flashIndex.scanned = chunkStart + bytesRead - (FLASH_LOG_MARKER_LENGTH - 1);
    }

    return true;
}

static uint8_t flashIndexProgress(void)
{
    if (flashIndex.end == 0) {
        return 100;
    }
    return (uint8_t)MIN((uint64_t)flashIndex.scanned * 100 / flashIndex.end, (uint64_t)99);
}

static uint32_t flashLogEnd(int index)
{
    return index + 1 < flashIndex.count ? flashIndex.start[index + 1] : flashIndex.end;
}

// Returns the index of the log with the given name, or -1.
static int flashFindLog(const char *name)
{
    uint32_t number = 0;
    for (int i = 3; i < 8; i++) {
        if (name[i] < '0' || name[i] > '9') {
            return -1;
        }
        number = number * 10 + name[i] - '0';
    }

    if (number < 1 || number > flashIndex.count) {
        return -1;
    }

    char expected[DOWNLOAD_NAME_LENGTH];
    formatLogName(number, expected);
    return memcmp(expected, name, DOWNLOAD_NAME_LENGTH) == 0 ? (int)number - 1 : -1;
}

#endif // USE_FLASHFS

/* ---------------------------------------------------------------------------------------------------- storage */

static logStorage_e selectStorage(void)
{
    const uint8_t device = blackboxConfig()->device;
    UNUSED(device);

#ifdef USE_FLASHFS
    if (device == BLACKBOX_DEVICE_FLASH && flashfsIsSupported()) {
        return STORAGE_FLASH;
    }
#endif
#ifdef USE_SDCARD
    if (sdcardConfig()->mode != SDCARD_MODE_NONE && (device == BLACKBOX_DEVICE_SDCARD || sdcard_isInserted())) {
        return STORAGE_SDCARD;
    }
#endif
#ifdef USE_FLASHFS
    if (flashfsIsSupported()) {
        return STORAGE_FLASH;
    }
#endif
    return STORAGE_NONE;
}

static bool storageReady(logStorage_e storage)
{
    switch (storage) {
#ifdef USE_SDCARD
    case STORAGE_SDCARD:
        return sdFilesystemReady();
#endif
#ifdef USE_FLASHFS
    case STORAGE_FLASH:
        return flashfsIsSupported() && flashfsIsReady();
#endif
    default:
        return false;
    }
}

static bool isHoldingStorage(void)
{
#ifdef USE_SDCARD
    if (sdIsHoldingCard()) {
        return true;
    }
#endif
    return dl.logOpen;
}

// Never blocks. Returns true once everything is released.
static bool releaseStep(void)
{
    dl.logOpen = false;
    dl.carryLen = 0;

#ifdef USE_SDCARD
    return sdReleaseStep();
#else
    return true;
#endif
}

static logDownloadStatus_e releaseBlocking(timeMs_t deadline)
{
    while (!releaseStep()) {
#ifdef USE_SDCARD
        if (!pollUntil(deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
#else
        UNUSED(deadline);
#endif
    }
    return LOG_DOWNLOAD_OK;
}

static logDownloadStatus_e checkAccess(logStorage_e storage)
{
    if (dl.erase.phase != ERASE_IDLE) {
        return LOG_DOWNLOAD_BUSY;
    }

    if (!storageReady(storage)) {
        return LOG_DOWNLOAD_NOT_READY;
    }

    if (ARMING_FLAG(ARMED) || !blackboxMayEditConfig() || cmp32(millis(), dl.lockoutUntilMs) < 0) {
        return LOG_DOWNLOAD_BUSY;
    }

#ifdef USE_SDCARD
    if (storage == STORAGE_SDCARD && !dl.sd.active && !blackboxSDCardIsIdle()) {
        return LOG_DOWNLOAD_BUSY;
    }
#endif

    return LOG_DOWNLOAD_OK;
}

static logDownloadStatus_e beginRequest(timeMs_t deadline)
{
    const logStorage_e storage = selectStorage();
    if (storage != dl.storage) {
        // e.g. blackbox_device was changed: drop everything that belongs to the old storage
        const logDownloadStatus_e status = releaseBlocking(deadline);
        if (status != LOG_DOWNLOAD_OK) {
            return status;
        }
        dl.storage = storage;
    }

    const logDownloadStatus_e status = checkAccess(storage);
    if (status != LOG_DOWNLOAD_OK) {
        return status;
    }

    dl.lastAccessMs = millis();

#ifdef USE_SDCARD
    if (storage == STORAGE_SDCARD) {
        if (dl.sd.orphan && afatfs_fclose(dl.sd.orphan, NULL)) {
            dl.sd.orphan = NULL;
        }
        return sdAcquire(deadline);
    }
#endif

    return LOG_DOWNLOAD_OK;
}

static logDownloadStatus_e closeLog(timeMs_t deadline)
{
    UNUSED(deadline);

    dl.logOpen = false;
    dl.carryLen = 0;

#ifdef USE_SDCARD
    while (dl.sd.file && !afatfs_fclose(dl.sd.file, NULL)) {
        if (!pollUntil(deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
    }
    dl.sd.file = NULL;
#endif

    return LOG_DOWNLOAD_OK;
}

static logDownloadStatus_e openLog(const char *name, timeMs_t deadline)
{
    if (dl.logOpen && memcmp(dl.fileName, name, DOWNLOAD_NAME_LENGTH) == 0) {
        return LOG_DOWNLOAD_OK;
    }

    logDownloadStatus_e status = closeLog(deadline);
    if (status != LOG_DOWNLOAD_OK) {
        return status;
    }

    switch (dl.storage) {
#ifdef USE_SDCARD
    case STORAGE_SDCARD:
        dl.sd.file = sdOpenBlocking(name, "r", deadline, &status);
        if (!dl.sd.file) {
            return status;
        }
        break;
#endif
#ifdef USE_FLASHFS
    case STORAGE_FLASH: {
        if (!flashIndexUpdate(deadline)) {
            return LOG_DOWNLOAD_IN_PROGRESS;
        }
        const int index = flashFindLog(name);
        if (index < 0) {
            return LOG_DOWNLOAD_NOT_FOUND;
        }
        dl.flash.start = flashIndex.start[index];
        dl.flash.end = flashLogEnd(index);
        dl.flash.cursor = dl.flash.start;
        break;
    }
#endif
    default:
        return LOG_DOWNLOAD_NOT_READY;
    }

    dl.logOpen = true;
    memcpy(dl.fileName, name, DOWNLOAD_NAME_LENGTH);
    return LOG_DOWNLOAD_OK;
}

#ifdef USE_SDCARD
// Waits until no seek is pending on the file, and returns the cursor position.
static bool sdTellBlocking(uint32_t *position, timeMs_t deadline)
{
    while (!afatfs_ftell(dl.sd.file, position)) {
        if (!pollUntil(deadline)) {
            return false;
        }
    }
    return true;
}
#endif

static logDownloadStatus_e seekLog(uint32_t offset, timeMs_t deadline)
{
    UNUSED(deadline);

    if (dl.carryLen > 0) {
        if (dl.carryOffset == offset) {
            return LOG_DOWNLOAD_OK;
        }
        dl.carryLen = 0;
    }

    switch (dl.storage) {
#ifdef USE_SDCARD
    case STORAGE_SDCARD: {
        uint32_t position;
        if (!sdTellBlocking(&position, deadline)) {
            return LOG_DOWNLOAD_IO_ERROR;
        }

        if (position != offset) {
            // Backwards seeks walk the cluster chain from the start of the file, so avoid them where possible
            while (afatfs_fseek(dl.sd.file, offset, AFATFS_SEEK_SET) == AFATFS_OPERATION_FAILURE) {
                if (!pollUntil(deadline)) {
                    return LOG_DOWNLOAD_IO_ERROR;
                }
            }
            if (!sdTellBlocking(&position, deadline)) {
                return LOG_DOWNLOAD_IO_ERROR;
            }
        }
        return LOG_DOWNLOAD_OK;
    }
#endif
#ifdef USE_FLASHFS
    case STORAGE_FLASH:
        dl.flash.cursor = dl.flash.start + MIN(offset, dl.flash.end - dl.flash.start);
        return LOG_DOWNLOAD_OK;
#endif
    default:
        return LOG_DOWNLOAD_NOT_READY;
    }
}

/*
 * Reads up to len bytes at the current position. Returns the number of bytes read, which is only smaller than len
 * at the end of the file or on timeout, or -1 if nothing could be read before the timeout.
 */
static int readLog(uint8_t *buffer, int len, timeMs_t deadline)
{
    UNUSED(deadline);
    int bytesRead = 0;

    if (dl.carryLen > 0) {
        const int n = MIN(len, dl.carryLen);
        memcpy(buffer, dl.carry, n);
        memmove(dl.carry, dl.carry + n, dl.carryLen - n);
        dl.carryLen -= n;
        dl.carryOffset += n;
        bytesRead += n;
    }

    switch (dl.storage) {
#ifdef USE_SDCARD
    case STORAGE_SDCARD:
        while (bytesRead < len) {
            const uint32_t n = afatfs_fread(dl.sd.file, buffer + bytesRead, len - bytesRead);
            bytesRead += n;

            if (n == 0) {
                if (afatfs_feof(dl.sd.file)) {
                    break;
                }
                if (!pollUntil(deadline)) {
                    return bytesRead > 0 ? bytesRead : -1;
                }
            }
        }
        break;
#endif
#ifdef USE_FLASHFS
    case STORAGE_FLASH:
        while (bytesRead < len && dl.flash.cursor < dl.flash.end) {
            const int n = flashfsReadAbs(dl.flash.cursor, buffer + bytesRead, MIN((uint32_t)(len - bytesRead), dl.flash.end - dl.flash.cursor));
            if (n <= 0) {
                return bytesRead > 0 ? bytesRead : -1;
            }
            dl.flash.cursor += n;
            bytesRead += n;
        }
        break;
#endif
    default:
        break;
    }

    return bytesRead;
}

// Puts back bytes that were read from the log at offset but could not be sent.
static void unreadLog(const uint8_t *buffer, int len, uint32_t offset)
{
#ifdef USE_FLASHFS
    if (dl.storage == STORAGE_FLASH) {
        // Random access is cheap, just read them again
        dl.flash.cursor = dl.flash.start + offset;
        return;
    }
#endif

    // readLog() drains the carry buffer before it reads a full chunk, so it is empty here
    dl.carryLen = MIN(len, (int)sizeof(dl.carry));
    dl.carryOffset = offset;
    memcpy(dl.carry, buffer, dl.carryLen);
}

/* ---------------------------------------------------------------------------------------------------- erase */

static void eraseFinish(logDownloadStatus_e result)
{
    dl.erase.phase = ERASE_IDLE;
    dl.erase.result = result;
}

static uint8_t eraseProgress(void)
{
    switch (dl.erase.phase) {
    case ERASE_SDCARD_DELETE:
        return dl.erase.total ? MIN(dl.erase.deleted * 90 / dl.erase.total, 90) : 90;
    case ERASE_SDCARD_REBUILD:
        return 90;
    case ERASE_SDCARD_WAIT_READY:
        return 95;
    case ERASE_FLASH:
        return PROGRESS_UNKNOWN;
    default:
        return dl.erase.result == LOG_DOWNLOAD_OK ? 100 : 0;
    }
}

static logDownloadStatus_e eraseStart(timeMs_t deadline)
{
    logDownloadStatus_e status = beginRequest(deadline);
    if (status != LOG_DOWNLOAD_OK) {
        return status;
    }

    status = closeLog(deadline);
    if (status != LOG_DOWNLOAD_OK) {
        return status;
    }

    switch (dl.storage) {
#ifdef USE_SDCARD
    case STORAGE_SDCARD: {
        int count = 0;
        status = sdForEachFile(sdCountVisitor, &count, deadline);
        if (status != LOG_DOWNLOAD_OK) {
            return status;
        }
        dl.erase.total = count;
        dl.erase.deleted = 0;
        dl.erase.phase = ERASE_SDCARD_DELETE;
        break;
    }
#endif
#ifdef USE_FLASHFS
    case STORAGE_FLASH:
        flashfsEraseCompletely();
        flashIndexReset();
        dl.erase.phase = ERASE_FLASH;
        break;
#endif
    default:
        return LOG_DOWNLOAD_NOT_READY;
    }

    dl.erase.result = LOG_DOWNLOAD_OK;
    return LOG_DOWNLOAD_OK;
}

// Advances the erase. If blocking, waits for the storage until the deadline.
static void eraseContinue(bool blocking, timeMs_t deadline)
{
    UNUSED(blocking);
    UNUSED(deadline);

    switch (dl.erase.phase) {
#ifdef USE_FLASHFS
    case ERASE_FLASH:
        if (flashfsIsReady()) {
            eraseFinish(LOG_DOWNLOAD_OK);
        }
        break;
#endif
#ifdef USE_SDCARD
    case ERASE_SDCARD_DELETE:
        if (blocking) {
            sdEraseDeleteStep(deadline);
        }
        if (dl.erase.phase != ERASE_SDCARD_REBUILD) {
            break;
        }
        FALLTHROUGH;
    case ERASE_SDCARD_REBUILD:
    case ERASE_SDCARD_WAIT_READY:
        // Once started, this has to finish, so it is also driven by blackboxDownloadUpdate()
        do {
            if (dl.erase.phase == ERASE_SDCARD_REBUILD && sdReleaseStep()) {
                const afatfsOperationStatus_e status = afatfs_rebuildFreeFile();
                if (status == AFATFS_OPERATION_SUCCESS) {
                    dl.erase.phase = ERASE_SDCARD_WAIT_READY;
                } else if (status == AFATFS_OPERATION_FAILURE) {
                    eraseFinish(LOG_DOWNLOAD_IO_ERROR);
                }
            }
            if (dl.erase.phase == ERASE_SDCARD_WAIT_READY) {
                const afatfsFilesystemState_e state = afatfs_getFilesystemState();
                if (state == AFATFS_FILESYSTEM_STATE_READY) {
                    eraseFinish(LOG_DOWNLOAD_OK);
                } else if (state == AFATFS_FILESYSTEM_STATE_FATAL) {
                    eraseFinish(LOG_DOWNLOAD_IO_ERROR);
                }
            }
        } while (blocking && dl.erase.phase != ERASE_IDLE && pollUntil(deadline));
        break;
#endif
    default:
        break;
    }
}

/* ---------------------------------------------------------------------------------------------------- MSP */

void blackboxDownloadMspList(sbuf_t *src, sbuf_t *dst)
{
    const uint16_t startIndex = sbufBytesRemaining(src) >= 2 ? sbufReadU16(src) : 0;
    const timeMs_t deadline = millis() + DOWNLOAD_TIMEOUT_MS;

    uint8_t *header = sbufPtr(dst);
    sbufWriteU8(dst, LOG_DOWNLOAD_OK);  // status
    sbufWriteU16(dst, startIndex);
    sbufWriteU8(dst, 0);                // count
    sbufWriteU8(dst, 0);                // flags
    sbufWriteU8(dst, 0);                // progress
    uint8_t *entries = sbufPtr(dst);

    logDownloadStatus_e status = beginRequest(deadline);
    const int maxCount = MIN(sbufBytesRemaining(dst) / DOWNLOAD_LIST_ENTRY_SIZE, 255);
    int count = 0;
    uint8_t flags = 0;
    uint8_t progress = 0;

    if (status == LOG_DOWNLOAD_OK) {
        switch (dl.storage) {
#ifdef USE_SDCARD
        case STORAGE_SDCARD: {
            sdListContext_t list = { .dst = dst, .startIndex = startIndex, .maxCount = maxCount };
            status = sdForEachFile(sdListVisitor, &list, deadline);
            count = list.count;
            flags |= list.more ? LIST_FLAG_MORE : 0;
            break;
        }
#endif
#ifdef USE_FLASHFS
        case STORAGE_FLASH:
            flags |= LIST_FLAG_FLASH;
            if (!flashIndexUpdate(deadline)) {
                status = LOG_DOWNLOAD_IN_PROGRESS;
                progress = flashIndexProgress();
                break;
            }
            progress = 100;
            for (int i = startIndex; i < flashIndex.count; i++) {
                if (count >= maxCount) {
                    flags |= LIST_FLAG_MORE;
                    break;
                }
                char name[DOWNLOAD_NAME_LENGTH];
                formatLogName(i + 1, name);
                sbufWriteData(dst, name, DOWNLOAD_NAME_LENGTH);
                sbufWriteU32(dst, flashLogEnd(i) - flashIndex.start[i]);
                sbufWriteU16(dst, 0);
                sbufWriteU16(dst, 0);
                count++;
            }
            break;
#endif
        default:
            status = LOG_DOWNLOAD_NOT_READY;
            break;
        }
    }

    if (status != LOG_DOWNLOAD_OK) {
        // Discard a partial page
        sbufAdvance(dst, -(int)(sbufPtr(dst) - entries));
        count = 0;
        flags &= ~LIST_FLAG_MORE;
    }

    header[0] = status;
    header[3] = count;
    header[4] = flags;
    header[5] = progress;
}

static logDownloadStatus_e serializeRead(sbuf_t *dst, uint32_t offset, uint16_t size, bool allowCompression, timeMs_t deadline)
{
    const int bytesRemainingInBuf = sbufBytesRemaining(dst) - DOWNLOAD_READ_HEADER_SIZE;
    if (bytesRemainingInBuf <= 0) {
        return LOG_DOWNLOAD_BAD_REQUEST;
    }
    const uint16_t readLen = MIN(size, bytesRemainingInBuf);

#ifdef USE_HUFFMAN
    const bool compress = allowCompression && readLen >= DOWNLOAD_HUFFMAN_MIN_BUFFER;
#else
    const bool compress = false;
    UNUSED(allowCompression);
#endif

    if (!compress) {
        uint8_t *dataPtr = sbufPtr(dst) + sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint8_t);
        const int bytesRead = readLog(dataPtr, readLen, deadline);
        if (bytesRead < 0) {
            return LOG_DOWNLOAD_IO_ERROR;
        }

        sbufWriteU32(dst, offset);
        sbufWriteU16(dst, bytesRead);
        sbufWriteU8(dst, COMPRESSION_NONE);
        sbufAdvance(dst, bytesRead);
        return LOG_DOWNLOAD_OK;
    }

#ifdef USE_HUFFMAN
    // Same scheme as the MSP_DATAFLASH_READ reply
    __attribute__ ((aligned(32))) uint8_t chunk[DOWNLOAD_HUFFMAN_CHUNK_SIZE];

    huffmanState_t state = {
        .bytesWritten = 0,
        .outByte = sbufPtr(dst) + sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint8_t) + HUFFMAN_INFO_SIZE,
        .outBufLen = readLen,
        .outBit = 0x80,
    };
    *state.outByte = 0;

    uint16_t bytesReadTotal = 0;
    bool readFailed = false;
    bool overflowed = false;
    while (state.bytesWritten < state.outBufLen) {
        const int bytesRead = readLog(chunk, sizeof(chunk), deadline);
        if (bytesRead <= 0) {
            readFailed = bytesRead < 0;
            break;
        }

        if (huffmanEncodeBufStreaming(&state, chunk, bytesRead, huffmanTable) == -1) {
            // Output buffer full, keep the chunk for the next request instead of seeking back
            unreadLog(chunk, bytesRead, offset + bytesReadTotal);
            overflowed = true;
            break;
        }

        bytesReadTotal += bytesRead;

        if (bytesRead < (int)sizeof(chunk)) {
            // End of file or timeout
            break;
        }
    }

    if (bytesReadTotal == 0) {
        if (readFailed || overflowed) {
            return LOG_DOWNLOAD_IO_ERROR;
        }
        // End of file
        sbufWriteU32(dst, offset);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, COMPRESSION_NONE);
        return LOG_DOWNLOAD_OK;
    }

    if (state.outBit != 0x80) {
        ++state.bytesWritten;
    }

    sbufWriteU32(dst, offset);
    sbufWriteU16(dst, HUFFMAN_INFO_SIZE + state.bytesWritten);
    sbufWriteU8(dst, COMPRESSION_HUFFMAN);
    sbufWriteU16(dst, bytesReadTotal);
    sbufAdvance(dst, state.bytesWritten);
#endif

    return LOG_DOWNLOAD_OK;
}

static bool readFileName(sbuf_t *src, char *name)
{
    if (sbufBytesRemaining(src) < DOWNLOAD_NAME_LENGTH) {
        return false;
    }
    sbufReadData(src, name, DOWNLOAD_NAME_LENGTH);
    sbufAdvance(src, DOWNLOAD_NAME_LENGTH);

    // Only plain 8.3 names in the log directory
    if (name[0] == '\0' || name[0] == '.') {
        return false;
    }
    for (int i = 0; i < DOWNLOAD_NAME_LENGTH && name[i] != '\0'; i++) {
        if (name[i] == '/' || name[i] == '\\' || name[i] == ' ') {
            return false;
        }
    }
    return true;
}

void blackboxDownloadMspRead(sbuf_t *src, sbuf_t *dst)
{
    const timeMs_t deadline = millis() + DOWNLOAD_TIMEOUT_MS;
    uint8_t *statusPtr = sbufPtr(dst);
    sbufWriteU8(dst, LOG_DOWNLOAD_OK);

    // One spare byte so the name is always NUL-terminated
    char name[DOWNLOAD_NAME_LENGTH + 1] = { 0 };
    if (!readFileName(src, name) || sbufBytesRemaining(src) < 6) {
        *statusPtr = LOG_DOWNLOAD_BAD_REQUEST;
        return;
    }
    const uint32_t offset = sbufReadU32(src);
    const uint16_t size = sbufReadU16(src);
    const bool allowCompression = sbufBytesRemaining(src) >= 1 ? sbufReadU8(src) : false;
    if (size == 0) {
        *statusPtr = LOG_DOWNLOAD_BAD_REQUEST;
        return;
    }

    logDownloadStatus_e status = beginRequest(deadline);

    if (status == LOG_DOWNLOAD_OK) {
        status = openLog(name, deadline);
    }

    if (status == LOG_DOWNLOAD_OK) {
        status = seekLog(offset, deadline);
    }

    if (status == LOG_DOWNLOAD_OK) {
        status = serializeRead(dst, offset, size, allowCompression, deadline);
    }

    *statusPtr = status;
}

void blackboxDownloadMspClose(sbuf_t *dst)
{
    const timeMs_t deadline = millis() + DOWNLOAD_TIMEOUT_MS;

    // If it times out, blackboxDownloadUpdate() finishes the job
    sbufWriteU8(dst, releaseBlocking(deadline));
}

void blackboxDownloadMspEraseAll(sbuf_t *src, sbuf_t *dst)
{
    const bool start = sbufBytesRemaining(src) >= 1 ? sbufReadU8(src) : false;
    const timeMs_t deadline = millis() + DOWNLOAD_TIMEOUT_MS;

    if (dl.erase.phase == ERASE_IDLE && start) {
        const logDownloadStatus_e status = eraseStart(deadline);
        if (status != LOG_DOWNLOAD_OK) {
            sbufWriteU8(dst, status);
            sbufWriteU8(dst, 0);
            return;
        }
    }

    if (dl.erase.phase != ERASE_IDLE) {
        dl.lastAccessMs = millis();
        eraseContinue(true, deadline);
    }

    sbufWriteU8(dst, dl.erase.phase != ERASE_IDLE ? LOG_DOWNLOAD_IN_PROGRESS : dl.erase.result);
    sbufWriteU8(dst, eraseProgress());
}

/*
 * Call periodically. Finishes erasing, and releases the SD card when the client went away.
 */
void blackboxDownloadUpdate(timeMs_t currentTimeMs)
{
    const bool idle = cmp32(currentTimeMs, dl.lastAccessMs) > DOWNLOAD_IDLE_TIMEOUT_MS;

    if (dl.erase.phase == ERASE_SDCARD_DELETE) {
        if (idle) {
            // Client went away, keep the files that are left
            eraseFinish(LOG_DOWNLOAD_IO_ERROR);
        }
    } else if (dl.erase.phase != ERASE_IDLE) {
        eraseContinue(false, currentTimeMs);
    }

    if (dl.erase.phase == ERASE_IDLE && idle && isHoldingStorage()) {
        releaseStep();
    }
}

/*
 * Called by blackbox before it starts a log. Never blocks. Returns true once the storage is free for logging; until
 * then, keep calling it and don't start the log.
 */
bool blackboxDownloadReleaseForLogging(void)
{
    dl.lockoutUntilMs = millis() + DOWNLOAD_LOCKOUT_MS;

    if (dl.erase.phase == ERASE_SDCARD_DELETE) {
        // Logging wins, keep the files that are left
        eraseFinish(LOG_DOWNLOAD_BUSY);
    } else if (dl.erase.phase != ERASE_IDLE) {
        if (dl.erase.phase == ERASE_SDCARD_WAIT_READY || dl.erase.phase == ERASE_FLASH) {
            // Cheap, only checks whether it has finished
            eraseContinue(false, millis());
        }
        if (dl.erase.phase != ERASE_IDLE) {
            // An erasing flash or a filesystem being re-initialised can't be logged to anyway
            return false;
        }
    }

    if (!isHoldingStorage()) {
        return true;
    }

    return releaseStep();
}

#endif // USE_MSP_LOG_DOWNLOAD
