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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "build/debug.h"

// Debugging code that become useful when output bandwidth saturation is suspected.
// Set debug_mode = BLACKBOX_OUTPUT to see following debug values.
//
// 0: Average output bandwidth in last 100ms
// 1: Maximum hold of above.
// 2: Bytes dropped due to output buffer full.
//
// Note that bandwidth usage slightly increases when DEBUG_BB_OUTPUT is enabled,
// as output will include debug variables themselves.

#define DEBUG_BB_OUTPUT

#include "blackbox.h"
#include "blackbox_io.h"

#include "common/maths.h"

#include "flight/pid.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/serial.h"

#include "msp/msp_serial.h"

#ifdef USE_SDCARD
#include "drivers/sdcard.h"
#endif

#if defined(SITL) || defined(MOCKUP)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <linux/limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif

#define BLACKBOX_SERIAL_PORT_MODE MODE_TX

// How many bytes can we transmit per loop iteration when writing headers?
static uint8_t blackboxMaxHeaderBytesPerIteration;

// How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog?
int32_t blackboxHeaderBudget;

static serialPort_t *blackboxPort = NULL;
static portSharing_e blackboxPortSharing;

#ifdef USE_SDCARD

static struct {
    afatfsFilePtr_t logFile;
    afatfsFilePtr_t logDirectory;
    afatfsFinder_t logDirectoryFinder;
    int32_t largestLogFileNumber;

    enum {
        BLACKBOX_SDCARD_INITIAL,
        BLACKBOX_SDCARD_WAITING,
        BLACKBOX_SDCARD_ENUMERATE_FILES,
        BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY,
        BLACKBOX_SDCARD_READY_TO_CREATE_LOG,
        BLACKBOX_SDCARD_READY_TO_LOG
    } state;
} blackboxSDCard;

#define LOGFILE_PREFIX "LOG"
#define LOGFILE_SUFFIX "BFL"

#endif // USE_SDCARD

#if defined(SITL) || defined(MOCKUP)

#define LOGFILE_SITL_BUFFER_MAX 4096

static struct {
    int fd;
    char filename[PATH_MAX];
    uint8_t buf[LOGFILE_SITL_BUFFER_MAX];
    uint16_t buf_fill;
    int32_t largestLogFileNumber;
    enum {
        BLACKBOX_SITL_INITIAL,
        BLACKBOX_SITL_WAITING,
        BLACKBOX_SITL_ENUMERATE_FILES,
        BLACKBOX_SITL_CHANGE_INTO_LOG_DIRECTORY,
        BLACKBOX_SITL_READY_TO_CREATE_LOG,
        BLACKBOX_SITL_READY_TO_LOG
    } state;
} blackboxSITLFile;

#define LOGFILE_DIR "./logs"
#define LOGFILE_PREFIX "LOG"
#define LOGFILE_SUFFIX ".BFL"

#define LOGFILE_NAME_LENGTH 12  // LOGxxxxx.BFL
#define LOGFILE_MAX_NUMBER 99999


void blackboxSITLBufferWrite(const uint8_t *new, const int len)
{
    if (blackboxSITLFile.buf_fill + len > LOGFILE_SITL_BUFFER_MAX) {
        // new buffer will not fit --> write out current buffer, then write 
        // LOGFILE_SITL_BUFFER_MAX bytes from new until new is smaller than
        // LOGFILE_SITL_BUFFER_MAX. This is not the optimal way, but oh well
        int bytes_written = write(
            blackboxSITLFile.fd, 
            blackboxSITLFile.buf,
            blackboxSITLFile.buf_fill);
        UNUSED(bytes_written);
        blackboxSITLFile.buf_fill = 0;

        int togo = len;
        const uint8_t *mutable = new;
        while (togo >= LOGFILE_SITL_BUFFER_MAX) {
            int bytes_written = write(
                blackboxSITLFile.fd,
                mutable,
                LOGFILE_SITL_BUFFER_MAX);
            UNUSED(bytes_written);
            togo -= LOGFILE_SITL_BUFFER_MAX;
            mutable += LOGFILE_SITL_BUFFER_MAX;
        }

        // write the leftovers in the buffer
        if (togo > 0) {
            memcpy(blackboxSITLFile.buf, mutable, togo);
            blackboxSITLFile.buf_fill = togo;
        }
    } else {
        // buffer fits, just append
        memcpy(blackboxSITLFile.buf + blackboxSITLFile.buf_fill, new, len);
        blackboxSITLFile.buf_fill += len;
    }
}

#endif

void blackboxOpen(void)
{
    serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
    if (sharedBlackboxAndMspPort) {
        mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
    }
}

#ifdef DEBUG_BB_OUTPUT
static uint32_t bbBits;
static timeMs_t bbLastclearMs;
static uint16_t bbRateMax;
static uint32_t bbDrops;
#endif

void blackboxWrite(uint8_t value)
{
#ifdef DEBUG_BB_OUTPUT
    bbBits += 8;
#endif

    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        flashfsWriteByte(value); // Write byte asynchronously
        break;
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        afatfs_fputc(blackboxSDCard.logFile, value);
        break;
#endif
#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        blackboxSITLBufferWrite(&value, 1);
        break;
#endif
    case BLACKBOX_DEVICE_SERIAL:
    default:
        {
            int txBytesFree = serialTxBytesFree(blackboxPort);

#ifdef DEBUG_BB_OUTPUT
            bbBits += 2;
            DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 3, txBytesFree);
#endif

            if (txBytesFree == 0) {
#ifdef DEBUG_BB_OUTPUT
                ++bbDrops;
                DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 2, bbDrops);
#endif
                return;
            }
            serialWrite(blackboxPort, value);
        }
        break;
    }

#ifdef DEBUG_BB_OUTPUT
    timeMs_t now = millis();

    if (now > bbLastclearMs + 100) {  // Debug log every 100[msec]
        uint16_t bbRate = ((bbBits * 10 + 5) / (now - bbLastclearMs)) / 10; // In unit of [Kbps]
        DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 0, bbRate);
        if (bbRate > bbRateMax) {
            bbRateMax = bbRate;
            DEBUG_SET(DEBUG_BLACKBOX_OUTPUT, 1, bbRateMax);
        }
        bbLastclearMs = now;
        bbBits = 0;
    }
#endif
}

// Print the null-terminated string 's' to the blackbox device and return the number of bytes written
int blackboxWriteString(const char *s)
{
    int length;
    const uint8_t *pos;

    switch (blackboxConfig()->device) {

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        length = strlen(s);
        flashfsWrite((const uint8_t*) s, length, false); // Write asynchronously
        break;
#endif // USE_FLASHFS

#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        length = strlen(s);
        afatfs_fwrite(blackboxSDCard.logFile, (const uint8_t*) s, length); // Ignore failures due to buffers filling up
        break;
#endif // USE_SDCARD

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        length = strlen(s);
        blackboxSITLBufferWrite((const uint8_t*) s, length);
        break;
#endif

    case BLACKBOX_DEVICE_SERIAL:
    default:
        pos = (uint8_t*) s;
        while (*pos) {
            blackboxWrite(*pos);
            pos++;
        }

        length = pos - (uint8_t*) s;
        break;
    }

    return length;
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Intended to be called regularly for the blackbox device to perform housekeeping.
 */
void blackboxDeviceFlush(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
        /*
         * This is our only output device which requires us to call flush() in order for it to write anything. The other
         * devices will progressively write in the background without Blackbox calling anything.
         */
    case BLACKBOX_DEVICE_FLASH:
        flashfsFlushAsync(false);
        break;
#endif // USE_FLASHFS

    default:
        ;
    }
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Returns true if all data has been written to the device.
 */
bool blackboxDeviceFlushForce(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        // Nothing to speed up flushing on serial, as serial is continuously being drained out of its buffer
        return isSerialTransmitBufferEmpty(blackboxPort);

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        return flashfsFlushAsync(true);
#endif // USE_FLASHFS

#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        // SD card will flush itself without us calling it, but we need to call flush manually in order to check
        // if it's done yet or not!
        // However the "flush" only queues one dirty sector each time and the process is asynchronous. So after
        // the last dirty sector is queued the flush returns true even though the sector may not actually have
        // been physically written to the SD card yet.
        return afatfs_flush();
#endif // USE_SDCARD

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL: {
        // first, write out and reset buffer
        int bytes_written = write(
            blackboxSITLFile.fd, 
            blackboxSITLFile.buf,
            blackboxSITLFile.buf_fill);
        UNUSED(bytes_written);
        blackboxSITLFile.buf_fill = 0;

        return fsync(blackboxSITLFile.fd) != -1;
    }
#endif

    default:
        return false;
    }
}

// Flush the blackbox device and only return true if sync is actually complete.
// Primarily to ensure the async operations of SD card sector writes complete thus freeing the cache entries.
bool blackboxDeviceFlushForceComplete(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (afatfs_sectorCacheInSync()) {
            return true;
        } else {
            blackboxDeviceFlushForce();
            return false;
        }
#endif // USE_SDCARD

    default:
        return blackboxDeviceFlushForce();
    }
}

/**
 * Attempt to open the logging device. Returns true if successful.
 */
bool blackboxDeviceOpen(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        {
            const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_BLACKBOX);
            baudRate_e baudRateIndex;
            portOptions_e portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED;

            if (!portConfig) {
                return false;
            }

            blackboxPortSharing = determinePortSharing(portConfig, FUNCTION_BLACKBOX);
            baudRateIndex = portConfig->blackbox_baudrateIndex;

            if (baudRates[baudRateIndex] == 230400) {
                /*
                 * OpenLog's 230400 baud rate is very inaccurate, so it requires a larger inter-character gap in
                 * order to maintain synchronization.
                 */
                portOptions |= SERIAL_STOPBITS_2;
            } else {
                portOptions |= SERIAL_STOPBITS_1;
            }

            blackboxPort = openSerialPort(portConfig->identifier, FUNCTION_BLACKBOX, NULL, NULL, baudRates[baudRateIndex],
                BLACKBOX_SERIAL_PORT_MODE, portOptions);

            /*
             * The slowest MicroSD cards have a write latency approaching 400ms. The OpenLog's buffer is about 900
             * bytes. In order for its buffer to be able to absorb this latency we must write slower than 6000 B/s.
             *
             * The OpenLager has a 125KB buffer for when the the MicroSD card is busy, so when the user configures
             * high baud rates, assume the OpenLager is in use and so there is no need to constrain the writes.
             *
             * In all other cases, constrain the writes as follows:
             *
             *     Bytes per loop iteration = floor((looptime_ns / 1000000.0) * 6000)
             *                              = floor((looptime_ns * 6000) / 1000000.0)
             *                              = floor((looptime_ns * 3) / 500.0)
             *                              = (looptime_ns * 3) / 500
             */


            switch (baudRateIndex) {
            case BAUD_1000000:
            case BAUD_1500000:
            case BAUD_2000000:
            case BAUD_2470000:
                // assume OpenLager in use, so do not constrain writes
                blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
                break;
            default:
                blackboxMaxHeaderBytesPerIteration = constrain((targetPidLooptime * 3) / 500, 1, BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION);
                break;
            };

            return blackboxPort != NULL;
        }
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        if (!flashfsIsSupported() || isBlackboxDeviceFull()) {
            return false;
        }

        blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;

        return true;
        break;
#endif // USE_FLASHFS
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL || afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_UNKNOWN || afatfs_isFull()) {
            return false;
        }

        blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;

        return true;
        break;
#endif // USE_SDCARD
#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL: {
        // create dir
        struct stat st = { 0 };
        if (stat(LOGFILE_DIR, &st) == -1) {
            // directory doesnt exist yet, create it, and false if error
            if (mkdir(LOGFILE_DIR, 0755) == -1) {
                return false;
            }
        }

        // open log file directory
        DIR *dir;
        struct dirent *entry;
        int maxLogNumber = 0;

        dir = opendir(LOGFILE_DIR);
        if (!dir) {
            return false;
        }

        // go over each file and find the one matching "LOGxxxxx.BFL" with the highest number xxxxx
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG && 
                strncmp(entry->d_name, LOGFILE_PREFIX, strlen(LOGFILE_PREFIX)) == 0 &&
                strcmp(entry->d_name + LOGFILE_NAME_LENGTH - strlen(LOGFILE_SUFFIX), LOGFILE_SUFFIX) == 0) {
                    int number = atoi(entry->d_name + strlen(LOGFILE_PREFIX));
                    maxLogNumber = MAX(maxLogNumber, number);
                }
        }

        closedir(dir);

        if (maxLogNumber == LOGFILE_MAX_NUMBER) {
            return false;
        }

        snprintf(blackboxSITLFile.filename, sizeof(blackboxSITLFile.filename), "%s/%s%05d%s", LOGFILE_DIR, LOGFILE_PREFIX, maxLogNumber + 1, LOGFILE_SUFFIX);

        blackboxSITLFile.fd = open(blackboxSITLFile.filename, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
        if (blackboxSITLFile.fd != -1) {
            blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
            return true;
        } else {
            return false;
        }
        break;
    }
#endif
    default:
        return false;
    }
}

/**
 * Erase all blackbox logs
 */
#ifdef USE_FLASHFS
void blackboxEraseAll(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_FLASH:
        /* Stop the recorder as if blackbox_mode = ALWAYS it will attempt to resume writing after
         * the erase and leave a corrupted first log.
         * Possible enhancement here is to restart logging after erase.
         */
        blackboxInit();
        flashfsEraseCompletely();
        break;
    default:
        //not supported
        break;
    }
}

/**
 * Check to see if erasing is done
 */
bool isBlackboxErased(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_FLASH:
        return flashfsIsReady();
        break;
    default:
    //not supported
        return true;
        break;
    }
}
#endif

/**
 * Close the Blackbox logging device.
 */
void blackboxDeviceClose(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        // Can immediately close without attempting to flush any remaining data.
        // Since the serial port could be shared with other processes, we have to give it back here
        closeSerialPort(blackboxPort);
        blackboxPort = NULL;

        /*
         * Normally this would be handled by mw.c, but since we take an unknown amount
         * of time to shut down asynchronously, we're the only ones that know when to call it.
         */
        if (blackboxPortSharing == PORTSHARING_SHARED) {
            mspSerialAllocatePorts();
        }
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        // Some flash device, e.g., NAND devices, require explicit close to flush internally buffered data.
        flashfsClose();
        break;
#endif

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        close(blackboxSITLFile.fd);
        break;
#endif

    default:
        ;
    }
}

#ifdef USE_SDCARD

static void blackboxLogDirCreated(afatfsFilePtr_t directory)
{
    if (directory) {
        blackboxSDCard.logDirectory = directory;

        afatfs_findFirst(blackboxSDCard.logDirectory, &blackboxSDCard.logDirectoryFinder);

        blackboxSDCard.state = BLACKBOX_SDCARD_ENUMERATE_FILES;
    } else {
        // Retry
        blackboxSDCard.state = BLACKBOX_SDCARD_INITIAL;
    }
}

static void blackboxLogFileCreated(afatfsFilePtr_t file)
{
    if (file) {
        blackboxSDCard.logFile = file;

        blackboxSDCard.largestLogFileNumber++;

        blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_LOG;
    } else {
        // Retry
        blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_CREATE_LOG;
    }
}

static void blackboxCreateLogFile(void)
{
    int32_t remainder = blackboxSDCard.largestLogFileNumber + 1;

    char filename[] = LOGFILE_PREFIX "00000." LOGFILE_SUFFIX;

    for (int i = 7; i >= 3; i--) {
        filename[i] = (remainder % 10) + '0';
        remainder /= 10;
    }

    blackboxSDCard.state = BLACKBOX_SDCARD_WAITING;

    afatfs_fopen(filename, "as", blackboxLogFileCreated);
}

/**
 * Begin a new log on the SDCard.
 *
 * Keep calling until the function returns true (open is complete).
 */
static bool blackboxSDCardBeginLog(void)
{
    fatDirectoryEntry_t *directoryEntry;

    doMore:
    switch (blackboxSDCard.state) {
    case BLACKBOX_SDCARD_INITIAL:
        if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY) {
            blackboxSDCard.state = BLACKBOX_SDCARD_WAITING;

            afatfs_mkdir("logs", blackboxLogDirCreated);
        }
        break;

    case BLACKBOX_SDCARD_WAITING:
        // Waiting for directory entry to be created
        break;

    case BLACKBOX_SDCARD_ENUMERATE_FILES:
        while (afatfs_findNext(blackboxSDCard.logDirectory, &blackboxSDCard.logDirectoryFinder, &directoryEntry) == AFATFS_OPERATION_SUCCESS) {
            if (directoryEntry && !fat_isDirectoryEntryTerminator(directoryEntry)) {
                // If this is a log file, parse the log number from the filename
                if (strncmp(directoryEntry->filename, LOGFILE_PREFIX, strlen(LOGFILE_PREFIX)) == 0
                    && strncmp(directoryEntry->filename + 8, LOGFILE_SUFFIX, strlen(LOGFILE_SUFFIX)) == 0) {
                    char logSequenceNumberString[6];

                    memcpy(logSequenceNumberString, directoryEntry->filename + 3, 5);
                    logSequenceNumberString[5] = '\0';

                    blackboxSDCard.largestLogFileNumber = MAX((int32_t)atoi(logSequenceNumberString), blackboxSDCard.largestLogFileNumber);
                }
            } else {
                // We're done checking all the files on the card, now we can create a new log file
                afatfs_findLast(blackboxSDCard.logDirectory);

                blackboxSDCard.state = BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY;
                goto doMore;
            }
        }
        break;

    case BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY:
        // Change into the log directory:
        if (afatfs_chdir(blackboxSDCard.logDirectory)) {
            // We no longer need our open handle on the log directory
            afatfs_fclose(blackboxSDCard.logDirectory, NULL);
            blackboxSDCard.logDirectory = NULL;

            blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_CREATE_LOG;
            goto doMore;
        }
        break;

    case BLACKBOX_SDCARD_READY_TO_CREATE_LOG:
        blackboxCreateLogFile();
        break;

    case BLACKBOX_SDCARD_READY_TO_LOG:
        return true; // Log has been created!
    }

    // Not finished init yet
    return false;
}

#endif // USE_SDCARD

/**
 * Begin a new log (for devices which support separations between the logs of multiple flights).
 *
 * Keep calling until the function returns true (open is complete).
 */
bool blackboxDeviceBeginLog(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        return blackboxSDCardBeginLog();
#endif // USE_SDCARD
#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        blackboxSITLFile.buf_fill = 0;
        return true;
#endif
    default:
        return true;
    }

}

/**
 * Terminate the current log (for devices which support separations between the logs of multiple flights).
 *
 * retainLog - Pass true if the log should be kept, or false if the log should be discarded (if supported).
 *
 * Keep calling until this returns true
 */
bool blackboxDeviceEndLog(bool retainLog)
{
#ifndef USE_SDCARD
    UNUSED(retainLog);
#endif

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        // Keep retrying until the close operation queues
        if (
            (retainLog && afatfs_fclose(blackboxSDCard.logFile, NULL))
            || (!retainLog && afatfs_funlink(blackboxSDCard.logFile, NULL))
        ) {
            // Don't bother waiting the for the close to complete, it's queued now and will complete eventually
            blackboxSDCard.logFile = NULL;
            blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_CREATE_LOG;
            return true;
        }
        return false;
#endif // USE_SDCARD
    default:
        return true;
    }
}

bool isBlackboxDeviceFull(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        return false;

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        return flashfsIsEOF();
#endif // USE_FLASHFS

#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        return afatfs_isFull();
#endif // USE_SDCARD

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
#endif

    default:
        return false;
    }
}

bool isBlackboxDeviceWorking(void)
{
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        return blackboxPort != NULL;

#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        return sdcard_isInserted() && sdcard_isFunctional() && (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY);
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        return flashfsIsReady();
#endif

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        return blackboxSITLFile.fd >= -1;
#endif
    default:
        return false;
    }
}

int32_t blackboxGetLogNumber(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        return blackboxSDCard.largestLogFileNumber;
#endif

    default:
        return -1;
    }
}

/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
void blackboxReplenishHeaderBudget(void)
{
    int32_t freeSpace;

    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        freeSpace = serialTxBytesFree(blackboxPort);
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        freeSpace = flashfsGetWriteBufferFreeSpace();
        break;
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        freeSpace = afatfs_getFreeBufferSpace();
        break;
#endif
#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        freeSpace = 0x3FF; // assume 1023 always. should be fine
        break;
#endif
    default:
        freeSpace = 0;
    }
    blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
}

/**
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that Cleanflight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }

    // Handle failure:
    switch (blackboxConfig()->device) {
    case BLACKBOX_DEVICE_SERIAL:
        /*
         * One byte of the tx buffer isn't available for user data (due to its circular list implementation),
         * hence the -1. Note that the USB VCP implementation doesn't use a buffer and has txBufferSize set to zero.
         */
        if (blackboxPort->txBufferSize && bytes > (int32_t) blackboxPort->txBufferSize - 1) {
            return BLACKBOX_RESERVE_PERMANENT_FAILURE;
        }
        return BLACKBOX_RESERVE_TEMPORARY_FAILURE;

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        if (bytes > (int32_t) flashfsGetWriteBufferSize()) {
            return BLACKBOX_RESERVE_PERMANENT_FAILURE;
        }

        if (bytes > (int32_t) flashfsGetWriteBufferFreeSpace()) {
            /*
             * The write doesn't currently fit in the buffer, so try to make room for it. Our flushing here means
             * that the Blackbox header writing code doesn't have to guess about the best time to ask flashfs to
             * flush, and doesn't stall waiting for a flush that would otherwise not automatically be called.
             */
            flashfsFlushAsync(true);
        }
        return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif // USE_FLASHFS

#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        // Assume that all writes will fit in the SDCard's buffers
        return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif // USE_SDCARD

#if defined(SITL) || defined(MOCKUP)
    case BLACKBOX_DEVICE_SITL:
        return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif // SITL

    default:
        return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }
}

int8_t blackboxGetLogFileNo(void)
{   
#ifdef USE_BLACKBOX
#ifdef USE_SDCARD
    // return current file number or -1 
    if (blackboxSDCard.state == BLACKBOX_SDCARD_READY_TO_LOG) {
        return blackboxSDCard.largestLogFileNumber;
    } else {
        return -1;
    }
#else
    // will be implemented later for flash based storage
    return -1;
#endif
#endif    
}
#endif // BLACKBOX
