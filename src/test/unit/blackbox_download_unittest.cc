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
 * Tests for blackbox/blackbox_download.c (listing, downloading and erasing blackbox logs over MSP).
 *
 * The SD card filesystem is replaced by a fake asyncfatfs that behaves like the real one in the ways that matter,
 * but adversarially: operations randomly don't complete until later polls, reads stop at sector boundaries or when a
 * "cluster lookup" is pending, seeks and opens complete asynchronously, and there are only 3 file handles. The flash
 * is a byte array. The MSP client side mirrors what the configurator does.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_download.h"

    #include "common/huffman.h"
    #include "common/maths.h"
    #include "common/streambuf.h"

    #include "fc/runtime_config.h"

    #include "io/asyncfatfs/asyncfatfs.h"
    #include "io/flashfs.h"

    #include "pg/sdcard.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/* ------------------------------------------------------------------------------------------------- stubs */

extern "C" {
    armingFlag_e armingFlags;
    sdcardConfig_t sdcardConfig_System;
    blackboxConfig_t blackboxConfig_System;

    static uint32_t nowMs = 1000;
    uint32_t millis(void) { return nowMs; }

    bool sdcard_isInserted(void) { return true; }
    bool sdcard_isFunctional(void) { return true; }

    static bool blackboxStopped = true;
    bool blackboxMayEditConfig(void) { return blackboxStopped; }

    static bool blackboxSdIdle = true;
    static int blackboxSdResets;
    bool blackboxSDCardIsIdle(void) { return blackboxSdIdle; }
    void blackboxSDCardResetWorkingDirectory(void) { blackboxSdResets++; }
}

/* ------------------------------------------------------------------------------------------------- fake asyncfatfs */

static int pBusy;               // percent chance that an operation is not ready yet
static bool stall;              // nothing completes while set
static int openDelayPolls;      // >= 0: fixed number of polls until an open completes, otherwise random
static afatfsFilesystemState_e fsState;

struct fakeFile_t {
    fatDirectoryEntry_t entry;
    std::vector<uint8_t> data;
};

static std::vector<fakeFile_t> dirEntries;

struct afatfsFile_t {
    bool used;
    bool isDir;
    int file;                   // index into dirEntries, -1 if none
    uint32_t cursor;
    int busyPolls;              // pending seek or cluster lookup
    uint32_t pendingCursor;
    int openPolls;              // pending open
    afatfsFileCallback_t openCallback;
    bool openOk;
    int unlinkPolls;            // pending unlink
    afatfsCallback_t unlinkCallback;
};

static afatfsFile_t handles[3];
static bool cwdIsLogs;
static int seekCount, backwardSeeks;
static int rebuildCalls, rebuildPollsLeft, initPollsLeft, initPollsAfterRebuild;

static int rnd(int n) { return rand() % n; }
static bool busy(void) { return stall || rnd(100) < pBusy; }

static int handlesInUse(void)
{
    int n = 0;
    for (auto &h : handles) {
        n += h.used;
    }
    return n;
}

static bool isDeleted(const fatDirectoryEntry_t &e) { return (uint8_t)e.filename[0] == FAT_DELETED_FILE_MARKER; }

static afatfsFilePtr_t allocHandle(void)
{
    for (auto &h : handles) {
        if (!h.used) {
            memset(&h, 0, sizeof(h));
            h.used = true;
            h.file = -1;
            return &h;
        }
    }
    return NULL;
}

static void startOpen(afatfsFilePtr_t h, bool ok, afatfsFileCallback_t callback)
{
    h->openCallback = callback;
    h->openOk = ok;
    h->openPolls = openDelayPolls >= 0 ? openDelayPolls : rnd(4);
    if (h->openPolls == 0 && !stall) {
        h->openCallback = NULL;
        if (!ok) {
            h->used = false;
        }
        callback(ok ? h : NULL);
    }
}

extern "C" {

bool afatfs_mkdir(const char *filename, afatfsFileCallback_t complete)
{
    EXPECT_STREQ("logs", filename);
    EXPECT_FALSE(cwdIsLogs) << "would create logs/logs";
    afatfsFilePtr_t h = allocHandle();
    if (!h) {
        complete(NULL);
        return false;
    }
    h->isDir = true;
    startOpen(h, true, complete);
    return true;
}

bool afatfs_fopen(const char *filename, const char *mode, afatfsFileCallback_t complete)
{
    EXPECT_TRUE(strcmp(mode, "r") == 0 || strcmp(mode, "r+") == 0);
    EXPECT_TRUE(cwdIsLogs);
    afatfsFilePtr_t h = allocHandle();
    if (!h) {
        complete(NULL);
        return false;
    }
    uint8_t fatName[FAT_FILENAME_LENGTH];
    fat_convertFilenameToFATStyle(filename, fatName);
    for (size_t i = 0; i < dirEntries.size(); i++) {
        const fatDirectoryEntry_t &e = dirEntries[i].entry;
        if (!isDeleted(e) && !(e.attrib & (FAT_FILE_ATTRIBUTE_VOLUME_ID | FAT_FILE_ATTRIBUTE_DIRECTORY))
            && memcmp(e.filename, fatName, FAT_FILENAME_LENGTH) == 0) {
            h->file = i;
        }
    }
    startOpen(h, h->file >= 0, complete);
    return true;
}

bool afatfs_fclose(afatfsFilePtr_t file, afatfsCallback_t callback)
{
    EXPECT_TRUE(file->used);
    EXPECT_EQ(0, file->openPolls);
    if (file->busyPolls > 0 || busy()) {
        return false;
    }
    file->used = false;
    if (callback) {
        callback();
    }
    return true;
}

bool afatfs_funlink(afatfsFilePtr_t file, afatfsCallback_t callback)
{
    EXPECT_TRUE(file->used && !file->isDir && file->file >= 0);
    if (file->busyPolls > 0 || busy()) {
        return false;
    }
    file->unlinkCallback = callback;
    file->unlinkPolls = 1 + rnd(5);
    return true;
}

bool afatfs_chdir(afatfsFilePtr_t dir)
{
    if (busy()) {
        return false;
    }
    if (dir) {
        EXPECT_TRUE(dir->used && dir->isDir);
        cwdIsLogs = true;
    } else {
        cwdIsLogs = false;
    }
    return true;
}

void afatfs_findFirst(afatfsFilePtr_t directory, afatfsFinder_t *finder)
{
    EXPECT_TRUE(directory->used && directory->isDir);
    finder->entryIndex = -1;
}

afatfsOperationStatus_e afatfs_findNext(afatfsFilePtr_t directory, afatfsFinder_t *finder, fatDirectoryEntry_t **dirEntry)
{
    EXPECT_TRUE(directory->used && directory->isDir);
    if (busy()) {
        return AFATFS_OPERATION_IN_PROGRESS;
    }
    static fatDirectoryEntry_t terminator;
    finder->entryIndex++;
    if (finder->entryIndex >= (int)dirEntries.size()) {
        memset(&terminator, 0, sizeof(terminator));
        *dirEntry = &terminator;
    } else {
        *dirEntry = &dirEntries[finder->entryIndex].entry;
    }
    return AFATFS_OPERATION_SUCCESS;
}

void afatfs_findLast(afatfsFilePtr_t directory) { UNUSED(directory); }

bool afatfs_feof(afatfsFilePtr_t file) { return file->cursor >= dirEntries[file->file].entry.fileSize; }

uint32_t afatfs_fread(afatfsFilePtr_t file, uint8_t *buffer, uint32_t len)
{
    EXPECT_TRUE(file->used && !file->isDir && file->openPolls == 0);
    if (file->busyPolls > 0) {
        return 0;
    }
    const fakeFile_t &f = dirEntries[file->file];
    const uint32_t size = f.entry.fileSize;
    if (file->cursor >= size) {
        return 0;
    }
    len = MIN(len, size - file->cursor);
    uint32_t bytesRead = 0;
    // Like the real thing: sector by sector, stopping when a sector isn't cached
    while (bytesRead < len) {
        if (busy()) {
            break;
        }
        const uint32_t n = MIN(len - bytesRead, 512 - file->cursor % 512);
        memcpy(buffer + bytesRead, f.data.data() + file->cursor, n);
        bytesRead += n;
        file->cursor += n;
        if (rnd(100) < 10) {
            // Moving to the next cluster needs a FAT lookup
            file->busyPolls = 1 + rnd(3);
            file->pendingCursor = file->cursor;
            break;
        }
    }
    return bytesRead;
}

afatfsOperationStatus_e afatfs_fseek(afatfsFilePtr_t file, int32_t offset, afatfsSeek_e whence)
{
    EXPECT_EQ(AFATFS_SEEK_SET, whence);
    if (file->busyPolls > 0) {
        return AFATFS_OPERATION_FAILURE;
    }
    seekCount++;
    const uint32_t target = MIN((uint32_t)offset, dirEntries[file->file].entry.fileSize);
    if (target < file->cursor) {
        backwardSeeks++;
    }
    if (busy()) {
        file->busyPolls = 1 + rnd(5);
        file->pendingCursor = target;
        file->cursor = 0;
        return AFATFS_OPERATION_IN_PROGRESS;
    }
    file->cursor = target;
    return AFATFS_OPERATION_SUCCESS;
}

bool afatfs_ftell(afatfsFilePtr_t file, uint32_t *position)
{
    if (file->busyPolls > 0) {
        return false;
    }
    *position = file->cursor;
    return true;
}

afatfsOperationStatus_e afatfs_rebuildFreeFile(void)
{
    EXPECT_EQ(0, handlesInUse()) << "no files may be open";
    EXPECT_FALSE(cwdIsLogs);
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, fsState);
    if (rebuildPollsLeft-- > 0) {
        return AFATFS_OPERATION_IN_PROGRESS;
    }
    rebuildCalls++;
    fsState = AFATFS_FILESYSTEM_STATE_INITIALIZATION;
    initPollsLeft = initPollsAfterRebuild;
    return AFATFS_OPERATION_SUCCESS;
}

void afatfs_poll(void)
{
    nowMs++;
    if (stall) {
        return;
    }
    if (fsState == AFATFS_FILESYSTEM_STATE_INITIALIZATION) {
        if (--initPollsLeft <= 0) {
            fsState = AFATFS_FILESYSTEM_STATE_READY;
        }
        return;
    }
    for (auto &h : handles) {
        if (!h.used) {
            continue;
        }
        if (h.busyPolls > 0 && --h.busyPolls == 0) {
            h.cursor = h.pendingCursor;
        }
        if (h.openCallback && h.openPolls > 0 && --h.openPolls == 0) {
            afatfsFileCallback_t callback = h.openCallback;
            h.openCallback = NULL;
            if (!h.openOk) {
                h.used = false;
            }
            callback(h.openOk ? &h : NULL);
        }
        if (h.unlinkPolls > 0 && --h.unlinkPolls == 0) {
            dirEntries[h.file].entry.filename[0] = (char)FAT_DELETED_FILE_MARKER;
            h.used = false;
            if (h.unlinkCallback) {
                h.unlinkCallback();
            }
        }
    }
}

afatfsFilesystemState_e afatfs_getFilesystemState(void) { return fsState; }

} // extern "C"

/* ------------------------------------------------------------------------------------------------- fake flash */

static std::vector<uint8_t> flashData;     // contents up to the used size
static const uint32_t flashSize = 8 * 1024 * 1024;
static bool flashErasing;

extern "C" {
    bool flashfsIsSupported(void) { return true; }
    bool flashfsIsReady(void) { return !flashErasing; }
    uint32_t flashfsGetOffset(void) { return flashData.size(); }
    void flashfsEraseCompletely(void) { flashErasing = true; flashData.clear(); }

    int flashfsReadAbs(uint32_t address, uint8_t *data, unsigned int len)
    {
        EXPECT_FALSE(flashErasing);
        nowMs++;    // reading takes time
        len = MIN(len, flashSize - address);
        for (unsigned i = 0; i < len; i++) {
            data[i] = address + i < flashData.size() ? flashData[address + i] : 0xff;
        }
        return len;
    }
}

/* ------------------------------------------------------------------------------------------------- MSP client */

#define OUTBUF_SIZE 4112
static uint8_t outBuf[OUTBUF_SIZE];
static uint8_t inBuf[192];
static int replyLen;

#define FLASH_HEADER "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"

struct listEntry_t {
    std::string name;
    uint32_t size;
    uint16_t date;
    uint16_t time;
};

struct listResult_t {
    std::vector<listEntry_t> entries;
    bool flash;
    int requests;
};

static sbuf_t replyReader(void) { sbuf_t b = { outBuf, outBuf + replyLen }; return b; }

static void callList(uint16_t startIndex, int bufSize)
{
    inBuf[0] = startIndex & 0xff;
    inBuf[1] = startIndex >> 8;
    sbuf_t src = { inBuf, inBuf + 2 };
    sbuf_t dst = { outBuf, outBuf + bufSize };
    blackboxDownloadMspList(&src, &dst);
    replyLen = dst.ptr - outBuf;
}

// Fetches the whole list, retrying I/O errors and waiting for the flash scan. Returns false on another status.
static bool listAll(listResult_t *result, int bufSize = OUTBUF_SIZE, uint8_t *failStatus = NULL)
{
    result->entries.clear();
    result->requests = 0;
    uint16_t start = 0;
    while (true) {
        EXPECT_LT(result->requests++, 2000);
        callList(start, bufSize);
        sbuf_t r = replyReader();
        const uint8_t status = sbufReadU8(&r);
        EXPECT_EQ(start, sbufReadU16(&r));
        const int count = sbufReadU8(&r);
        const uint8_t flags = sbufReadU8(&r);
        const uint8_t progress = sbufReadU8(&r);
        EXPECT_EQ(6 + count * 20, replyLen);
        result->flash = flags & 2;

        if (status == LOG_DOWNLOAD_IO_ERROR) {
            continue;
        }
        if (status == LOG_DOWNLOAD_IN_PROGRESS) {
            EXPECT_EQ(0, count);
            EXPECT_LT(progress, 100);
            continue;
        }
        if (status != LOG_DOWNLOAD_OK) {
            if (failStatus) {
                *failStatus = status;
            }
            EXPECT_EQ(0, count);
            return false;
        }

        for (int i = 0; i < count; i++) {
            listEntry_t e;
            char name[13] = { 0 };
            sbufReadData(&r, name, 12);
            sbufAdvance(&r, 12);
            e.name = name;
            e.size = sbufReadU32(&r);
            e.date = sbufReadU16(&r);
            e.time = sbufReadU16(&r);
            result->entries.push_back(e);
        }
        if (!(flags & 1)) {
            return true;
        }
        EXPECT_GT(count, 0);
        start += count;
    }
}

static int huffmanDecode(const uint8_t *in, int inLen, uint8_t *out, int count)
{
    int bit = 0;
    for (int n = 0; n < count; n++) {
        uint16_t code = 0;
        for (int len = 1; ; len++) {
            if (bit >= inLen * 8 || len > 16) {
                ADD_FAILURE() << "bad huffman data";
                return n;
            }
            code = (code << 1) | ((in[bit / 8] >> (7 - bit % 8)) & 1);
            bit++;
            int symbol = -1;
            for (int s = 0; s < 256; s++) {
                if (huffmanTable[s].codeLen == len && (huffmanTable[s].code >> (16 - len)) == code) {
                    symbol = s;
                    break;
                }
            }
            if (symbol >= 0) {
                out[n] = symbol;
                break;
            }
        }
    }
    return count;
}

static void callRead(const char *name, uint32_t offset, uint16_t size, bool compression, int bufSize)
{
    memset(inBuf, 0, sizeof(inBuf));
    strncpy((char *)inBuf, name, 12);
    uint8_t *p = inBuf + 12;
    p[0] = offset;
    p[1] = offset >> 8;
    p[2] = offset >> 16;
    p[3] = offset >> 24;
    p[4] = size;
    p[5] = size >> 8;
    p[6] = compression;
    sbuf_t src = { inBuf, inBuf + 19 };
    sbuf_t dst = { outBuf, outBuf + bufSize };
    blackboxDownloadMspRead(&src, &dst);
    replyLen = dst.ptr - outBuf;
}

// Reads one chunk, retrying I/O errors. Returns the number of bytes, or -status.
static int readChunk(const char *name, uint32_t offset, bool compression, uint8_t *data, int bufSize = OUTBUF_SIZE)
{
    for (int attempt = 0; attempt < 1000; attempt++) {
        callRead(name, offset, 4096, compression, bufSize);
        sbuf_t r = replyReader();
        const uint8_t status = sbufReadU8(&r);
        if (status == LOG_DOWNLOAD_IO_ERROR || status == LOG_DOWNLOAD_IN_PROGRESS) {
            continue;
        }
        if (status != LOG_DOWNLOAD_OK) {
            EXPECT_EQ(1, replyLen);
            return -(int)status;
        }
        EXPECT_EQ(offset, sbufReadU32(&r));
        const int dataSize = sbufReadU16(&r);
        const int compressionType = sbufReadU8(&r);
        EXPECT_EQ(8 + dataSize, replyLen);
        if (compressionType == 0) {
            memcpy(data, r.ptr, dataSize);
            return dataSize;
        }
        EXPECT_EQ(1, compressionType);
        EXPECT_TRUE(compression);
        const int count = sbufReadU16(&r);
        EXPECT_GT(count, 0);
        return huffmanDecode(r.ptr, dataSize - 2, data, count);
    }
    ADD_FAILURE() << "too many retries";
    return 0;
}

// Downloads a whole log. dropEvery simulates lost replies (the client asks for the same offset again).
static std::vector<uint8_t> download(const std::string &name, bool compression, int bufSize = OUTBUF_SIZE, int dropEvery = 0)
{
    std::vector<uint8_t> result;
    uint8_t chunk[8192];
    int requests = 0;
    while (true) {
        const int n = readChunk(name.c_str(), result.size(), compression, chunk, bufSize);
        EXPECT_GE(n, 0);
        if (n < 0) {
            break;
        }
        if (dropEvery && ++requests % dropEvery == 0) {
            continue;
        }
        if (n == 0) {
            break;
        }
        result.insert(result.end(), chunk, chunk + n);
        if (result.size() > 64 * 1024 * 1024) {
            ADD_FAILURE() << "runaway download";
            break;
        }
    }
    return result;
}

static uint8_t callClose(void)
{
    sbuf_t dst = { outBuf, outBuf + OUTBUF_SIZE };
    blackboxDownloadMspClose(&dst);
    replyLen = dst.ptr - outBuf;
    return outBuf[0];
}

static void callErase(bool start, uint8_t *status, uint8_t *progress)
{
    inBuf[0] = start;
    sbuf_t src = { inBuf, inBuf + 1 };
    sbuf_t dst = { outBuf, outBuf + OUTBUF_SIZE };
    blackboxDownloadMspEraseAll(&src, &dst);
    replyLen = dst.ptr - outBuf;
    EXPECT_EQ(2, replyLen);
    *status = outBuf[0];
    *progress = outBuf[1];
}

// Erases everything, polling like the configurator. Returns the final status.
static uint8_t eraseAll(int *polls = NULL)
{
    uint8_t status, progress;
    callErase(true, &status, &progress);
    int n = 0;
    uint8_t lastProgress = 0;
    while (status == LOG_DOWNLOAD_IN_PROGRESS) {
        if (progress != 255) {
            EXPECT_GE(progress, lastProgress);
            lastProgress = progress;
        }
        nowMs += 100;
        blackboxDownloadUpdate(nowMs);
        callErase(false, &status, &progress);
        EXPECT_LT(n++, 10000);
    }
    if (polls) {
        *polls = n;
    }
    if (status == LOG_DOWNLOAD_OK) {
        EXPECT_EQ(100, progress);
    }
    return status;
}

/* ------------------------------------------------------------------------------------------------- fixtures */

static void addEntry(const char *name, uint8_t attrib, uint32_t size, bool deleted = false)
{
    fakeFile_t f;
    memset(&f.entry, 0, sizeof(f.entry));
    fat_convertFilenameToFATStyle(name, (uint8_t *)f.entry.filename);
    f.entry.attrib = attrib;
    f.entry.fileSize = size;
    f.entry.lastWriteDate = FAT_MAKE_DATE(2026, 11, 18);
    f.entry.lastWriteTime = FAT_MAKE_TIME(12, 34, 56);
    if (deleted) {
        f.entry.filename[0] = (char)FAT_DELETED_FILE_MARKER;
    }
    f.data.resize(size);
    for (uint32_t i = 0; i < size; i++) {
        // Alternate between compressible (blackbox-like small values) and random data
        f.data[i] = (i / 3000) % 2 ? (uint8_t)rand() : (uint8_t)(rand() % 4);
    }
    dirEntries.push_back(f);
}

static const fakeFile_t *findFile(const std::string &name)
{
    uint8_t fatName[FAT_FILENAME_LENGTH];
    fat_convertFilenameToFATStyle(name.c_str(), fatName);
    for (auto &f : dirEntries) {
        if (!isDeleted(f.entry) && memcmp(f.entry.filename, fatName, FAT_FILENAME_LENGTH) == 0) {
            return &f;
        }
    }
    return NULL;
}

static void releaseAndReset(void)
{
    // Drop whatever the module still holds from an earlier test, and let lockouts expire
    nowMs += 10000;
    for (int i = 0; i < 10000; i++) {
        blackboxDownloadUpdate(nowMs);
        afatfs_poll();
        if (handlesInUse() == 0 && !cwdIsLogs) {
            break;
        }
    }
    nowMs += 10000;
}

class BlackboxDownloadSdTest : public ::testing::Test {
protected:
    static const int numLogs = 45;

    void SetUp() override
    {
        stall = false;
        pBusy = 30;
        openDelayPolls = -1;
        fsState = AFATFS_FILESYSTEM_STATE_READY;
        armingFlags = (armingFlag_e)0;
        blackboxStopped = true;
        blackboxSdIdle = true;
        sdcardConfig_System.mode = SDCARD_MODE_SPI;
        blackboxConfig_System.device = BLACKBOX_DEVICE_SDCARD;
        releaseAndReset();

        srand(1);
        memset(handles, 0, sizeof(handles));
        cwdIsLogs = false;
        seekCount = backwardSeeks = 0;
        rebuildCalls = rebuildPollsLeft = initPollsLeft = 0;
        initPollsAfterRebuild = 20;
        blackboxSdResets = 0;

        // Directory: ., .., a subdirectory, a long filename entry, a deleted file, a volume label, a file without
        // an extension and the logs, with sizes around sector and chunk boundaries
        dirEntries.clear();
        addEntry(".", FAT_FILE_ATTRIBUTE_DIRECTORY, 0);
        addEntry("..", FAT_FILE_ATTRIBUTE_DIRECTORY, 0);
        addEntry("SUBDIR", FAT_FILE_ATTRIBUTE_DIRECTORY, 0);
        addEntry("LFNENTRY", 0x0F, 0);
        addEntry("LOG99999.BFL", FAT_FILE_ATTRIBUTE_ARCHIVE, 100, true);
        addEntry("VOLUME", FAT_FILE_ATTRIBUTE_VOLUME_ID, 0);
        addEntry("NOEXT", FAT_FILE_ATTRIBUTE_ARCHIVE, 1000);
        const uint32_t sizes[] = { 0, 1, 255, 256, 257, 511, 512, 4095, 4096, 4097, 100000, 333333 };
        for (int i = 0; i < numLogs; i++) {
            char name[16];
            snprintf(name, sizeof(name), "LOG%05d.BFL", i + 1);
            addEntry(name, FAT_FILE_ATTRIBUTE_ARCHIVE, i < 12 ? sizes[i] : (uint32_t)rnd(20000));
        }
    }

    void TearDown() override
    {
        stall = false;
        pBusy = 30;
        openDelayPolls = -1;
        for (int i = 0; i < 20 && callClose() != LOG_DOWNLOAD_OK; i++) {
        }
        EXPECT_EQ(0, handlesInUse()) << "file handle leaked";
        EXPECT_FALSE(cwdIsLogs) << "working directory not restored";
    }
};

TEST_F(BlackboxDownloadSdTest, ListsOnlyRegularFilesInPages)
{
    for (int bufSize : { OUTBUF_SIZE, 400 }) {
        listResult_t list;
        ASSERT_TRUE(listAll(&list, bufSize));
        EXPECT_FALSE(list.flash);
        ASSERT_EQ(numLogs + 1u, list.entries.size());
        EXPECT_EQ("NOEXT", list.entries[0].name);
        EXPECT_EQ("LOG00001.BFL", list.entries[1].name);
        EXPECT_EQ(0u, list.entries[1].size);
        EXPECT_EQ(333333u, list.entries[12].size);
        EXPECT_EQ(FAT_MAKE_DATE(2026, 11, 18), list.entries[1].date);
        EXPECT_EQ(FAT_MAKE_TIME(12, 34, 56), list.entries[1].time);
        EXPECT_EQ("LOG00045.BFL", list.entries[numLogs].name);
        if (bufSize == 400) {
            EXPECT_EQ(3, list.requests);   // 19 entries per page
        }
    }
    EXPECT_EQ(1, blackboxSdResets);
}

TEST_F(BlackboxDownloadSdTest, DownloadsAllFilesExactlyWithoutSeekingBack)
{
    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    for (size_t i = 0; i < list.entries.size(); i++) {
        const std::vector<uint8_t> data = download(list.entries[i].name, i % 2);
        EXPECT_EQ(findFile(list.entries[i].name)->data, data) << list.entries[i].name;
        EXPECT_EQ(list.entries[i].size, data.size());
    }
    // Sequential reads continue where the previous one stopped, also with compression
    EXPECT_EQ(0, backwardSeeks);
    EXPECT_LE(handlesInUse(), 2);
}

TEST_F(BlackboxDownloadSdTest, RetriesOfLostRepliesGetTheSameData)
{
    EXPECT_EQ(findFile("LOG00012.BFL")->data, download("LOG00012.BFL", true, OUTBUF_SIZE, 3));
    EXPECT_EQ(findFile("LOG00011.BFL")->data, download("LOG00011.BFL", false, OUTBUF_SIZE, 4));
}

TEST_F(BlackboxDownloadSdTest, SmallMspBuffer)
{
    EXPECT_EQ(findFile("LOG00011.BFL")->data, download("LOG00011.BFL", true, 400));
}

TEST_F(BlackboxDownloadSdTest, EndOfFileMissingFileAndBadRequests)
{
    uint8_t data[8192];
    EXPECT_EQ(0, readChunk("LOG00002.BFL", 1, true, data));
    EXPECT_EQ(0, readChunk("LOG00002.BFL", 5000, false, data));
    EXPECT_EQ(-LOG_DOWNLOAD_NOT_FOUND, readChunk("LOG77777.BFL", 0, false, data));
    EXPECT_EQ(-LOG_DOWNLOAD_NOT_FOUND, readChunk("LOG99999.BFL", 0, false, data));
    EXPECT_EQ(-LOG_DOWNLOAD_BAD_REQUEST, readChunk("../X", 0, false, data));

    callRead("LOG00002.BFL", 0, 0, false, OUTBUF_SIZE);
    EXPECT_EQ(LOG_DOWNLOAD_BAD_REQUEST, outBuf[0]);
}

TEST_F(BlackboxDownloadSdTest, RefusedWhileArmedOrLogging)
{
    uint8_t data[8192];
    armingFlags = ARMED;
    EXPECT_EQ(-LOG_DOWNLOAD_BUSY, readChunk("LOG00003.BFL", 0, false, data));
    armingFlags = (armingFlag_e)0;

    blackboxStopped = false;
    EXPECT_EQ(-LOG_DOWNLOAD_BUSY, readChunk("LOG00003.BFL", 0, false, data));
    blackboxStopped = true;

    blackboxSdIdle = false;
    EXPECT_EQ(-LOG_DOWNLOAD_BUSY, readChunk("LOG00003.BFL", 0, false, data));
    blackboxSdIdle = true;

    uint8_t status = 0;
    armingFlags = ARMED;
    listResult_t list;
    EXPECT_FALSE(listAll(&list, OUTBUF_SIZE, &status));
    EXPECT_EQ(LOG_DOWNLOAD_BUSY, status);
    armingFlags = (armingFlag_e)0;

    EXPECT_EQ(0, handlesInUse());
}

TEST_F(BlackboxDownloadSdTest, BlackboxStartTakesTheCardBack)
{
    uint8_t data[8192];
    EXPECT_GT(readChunk("LOG00012.BFL", 0, false, data), 0);
    EXPECT_EQ(2, handlesInUse());

    int steps = 0;
    while (!blackboxDownloadReleaseForLogging()) {
        afatfs_poll();
        ASSERT_LT(steps++, 1000);
    }
    EXPECT_EQ(0, handlesInUse());
    EXPECT_FALSE(cwdIsLogs);

    // Locked out for a moment, so blackbox can start
    EXPECT_EQ(-LOG_DOWNLOAD_BUSY, readChunk("LOG00012.BFL", 4096, false, data));
    nowMs += 600;

    // Afterwards the download can continue (reopen and seek)
    ASSERT_EQ(4096, readChunk("LOG00012.BFL", 4096, false, data));
    EXPECT_EQ(0, memcmp(data, findFile("LOG00012.BFL")->data.data() + 4096, 4096));
}

TEST_F(BlackboxDownloadSdTest, IdleTimeoutReleasesTheCard)
{
    uint8_t data[8192];
    EXPECT_GT(readChunk("LOG00012.BFL", 0, false, data), 0);
    EXPECT_EQ(2, handlesInUse());

    nowMs += 2500;
    for (int i = 0; i < 1000 && (handlesInUse() || cwdIsLogs); i++) {
        blackboxDownloadUpdate(nowMs);
        afatfs_poll();
    }
    EXPECT_EQ(0, handlesInUse());
    EXPECT_FALSE(cwdIsLogs);
}

TEST_F(BlackboxDownloadSdTest, TimeoutsDoNotLeakHandles)
{
    // Card stops responding
    stall = true;
    callRead("LOG00014.BFL", 0, 4096, false, OUTBUF_SIZE);
    EXPECT_EQ(LOG_DOWNLOAD_IO_ERROR, outBuf[0]);
    stall = false;

    // Opening the file takes longer than a request may take, twice
    pBusy = 0;
    openDelayPolls = 1000;
    for (int i = 0; i < 2; i++) {
        callRead("LOG00014.BFL", 0, 4096, false, OUTBUF_SIZE);
        EXPECT_EQ(LOG_DOWNLOAD_IO_ERROR, outBuf[0]);
    }
    openDelayPolls = -1;
    pBusy = 30;

    // The abandoned open completes later; its handle must be closed, not leaked
    for (int i = 0; i < 2000; i++) {
        afatfs_poll();
    }
    EXPECT_EQ(findFile("LOG00014.BFL")->data, download("LOG00014.BFL", true));
    EXPECT_EQ(2, handlesInUse());
}

TEST_F(BlackboxDownloadSdTest, RandomisedDownloads)
{
    for (unsigned seed = 1; seed <= 30; seed++) {
        srand(seed);
        pBusy = rnd(60);
        listResult_t list;
        ASSERT_TRUE(listAll(&list));
        for (int k = 0; k < 4; k++) {
            const listEntry_t &e = list.entries[rnd(list.entries.size())];
            EXPECT_EQ(findFile(e.name)->data, download(e.name, rnd(2), rnd(2) ? OUTBUF_SIZE : 400, rnd(3) ? 0 : 2 + rnd(4)))
                << "seed " << seed << " " << e.name;
        }
    }
}

TEST_F(BlackboxDownloadSdTest, EraseAllDeletesTheLogsAndFreesTheirSpace)
{
    uint8_t data[8192];
    EXPECT_GT(readChunk("LOG00012.BFL", 0, false, data), 0);

    rebuildPollsLeft = 5;
    EXPECT_EQ(LOG_DOWNLOAD_OK, eraseAll());

    for (auto &f : dirEntries) {
        const bool listed = !(f.entry.attrib & (FAT_FILE_ATTRIBUTE_VOLUME_ID | FAT_FILE_ATTRIBUTE_DIRECTORY));
        EXPECT_TRUE(!listed || isDeleted(f.entry));
    }
    EXPECT_EQ(1, rebuildCalls);
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, fsState);

    // Asking again reports the finished erase
    uint8_t status, progress;
    callErase(false, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_OK, status);
    EXPECT_EQ(100, progress);

    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    EXPECT_EQ(0u, list.entries.size());
}

TEST_F(BlackboxDownloadSdTest, EraseAllIsAbandonedWhenTheClientStopsPolling)
{
    pBusy = 0;
    openDelayPolls = 100;   // slow enough that one request can't delete everything
    uint8_t status, progress;
    callErase(true, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_IN_PROGRESS, status);

    nowMs += 3000;
    for (int i = 0; i < 1000; i++) {
        blackboxDownloadUpdate(nowMs);
        afatfs_poll();
    }
    callErase(false, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_IO_ERROR, status);
    EXPECT_EQ(0, rebuildCalls);

    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    EXPECT_GT(list.entries.size(), 0u);
    EXPECT_LT(list.entries.size(), numLogs + 1u);
}

TEST_F(BlackboxDownloadSdTest, BlackboxStartStopsDeletingButWaitsForTheRebuild)
{
    pBusy = 0;
    openDelayPolls = 100;           // slow enough that one request can't delete everything
    initPollsAfterRebuild = 1000;   // the filesystem takes longer than a request to come back
    uint8_t status, progress;
    callErase(true, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_IN_PROGRESS, status);

    // Blackbox wants to log while files are being deleted: deleting stops
    int steps = 0;
    while (!blackboxDownloadReleaseForLogging()) {
        afatfs_poll();
        ASSERT_LT(steps++, 1000);
    }
    callErase(false, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_BUSY, status);
    EXPECT_EQ(0, rebuildCalls);

    // But once the space is being rebuilt, it must finish first
    nowMs += 1000;
    rebuildPollsLeft = 3;
    callErase(true, &status, &progress);
    for (int i = 0; status == LOG_DOWNLOAD_IN_PROGRESS && rebuildCalls == 0; i++) {
        ASSERT_LT(i, 1000);
        callErase(false, &status, &progress);
    }
    ASSERT_EQ(1, rebuildCalls);
    ASSERT_EQ(LOG_DOWNLOAD_IN_PROGRESS, status);
    EXPECT_FALSE(blackboxDownloadReleaseForLogging());

    // Finishes without the client
    for (int i = 0; i < 2000 && fsState != AFATFS_FILESYSTEM_STATE_READY; i++) {
        blackboxDownloadUpdate(nowMs);
        afatfs_poll();
    }
    EXPECT_TRUE(blackboxDownloadReleaseForLogging());
    callErase(false, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_OK, status);
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, fsState);
}

/* ------------------------------------------------------------------------------------------------- flash */

class BlackboxDownloadFlashTest : public ::testing::Test {
protected:
    std::vector<uint32_t> logStarts;

    void SetUp() override
    {
        armingFlags = (armingFlag_e)0;
        blackboxStopped = true;
        flashErasing = false;
        sdcardConfig_System.mode = SDCARD_MODE_NONE;
        blackboxConfig_System.device = BLACKBOX_DEVICE_FLASH;
        releaseAndReset();

        // An empty flash invalidates anything the module still knows about the old contents
        flashData.clear();
        listResult_t list;
        EXPECT_TRUE(listAll(&list));
        EXPECT_EQ(0u, list.entries.size());
        srand(2);
        logStarts.clear();
    }

    void appendLog(uint32_t size)
    {
        logStarts.push_back(flashData.size());
        const std::string header = FLASH_HEADER;
        for (uint32_t i = 0; i < size; i++) {
            flashData.push_back(i < header.size() ? header[i] : (uint8_t)(rand() % 8));
        }
    }

    // Puts the next log header so that it crosses a 512 byte boundary
    void padToCrossBoundary(void)
    {
        while ((flashData.size() + 20) % 512 != 0) {
            flashData.push_back(rand() % 8);
        }
    }

    std::vector<uint8_t> logData(size_t i)
    {
        const uint32_t end = i + 1 < logStarts.size() ? logStarts[i + 1] : flashData.size();
        return std::vector<uint8_t>(flashData.begin() + logStarts[i], flashData.begin() + end);
    }
};

TEST_F(BlackboxDownloadFlashTest, FindsLogsByTheirHeaders)
{
    appendLog(300000);
    padToCrossBoundary();
    appendLog(1);           // only the first byte of a header: not a log on its own
    appendLog(700000);
    padToCrossBoundary();
    appendLog(5000);
    appendLog(200000);

    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    EXPECT_TRUE(list.flash);
    EXPECT_GT(list.requests, 2) << "scanning should take several requests";

    // The padding and the truncated header are part of the log before
    ASSERT_EQ(4u, list.entries.size());
    EXPECT_EQ("LOG00001.BFL", list.entries[0].name);
    EXPECT_EQ("LOG00004.BFL", list.entries[3].name);
    EXPECT_EQ(logStarts[2] - logStarts[0], list.entries[0].size);
    EXPECT_EQ(logStarts[3] - logStarts[2], list.entries[1].size);
    EXPECT_EQ(5000u, list.entries[2].size);
    EXPECT_EQ(200000u, list.entries[3].size);
    EXPECT_EQ(0, list.entries[3].date);

    const std::vector<uint8_t> data = download("LOG00002.BFL", true);
    EXPECT_TRUE(std::equal(data.begin(), data.end(), flashData.begin() + logStarts[2]));
    EXPECT_EQ(list.entries[1].size, data.size());
    EXPECT_EQ(logData(4), download("LOG00004.BFL", false));
    EXPECT_EQ(logData(4), download("LOG00004.BFL", true, OUTBUF_SIZE, 3));

    uint8_t chunk[8192];
    EXPECT_EQ(-LOG_DOWNLOAD_NOT_FOUND, readChunk("LOG00005.BFL", 0, false, chunk));
    EXPECT_EQ(-LOG_DOWNLOAD_NOT_FOUND, readChunk("LOG0001.BFL", 0, false, chunk));
    EXPECT_EQ(0, readChunk("LOG00003.BFL", 5000, false, chunk));
}

TEST_F(BlackboxDownloadFlashTest, DataBeforeTheFirstHeaderIsALog)
{
    for (int i = 0; i < 1000; i++) {
        flashData.push_back(0x42);
    }
    appendLog(3000);

    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    ASSERT_EQ(2u, list.entries.size());
    EXPECT_EQ(1000u, list.entries[0].size);
    EXPECT_EQ(3000u, list.entries[1].size);
}

TEST_F(BlackboxDownloadFlashTest, NewLogsAreAddedToTheIndex)
{
    appendLog(400000);
    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    ASSERT_EQ(1u, list.entries.size());

    appendLog(10000);
    ASSERT_TRUE(listAll(&list));
    ASSERT_EQ(2u, list.entries.size());
    EXPECT_EQ(400000u, list.entries[0].size);
    EXPECT_EQ(10000u, list.entries[1].size);
    EXPECT_LE(list.requests, 2) << "only the new data should be scanned";
}

TEST_F(BlackboxDownloadFlashTest, TooManyLogsEndUpInTheLastOne)
{
    for (int i = 0; i < 130; i++) {
        appendLog(100);
    }
    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    ASSERT_EQ(128u, list.entries.size());
    EXPECT_EQ(300u, list.entries[127].size);
}

TEST_F(BlackboxDownloadFlashTest, RefusedWhileArmed)
{
    appendLog(1000);
    armingFlags = ARMED;
    uint8_t status = 0;
    listResult_t list;
    EXPECT_FALSE(listAll(&list, OUTBUF_SIZE, &status));
    EXPECT_EQ(LOG_DOWNLOAD_BUSY, status);
    armingFlags = (armingFlag_e)0;
}

TEST_F(BlackboxDownloadFlashTest, EraseAllErasesTheFlash)
{
    appendLog(5000);
    appendLog(5000);
    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    ASSERT_EQ(2u, list.entries.size());

    uint8_t status, progress;
    callErase(true, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_IN_PROGRESS, status);
    EXPECT_EQ(255, progress);
    EXPECT_TRUE(flashErasing);

    // Nothing else while erasing, and blackbox has to wait
    EXPECT_FALSE(listAll(&list, OUTBUF_SIZE, &status));
    EXPECT_EQ(LOG_DOWNLOAD_BUSY, status);
    EXPECT_FALSE(blackboxDownloadReleaseForLogging());

    flashErasing = false;
    callErase(false, &status, &progress);
    EXPECT_EQ(LOG_DOWNLOAD_OK, status);
    EXPECT_EQ(100, progress);

    nowMs += 1000;
    ASSERT_TRUE(listAll(&list));
    EXPECT_EQ(0u, list.entries.size());
}

TEST_F(BlackboxDownloadFlashTest, StorageFollowsTheBlackboxDevice)
{
    appendLog(1000);
    listResult_t list;
    ASSERT_TRUE(listAll(&list));
    EXPECT_TRUE(list.flash);

    // With an SD card configured, the SD card is used
    sdcardConfig_System.mode = SDCARD_MODE_SPI;
    blackboxConfig_System.device = BLACKBOX_DEVICE_SDCARD;
    fsState = AFATFS_FILESYSTEM_STATE_READY;
    dirEntries.clear();
    pBusy = 0;
    ASSERT_TRUE(listAll(&list));
    EXPECT_FALSE(list.flash);
    EXPECT_EQ(0u, list.entries.size());

    sdcardConfig_System.mode = SDCARD_MODE_NONE;
    blackboxConfig_System.device = BLACKBOX_DEVICE_FLASH;
    ASSERT_TRUE(listAll(&list));
    EXPECT_TRUE(list.flash);
    EXPECT_EQ(0, handlesInUse());
    EXPECT_FALSE(cwdIsLogs);
}
