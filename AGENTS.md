# Notes for contributors and coding agents

Indiflight is a Betaflight fork. The configurator lives in a separate repo
(`tudelft/indiflight-configurator`, branch `indi-web`). It has its own `AGENTS.md`.

## Building

```sh
make BOARD=MTKS-H743 YES=y          # board configs are in configs/boards/*.txt
```

`YES=y` skips the "uncommitted changes" prompt. Bare MCU targets such as `make STM32H743` don't build,
because they define no sensors. Use a `BOARD=`.

## Adding an MSP command

**Pick a code.** Indiflight-specific commands are MSPv2 codes in the `0x40xx` range, defined in
`src/main/msp/msp_protocol_v2_indiflight.h`. Upstream Betaflight uses `0x30xx`
(`msp_protocol_v2_betaflight.h`). Don't reuse or renumber a code, because released configurators depend on it.
Codes above 255 are sent as MSPv2 automatically, on both sides.

**Handle it in `src/main/msp/msp.c`.** `mspFcProcessCommand()` tries these handlers in order:

| Function | Use for |
|---|---|
| `mspCommonProcessOutCommand()` / `mspProcessOutCommand()` | Replies that don't need any request payload |
| `mspFcProcessOutCommandWithArg()` | Replies that read a request payload (`src`) and write a reply (`dst`) |
| `mspProcessInCommand()` / `mspCommonProcessInCommand()` | "Set" commands with no reply data |

- **Unknown or failed commands:** returning `MSP_RESULT_ERROR`, or falling through as unknown, sends an MSP error reply (direction `!`). The configurator treats that as "not supported". For anything that can fail at runtime, put a status byte first in the reply instead, so the client can tell what went wrong.
- **Sizes:** use `sbufBytesRemaining()` on both buffers, because the reply buffer size depends on the transport. `MSP_PORT_OUTBUF_SIZE` (see `msp_serial.h`) is 4112 bytes on targets with flash or SD logging, and 400 bytes otherwise. Requests are at most `MSP_PORT_INBUF_SIZE` (192 bytes).
- **Larger features:** put the logic in their own module and keep only a `case` in `msp.c`, guarded by a feature macro. See `blackbox/blackbox_download.c` and `USE_MSP_LOG_DOWNLOAD` in `target/common_post.h`.
- **Payload format:** document it next to the code. All integers are little-endian (`sbufWriteU16/U32`).

**Update the configurator.** Add the code to `src/js/msp/MSPCodes.js`, using the same name as the firmware.
Put the encode/decode in `src/js/msp/MSPHelper.js`, or in a pure module it imports so the code can be
unit-tested. Call it only from a store or composable, never from a component.

**Version.** Bump `API_VERSION_MINOR` in `msp_protocol.h` only when releasing. Detect new commands by their
reply, not by the API version.

## Unit tests

The tests are in `src/test/unit/*_unittest.cc`, use googletest, and are built by `src/test/Makefile`.

- **Adding a test:** any new `foo_unittest.cc` is picked up automatically. Declare what it compiles in the Makefile:

  ```make
  foo_unittest_SRC := \
          $(USER_DIR)/path/to/foo.c \
          $(USER_DIR)/common/streambuf.c
  foo_unittest_DEFINES := \
          USE_SOMETHING=
  ```

- **Headers:** only the listed sources are compiled, against `src/test/unit/platform.h` and `target.h`. `common_pre.h` and `common_post.h` are *not* included, so feature macros must be set in `_DEFINES`.
- **Stubs:** everything else the code calls has to be stubbed in the test file, inside `extern "C"`. For parameter groups, define the `xxx_System` variable (e.g. `blackboxConfig_t blackboxConfig_System;`).
- **Running:** `make test` (or `test-all`, as CI runs it with `EXTRA_FLAGS=-Werror`) from the repo root. A single test is `make -C src/test test_foo_unittest`. The binary ends up in `obj/test/foo_unittest/`, and accepts `--gtest_filter=...`.
- **Compiler:** the Makefile requires clang 7–14 (the newer `clang` 18 is rejected). On a machine with clang-14 and the GCC 13 C++ headers:

  ```sh
  make -C src/test CC=clang-14 \
    CXX="clang++-14 -isystem /usr/include/c++/13 -isystem /usr/include/x86_64-linux-gnu/c++/13 -L<dir>" \
    test_foo_unittest
  ```

  `<dir>` must contain a `libBlocksRuntime.a`. An empty one is enough if the tests don't use blocks: `ar rcs libBlocksRuntime.a empty.o`.
- **Known failures on this branch:** `blackbox_unittest` currently doesn't build, because `solveActiveSet.h` is not on the include path.
- **What makes a useful test:** see `blackbox_download_unittest.cc` for a test that replaces a driver with an adversarial fake. Its fake asyncfatfs randomly delays, splits and stalls operations, and the test checks the module's invariants, such as that no file handles leak and the working directory is restored.

## Blackbox log transfer over MSP (brief)

Implemented in `src/main/blackbox/blackbox_download.c`. The wire format is documented in `blackbox_download.h`.
The configurator side is the "Log Download" tab.

The commands are `MSP2_INDI_BLACKBOX_LOG_LIST/READ/CLOSE/ERASE_ALL` (`0x4006`–`0x4009`). Every reply starts with
a status byte.

- **Where logs come from:** the configured blackbox device. If that is neither flash nor SD, the SD card is used if one is inserted, otherwise the flash.
  - SD card: the files in `/logs`.
  - Flash: logs are stored back to back. They are found by scanning for the blackbox header line, in steps of about 250 ms (status `IN_PROGRESS` with a percentage until done), and are named `LOG00001.BFL`, ….
- **Reading:** a READ returns a chunk at a given offset, in the same layout as `MSP_DATAFLASH_READ`, optionally Huffman-compressed.
  - The SD file stays open between requests, so sequential reads don't seek.
  - asyncfatfs is asynchronous, so the handlers poll it themselves, for at most 250 ms per request.
- **Safety:** requests are refused while armed or while blackbox is logging. Blackbox always wins:
  - Before starting a log it calls `blackboxDownloadReleaseForLogging()`, which releases the card without blocking.
  - The card is also released after 2 s without requests.
  - While a download holds the card, the working directory is `/logs`. It is restored to the root afterwards, so blackbox still finds `/logs`.
- **Erase all:**
  - Flash: a chip erase.
  - SD card: deletes the files in `/logs` (only while the client keeps polling), then calls `afatfs_rebuildFreeFile()`. Without that, deleted files' space never returns to `FREESPAC.E`, the contiguous region new logs are allocated from.
- **Integrity:** each MSP frame has a CRC8, and the client re-requests a chunk on a CRC error, a wrong offset or a timeout. There is no whole-file checksum; the client only compares the received size with the listed size.
