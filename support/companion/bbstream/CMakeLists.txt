# Builds the companion-side blackbox stream decoder (bbstream_cli).
# Same layout as groundstation/relay, so the cross-compiler container can build and deploy it:
#   docker run -v rootfs:/rootfs --mount type=bind,src=./,dst=/package --env-file=./remote.env \
#     aarch64-linux-gnu-cross --processes=8 --clean-build --deploy=/home/pi --wd=support/companion/bbstream
# Native:  mkdir build && cd build && cmake .. && make && make test
cmake_minimum_required (VERSION 3.18)
project(bbstream)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
add_compile_options(-Wall -Wextra)
# blackbox-tools stores stream bytes/EOF in plain `char` (e.g. `char command = streamPeekChar()` in
# flightLogParse). On aarch64 Linux char is unsigned, EOF never compares equal and the parser loops forever at
# the end of every window. Make char signed, as on x86, instead of patching the library.
add_compile_options(-fsigned-char)

# the blackbox-tools sources are used as they are
set(BBT ${CMAKE_SOURCE_DIR}/../../LogAnalysis/indiflight_log_tools/indiflight_log_tools/blackbox-tools/src)
set(BBT_LIB
    ${BBT}/parser.c ${BBT}/tools.c ${BBT}/platform.c ${BBT}/stream.c
    ${BBT}/decoders.c ${BBT}/units.c ${BBT}/blackbox_fielddefs.c)

find_package(Threads REQUIRED)

add_executable(bbstream_cli bbstream.c bbstream_cli.c ${BBT_LIB})
target_include_directories(bbstream_cli PRIVATE ${BBT})
target_link_libraries(bbstream_cli m Threads::Threads)

# ----- tests: `make test` (or `ctest -V`) in the build directory -----
# The test replays a real log through pipes and a pty and compares with the reference decoder. Everything it
# needs is copied into the build directory, so it also runs on the companion after a deploy:
#   cd ~/bbstream/build-aarch64-linux-gnu && python3 test_bbstream.py
option(BBSTREAM_TESTS "Build blackbox_decode and set up the tests" ON)
if(BBSTREAM_TESTS)
    add_executable(blackbox_decode ${BBT_LIB} ${BBT}/blackbox_decode.c ${BBT}/stats.c)
    target_include_directories(blackbox_decode PRIVATE ${BBT})
    target_compile_options(blackbox_decode PRIVATE -w)
    target_link_libraries(blackbox_decode m Threads::Threads)

    configure_file(${CMAKE_SOURCE_DIR}/test_bbstream.py ${CMAKE_BINARY_DIR}/test_bbstream.py COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../blackbox_packets.py ${CMAKE_BINARY_DIR}/blackbox_packets.py COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../../../logs-mockup/LOG00043.BFL ${CMAKE_BINARY_DIR}/LOG00043.BFL COPYONLY)

    enable_testing()
    add_test(NAME bbstream COMMAND python3 ${CMAKE_BINARY_DIR}/test_bbstream.py)
    set_tests_properties(bbstream PROPERTIES TIMEOUT 300)
endif()
