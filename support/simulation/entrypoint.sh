#!/bin/bash
set -e

if [[ -z ${PROFILE} ]]; then
    echo "Must have PROFILE defined as environment variable"
    exit 1
fi

if [[ -z ${SIM} ]]; then
    echo "Must have SIM defined as environment variable"
    exit 1
fi

make -j TARGET=MOCKUP

GDBSERVER_CMD=
if [[ ${GDBSERVER} == "y" ]]; then
    GDBSERVER_CMD="gdbserver localhost:3333"
fi

echo ${GDBSERVER}
echo ${GDBSERVER_CMD}

${GDBSERVER_CMD} /usr/bin/python3 ./support/simulation/${SIM}.py   \
    --sil ./obj/main/indiflight_MOCKUP.so                          \
    --sil-profile-txt ./configs/profiles/${PROFILE}.txt            \
    "$@"
