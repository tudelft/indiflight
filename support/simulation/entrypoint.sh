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

source /opt/ros/humble/setup.bash
source /uros_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds

make -j -e DEBUG=${DEBUG} -e PROFILE=${PROFILE} TARGET=MOCKUP

GDBSERVER_CMD=
if [[ ${GDBSERVER} == "y" ]]; then
    GDBSERVER_CMD="gdbserver localhost:3333"
fi

#${GDBSERVER_CMD} /usr/bin/python3 -m cProfile -o profile.prof ./support/simulation/${SIM}.py   \
${GDBSERVER_CMD} /python-venv/bin/python3 ./support/simulation/${SIM}.py   \
    --sil ./obj/main/indiflight_MOCKUP.so                          \
    --sil-profile-txt ./configs/profiles/${PROFILE}.txt            \
    "$@"
