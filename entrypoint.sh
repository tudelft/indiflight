#!/bin/bash

cd /indiflight

make -j "$@"

if [[ $UPLOAD_PARAMETERS == "y" ]]; then
    echo "Wait for serial device to come up"
    sleep 5

    VENDOR_ID="0483"
    PRODUCT_ID="5740"
    DEVICE=
    for dev in /dev/ttyACM*; do
        udevadm info --name $device | grep VENDOR_ID=$VENDOR_ID -A 2 | grep MODEL_ID=$PRODUCT_ID;
        if [[ $? -eq 0 ]]; then
            DEVICE=${dev}
        fi
    done

    if [[ -n "$DEVICE" ]]; then
        echo "/dev/serial/by-id/$DEVICE"
    else
        echo "No STM32 serial device found." >&2
        exit 1
    fi

    /usr/bin/python3 support/scripts/uploadProfile.py /dev/serial/by-id/$DEVICE \
        /indiflight/configs/boards/${BOARD}.txt \
        /indiflight/configs/profiles/${PROFILE}.txt
fi
