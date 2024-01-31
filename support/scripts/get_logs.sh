#!/bin/bash

function help_and_exit {
    echo "$0 [device_on_remote] [local_path]"
    echo "One-way sync from device to local path. Deletes local files that are not on the remote anymore!"
    exit 1
}

if [ $# -ne 2 ]; then
    echo "incorrect number of arguments"
    help_and_exit
fi

DEV=$1
DEST_PATH=$2

# betaflight specific: fucntion to reset flight controller in non MSC mode without reconnecting power.
reset () {
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 '/usr/bin/openocd -f /opt/openocd/openocd.cfg -c "init; reset; exit"'
}

# check if disk available
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo lsblk -p | grep ${DEV}"
if [[ $? -gt 0 ]]; then
    # not yet available
    echo "Device not available on remote."

    # check if configurator is connected to USB port
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo netstat -tn | grep 5761"
    if [[ $? -eq 0 ]]; then
        which tcpkill
        if [[ $? -gt 0 ]]; then
            echo "PI still connected to a configurator, but tcpkill (dsniff package) not installed. Disconnect configurator and re-run."
            exit 1
        else
            # try to kill existing tcp connections from configurator
            echo -n "PI still connected to a configurator. Attempting to kill the connection... "
            sudo timeout 3 tcpkill -9 dst host 10.0.0.1 and dst port 5761 > /dev/null
        fi
    fi

    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo netstat -tn | grep 5761"
    if [[ $? -gt 0 ]]; then
        echo "failed! Disconnect configurator and re-run."
        exit 1
    fi

    echo "succesful"
    echo -n "Send reboot-to-MSC signal to FC... "

    # use MSP protocol over TCP (command 68, one Byte of value 0x03)
    timeout 1 $(dirname "$0")/sendMSPoverTCP.py --host 10.0.0.1 --port 5761 68 B 3
    if [[ $? -gt 0 ]]; then
        echo "Failed. exiting"
        exit 1
    fi
    echo "Succesful."
fi

sleep 3

sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo findmnt ${DEV}"
if [[ $? -gt 0 ]]; then
    # try to find blk device
    i=3
    while 
        sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo lsblk -p | grep ${DEV}"
        [[ $? -gt 0  ]] && [[ $i -gt 0 ]]
    do
        echo "MSC device not (yet) available on remote. Waiting..."
        ((i=i-1))
        sleep 1
        false
    done

    if [[ $? -gt 0 ]]; then
        # mount failed, only path here
        echo "Failed to bring up MSC device. Resetting FC..."
        reset
        exit 1
    fi

    echo "MSC device available! Attempting checking/fixing of FAT filesystem on ${DEV}:"
    echo ""
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo fsck.vfat -l -a -v -w -V /dev/mmcblk0p1"

    echo ""
    echo "Attempting mounting ${DEV} on remote:/mnt..."
    i=3
    while
        sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo timeout 3 mount ${DEV} /mnt"
        [[ $? -gt 0 ]] && [[ $i -gt 0 ]]
    do
        ((i=i-1))
        false
    done

    if [[ $? -gt 0 ]]; then
        # mount failed, only path here
        echo "Mounting failed! Resetting FC..."
        reset
        exit 1
    fi
fi


exit_code=0

echo "Mounting succesful! Starting rsync..."
rsync -a --rsh "sshpass -p pi ssh -o StrictHostKeyChecking=no -l pi" --timeout=3 --delete pi@10.0.0.1:/mnt/LOGS/ "$DEST_PATH"
if [[ $? -eq 0 ]]; then
    echo "Transfer successful! Unmounting ${DEV}..."
else
    echo "Transfer failed! Unmounting ${DEV}..."
    exit_code=1
fi

# always unmount
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo umount ${DEV}"

# always reset
echo "Resetting FC..."
reset
exit ${exit_code}

