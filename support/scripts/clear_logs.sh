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
    echo "MSC Device not (yet) available on remote."


    echo -n "Sending reboot-to-MSC signal to FC... "

    # restart ser2net to terminate any existing connections, then hope we are faster with sending the reboot signal than auto reconnect of the configurator
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo systemctl restart ser2net"

    # use MSP protocol over TCP (command 68, one Byte of value 0x03)
    timeout 3 $(dirname "$0")/sendMSPoverTCP.py --host 10.0.0.1 --port 5761 68 B 3
    if [[ $? -gt 0 ]]; then
        echo "Failed. exiting. restart into MSC manually through the configurator"
        exit 1
    fi
fi

sleep 3

echo
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
        echo
        reset
        exit 1
    fi

    echo "MSC device available! Attempting checking/fixing of FAT filesystem on ${DEV}:"
    echo ""
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo fsck.vfat -l -a -v -w -V ${DEV}"

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
        echo
        reset
        exit 1
    fi
fi

exit_code=0

echo "Mounting succesful! Starting delete..."
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo timeout 3 rm /mnt/LOGS/LOG0*"
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo timeout 3 rm /mnt/LOG0*"


# rsync -a --rsh "sshpass -p pi ssh -o StrictHostKeyChecking=no -l pi" --timeout=3 --delete pi@10.0.0.1:/mnt/LOGS/ "$DEST_PATH"
# if [[ $? -eq 0 ]]; then
#     echo "Transfer successful! Unmounting ${DEV}..."
# else
#     echo "Transfer failed! Unmounting ${DEV}..."
#     exit_code=1
# fi

# always unmount
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo umount ${DEV}"

# always reset
echo "Resetting FC..."
echo
reset
exit ${exit_code}
