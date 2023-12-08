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

sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo findmnt ${DEV}"
if [[ $? -gt 0 ]]; then
    echo "Not yet mounted. Mounting ${DEV} on /mnt on remote..."
    i=2
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo mount ${DEV} /mnt"
    while [[ $? -gt 0  ]] && [ $i -gt 0 ]; do
        ((i=i-1))
        sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 "sudo mount ${DEV} /mnt"
    done
fi

if [[ $? -gt 0 ]]; then
    # mount failed, only path here
    echo "Mounting failed! Resetting FC"
    reset
    exit 1
fi

exit_code=0

echo "starting rsync..."
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

