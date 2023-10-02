#!/bin/bash

if [[ -z "$1" ]]; then
    echo "First and only argument must be log destination path."
    exit 1
fi
DEST_PATH=$1

reset () {
    # always reboot fxc 
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 '/usr/bin/openocd -f /opt/openocd/openocd.cfg -c "init; reset; exit"'
}

sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 'sudo findmnt /dev/sda1'
if [[ $? -gt 0 ]]; then
    echo "Not yet mounted..."
    sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 'sudo mount /dev/sda1 /mnt'
fi

if [[ $? -gt 0 ]]; then
    # mount failed, only path here
    echo "Mounting failed!"
    reset
    exit 1
fi

rsync -a --rsh "sshpass -p pi ssh -o StrictHostKeyChecking=no -l pi" --timeout=3 --delete pi@10.0.0.1:/mnt/LOGS "$DEST_PATH"

# always unmount
sshpass -p pi ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 pi@10.0.0.1 'sudo umount /dev/sda1'

# always reset
reset

