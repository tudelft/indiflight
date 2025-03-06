#!/bin/bash

# Download logs from an indiflight flight controller via RPI companion computer
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

if ! [ -f remote.env ]; then
  echo "Fatal: start.sh expects a remote.env file with the following contents:

# NO SPACES BEFORE AND AFTER THE '='
# NO quotation marks and no spaces in the variables
REMOTE_NAME=CurrentUnusedCanBeEmpty
REMOTE_IP=192.168.1.42
REMOTE_USER=<username>
REMOTE_PASSWORD=<ssh_password>
    "
fi

source remote.env

if [ -z $REMOTE_IP ] || [ -z $REMOTE_USER ] || [ -z $REMOTE_PASSWORD ]; then
    echo "Fatal: Not all of REMOTE_IP, REMOTE_USER, REMOTE_PASSWORD set in remote.env"
fi


function help_and_exit {
    echo "$0 [--no-reset] local_path"
    echo "One-way sync from device to local path. !! This script deletes local files that are not on the remote anymore !!"
    exit 1
}

NO_RESET=
if [[ $1 == "--no-reset" ]]; then
    NO_RESET="t"
    shift
fi

if [[ $1 == "--help" ]] || [[ $1 == "-h" ]]; then
    help_and_exit
fi

if [ $# -ne 1 ]; then
    echo "incorrect number of arguments"
    help_and_exit
fi

#DEV=$1 # not a thing anymore, will be inferred in this script
#shift
DEST_PATH=$1

REMOTE_MOUNTPOINT=/mnt/fc_logs_mountpoint
REMOTE_LOGDEVICE_GLOB='usb-STM_Produ*part1'

# betaflight specific: fucntion to reset flight controller in non MSC mode without reconnecting power.
reset () {
    if [[ $NO_RESET != "t" ]]; then
        sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP 'sudo systemctl stop openocd.service; /usr/bin/openocd -f /opt/openocd/openocd.cfg -c "init; reset; exit"'
    fi
}

# check if disk available
REMOTE_DEVICE=$(sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "find /dev/disk/by-id/ -name $REMOTE_LOGDEVICE_GLOB | head -n1")
if [[ -z $REMOTE_DEVICE ]]; then
    # not yet available
    echo "MSC Device not (yet) available on remote."

    echo -n "Sending reboot-to-MSC signal to FC... "

    # restart ser2net to terminate any existing connections, then hope we are faster with sending the reboot signal than auto reconnect of the configurator
    sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo systemctl restart ser2net"
    sleep 1

    # use MSP protocol over TCP (command 68, one Byte of value 0x03)
    timeout 3 /usr/bin/env python3 $(dirname "$0")/sendMSPoverTCP.py --host $REMOTE_IP --port 5761 68 B 3
    if [[ $? -gt 0 ]]; then
        echo "Failed. exiting. restart into MSC manually through the configurator"
        exit 1
    fi
fi

sleep 3

echo
# check if already mounted, if not try to mount
sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo findmnt ${REMOTE_MOUNTPOINT}"
if [[ $? -gt 0 ]]; then
    # check if device is available for mounting
    i=3
    while 
        #sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo lsblk -p | grep ${DEV}"
        REMOTE_DEVICE=$(sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "find /dev/disk/by-id/ -name $REMOTE_LOGDEVICE_GLOB | head -n1")
        [[ -z $REMOTE_DEVICE  ]] && [[ $i -gt 0 ]]
    do
        echo "MSC device not (yet) available on remote. Waiting..."
        ((i=i-1))
        sleep 1
        false
    done

    if [[ -z $REMOTE_DEVICE ]]; then
        # mount failed, only path here
        echo "Failed to bring up MSC device. Resetting FC..."
        echo
        reset
        exit 1
    fi

    echo "MSC device available!"
    #echo "MSC device available! Attempting checking/fixing of FAT filesystem on ${REMOTE_DEVICE}:"
    #echo ""
    #sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo fsck.vfat -l -a -v -w -V ${REMOTE_DEVICE}"

    echo ""
    echo "Attempting mounting ${REMOTE_DEVICE} on remote:${REMOTE_MOUNTPOINT}..."
    i=3
    while
        sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo mkdir -p ${REMOTE_MOUNTPOINT}; sudo timeout 5 mount ${REMOTE_DEVICE} ${REMOTE_MOUNTPOINT}"
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

echo "Mounting succesful! Starting rsync..."
echo
rsync -a --rsh "sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -l $REMOTE_USER" --timeout=3 --delete -v --progress $REMOTE_USER@$REMOTE_IP:${REMOTE_MOUNTPOINT}/LOGS/ "$DEST_PATH"
if [[ $? -eq 0 ]]; then
    echo
    echo "Transfer successful! Unmounting ${REMOTE_DEVICE}..."
else
    echo
    echo "Transfer failed! Unmounting ${REMOTE_DEVICE}..."
    exit_code=1
fi

# always and eject
sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 $REMOTE_USER@$REMOTE_IP "sudo umount ${REMOTE_MOUNTPOINT}; sudo eject ${REMOTE_MOUNTPOINT}"

# always reset
echo "Resetting FC..."
echo
reset
exit ${exit_code}
