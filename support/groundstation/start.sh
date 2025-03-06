#!/bin/bash

# Partitions a maximized terminal using tmux and launches relevant tools
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


# Watch out! Stop all commands before closing the terminal, catching that still
# doesnt work

# example command to run this in a new terminal
#   alacritty -o "window.startup_mode=Maximized" -e ./dashboard.sh 1003 
#

echo_help_and_exit() {
    echo "usage: $0 <rigid_body_id> [--test] [--help/-h]"
    exit 1
}

source ../../remote.env

if [[ -z $REMOTE_IP ]]; then
    echo "../../remote.env has the contain a REMOTE_IP= statement"
fi

# check for arguments
if [[ $# -gt 2 ]] || [[ $# -lt 1 ]]; then
    echo_help_and_exit
fi

while test $# != 0
do
    left=$(echo "$1" | cut -d "=" -f 1)
    right=$(echo "$1" | cut -d "=" -f 2)
    case "$left" in
    --test) TEST_FLAG=--test ;;
    --help) echo_help_and_exit ;;
    -h) echo_help_and_exit ;;
    *) RB_ID=$left ;;
    esac
    shift
done

if [[ $RB_ID -lt 0 ]] || [[ -z $RB_ID ]]; then
    echo "Invalid Rigid Body ID"
    echo_help_and_exit
fi

# define ssh stuff
#ON_PI="/usr/bin/sshpass -p pi ssh -o StrictHostKeyChecking=no pi@10.0.0.1"

# make sure program exits properly
trap "exit" INT TERM
trap "kill 0" EXIT

# setup tmux
session="dashboard"
tmux kill-server
tmux start-server
tmux new-session -d -s $session
tmux set mouse on

# make panes like this 
# +-----+------+
# | .0  |  .3  |
# +-----|------+
# | .1  |  .4  |
# +-----+------+
# | .2  |  .5  |
# +-----+------+
#
tmux splitw -h -t "$session:0.0"
tmux splitw -v -t "$session:0.0"
tmux splitw -v -t "$session:0.2"
tmux splitw -v -t "$session:0.1"
tmux splitw -v -t "$session:0.4"

# start uart comms. UPDATE: now assumed that relay.service is running, check README
#tmux send-keys -t $session:0.3 "sudo killall relay\;"
tmux send-keys -t $session:0.3 "relay/build/relay "
#tmux send-keys -t $session:0.3 "$ON_PI sudo systemctl status relay.service" ENTER

# start natnet2udp.py in udp mode
#tmux send-keys -t $session:0.0 "./optitrack_forwarder/build/natnet2udp.py -ac $RB_ID 0 -f 20 -le right -an far -xs right -up z_up -udp $TEST_FLAG" ENTER
#tmux send-keys -t $session:0.0 "./UnifiedOptitrackClients/build/mocap2udp -s $RB_ID --ac 0 -f 20 -i 10.0.0.1 -p 5005 -c NED $TEST_FLAG" ENTER
if [[ -z $TEST_FLAG ]]; then
    tmux send-keys -t $session:0.0 "./ext/UnifiedMocapRouter/build/mocap-router optitrack udp -d 14 -f 20 -c NED -r far -n far -s $RB_ID -i $REMOTE_IP" ENTER
else
    tmux send-keys -t $session:0.0 "./ext/UnifiedMocapRouter/build/mocap-router test udp -d 14 -f 20 -c NED -r far -n far -s 3 -i $REMOTE_IP --test_freq 20" ENTER
fi

# setpoints
tmux send-keys -t $session:0.1 "/usr/bin/env python3 setpointSender.py --host $REMOTE_IP --pos 0 0 -1.0 --yaw 0"

# keyboards
#tmux send-keys -t $session:0.2 "/usr/bin/env python3 keyInputs.py --host $REMOTE_IP" ENTER
tmux send-keys -t $session:0.2 "/usr/bin/env python3 configMenu.py --host $REMOTE_IP" ENTER

# socat
tmux send-keys -t $session:0.4 "sudo socat -d -d PTY,link=/dev/ttyDB,raw,echo=0,mode=666 TCP:dronebridge.local:5760" ENTER

# ping
tmux send-keys -t $session:0.5 "ping dronebridge.local" ENTER

# periodically get stats by sending USER 1 signal to process
#$ON_PI "while [[1]]; do; kill -USE2 `pidof connect`; sleep 5; done" &

tmux attach-session -t $session

