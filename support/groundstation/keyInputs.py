# Sends keystrokes in the terminal via UDP to issue commands to the vehicle
#
# Copyright 2024 Robin Ferede (Delft University of Technology)
#                Till Blaha (Delft University of Technology)
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


import curses
from argparse import ArgumentParser
import socket
import struct

# HID codes for various keys
hid_codes = {
    'a': 0x04, 'b': 0x05, 'c': 0x06, 'd': 0x07, 'e': 0x08,
    'f': 0x09, 'g': 0x0A, 'h': 0x0B, 'i': 0x0C, 'j': 0x0D,
    'k': 0x0E, 'l': 0x0F, 'm': 0x10, 'n': 0x11, 'o': 0x12,
    'p': 0x13, 'q': 0x14, 'r': 0x15, 's': 0x16, 't': 0x17,
    'u': 0x18, 'v': 0x19, 'w': 0x1A, 'x': 0x1B, 'y': 0x1C,
    'z': 0x1D, '0': 0x27, '1': 0x1E, '2': 0x1F, '3': 0x20,
    '4': 0x21, '5': 0x22, '6': 0x23, '7': 0x24, '8': 0x25,
    '9': 0x26, ' ': 0x2C, '\n': 0x28, '\b': 0x2A, '\t': 0x2B,
    'ESC': 0x29, 'LEFT': 0x50, 'UP': 0x48, 'RIGHT': 0x4D, 'DOWN': 0x50
}

# msg definitions for udp sending
KEYBOARD = {
    'time_us': 0,
    'key': 0,
}

KEYBOARD_format = '!IB'


def main(stdscr, sock, host, port):
    msg = KEYBOARD.copy()
    msg['time_us'] = int(0)
    msg['key'] = 0x0

    # detect overruns
    rows, _ = stdscr.getmaxyx()
    if rows < 15:
        raise TypeError(f"Terminal must have height of at least 15 rows, has {rows}")

    # Clear screen
    stdscr.clear()
    stdscr.addstr("Press keys to see their HID codes. Press ESC to exit.\n")
    info = '''
0 = go to center
p = go to above nn_init
t = takeoff
s = set waypoint here
1 = initTrajectoryTracker
2 = decrease speed by 0.5 m/s
3 = increase speed by 0.5 m/s
4 = stopTrajectoryTracker
5 = land
6 = nn_init
7 = nn_activate
8 = recovery_mode
9 = kill

    '''
    stdscr.addstr(info)

    while True:
        key = stdscr.getch()  # Wait for a key press
        if key == 27:  # ESC key
            break

        # Clear the previous output
        move_down_by = info.count('\n') + 1
        stdscr.move(move_down_by, 0)
        stdscr.clrtoeol()  # Clear the line

        # Check if the key is an ASCII character
        key_hid = None
        if 32 <= key <= 126:  # Printable characters
            char = chr(key)
            if char in hid_codes:
                key_hid = hid_codes[char]
                stdscr.addstr(f"HID Code for '{char}': 0x{key_hid:02X}\n")
        elif key == curses.KEY_LEFT:
            key_hid = hid_codes['LEFT']
            stdscr.addstr("HID Code for Left Arrow: 0x{:02X}\n".format(key_hid))
        elif key == curses.KEY_RIGHT:
            key_hid = hid_codes['RIGHT']
            stdscr.addstr("HID Code for Right Arrow: 0x{:02X}\n".format(key_hid))
        elif key == curses.KEY_UP:
            key_hid = hid_codes['UP']
            stdscr.addstr("HID Code for Up Arrow: 0x{:02X}\n".format(key_hid))
        elif key == curses.KEY_DOWN:
            key_hid = hid_codes['DOWN']
            stdscr.addstr("HID Code for Down Arrow: 0x{:02X}\n".format(key_hid))
        elif key == 8:  # Backspace key
            key_hid = hid_codes['\b']
            stdscr.addstr("HID Code for Backspace: 0x{:02X}\n".format(key_hid))
        elif key == 9:  # Tab key
            key_hid = hid_codes['\t']
            stdscr.addstr("HID Code for Tab: 0x{:02X}\n".format(key_hid))
        elif key == 10:  # Enter key
            key_hid = hid_codes['\n']
            stdscr.addstr("HID Code for Enter: 0x{:02X}\n".format(key_hid))

        # send over UDP
        if key_hid is not None:
            msg['time_us'] = int(0)
            msg['key'] = key_hid
            msg_packed = struct.pack(KEYBOARD_format, *msg.values())
            sock.sendto(msg_packed, (args.host, args.port))

        # refresh terminal
        stdscr.refresh()

if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument('--host', required=False, default="10.0.0.1", type=str)
    parser.add_argument('--port', required=False, default=5007, type=int)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    curses.wrapper(main, sock, args.host, args.port)