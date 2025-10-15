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


# import curses
from dialog import Dialog
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


#% menu definitions

# menu of keystrokes must be a list of tuples like below. The first item must
# be a bool that indicates whether we should return to the previous menu on
# selection
NN_ITEMS = [
    True,
    ("1", "NN SLOT 1"),
    ("2", "NN SLOT 2"),
    ("3", "NN SLOT 3"),
    ("4", "NN SLOT 4"),
    ("5", "NN SLOT 5"),
    ("6", "NN SLOT 6"),
    ("7", "NN SLOT 7"),
    ("8", "NN SLOT 8"),
    ("9", "NN SLOT 9"),
]

TT_SPEED_ITEMS = [
    True,
    ("x", "70%"),
    ("y", "80%"),
    ("z", "90%"),
    ("a", "95%"),
    ("b", "100%"),
    ("c", "105%"),
    ("d", "110%"),
    ("e", "115%"),
    ("f", "120%"),
]

TT_HEADING_ITEMS = [
    True,
    ("r", "look at ref"),
    ("n", "look at nothing"),
    ("v", "look at velocity"),
    ("g", "look at gate"),
]

FC_AUTOSTART_ITEMS = [
    True,
    ("q", "True"),
    ("w", "False"),
]

FC_OVERWRITE_ITEMS = [
    True,
    ("o", "True"),
    ("u", "False"),
]

FC_FLY_ITEMS = [
    False,
    ("0", "go to center"),
    ("p", "go to above nn_init"),
    ("t", "takeoff"),
    ("s", "set waypoint here"),
    ("h", "toggle heading mode"),
    ("1", "initTrajectoryTracker"),
    ("2", "decrease speed by 0.1"),
    ("3", "increase speed by 0.1"),
    ("4", "stopTrajectoryTracker"),
    ("8", "recovery_mode"),
    ("5", "land"),
    ("6", "nn_init"),
    ("7", "nn_activate / nn_deactivate"),
]

# menu of menu's must be a dict where the keys are the displayed text, and the submenu is the item
TT_ITEMS = {"Set initial speed": TT_SPEED_ITEMS,
            "Set heading mode": TT_HEADING_ITEMS,
            "Exit": 0}
FC_ITEMS = {"Configure autostart": FC_AUTOSTART_ITEMS,
            "Configure state overwrite": FC_OVERWRITE_ITEMS,
            "Go Fly!": FC_FLY_ITEMS,
            "Exit": 0}
MAIN_ITEMS = {"Configure Neural Nets": NN_ITEMS,
              "Configure Trajectory Tracker": TT_ITEMS,
              "Flight Control": FC_ITEMS,
              "Exit": 0}


def send_key(key, sock, hostport, dialog):
    msg = KEYBOARD.copy()
    if key in hid_codes.keys():
        msg['time_us'] = int(0)
        msg['key'] = hid_codes[key]
        msg_packed = struct.pack(KEYBOARD_format, *msg.values())
        sock.sendto(msg_packed, hostport)
    else:
        dialog.msgbox(f"ERROR: key {key} not found in hid_codes")


if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument('--host', required=False, default="10.0.0.1", type=str)
    parser.add_argument('--port', required=False, default=5007, type=int)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    d = Dialog(dialog="dialog")
    d.set_background_title("gotta go faaast")

    make_tuples = lambda myList: [(x, "") for x in myList]

    menu_stack = [MAIN_ITEMS]
    menu_title_stack = ["Main Menu"]

    try:
        while True:
            if len(menu_stack) == 0:
                break

            menu = menu_stack[-1]

            if isinstance(menu, dict): # menu of submenues
                choices = [(choice, "") for choice in menu.keys()]
            else: # menu of keystrokes
                choices = menu[1:]

            okcancel, choice = d.menu(" > ".join(menu_title_stack), choices=choices, height=25, width=70)

            if choice == "Exit" or choice == '' or okcancel == 'cancel':
                menu_stack.pop()
                menu_title_stack.pop()
                continue

            if isinstance(menu, dict): # menu of submenues
                menu_stack.append(menu[choice])
                menu_title_stack.append(choice)
            else: # send key!
                send_key(choice, sock, (args.host, args.port), d)
                d.set_background_title(f"Sent {choice}")
                if menu[0]:
                    # return requested
                    menu_stack.pop()
                    menu_title_stack.pop()

    except KeyboardInterrupt:
        print("\nExiting...")
