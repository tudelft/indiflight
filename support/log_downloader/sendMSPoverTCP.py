# Send an MSP command via TCP (e.g. to be forwarded to a flight controller)
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

import socket
import struct
from argparse import ArgumentParser

# reboot
# command: MSP_SET_REBOOT 68
# data: single uint8 determining the type of reboot
#    FIRMWARE: 0, # normal
#    BOOTLOADER: 1,
#    MSC: 2,
#    MSC_UTC: 3, # for Linux to recognize FAT32 timestamps properly apparently
#    BOOTLOADER_FLASH: 4, # no clue
#
# command to reboot into linux FAT32 MSC mode: ./sendMsp.py 68 B 3

#%% MSP "implementation"

#http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
MSP_PREAMBLE = b'$M'
MSP_DIRECTION_TO_FC = b'<'

def MSP_send(sock, command, data, packing):
    size = len(data)
    if (size):
        data_packed = struct.pack('<'+packing, *data)
    else:
        data_packed = b""

    crc = size ^ command
    for val in data_packed:
        crc ^= val

    #            little-endian $M           <             size command      data       crc
    msg = struct.pack('<       2s           c              B       B'+   str(size)+'s   B',
                      MSP_PREAMBLE, MSP_DIRECTION_TO_FC, size, command, data_packed,   crc)

    print(msg)
    sock.sendall(msg)

#%%

if __name__=="__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--host", default='10.0.0.1', help="TCP MSP Host IP address")
    parser.add_argument("-p", "--port", default=5761, type=int, help="TCP MSP Host port")
    #parser.add_argument("-D", "--dev", required=False, help="MSP device") # not implemented

    parser.add_argument("command", help="Command ID in decimal", type=int)
    parser.add_argument("packing", nargs='?', help="Data format used in struct.pack", type=str)
    parser.add_argument("data", nargs='*', help="Data separated by spaces", type=int)

    args = parser.parse_args()

    # do stuff
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((args.host, args.port))
        MSP_send(sock, args.command, args.data, args.packing)

