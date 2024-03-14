#!/usr/bin/env python3
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
    data_packed = struct.pack('<'+packing, *data)

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
    parser.add_argument("packing", help="Data format used in struct.pack", type=str)
    parser.add_argument("data", nargs='+', help="Data separated by spaces", type=int)

    args = parser.parse_args()

    # do stuff
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((args.host, args.port))
        MSP_send(sock, args.command, args.data, args.packing)

