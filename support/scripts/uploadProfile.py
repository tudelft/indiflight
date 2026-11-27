#!/usr/bin/env python3

import sys
import time
import serial
from tqdm import tqdm

from argparse import ArgumentParser
parser = ArgumentParser("Upload run-time configuration")
parser.add_argument("serial_port", type=str, help="Write to this serial port")
parser.add_argument("command_files", type=str, nargs="+", action="extend", help="Pull commands from these files")
args = parser.parse_args()

port = args.serial_port
files = args.command_files

# tested chatGPT code
def main():
    lines = []
    for file in files:
        try:
            # Read and process the command file
            with open(file, 'r') as f:
                lines.extend(f.readlines())
        except:
            print(f"Error: Could not read command file '{file}'.")
            sys.exit(1)

    try:
        success = False
        # Open the serial port
        with serial.Serial(port, baudrate=115200, timeout=2) as ser:
            # Send initial "#" to the serial port
            print(f"Attempting connection to {port}... ", end="", flush=True)
            ser.write(b"#\n")

            hello = ["\r\n", "Entering CLI Mode, type 'exit' to return, or 'help'\r\n", "\r\n", "# "]
            for exp in hello:
                response = ser.readline().decode('utf-8')
                if response != exp:
                    print(f'Unexpected response from the serial port! Expected "{exp}", got "{response}"')
                    sys.exit(1)
                time.sleep(0.1)

            print(f"Connected!\n")

            for line in tqdm(lines, desc=f"Sending {files}", unit="line"):
                # Strip comments and whitespace
                line = line.split("#")[0].strip()

                if not line:
                    continue  # Skip empty lines, which is now also commented lines

                # Send the line to the serial port
                #print(line)
                ser.write((line + "\n").encode('utf-8'))
                ser.flush()

                # discard all input (if we dont do this, we get input buffer overruns)
                ser.reset_input_buffer()

                # Wait based on the line content
                if "profile" in line:
                    time.sleep(0.1)  # 100ms for lines containing "profile"
                else:
                    time.sleep(0.02)  # 15ms for other lines

            print("\nAll commands sent. Saving and resetting... REMEMBER TO CHECK CONFIGURATOR!!!\n")
            ser.write(b"save\n")
            success = True

    except serial.SerialException as e:
        print(f"Error: Could not open serial port '{port}': {e}")
    except:
        print(f"Error: unknown error")
    finally:
        if not success:
            print("UNSUCCESFUL: Resetting flight controller")
            with serial.Serial(port, baudrate=115200, timeout=2) as ser:
                ser.write(b"exit\n")

if __name__ == "__main__":
    main()
