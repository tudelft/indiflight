#!/usr/bin/env python3

import sys
import time
import serial
from tqdm import tqdm

# untested chatGPT code
def main():
    # Validate input arguments
    if len(sys.argv) != 3:
        print("Usage: python script.py <serial_port> <command_file>")
        sys.exit(1)

    serial_port = sys.argv[1]
    profile_txt = sys.argv[2]

    try:
        # Open the serial port
        with serial.Serial(serial_port, baudrate=115200, timeout=2) as ser:
            # Send initial "#" to the serial port
            print(f"Attempting connection to {serial_port}... ", end="", flush=True)
            ser.write(b"#\n")

            hello = ["\r\n", "Entering CLI Mode, type 'exit' to return, or 'help'\r\n", "\r\n", "# "]
            for exp in hello:
                response = ser.readline().decode('utf-8')
                if response != exp:
                    print(f'Unexpected response from the serial port! Expected "{exp}", got "{response}"')
                    sys.exit(1)
                time.sleep(0.1)

            print(f"Connected!\n")

            # Read and process the command file
            with open(profile_txt, 'r') as file:
                lines = file.readlines()

            for line in tqdm(lines, desc=f"Sending {profile_txt}", unit="line"):
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
            ser.write(b"save\n")
            ser.write(b"save\n")

    except FileNotFoundError:
        print(f"Error: Command file '{profile_txt}' not found.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port '{serial_port}': {e}")

if __name__ == "__main__":
    main()
