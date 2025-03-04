![Indiflight](docs/assets/images/IndiflightLogoFull.png)

This is a direct fork of the 4.4-maintenance branch of Betaflight, implementing an Incremental Nonlinear Dynamic Inversion controller. Also, UART serial telemetry and uplink is improved, so that offboard position estimation via optical cameras is possible. This version is meant as research software into the control of UAV and is extremely experimental: you will probably hurt yourself or others if you just flash it and expect it to work.

Supporting software:
[![Github](https://img.shields.io/badge/Github-indiflight_support-blue?logo=github)](https://github.com/tudelft/indiflightSupport) Contains information on research papers connected with Indiflight and  Documentation on Drone builds/setups. N.B. until recently it also contained simulation and groundstation code, but this is know included in this repo.

## Introduction to Indiflight Configuration

INDIflight has compile-time configuration and run-time configuration.

- Compile-time configuration consists of macros that decide for which MCU chip to 
build and which components are included in the firmware. One reason not to 
simply include all features is to reduce firmware size, RAM usage and
computation overhead. These compiler flags are just C compiler defines that usually 
take the form `-DUSE_FEATUREXYZ`.
- Run-time configuration are parameters that the firmware reads at boot from a
separate onboard storage chip ("EEPROM"), but this chip can also be written to.

In INDIflight we now include both compile-time and run-time in the same files:

- One file (`./configs/boards/*.txt`) contains compile-time and run-time 
configuration for a specific flight control board, e.g. MCU type, compile-time 
defines for sensors present on the board, run-time config for MCU pins, ...
- another file (`./configs/profile/*.txt`) contains compile-time and run-time
configuration for a certain drone configuration, including e.g. ESC setup,
tuning, RC switch configuration, serial port setup, ...

**N.B.** previously, a `local.mk` file was used to set the build arguments. This is
not necessary anymore

**N.B.2** we don't use "Manufacturer Defaults" anymore that used to be put by
the manufacturer of a board into a special section of MCU chip flash memory 
(not EEPROM!). After flashing INDIflight, the run-time config is "naked", 
namely the defaults set with the `PG_RESET_TEMPLATE` and `PG_REGISTER_WITH_RESET_FN` macros in the c-code.


### Note on runtime configuration

The runtime parameters can be modified through:
- the indiflight configurator interface. Install the `.deb` or `.apk` from [![Github](https://img.shields.io/badge/Github-indiflight_configurator-blue?logo=github)](https://github.com/tudelft/indiflight-configurator)
- `set` commands, either via the indiflight configurator CLI, or loading `.txt` 
files of `set` commands 



## Building Indiflight

We're building with docker, because of the simplicity and reproducibility. So, first install docker engine, e.g. https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

Run all the commands below from the root of this repository.

### Step 1 -- Create a docker image of the builder

    docker build . -t indiflight-builder

### Step 2 -- Building

As mentioned above, building needs a compile-time configuration for the board 
used and the features required. These need to be supplied as environment
to the docker image:

    docker run --privileged -it                \
        -v ./:/indiflight                      \
        -e BOARD=MTKS-H743 -e PROFILE=CineRat  \
        indiflight-builder

The `.hex` binary is now available in `./obj`.

### Step 3 -- Flashing

You can flash the `.hex` using the indiflight configurator. If at any point you
get the message "Load custom defaults?", click "No".


### Step 4 -- Configuring

In the configurator, go to "pid", click "load profile" and load first the board
`.txt`, then do the same for the profile `.txt`.


### Combine step 2, 3 and 4 in one

Disconnect the indiflight configurator, then run (sometimes twice):

    docker run --privileged -it                                      \
        -v ./:/indiflight                                            \
        -e BOARD=MTKS-H743 -e PROFILE=CineRat -e UPLOAD_PARAMETERS=y \
        indiflight-builder dfu_flash

(an error such as "could not download status" is fine)

### Other useful arguments to the container

Environment variables
```sh
-e DEBUG=INFO       # compiler optimisations but with debug symbols (run clean before!)
-e DEBUG=GDB        # no compiler optimisations and with debug symbols (run clean before!)
-e EXTRA_FLAGS=...  # pass additional defines to the c compiler
```

All arguments after the image tag `indiflight`, are directly passed to `make`. Examples are:
```sh
clean             # delete all relevant object files
dfu_flash         # dito, but then flash via dfu after
remote_flash_swd  # dito, but then flash via swd on a companion computer
```


## Flashing and Debugging over raspberry pi companion computer

For context, see REAMDME's of https://github.com/tudelft/racebian.

Furthermore, install:

    apt install gdb-multiarch binutils-multiarch sshpass

Create a `remote.env` in the root of this repo:
```sh
REMOTE_IP=10.0.0.1  # ip
REMOTE_USER=pi
REMOTE_PASSWORD=pi
REMOTE_NAME=pi      # arbitrary name
```

### Remote flash

If connected to the racebian raspberry via wifi (such that it has ip 10.0.0.1, user pi and password pi), the following can be used to flash (UPLOAD_PARAMETERS not supported):

    docker run --privileged -it                \
        -v ./:/indiflight                      \
        -e BOARD=MTKS-H743 -e PROFILE=CineRat  \
        indiflight-builder remote_flash_dfu  # or remote_flash_swd


### Remote debug

(unfortunately the credentials in `remote.env` are hardcoded in `.vscode/tasks.json` and `launch.json`)

To debug within VScode:
1. Build and flash with `-e DEBUG=GDB`
2. Hit `CTRL+SHIFT+D`,
3. Select "Cortex OpenOCD"
4. Hit play and be a little bit patient (15sec or so? Then youll be taken to the start
of `main()`).

DO NOT CLICK ON `Global` variables, this froze and crashed by VScode.

You can reset and execution of betaflight by typing `monitor reset` in the `DEBUG CONSOLE`.


# From Original Betaflight README

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those whom have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
