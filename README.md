![Indiflight](docs/assets/images/IndiflightLogoFull.png)

This is a direct fork of the 4.4-maintenance branch of Betaflight, implementing an Incremental Nonlinear Dynamic Inversion controller. Also, UART serial telemetry and uplink is improved, so that offboard position estimation via optical cameras, or even offboard control is possible. This version is meant as research software into the control of UAV and is extremely experimental: you will probably hurt yourself or others if you just flash it and expect it to work.

## Building using docker -- the preferred way

(tested on Ubuntu 22.04, install docker like https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

### Step 1 -- Create a docker image:

    docker build . -t indiflight


### Step 2 -- Create a `make/local.mk`

This defines the build configuration. For the Mateksys H743 you can use the 
following file (to do that for other FCs, check https://github.com/betaflight/config/):

```Makefile
########## TARGET CONFIG
TARGET = STM32H743
EXTRA_FLAGS = -D'BUILD_KEY=4880cd41e59e44642e41c3f6344b3993' -D'RELEASE_NAME=4.4.2' -D'BOARD_NAME=MATEKH743' -D'MANUFACTURER_ID=MTKS' -DCLOUD_BUILD -DUSE_GYRO -DUSE_GYRO_SPI_ICM42605 -DUSE_GYRO_SPI_ICM42688P -DUSE_GYRO_SPI_MPU6000 -DUSE_GYRO_SPI_MPU6500 -DUSE_ACC -DUSE_ACC_SPI_ICM42605 -DUSE_ACC_SPI_ICM42688P -DUSE_ACC_SPI_MPU6000 -DUSE_ACC_SPI_MPU6500 -DUSE_DSHOT -DUSE_LED_STRIP -DUSE_MAX7456 -DUSE_OSD -DUSE_OSD_HD -DUSE_OSD_SD -DUSE_PINIO -DUSE_BLACKBOX -DUSE_SDCARD -DUSE_SERIALRX -DUSE_SERIALRX_SBUS -DUSE_SERIALRX_CRSF -DUSE_TELEMETRY -DUSE_TELEMETRY_SMARTPORT

# Remove propellers: hardware in the loop simulation over UART
#EXTRA_FLAGS += -DHIL_BUILD

### OR

#TARGET = MOCKUP
#EXTRA_FLAGS = -Wno-double-promotion -Wno-misleading-indentation
#EXTRA_FLAGS += -DUSE_IMU_CALC -DUSE_ACC -DUSE_FAKE_ACC -DUSE_GYRO -DUSE_FAKE_GYRO -DUSE_GPS_PI -DUSE_BLACKBOX

########## FURTHER OPTIONS
# Developer options
#EXTRA_FLAGS += -DUSE_BENCHMARK -DUSE_STACK_CHECK

# indi config:
EXTRA_FLAGS += -DUSE_INDI

# telemetry config:
EXTRA_FLAGS += -DUSE_TELEMETRY -DUSE_TELEMETRY_PI -DPI_STATS -DPI_USE_PRINT_MSGS -DUSE_GPS_PI

# higher level controllers / estimators
#EXTRA_FLAGS += -DUSE_EKF
EXTRA_FLAGS += -DUSE_POS_CTL -DUSE_VEL_CTL
#EXTRA_FLAGS += -DUSE_GPS
EXTRA_FLAGS += -DUSE_TRAJECTORY_TRACKER

# DANGEROUS modes!!
#EXTRA_FLAGS += -DUSE_ACCEL_RPM_FILTER 
#EXTRA_FLAGS += -DUSE_THROW_TO_ARM -DUSE_THROWING_WITHOUT_POSITION
#EXTRA_FLAGS += -DUSE_CATAPULT
#EXTRA_FLAGS += -DUSE_LEARNER
#EXTRA_FLAGS += -DUSE_NN_CONTROL
```

### Step 3 -- First flash and configuration

Run docker image, which outputs a `./obj/*.hex` file, which can be flashed via
the [configurator](https://github.com/tudelft/indiflight-configurator) (**use "Full chip erase" option**):

    docker run -v ./:/indiflight indiflight

Now you still need to load a suitable profile via the [configurator](https://github.com/tudelft/indiflight-configurator) 
that defines the runtime-config, which is board-dependent, but also drone-dependent.
See https://github.com/tudelft/indiflightSupport for that.


### Step 4 -- Subsequent compilations

The next compilations can be much faster, without full chip erase, and even
without the configurator at all. Boot the flight controller into DFU-mode by
pressing the boot button while powering on. Then compile and flash at the same
time:

    docker run --privileged -v ./:/indiflight indiflight dfu_flash

See below for how to use a companion computer to flash and debug even faster


### Step 5 -- Other useful arguments to the container

All arguments after the image tag `indiflight`, are directly passed to `make`. Examples are:
```sh
clean                       # delete all relevant object files
DEBUG=GDB                   # no optimisations and with debug symbols (run clean before!)
DEBUG=GDB dfu_flash         # dito, but then flash via dfu after
DEBUG=GDB remote_flash_swd  # dito, but then flash via swd on a companion computer
```


## Flashing and Debugging over raspberry pi companion computer

For context, see REAMDME's of https://github.com/tudelft/racebian.

Furthermore, install:

    apt install gdb-multiarch binutils-multiarch sshpass

If connected to the racebian raspberry via wifi (such that it has ip 10.0.0.1, user pi and password pi), the following can be used to flash:

    docker run -v ./:/indiflight indiflight remote_flash_swd
    #docker run -v ./:/indiflight indiflight DEBUG=GDB remote_flash_swd

To debug within VScode, just hit `CTRL+SHIFT+D`, hit play and be a little bit patient (15sec or so? Then youll be taken to the start of `main()`).

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
