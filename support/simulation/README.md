# PyNDIflight -- Simulation environment for Indiflight

This README is indended to simulate your own crafts with INDIflight.

Additionally, this simulation environment was also used to generate the benchmark data linked below, which was also used in two conference papers. 
To reproduce these, the [README](README.md#References) of this repo for details.

[![Dataset](http://img.shields.io/badge/Dataset-10.4121/0530be90--cc6c--4029--9774--670657882906-yellow?logo=doi)](http://doi.org/10.4121/0530be90-cc6c-4029-9774-670657882906) T. M. Blaha, E. J. J. Smeur, and B. D. W. Remes "Flight Data of A Quadrotor Launched in the Air while Learning its own Flight Model and Controller" 2024. 4TU.ResearchData repository.


## (partial) Software in the Loop simulation

This variant runs part of the Indiflight code on your local workstation, with 
the purpose of testing and debugging estimation and control code, but not
debugging device drivers, schedulers, or interfaces. We call such a build a 
`MOCKUP`, as it's not the whole package.

### Quickstart using Docker

(tested on Ubuntu 22.04, install docker like https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

Run all commands from the root directory of this repo.

#### Step 1 -- Build the container:

    docker build . -t pyndiflight -f Simulation/sil-githash.Dockerfile \
        --build-arg COMMIT=85e336c1e \
        --build-arg BUILD_CONFIG=./Simulation/config/mockupConfig.mk

- downloads an `indiflight` commit
- builds the `MOCKUP` library using a build configuration (aka. `local.mk` file)
- provides facilities to pass in an `indiflight` configuration profile and python simulation script

#### Step 2 -- Run the container (sorry for the long arguments...) and open http://localhost:5000

    docker run -it -p 5000:5000                                           \
        -v ./Simulation/exampleQuadSim.py:/sim.py                         \
        -v ./Simulation/config/exampleINDIflightProfile.txt:/profile.txt  \
        pyndiflight --throw --learn --sil-log

Explanation of arguments:

```
-it                                                               # ensure proper printing of status bar
-p 5000:5000                                                      # Map ports for visualisation
-v ./Simulation/exampleQuadSim.py:/sim.py                         # Map simulation script into container
-v ./Simulation/config/exampleINDIflightProfile.txt:/profile.txt  # Map runtime config into container
pyndiflight                                                       # Container tag (see build command)
--throw --learn --sil-log                                         # see exampleQuadSim.py --help
```

#### Step 3 -- Get log from last-run container

    docker cp $(docker ps -alq):/logs/. ./logs/

NB: logs are not persistent (unless you mount another volume to `:/logs`)


### Development setup using Docker

This variant uses a local copy of the `indiflight` source code, which is build
at container runtime.

#### Step 1 -- Build indiflight docker builder

Clone the indiflight firmware repo (https://github.com/tudelft/indiflight), 
and build the builder container as indicated in its `README.md`.

#### Step 2 -- Build local pyndiflight docker container

**NOTE**: Run all following commands from the root folder of this repo.

    docker build . -t pyndiflight-local -f ./Simulation/sil-local.Dockerfile

If you want to debug the indiflight code using GDB, you can use this instead:

    docker build . -t pyndiflight-local-gdb -f ./Simulation/sil-local-gdb.Dockerfile

(rebuild it only when `indiflight` build container or `PyNDIflight` simulation
code changes)

#### Step 3 -- Run the container and open http://localhost:5000

Don't forget to provide a `make/local.mk` file in the `indiflight` repo (see
its `README`). Also enable `USE_THROW_TO_ARM` and `USE_LEARNER`, for the
example simulation to make sense.

    docker run -it -p 5000:5000 -p 3333:3333                             \
        -v ./Simulation/exampleQuadSim.py:/sim.py                        \
        -v ./Simulation/config/exampleINDIflightProfile.txt:/profile.txt \
        -v <path/to/indiflight>:/indiflight                              \
        pyndiflight-local --throw --learn --sil-log

Explanation of arguments:

```
-it                                                               # ensure proper printing of status bar
-p 5000:5000 -p 3333:3333                                         # Map ports for visualisation and, if used gdbserver
-v ./Simulation/exampleQuadSim.py:/sim.py                         # Map simulation script into container
-v ./Simulation/config/exampleINDIflightProfile.txt:/profile.txt  # Map runtime config into container
-v <path/to/indiflight>:/indiflight                               # Map indiflight into the container
pyndiflight-local                                                 # Container tag (see build command)
--throw --learn --sil-log                                         # see ./exampleQuadSim.py --help
```

To debug INDIflight, replace `pyndiflight-local` with `pyndiflight-local-gdb`.
This will launch a `gdbserver` that you can connect to, e.g. from VSCode. In
the `indiflight` repo, there is a `launch.json` configuration that can be 
started from within VSCode. 

#### Step 4 -- Connect to the debug server from VSCode

Open a VSCode window in the `indiflight` repo that you cloned. Install the 
`Native Debug` extension and run the `MOCKUP docker` debug launch configuration.
It will connect to the container and allow pausing the app and settings 
breakpoints and so on. Open your browser at `http://localhost:5000` for the visualization

Of course you can also connect other debuggers, see the `.vscode/launch.json`
in `indiflight` for the arguments used.


#### Step 5 -- Get log from latest container

    docker cp $(docker ps -alq):/logs/. ./logs/

NB: logs are not persistent (unless you mount another volume to `:/logs`)


### Software architecture and limitations

Interfacing is done using direct C API and the simulation is advanced using 
a `tick` function. The only parts of Indiflight that are run here are the 
following tasks:
- `FILTER` (gyro filtering) at every `tick()`
- `INNER_LOOP` (attitude/rate control) at every `tick()`
- `ACCEL` (accelerometer processing) at every second `tick()`
- `ATTITUDE` (attitude and position estimation using complementary filter) at every second `tick()`
- `EKF` at every second `tick()`
- `POS_CTL` at every second `tick()`



## Hardware in the Loop simulation

With HIL simulation, Indiflight runs on the target hardware as it would during 
flight. However, a serial communications link provides simulated sensor values
in and accepts the computed actuator commands, instead of sending them to the 
real ESCs.

This can be used to test all components except the IMU and DSHOT drivers, but
requires a full Flight Controller + Remote Control setup, and additionally 
an FTDI serial-to-usb connection.

**NOTE: ALWAYS** power the FTDI adapter before the flight controller.

(Limitation: only up to 4 actuators for now)

### Setup

#### Step 1 -- Build indiflight docker builder (same as Step 1 of Dev Setup for SIL)

Clone the indiflight repo (https://github.com/tudelft/indiflight), and build 
the builder container as indicated in its `README.md`.


#### Step 2 -- Compile for target hardware

Provide a `make/local.mk` for your target hardware in the cloned indiflight
repo (see its README).

Now compile and flash the resulting binary (also check README):

    docker run --privileged                                  \
        -v </path/to/indiflight>:/indiflight                 \
        indiflight                                           \
        DEBUG=GDB EXTRA_FLAGS_CMDLINE=-DHIL_BUILD dfu_flash

(if configured, you can also use `remote_flash_swd`, see indiflight README)


#### Step 3 -- Configure using indiflight-configurator

Setup the HIL interface in the Ports tab of the Configurator (use `auto` baudrate).


#### Step 4 -- Build the hil-simulation container

    docker build . -t pyndiflight-hil -f Simulation/hil.Dockerfile


#### Step 5 -- Run the container and open http://localhost:5000

    docker run --privileged -it -p 5000:5000                     \
        -v ./Simulation/exampleQuadSim.py:/sim.py                \
        pyndiflight-hil --hil /dev/ttyUSB0 --mocap 10.0.0.1 5005 \
        --throw

