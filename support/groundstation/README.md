
# Indiflight Support Groundstation

This software is needed to control automatic flight of the drone without remote control. If you don't need this, don't bother with it!

You will need a Raspberry PI Zero 2W, or comparable, companion computer mounted on the drone and connected to the flight controller as shown in [../Documentation/Drones/CineRat](../Documentation/Drones/CineRat).

Supported only for Ubuntu 22.04 (and only on x86 if you're using motion capture with the optitrack system).

## Quickstart (Ubuntu 22.04)

Steps 1 through 3 are better explained in the [wiki](https://github.com/tudelft/indiflightSupport/wiki/CineRat----04-Software-Installation-and-Setup#Raspberry-Pi-installation)

1. Download Racebian 0.5.0 <https://surfdrive.surf.nl/files/index.php/s/kndkZOelKWptZtV>
2. Flash to companion computer by inserting its SD card. Modify the command for your path and sd-card number:
```
sudo dd bs=4M if=/path/to/the/image.img of=/dev/mmcblk? status=progress
```
3. Boot the companion computer and connect to its `racebian` wifi access point with password `betaflight`.
4. Download and extract `groundstation-build.zip` from <https://github.com/tudelft/indiflightSupport/releases>. 
5. Install `relay` on the companion computer as `/home/pi/relay`. Make sure `/home/pi/relay/build-aarch64-linux-gnu/relay` is executable.
6. Run `./start.sh --help` in a maximized terminal and you're on your way.

More info on `racebian`, see <https://github.com/tudelft/racebian>. 

More info on the intended hardware configuration, see [../Documentation/Drones/CineRat](../Documentation/Drones/CineRat)




## Alternative: Building the Tools yourself

### Relay program

This program runs on the companion computer and relays motion capture position data comming in from a groundstation laptop to the flight controller. It can also relays setpoints and commands.

You can compile it on the target hardware, or use a cross-compiler.

To setup the cross-compiler install docker (<https://docs.docker.com/engine/install/ubuntu/>) and do the steps in [Cross-Compiler/README.md].

Then cross-compile and deploy:
```
cd relay
docker run -v rootfs:/rootfs --mount type=bind,src=./,dst=/package pi-cross --processes=8 --clean-build --deploy=/home/pi
```

### Unified Optitrack Clients

The program to receive Motion capture data on your laptop and forward it to
the companion computer. See its README for detail on how to build.


### Scripts

This directory also contains some scripts:
```shell
.
├── keyInputs.py       # allows using keystrokes to send commands to the drone
├── setpointSender.py  # can send more custom position setpoints to the drone
└── start.sh           # in maximized terminal: starts up all components simulatneously
```


