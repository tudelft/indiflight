if you want make changes: do it in src/relay.cpp

to build:

    mkdir build && cd build
    cmake ..
    make

to build for Orin (on the Orin)

    mkdir build && cd build
    cmake .. -DORIN=ON
    make

start connection:

    sudo ./relay

## Optional: unit file

A systemd unit file is provided so systemd can manage that the client is always running, even after reboot.

Install, enable and run on the pi:
```shell
sudo install -m 644 /home/pi/relay/build-aarch64-linux-gnu/relay.service /lib/systemd/system/relay.service
sudo systemctl daemon-reload
sudo systemctl enable relay.service
sudo systemctl start relay.service
```
