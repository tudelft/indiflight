# RPi cross compile solution

This piece of software cross compiles any cmake project for a remote target. 
It is not meant to compile Indiflight, but mostly to run supporting applications on a companion computer.

## Idea

Create a docker image that does all the compilations and maintains a synched copy of the RPi rootfs.

1. Derive a docker image from debian:bullseye (to get identical environment including glibc2.31)
2. install the cross-compile toolchains in it
3. expose an ENTRYPOINT that handles the building by
  1. keeping an updated copy of the raspberries `lib` and `usr` directies available to the container, for correct includes/linking.
  2. using cmake and make on a specified local package
  3. optionally deploys the build-files to a specified location on the pi

Read further for setup and the eventual build-commands

## Docker image setup

Prerequesites (execute as local user) https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04:
```bash
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce
sudo usermod -aG docker ${USER}
reboot # or open use su - ${USER} for the rest
```
<!--
Install a plugin to connect to the pi easier:
```bash
docker plugin install vieux/sshfs
```
-->

Setup of the image:
```bash
cd cross-compiler
docker build . -t pi-cross-base -f Dockerfile.cross
docker build . -t pi-cross -f Dockerfile.cross.custom
docker volume create rootfs # not sure where this is actually saved on disk... but somewhere
```

If you need more dependencies (excluding C libraries! Install those on the pi) in your image, modify or copy & modify the `Dockerfile.cross.custom` file, and rerun the last `docker buildx` command above.

## Running the build container to build for Raspberry Pi


Once setup is complete, this is the magic command:
```bash
sudo sysctl -w net.ipv4.ip_forward=1 # if you previously did make pi-routing-up or make pi-connect, this can be skipped
cd /path/to/package/root/you/want/to/build
docker run -v rootfs:/rootfs --mount type=bind,src=./,dst=/package pi-cross --sync
```

On exit, all build files are in the folder `build-aarch64-linux-gnu`.

On the first time running the container a lot of environment files are copied from the pi to the (persistent) docker volume we just created (takes roughly 10min on my wifi connection).
In subsequent builds they will only be updated when something on the pi changes, but checking for updates still takes time (+- 10sec). You can skip this step by ommitting `--sync`.

This means that the pi needs to be connected at least during the first time to do the initial sync.


### Optional container arguments 

put at the _end_ of the command above:
```bash
--sync                          # syncronize the rootfs with the pi. Do this on first command, or if libraries/includes changed in the /lib or /usr dir of the pi. Omitting is much faster, of course.
--clean-build                   # deletes the entire build-aarch64-linux-gnu folder from the local tree before compilation
--debug                         # sets -DCMAKE_BUILD_TYPE=Debug
--deploy=<DEPLOY_PATH>          # upload the build-aarch64-linux-gnu directory to the pi using rsync after building. Requires project() to be set in CMakeLists.txt and it will be uploaded to <DEPLAY_PATH>/<CMAKE_PROJECT_NAME>/build-aarch64-linux-gnu
--processes=8                   # passed to make as make -j <processes>. Default is 8.
```

## Handy Docker commands

The container can be run interactively (the building is skipped and a shell is opened). Do not use in conjunction with any of the arguments above.
```bash
docker run -it --entrypoint=/bin/bash -v rootfs:/rootfs --mount type=bind,src=./,dst=/package pi-cross
```

Pro-tip: add the following alias to your `~/.bashrc` so that you can omit `docker run -v rootfs:/rootfs --mount type=bind,src=./,dst=/package`:
```bash
alias pi-cross='docker run -v rootfs:/rootfs --mount type=bind,src=./,dst=/package pi-cross --processes=30'
```

Remove all existing containers:
```bash
docker ps -a                    # list all containers (even stopped)
docker rm -f $(docker ps -a -q) # remove all existing containers
docker image ls                 # list images (DO NOT MISTYPE AS images)
docker rmi [image_hash]         # delete image with image_hash
docker volume ls                # list volumes
docker volume rm [volume_name]  # remove volume with volume_name
```