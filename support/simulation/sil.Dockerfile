# Image to launch a PyNDIflight simulation from a local indiflight clone
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

FROM indiflight-builder

ENV DEBIAN_FRONTEND=noninteractive \
    TZ="Europe/Amsterdam"

RUN apt-get update \
    && apt-get --no-install-recommends install -y \
        gdb \
        gdbserver \
        locales \
        software-properties-common \
        curl \
        python3.10-venv \
    && rm -rf /usr/var/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN export LANG=en_us.UTF-8

# do requirements first, so rebuilding caches this even if Simulation code changes
COPY requirements.txt /requirements.txt
RUN python3 -m venv /python-venv \
    && /python-venv/bin/pip install -r /requirements.txt

# RUN add-apt-repository universe
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# RUN apt update \
#     && apt-get --no-install-recommends install -y \
#     ros-humble-ros-base \
#     ros-dev-tools \
#     && rm -rf /usr/var/apt/lists/*
# 
# ENV ROS_DISTRO=humble
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
#     && mkdir /uros_ws \
#     && cd /uros_ws \
#     && git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
#     && rosdep init \
#     && rosdep update \
#     && rosdep install --from-paths src --ignore-src -y \
#     && colcon build"
# 
# WORKDIR /uros_ws
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
#     && source install/local_setup.bash \
#     && ros2 run micro_ros_setup create_firmware_ws.sh host generic \
#     && ros2 run micro_ros_setup build_firmware.sh"

EXPOSE 5000
EXPOSE 3333

#ENV EXTRA="-Wno-double-promotion -Wno-misleading-indentation"
ADD --chmod=755 entrypoint.sh /entrypoint.sh

WORKDIR /indiflight
ENTRYPOINT [ "/entrypoint.sh" ]
