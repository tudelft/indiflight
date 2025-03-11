Generated using https://micro.ros.org/docs/tutorials/core/first_application_linux/

On Ubuntu 22.04
```bash
source /opt/ros/humble/setup.bash

cd lib/main/micro_ros_build
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# actually build shared libraries for host (takes a while)
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
```

Install and cleanup:
```bash
mkdir lib
find install -name "*generator_c.so*" -exec cp {} lib \;
find install -name "*typesupport*_c.so*" -exec cp {} lib \;
find install -name "libmicro_ros_utilities.so*" -exec cp {} lib \;
find install -name "libmicrocdr.so*" -exec cp {} lib \;
find install -name "libmicroxrcedds_client.so*" -exec cp {} lib \;
find install -name "librclc*.so*" -exec cp {} lib \;
find install -name "librmw_microxrcedds.so*" -exec cp {} lib \;

rm -rf build firmware install log src
```
