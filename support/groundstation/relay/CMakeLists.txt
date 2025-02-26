# Copyright 2024 Till Blaha (Delft University of Technology)
#                Erin Lucassen (Delft University of Technology)
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

cmake_minimum_required (VERSION 3.18)
project(relay)


set(PI_GEN_DIR ${CMAKE_BINARY_DIR}/pi-protocol)
include_directories(${PI_GEN_DIR})

# compile with pi-stats support and message printing
add_compile_options(-Wall -Wpedantic -Werror) #todo: find the proper way to do this
add_definitions(-DPI_STATS -DPI_USE_PRINT_MSGS -DPI_DEBUG)

add_executable(relay ${CMAKE_SOURCE_DIR}/src/relay.cpp
    ${PI_GEN_DIR}/pi-protocol.c
    ${PI_GEN_DIR}/pi-messages.c
)

# when compiling for the orin, enable the parts of the code that do ROS2 things
option(ORIN "Compile for Orin with ROS2 VIO support" OFF)
if(ORIN)
    add_definitions(-DORIN)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
    # ament_target_dependencies(client rclcpp geometry_msgs)
    ament_target_dependencies(relay rclcpp nav_msgs)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD C++11)

# target_link_libraries(connect ${PROJECT_NAME})

# ----- python generation of pi protocol headers -----
# find Py3, setting the Python3_EXECUTATBLE CMAKE variable
find_package(Python3 COMPONENTS Interpreter)
#message(${Python3_EXECUTABLE})

set(PI_SRC_DIR ${CMAKE_SOURCE_DIR}/../../../lib/main/pi-protocol)
set(PI_CONFIG_FILENAME config.yaml)
add_custom_command(
    OUTPUT
        ${PI_GEN_DIR}/pi-protocol.h
        ${PI_GEN_DIR}/pi-messages.h
        ${PI_GEN_DIR}/pi-messages.c
    DEPENDS
        ${PI_SRC_DIR}/${PI_CONFIG_FILENAME}
        ${PI_SRC_DIR}/msgs/*.yaml
        ${PI_SRC_DIR}/python/*.py
        ${PI_SRC_DIR}/templates/*.j2
    COMMAND mkdir -p ${PI_GEN_DIR} && ${Python3_EXECUTABLE}
        ${PI_SRC_DIR}/python/generate.py
        ${PI_CONFIG_FILENAME}
        --output-dir ${PI_GEN_DIR})
configure_file(${PI_SRC_DIR}/src/pi-protocol.c ${PI_GEN_DIR}/pi-protocol.c COPYONLY)
