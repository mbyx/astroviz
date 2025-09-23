#!/bin/bash
set -e  # Exit on error

# Ensure a device path is provided
if [[ -z "$1" ]]; then
    echo "Error: No IP address provided."
    echo "Usage: $0 <ip-address>"
    echo $1
    exit 1
fi

IP_ADDRESS="$1"
shift  # Remove the first argument to allow additional arguments

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Start zenoh-bridge-ros2dds
exec zenoh-bridge-ros2dds -l udp/"$IP_ADDRESS":7443

