#!/bin/bash

# Source ROS2 workspace setup
source /root/ws/install/setup.bash

# Check if the IP address is supplied
if [ -z "$1" ]
then
    echo "Please supply the IP address of the Zenoh router"
    exit 1
fi

# Start the Zenoh bridge connecting to the supplied IP address
zenoh-bridge-ros2dds -m client -e tcp/$1:7447 &

# Launch cerebra rviz
ros2 launch cerebra_rviz_plugins cerebra.launch.py

# Close Zenoh bridge on exit
kill %1
