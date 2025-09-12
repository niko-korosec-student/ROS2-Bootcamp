#!/bin/bash
# Source base ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source workspace install setup if it exists
if [ -f /home/ws/install/setup.bash ]; then
    source /home/ws/install/setup.bash
fi
