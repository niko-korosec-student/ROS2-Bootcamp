#!/bin/bash
# Source base ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source workspace install setup if it exists
if [ -f /workspaces/ROS2-Bootcamp/install/setup.bash ]; then
    source /workspaces/ROS2-Bootcamp/install/setup.bash
fi
