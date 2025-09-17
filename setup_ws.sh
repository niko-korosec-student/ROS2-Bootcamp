#!/bin/bash
# Source base ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source workspace install setup if it exists
if [ -n "$WORKSPACE_PATH" ] && [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    source "$WORKSPACE_PATH/install/setup.bash"
fi
