FROM ros:jazzy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Base system updates + tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3-pip \
        nano \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL /bin/bash

# Install ROS2 demo nodes
RUN apt-get update && apt-get install -y \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-ros-base \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2cli \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-turtlesim \
    #ros-jazzy-slam-toolbox \
    #ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Set the default user
USER $USERNAME

# Correct workspace inside container
WORKDIR /workspaces/ROS2-Bootcamp
CMD ["/bin/bash"]
