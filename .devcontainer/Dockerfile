FROM ros:jazzy-ros-base

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WsF=/workspaces/ROS2-Bootcamp

ENV SHELL=/bin/bash
ENV WORKSPACE_PATH=$WsF

RUN if id -u $USER_UID >/dev/null 2>&1; then \
        userdel -r $(id -un $USER_UID); \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        python3-pip \
        nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2cli \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-turtlesim \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
