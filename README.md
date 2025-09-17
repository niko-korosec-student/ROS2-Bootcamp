![ROS2 Bootcamp Banner](./notes/images/banner_ros2.gif)

# ROS2-Bootcamp

This repository is a learning workspace for practicing ROS 2.
It combines structured notes, cheat sheets, and Python exercises, with a ready-to-use development container and Docker setup.

## Repository Structure

```
.
├───.devcontainer      # Dev container & Docker setup for VS Code
├───cheatsheets        # Handy cheat sheets for quick reference
├───notes              # Lecture-style notes (with images)
│   └───images
└───python_intro       # Python introduction + OOP exercises
    └───OOP
```

## Quick Links

- [Notes](./notes)  
- [Cheat Sheets](./cheatsheets)  
- [Python Intro Exercises](./python_intro)  

## Development Container Setup and Usage

This repo is configured for use with **VS Code Dev Containers** + **Docker**. The container setup is defined under the `.devcontainer` folder.

### Key Path Convention: `/workspaces/ROS2-Bootcamp`

- Inside the container, the repository is mounted at `/workspaces/ROS2-Bootcamp`.
- This path is **defined once** in `.devcontainer/devcontainer.json` and `.devcontainer/Dockerfile`.
- All workspace-related commands and source scripts use this path, so it is crucial to keep this path consistent for proper operation.

### How to Set Up the Workspace on Your Machine

1. **Create a folder on your local machine** where you want to keep your projects, e.g., `~/ros2_workspace`.

2. **Open a terminal in that folder and clone this repository:**

   ```bash
   git clone https://github.com/yourusername/ROS2-Bootcamp.git

3. **Open the folder in VS Code:**

Open VS Code and select "Open Folder" → your `ROS2-Bootcamp` folder.

4. **Start the development container:**

- When prompted by VS Code, reopen the folder inside the Dev Container.
- This will build the container image (if not built) based on the `.devcontainer/Dockerfile`.
- The container mounts your local code at `/workspaces/ROS2-Bootcamp` inside the container.

5. **Access the workspace inside the container:**

The workspace folder inside the container is `/workspaces/ROS2-Bootcamp` and all commands, scripts (e.g., `setup_ws.sh`), and environment setups rely on this.

### Important Configuration Locations Using the Workspace Path

You must update path if changing the workspace directory:

*Devcontainer:*

- `"workspaceFolder": "/workspaces/ROS2-Bootcamp"`

*Docker file:*

- `ARG WsF=/workspaces/ROS2-Bootcamp`  

Any scripts or commands relying on this path

### Running and Using the Container

- The user inside the container is `nico` (default username), with passwordless sudo.
- The container runs with network, IPC, and PID namespaces shared (`--net=host`, etc.) to enable ROS 2 networking.
- Port forwarding is disabled by default in the container settings for simplicity.
- The container mounts device files for GUI access, enabling tools such as RViz.
- The `setup_ws.sh` script is added to the user's `.bashrc` to source the ROS 2 workspace environment automatically.

## Purpose
This repository is both a study resource and a practice environment:

 - Notes explain the theory behind ROS 2 concepts.
 - Cheat sheets give you quick reference material.
 - Python exercises build programming fundamentals.

Dev container setup ensures you can reproduce the environment on any machine with Docker + VS Code.