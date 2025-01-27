# MSD_Lab Development Environment

This guide provides instructions to set up and use the MSD_Lab development environment using Docker and Visual Studio Code's Dev Containers.

## Prerequisites

- **Docker**: Ensure Docker is installed and running on your system.
- **Visual Studio Code**: Install [Visual Studio Code](https://code.visualstudio.com/).
- **Dev Containers Extension**: Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for Visual Studio Code.

## Setup Steps

1. **Clone the Repository**:
   ```bash
   git clone git@github.com:affanadeeb/MSD.git
   ```
# Open in Visual Studio Code:

- Launch Visual Studio Code.
- Open the `MSD` folder.

# Reopen in Container:

- Press `F1` to open the command palette.
- Type and select `Dev Containers: Rebuild and Reopen in Container`.
- Visual Studio Code will build the Docker container defined in the `Dockerfile` and open the workspace inside it.

# Workspace Structure

- `/ros2_ws/src`: This directory is mounted to your local workspace folder and contains your ROS 2 packages.

# Available Commands

The development environment provides action buttons in Visual Studio Code for common tasks:

## Build Workspace:

- Command: `colcon build --symlink-install`
- Description: Builds the ROS 2 workspace.
- Button Color: Green

### Test Workspace:

- Command: `colcon test --event-handlers console_direct+`
- Description: Runs tests in the ROS 2 workspace.
- Button Color: Orange

### Import Libraries:

- Command: `rm -rf /ros2_ws/src/dep_repos && mkdir -p /ros2_ws/src/dep_repos && vcs import /ros2_ws/src/dep_repos < /ros2_ws/src/dep.repos`
- Description: Removes existing dependencies, creates a new directory, and imports libraries as specified.
- Button Color: Aqua


# Credits
## Based on the structure provided by Soham Patil
## GitHub: https://github.com/soham2560