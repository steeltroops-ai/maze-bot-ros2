# Environment Setup Guide

This guide covers setting up the development environment for the Autonomous Maze-Solving Bot on different operating systems.

## System Requirements

- **Memory**: 8GB RAM minimum, 16GB recommended
- **Storage**: 10GB free space for full installation
- **ROS2**: Humble Hawksbill (recommended)
- **Gazebo**: Version 11 or later
- **Python**: 3.8+
- **C++**: C++17 compatible compiler

## Windows with WSL2 (Recommended)

### Step 1: Enable WSL2

Open PowerShell as Administrator and run:

```powershell
wsl --install
```

Restart your computer when prompted.

### Step 2: Install Ubuntu 22.04

```powershell
# Install Ubuntu 22.04 LTS
wsl --install -d Ubuntu-22.04

# Set as default distribution
wsl --set-default Ubuntu-22.04

# Verify installation
wsl --list --verbose
```

### Step 3: Configure GUI Support

```bash
# In WSL Ubuntu terminal
sudo apt update && sudo apt upgrade -y

# Install GUI support packages
sudo apt install -y x11-apps mesa-utils

# Test GUI support (should open a clock window)
xclock
```

### Step 4: Install ROS2 and Dependencies

```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 and required packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-xacro
sudo apt install -y python3-rosdep
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Native Ubuntu (20.04/22.04)

### Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 and dependencies
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-xacro
sudo apt install -y python3-rosdep
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

## macOS

### Install Dependencies

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install required packages
brew install python@3.9
brew install cmake
brew install opencv
brew install pkg-config

# Install ROS2 Humble (binary installation)
# Follow the official ROS2 macOS installation guide at:
# https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html
```

## Project Setup

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/your-username/maze-bot-ros2.git
cd maze-bot-ros2

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the project
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Environment Configuration

Run the setup script to configure your environment:

```bash
./scripts/setup_wsl_environment.sh
```

This script will:
- Add ROS2 environment to your shell configuration
- Set up display variables for GUI applications
- Configure workspace environment variables

## Verification

Test your installation:

```bash
# Check ROS2 installation
ros2 --version

# Verify packages are available
ros2 pkg list | grep maze_bot

# Test GUI support (should open Gazebo)
gazebo --version
```

## Next Steps

Once your environment is set up, proceed to the [Usage Guide](usage-guide.md) to learn how to run the system.

For troubleshooting common issues, see the [Troubleshooting Guide](troubleshooting.md).
