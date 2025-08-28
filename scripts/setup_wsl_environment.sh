#!/bin/bash

# WSL Environment Setup Script for Maze Bot ROS2 Project
# This script sets up the complete environment for running the autonomous maze-solving robot

set -e  # Exit on any error

echo "=== WSL Environment Setup for Maze Bot ROS2 ==="
echo ""

# Set up display for GUI applications
export DISPLAY=:0
echo "DISPLAY set to: $DISPLAY"

# Add ROS2 environment to bashrc if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "✓ Added ROS2 environment to ~/.bashrc"
fi

# Add workspace environment to bashrc if not already present
WORKSPACE_PATH="/mnt/c/Omniverse/Projects/maze-bot-ros2"
if ! grep -q "source $WORKSPACE_PATH/install/setup.bash" ~/.bashrc; then
    echo "source $WORKSPACE_PATH/install/setup.bash" >> ~/.bashrc
    echo "✓ Added workspace environment to ~/.bashrc"
fi

# Add display export to bashrc if not already present
if ! grep -q "export DISPLAY=:0" ~/.bashrc; then
    echo "export DISPLAY=:0" >> ~/.bashrc
    echo "✓ Added DISPLAY export to ~/.bashrc"
fi

# Source the environments
source /opt/ros/humble/setup.bash
source $WORKSPACE_PATH/install/setup.bash

echo ""
echo "=== Environment Setup Complete ==="
echo ""
echo "Available commands:"
echo "  Launch simulation:     ros2 launch maze_bot_bringup maze_bot_simulation.launch.py"
echo "  Launch RViz:          ros2 launch maze_bot_bringup rviz.launch.py"
echo "  Test navigation:      python3 scripts/test_navigation.py"
echo "  Check robot status:   ros2 topic list | grep maze"
echo ""
echo "GUI applications should work with WSLg (Windows 11) or X11 forwarding."
echo "If you encounter display issues, ensure WSLg is enabled or use an X11 server."
echo ""
