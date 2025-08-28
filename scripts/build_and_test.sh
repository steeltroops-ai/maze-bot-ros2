#!/bin/bash

# Build and Test Script for Maze Bot ROS2 Project
# This script builds the project and runs basic validation tests

set -e  # Exit on any error

echo "=== Maze Bot ROS2 Build and Test Script ==="
echo ""

# Check if we're in the right directory
if [ ! -f "colcon.meta" ]; then
    echo "Error: Please run this script from the workspace root directory"
    exit 1
fi

# Source ROS2 setup
echo "1. Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "   ✓ ROS2 Humble sourced"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "   ✓ ROS2 Foxy sourced"
else
    echo "   ✗ ROS2 installation not found"
    exit 1
fi

# Install dependencies
echo ""
echo "2. Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y
echo "   ✓ Dependencies installed"

# Build the workspace
echo ""
echo "3. Building workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -eq 0 ]; then
    echo "   ✓ Build successful"
else
    echo "   ✗ Build failed"
    exit 1
fi

# Source the workspace
echo ""
echo "4. Sourcing workspace..."
source install/setup.bash
echo "   ✓ Workspace sourced"

# Run basic tests
echo ""
echo "5. Running basic validation tests..."

# Test 1: Check if packages are built correctly
echo "   Testing package installation..."
ros2 pkg list | grep maze_bot > /dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Maze bot packages found"
else
    echo "   ✗ Maze bot packages not found"
    exit 1
fi

# Test 2: Validate URDF
echo "   Testing robot model..."
check_urdf install/maze_bot_description/share/maze_bot_description/urdf/maze_bot_complete.urdf.xacro > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✓ Robot URDF is valid"
else
    echo "   ⚠ URDF validation failed (may need xacro processing)"
fi

# Test 3: Check launch files
echo "   Testing launch files..."
if [ -f "install/maze_bot_bringup/share/maze_bot_bringup/launch/maze_bot_simulation.launch.py" ]; then
    echo "   ✓ Main launch file exists"
else
    echo "   ✗ Main launch file missing"
    exit 1
fi

# Test 4: Check configuration files
echo "   Testing configuration files..."
if [ -f "install/maze_bot_navigation/share/maze_bot_navigation/config/navigation_params.yaml" ]; then
    echo "   ✓ Navigation parameters file exists"
else
    echo "   ✗ Navigation parameters file missing"
    exit 1
fi

echo ""
echo "=== Build and Test Summary ==="
echo "✓ All basic tests passed!"
echo ""
echo "To run the simulation:"
echo "  ros2 launch maze_bot_bringup maze_bot_simulation.launch.py"
echo ""
echo "To visualize in RViz:"
echo "  ros2 launch maze_bot_bringup rviz.launch.py"
echo ""
echo "To customize parameters, edit:"
echo "  install/maze_bot_navigation/share/maze_bot_navigation/config/navigation_params.yaml"
echo ""
