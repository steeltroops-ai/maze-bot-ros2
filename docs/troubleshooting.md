# Troubleshooting Guide

This guide covers common issues and their solutions when working with the Autonomous Maze-Solving Bot.

## Build Issues

### Package Not Found Errors

**Problem**: `Package 'maze_bot_*' not found`

**Solution**:
```bash
# Ensure you're in the workspace root
cd maze-bot-ros2

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Compilation Errors

**Problem**: C++ compilation errors

**Solution**:
```bash
# Check compiler version
gcc --version

# Install required build tools
sudo apt install build-essential cmake

# Update system packages
sudo apt update && sudo apt upgrade

# Rebuild with verbose output
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --verbose
```

### Missing Dependencies

**Problem**: `Could not find a package configuration file provided by...`

**Solution**:
```bash
# Install missing ROS2 packages
sudo apt install ros-humble-<package-name>

# For navigation packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# For SLAM
sudo apt install ros-humble-slam-toolbox

# Update rosdep database
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Runtime Issues

### Gazebo Won't Start

**Problem**: Gazebo fails to launch or crashes

**Solution**:
```bash
# Check if Gazebo is already running
killall gzserver gzclient

# Clear Gazebo cache
rm -rf ~/.gazebo/log

# Test Gazebo installation
gazebo --version
gazebo --verbose

# For WSL users - ensure GUI support
export DISPLAY=:0
xhost +local:
```

### No GUI Display (WSL)

**Problem**: GUI applications don't appear

**Solution**:
```bash
# Check WSL version
wsl --version

# For WSL2 with WSLg (Windows 11)
export DISPLAY=:0

# Test GUI support
xclock

# If xclock doesn't work, install X11 server on Windows:
# Download and install VcXsrv or Xming
# Configure with "Disable access control" checked

# Alternative: Use WSL2 with Windows 11 WSLg (recommended)
```

### Robot Not Moving

**Problem**: Robot spawns but doesn't respond to navigation commands

**Solution**:
```bash
# Check if navigation node is running
ros2 node list | grep navigation

# Verify topics are publishing
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel

# Check for error messages
ros2 topic echo /rosout

# Restart navigation
ros2 lifecycle set /navigation_node shutdown
ros2 launch maze_bot_navigation navigation.launch.py
```

### Navigation Failures

**Problem**: Robot gets stuck or moves erratically

**Solution**:
```bash
# Check sensor data
ros2 topic echo /scan

# Verify odometry
ros2 topic echo /odom

# Check parameter values
ros2 param list /navigation_node
ros2 param get /navigation_node max_linear_speed

# Reset robot position
ros2 service call /reset_simulation std_srvs/srv/Empty
```

## Performance Issues

### Slow Simulation

**Problem**: Gazebo runs slowly or lags

**Solution**:
```bash
# Reduce simulation complexity
# Edit launch file to set gui:=false for headless mode

# Adjust Gazebo physics settings
# In world file, increase step size or reduce iterations

# Check system resources
htop
nvidia-smi  # If using GPU

# Close unnecessary applications
# Allocate more RAM to WSL (if using WSL)
```

### High CPU Usage

**Problem**: System becomes unresponsive

**Solution**:
```bash
# Monitor ROS2 processes
ros2 wtf

# Check node CPU usage
top -p $(pgrep -f ros2)

# Reduce update frequencies in parameter files
# navigation_params.yaml: reduce controller_frequency
# slam_params.yaml: increase map_update_interval

# Use release build instead of debug
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Network and Communication Issues

### ROS2 Communication Problems

**Problem**: Nodes can't communicate

**Solution**:
```bash
# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Verify domain ID
echo $ROS_DOMAIN_ID

# Check network configuration
ros2 doctor

# Test communication
ros2 topic pub /test std_msgs/msg/String "data: 'hello'"
ros2 topic echo /test
```

### Multiple Robot Conflicts

**Problem**: Multiple robots interfere with each other

**Solution**:
```bash
# Use different domain IDs
export ROS_DOMAIN_ID=1  # Terminal 1
export ROS_DOMAIN_ID=2  # Terminal 2

# Or use namespaces
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py robot_name:=robot1
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py robot_name:=robot2
```

## Testing and Validation Issues

### Test Scripts Fail

**Problem**: Python test scripts don't run

**Solution**:
```bash
# Check Python version
python3 --version

# Install required Python packages
pip3 install numpy matplotlib

# Make scripts executable
chmod +x scripts/*.py

# Run with explicit Python interpreter
python3 scripts/test_navigation.py

# Check for missing ROS2 Python packages
pip3 install rclpy
```

### Stability Test Failures

**Problem**: Robot flips or becomes unstable

**Solution**:
```bash
# Check URDF parameters
# Verify mass distribution in maze_bot.urdf.xacro
# Ensure center of mass is low
# Increase base width for stability

# Adjust Gazebo physics
# Increase friction coefficients
# Reduce maximum velocities
# Check contact parameters
```

## Environment-Specific Issues

### WSL-Specific Problems

**Problem**: Various WSL-related issues

**Solution**:
```bash
# Update WSL
wsl --update

# Check WSL version
wsl --version

# Restart WSL
wsl --shutdown
wsl

# Fix file permissions
sudo chmod -R 755 /mnt/c/path/to/project

# Memory issues - increase WSL memory limit
# Create .wslconfig in Windows user directory:
# [wsl2]
# memory=8GB
```

### Ubuntu Version Compatibility

**Problem**: Package conflicts between Ubuntu versions

**Solution**:
```bash
# Check Ubuntu version
lsb_release -a

# For Ubuntu 20.04, use ROS2 Foxy
# For Ubuntu 22.04, use ROS2 Humble

# Update package sources for correct ROS2 version
sudo apt update
sudo apt list --upgradable
```

## Getting Help

### Diagnostic Commands

Run these commands to gather system information:

```bash
# System information
uname -a
lsb_release -a

# ROS2 information
ros2 --version
ros2 doctor

# Package information
ros2 pkg list | grep maze_bot
colcon list

# Runtime diagnostics
ros2 node list
ros2 topic list
ros2 service list
```

### Log Files

Check these locations for detailed error messages:

```bash
# ROS2 logs
ls ~/.ros/log/

# Gazebo logs
ls ~/.gazebo/log/

# Build logs
ls log/

# System logs
journalctl -u ros2
```

### Reporting Issues

When reporting issues, include:

1. Operating system and version
2. ROS2 version
3. Error messages (full output)
4. Steps to reproduce
5. Output of diagnostic commands
6. Relevant log files

### Community Support

- ROS2 Documentation: https://docs.ros.org/en/humble/
- ROS Answers: https://answers.ros.org/
- Gazebo Documentation: http://gazebosim.org/documentation
- Navigation2 Documentation: https://navigation.ros.org/

## Prevention Tips

### Best Practices

1. **Always source environments**: Source ROS2 and workspace before running commands
2. **Use virtual environments**: Isolate Python dependencies
3. **Regular updates**: Keep system and packages updated
4. **Clean builds**: Remove build artifacts when switching branches
5. **Monitor resources**: Keep an eye on CPU and memory usage
6. **Backup configurations**: Save working parameter files
7. **Version control**: Use git to track changes
8. **Test incrementally**: Test changes in small steps
9. **Document modifications**: Keep notes of custom changes
10. **Regular validation**: Run test scripts periodically
