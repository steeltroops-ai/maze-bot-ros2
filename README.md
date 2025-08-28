# Autonomous Maze Bot

A ROS2-based robot that can navigate through mazes using LiDAR sensors and navigation algorithms.

## Overview

This project implements a maze-solving robot using ROS2 Humble and Gazebo simulation. The robot uses sensor data to avoid obstacles and find paths through complex environments.

## Features

- Autonomous maze navigation
- LiDAR-based obstacle detection
- Computer vision target detection
- Real-time path planning
- Safety systems and collision avoidance

## Project Structure

```
src/
├── maze_bot_description/     # Robot URDF and meshes
├── maze_bot_gazebo/         # Gazebo simulation files
├── maze_bot_navigation/     # Navigation algorithms
└── maze_bot_bringup/        # Launch files

scripts/
├── test_autonomous_navigation.py    # Test script
└── autonomous_navigation_demo.py    # Demo script
```

## Build Instructions

1. **Install Dependencies**
   ```bash
sudo apt update
   sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   sudo apt install python3-colcon-common-extensions
```

2. **Build the Project**
   ```bash
cd maze-bot-ros2
   colcon build
   source install/setup.bash
```

## How to Run

### Basic Simulation
```bash
# Launch the complete system
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py

# In a new terminal, send navigation goals using RViz
# Click "2D Nav Goal" in RViz and click on the map
```

### Individual Components
```bash
# Run just the navigation algorithm
ros2 run maze_bot_navigation maze_navigator

# Run computer vision target detection
ros2 run maze_bot_navigation vision_target_detector
```

## Usage Examples

### Setting Navigation Goals
```bash
# Send a goal via command line
ros2 topic pub /move_base_simple/goal geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position: {x: 4.0, y: 4.0, z: 0.0}
  orientation: {w: 1.0}
"
```

### Running Tests
```bash
# Run the automated test suite
python3 scripts/test_autonomous_navigation.py

# Run the demonstration
python3 scripts/autonomous_navigation_demo.py
```

### Monitoring Robot Status
```bash
# View robot position
ros2 topic echo /odom

# View laser scan data
ros2 topic echo /scan

# View navigation commands
ros2 topic echo /cmd_vel
```

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `maze_bot_simulation.launch.py` | Complete system with Gazebo |
| `slam_navigation.launch.py` | SLAM-enabled navigation |
| `gazebo.launch.py` | Gazebo simulation only |
| `rviz.launch.py` | RViz visualization only |

## Configuration

Key parameters can be adjusted in:
- `config/navigation_params.yaml` - Navigation settings
- `config/robot_params.yaml` - Robot configuration
- `worlds/maze.world` - Simulation environment

## Troubleshooting

### Common Issues

**Build Errors:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build
```

**Gazebo Won't Start:**
```bash
# Check if Gazebo is already running
killall gzserver gzclient
```

**Robot Doesn't Move:**
- Check if navigation goals are being published
- Verify laser scan data is available
- Ensure the robot is not in an obstacle

### Getting Help

1. Check the logs: `ros2 topic echo /rosout`
2. Verify topics: `ros2 topic list`
3. Check node status: `ros2 node list`

## Requirements

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo 11
- Python 3.10+

## License

This project is open source and available under the MIT License.