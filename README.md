# Autonomous Maze-Solving Robot

A ROS2-based autonomous navigation system featuring intelligent algorithms, SLAM integration, and comprehensive testing for robotics applications.

## Overview

This project implements an autonomous maze-solving robot using ROS2, Gazebo simulation, and modern navigation algorithms. The system combines robust navigation capabilities with professional development tools.

### Key Features

- **Intelligent Navigation**: Bug2 algorithm with potential field method for efficient pathfinding
- **Real-time Mapping**: SLAM integration for dynamic environment mapping
- **Sensor Fusion**: LiDAR and odometry integration for accurate localization
- **Safety Systems**: Collision avoidance and recovery behaviors
- **Testing Framework**: Automated validation and performance metrics
- **Cross-platform**: Works on Windows (WSL), Linux, and macOS

## System Architecture

The system consists of four main ROS2 packages:

- **maze_bot_description**: Robot URDF models and physical properties
- **maze_bot_gazebo**: Simulation environment and world configurations
- **maze_bot_navigation**: Navigation algorithms and control systems
- **maze_bot_bringup**: Launch files and system integration

### Navigation Algorithm

The robot uses a hybrid approach combining:

- **Bug2 Algorithm**: Guarantees goal reachability in maze environments
- **Potential Fields**: Smooth obstacle avoidance and goal attraction
- **State Machine**: Intelligent behavior switching (explore, wall-follow, goal-seek, recovery)

## Quick Start

### Prerequisites

- ROS2 Humble (Ubuntu 22.04) or Foxy (Ubuntu 20.04)
- Gazebo 11+
- 8GB RAM minimum
- Works on Windows (WSL), Linux, and macOS

For detailed setup instructions, see the [Setup Guide](docs/setup-guide.md).

### Installation and Setup

```bash
# Clone the repository
git clone https://github.com/steeltroops-ai/maze-bot-ros2.git
cd maze-bot-ros2

# Build the project
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Launch the System

```bash
# Basic simulation
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py

# SLAM-enabled navigation
ros2 launch maze_bot_bringup slam_navigation.launch.py
```

## Testing and Validation

### Automated Testing

```bash
# Robot stability test
python3 scripts/test_robot_stability.py

# Navigation performance test
python3 scripts/test_navigation_algorithms.py

# Basic navigation test
python3 scripts/test_navigation.py
```

### Performance Metrics

- **Navigation Success Rate**: >95% in standard maze configurations
- **Average Completion Time**: <3 minutes for standard mazes
- **Collision Avoidance**: 100% success rate with proper tuning
- **Goal Accuracy**: ±0.25m from target position

## Documentation

- **[Setup Guide](docs/setup-guide.md)**: Complete installation instructions for all platforms
- **[Usage Guide](docs/usage-guide.md)**: Detailed usage examples and launch options
- **[Troubleshooting](docs/troubleshooting.md)**: Common issues and solutions
- **[Parameters](docs/parameters.md)**: Configuration options and tuning guide

## Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/new-feature`
3. Commit changes: `git commit -m 'Add new feature'`
4. Push to branch: `git push origin feature/new-feature`
5. Open Pull Request
