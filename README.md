# Autonomous Maze-Solving Robot System

A comprehensive ROS2-based autonomous navigation system featuring a differential drive robot with LiDAR sensor for maze-solving applications.

## 🎯 Overview

This project implements an autonomous maze-solving robot using ROS2, Gazebo simulation, and advanced navigation algorithms. The system features:

- **Differential drive robot** with LiDAR sensor
- **Wall-following navigation** algorithm with obstacle avoidance
- **Gazebo simulation environment** with customizable maze worlds
- **Real-time parameter tuning** and configuration
- **Comprehensive testing and validation** tools

## 🏗️ System Architecture

```
maze-bot-ros2/
├── src/
│   ├── maze_bot_description/     # Robot URDF models and configurations
│   ├── maze_bot_gazebo/          # Simulation worlds and launch files
│   ├── maze_bot_navigation/      # Navigation algorithms and nodes
│   └── maze_bot_bringup/         # System integration and launch files
├── scripts/                      # Build and test automation scripts
└── README.md                     # This documentation
```

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 or 22.04
- **ROS2**: Humble Hawksbill (recommended) or Foxy Fitzroy
- **Gazebo**: Version 11 or later
- **Python**: 3.8+
- **C++**: C++17 compatible compiler

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
```

## 🚀 Quick Start

### 1. Clone and Build
```bash
# Clone the repository
git clone https://github.com/steeltroops-ai/maze-bot-ros2.git
cd maze-bot-ros2

# Make build script executable
chmod +x scripts/build_and_test.sh

# Build and test the project
./scripts/build_and_test.sh
```

### 2. Launch the Simulation
```bash
# Source the workspace
source install/setup.bash

# Launch the complete simulation system
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py
```

### 3. Visualize with RViz (Optional)
```bash
# In a new terminal, source the workspace
source install/setup.bash

# Launch RViz for visualization
ros2 launch maze_bot_bringup rviz.launch.py
```

## 🎮 Usage Examples

### Basic Navigation
The robot will automatically start navigating toward the goal position (4.0, 4.0) using wall-following algorithm.

### Custom Goal Position
```bash
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py \
  params_file:=path/to/custom_params.yaml
```

### Different Starting Position
```bash
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py \
  x_pose:=-2.0 y_pose:=-2.0
```

### Manual Control (Emergency)
```bash
# Stop the navigation node
ros2 lifecycle set /maze_navigator shutdown

# Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ⚙️ Configuration Parameters

### Navigation Parameters (`navigation_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x` | 4.0 | Target X coordinate |
| `goal_y` | 4.0 | Target Y coordinate |
| `goal_tolerance` | 0.5 | Distance threshold for goal reaching |
| `max_linear_speed` | 0.5 | Maximum forward speed (m/s) |
| `max_angular_speed` | 1.0 | Maximum rotation speed (rad/s) |
| `wall_distance` | 0.7 | Desired distance from wall (m) |
| `front_distance` | 0.8 | Minimum front clearance (m) |
| `kp_wall` | 2.0 | Wall-following proportional gain |
| `kd_wall` | 0.5 | Wall-following derivative gain |

### Robot Spawn Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `x_pose` | -4.0 | Initial X position |
| `y_pose` | -4.0 | Initial Y position |
| `use_sim_time` | true | Use simulation time |

## 🧪 Testing and Validation

### Automated Testing
```bash
# Run comprehensive build and validation tests
./scripts/build_and_test.sh

# Run navigation performance test
chmod +x scripts/test_navigation.py
source install/setup.bash
python3 scripts/test_navigation.py
```

### Manual Testing Checklist

1. **Robot Spawning**: ✅ Robot appears in Gazebo at correct position
2. **Sensor Data**: ✅ LiDAR data visible in RViz
3. **Obstacle Avoidance**: ✅ Robot avoids walls and obstacles
4. **Wall Following**: ✅ Robot maintains consistent wall distance
5. **Goal Reaching**: ✅ Robot reaches target destination
6. **Safety Stops**: ✅ Robot stops when obstacles are too close

## 🔧 Troubleshooting

### Common Issues

**Issue**: Robot doesn't move
- **Solution**: Check if navigation node is running: `ros2 node list | grep maze_navigator`
- **Solution**: Verify cmd_vel topic: `ros2 topic echo /cmd_vel`

**Issue**: Gazebo crashes on startup
- **Solution**: Ensure sufficient system resources (4GB+ RAM recommended)
- **Solution**: Update graphics drivers

**Issue**: Robot gets stuck in corners
- **Solution**: Adjust `wall_distance` and PID parameters
- **Solution**: Reduce `max_linear_speed` for better control

**Issue**: Navigation too aggressive
- **Solution**: Reduce `kp_wall` gain
- **Solution**: Increase `front_distance` threshold

### Debug Commands
```bash
# Check active nodes
ros2 node list

# Monitor navigation status
ros2 topic echo /cmd_vel

# View laser scan data
ros2 topic echo /scan

# Check robot position
ros2 topic echo /odom

# View parameter values
ros2 param list /maze_navigator
ros2 param get /maze_navigator goal_x
```

## 📊 Performance Metrics

### Expected Performance
- **Navigation Success Rate**: >90% in standard maze configurations
- **Average Completion Time**: 2-5 minutes depending on maze complexity
- **Collision Avoidance**: 100% success rate with proper tuning
- **Goal Accuracy**: ±0.5m from target position

### Benchmarking
```bash
# Run performance benchmark
source install/setup.bash
python3 scripts/test_navigation.py
```

## 🔄 Advanced Extensions

### SLAM Integration (Future Enhancement)
```bash
# Install Nav2 stack
sudo apt install ros-humble-nav2-bringup

# Launch with SLAM
ros2 launch nav2_bringup slam_launch.py
```

### Custom Maze Worlds
1. Edit `src/maze_bot_gazebo/worlds/maze_world.world`
2. Add custom wall configurations
3. Rebuild: `colcon build --packages-select maze_bot_gazebo`

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request