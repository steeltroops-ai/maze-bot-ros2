# Usage Guide

This guide covers how to run and use the Autonomous Maze-Solving Bot system.

## Quick Start

### Basic Simulation

Launch the complete system with default settings:

```bash
# Source the workspace
source install/setup.bash

# Launch the simulation
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py
```

This will start:
- Gazebo simulation with the maze environment
- Robot model with sensors
- Navigation algorithms
- RViz visualization

### SLAM-Enabled Navigation

For real-time mapping and navigation:

```bash
# Launch SLAM navigation
ros2 launch maze_bot_bringup slam_navigation.launch.py
```

This includes:
- SLAM toolbox for mapping
- Nav2 navigation stack
- Enhanced navigation algorithms

## Launch Options

### Simulation Parameters

Customize the simulation with these parameters:

```bash
# Custom robot starting position
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py x_pose:=-2.0 y_pose:=-2.0

# Run headless (no GUI)
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py gui:=false

# Use custom parameters file
ros2 launch maze_bot_bringup maze_bot_simulation.launch.py params_file:=/path/to/custom_params.yaml
```

### Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `maze_bot_simulation.launch.py` | Complete simulation with navigation |
| `slam_navigation.launch.py` | SLAM-enabled navigation |
| `gazebo.launch.py` | Gazebo simulation only |
| `rviz.launch.py` | RViz visualization only |
| `navigation.launch.py` | Navigation algorithms only |

## Manual Control

### Teleoperation

Control the robot manually using keyboard:

```bash
# Install teleop package if not available
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use these keys:
- `i` - Move forward
- `k` - Stop
- `j` - Turn left
- `l` - Turn right
- `u` - Forward + left
- `o` - Forward + right

### Goal Setting

Set navigation goals using RViz:

1. Open RViz (should launch automatically with simulation)
2. Click "2D Nav Goal" button in toolbar
3. Click and drag on the map to set goal position and orientation
4. Robot will navigate to the goal automatically

Or set goals programmatically:

```bash
# Set goal using command line
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 4.0
    y: 4.0
    z: 0.0
  orientation:
    w: 1.0"
```

## Monitoring and Debugging

### View Robot Status

```bash
# Check robot topics
ros2 topic list | grep maze

# Monitor robot position
ros2 topic echo /odom

# Monitor laser scan data
ros2 topic echo /scan

# Check navigation status
ros2 topic echo /navigation_status
```

### Performance Monitoring

```bash
# Monitor system performance
ros2 run rqt_graph rqt_graph

# View computation graph
ros2 run rqt_tf_tree rqt_tf_tree

# Monitor resource usage
htop
```

## Testing and Validation

### Robot Stability Test

Test robot physics and stability:

```bash
python3 scripts/test_robot_stability.py
```

This test will:
- Monitor robot orientation and detect flipping
- Test various motion patterns
- Generate stability score and recommendations

### Navigation Performance Test

Evaluate navigation algorithm performance:

```bash
python3 scripts/test_navigation_algorithms.py
```

This test will:
- Run multiple navigation scenarios
- Measure success rate and efficiency
- Generate performance reports
- Save detailed results to JSON files

### Custom Testing

Run the enhanced navigation test:

```bash
python3 scripts/test_navigation.py
```

## Data Collection

### Recording Simulation Data

Record simulation data for analysis:

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /odom /scan /cmd_vel /goal_pose
```

### Playback Recorded Data

```bash
# Play back recorded data
ros2 bag play <bag_file_name>
```

## Configuration

### Parameter Tuning

Key parameters can be adjusted in:
- `src/maze_bot_navigation/config/navigation_params.yaml`
- `src/maze_bot_bringup/config/nav2_params.yaml`
- `src/maze_bot_bringup/config/slam_params.yaml`

See the [Parameters Guide](parameters.md) for detailed configuration options.

### Environment Variables

Important environment variables:

```bash
# ROS2 domain (for multiple robots)
export ROS_DOMAIN_ID=0

# Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/maze_bot_gazebo/models

# Display for GUI applications (WSL)
export DISPLAY=:0
```

## Troubleshooting

For common issues and solutions, see the [Troubleshooting Guide](troubleshooting.md).

## Advanced Usage

### Multi-Robot Simulation

Run multiple robots:

```bash
# Robot 1
ROS_DOMAIN_ID=1 ros2 launch maze_bot_bringup maze_bot_simulation.launch.py robot_name:=robot1

# Robot 2 (in new terminal)
ROS_DOMAIN_ID=2 ros2 launch maze_bot_bringup maze_bot_simulation.launch.py robot_name:=robot2
```

### Custom Worlds

Create custom maze environments by modifying:
- `src/maze_bot_gazebo/worlds/maze_world.world`
- Add new world files and update launch files accordingly

### Integration with External Systems

The system can be integrated with:
- External path planning systems
- Computer vision pipelines
- Machine learning frameworks
- Real robot hardware (with appropriate drivers)
