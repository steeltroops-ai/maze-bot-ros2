# Parameter Configuration Guide

All configurable parameters for the Autonomous Maze-Solving Bot system.

## Navigation Parameters

### Core Navigation Settings

Located in `src/maze_bot_navigation/config/navigation_params.yaml`:

```yaml
navigation_node:
  ros__parameters:
    # Goal settings
    goal_x: 4.0                    # Target X coordinate (meters)
    goal_y: 4.0                    # Target Y coordinate (meters)
    goal_tolerance: 0.5            # Distance tolerance to goal (meters)
    
    # Speed limits
    max_linear_speed: 0.8          # Maximum forward speed (m/s)
    max_angular_speed: 1.0         # Maximum rotation speed (rad/s)
    
    # Wall following parameters
    wall_distance: 0.4             # Desired distance from wall (meters)
    front_distance: 0.8            # Safe distance to front obstacles (meters)
    
    # PID controller gains
    kp_wall: 2.0                   # Proportional gain for wall following
    kd_wall: 0.5                   # Derivative gain for wall following
    
    # Safety parameters
    min_obstacle_distance: 0.2     # Minimum safe distance to obstacles (meters)
    emergency_stop_distance: 0.15  # Distance for emergency stop (meters)
```

### Algorithm-Specific Parameters

#### Bug2 Algorithm Settings

```yaml
bug2_algorithm:
  ros__parameters:
    # M-line tolerance
    m_line_tolerance: 0.3          # Tolerance for M-line detection (meters)
    
    # Hit point tracking
    max_hit_points: 10             # Maximum stored hit points
    hit_point_tolerance: 0.2       # Minimum distance between hit points (meters)
    
    # Recovery behavior
    recovery_timeout: 30.0         # Time before triggering recovery (seconds)
    max_recovery_attempts: 3       # Maximum recovery attempts
    
    # Progress monitoring
    progress_timeout: 15.0         # Time without progress before stuck detection (seconds)
    min_progress_distance: 0.1     # Minimum movement to count as progress (meters)
```

#### Potential Field Parameters

```yaml
potential_field:
  ros__parameters:
    # Attractive force
    attractive_gain: 1.0           # Strength of goal attraction
    attractive_max_distance: 5.0   # Maximum distance for attraction (meters)
    
    # Repulsive force
    repulsive_gain: 2.0            # Strength of obstacle repulsion
    repulsive_max_distance: 1.5    # Maximum distance for repulsion (meters)
    
    # Force combination
    force_scaling: 0.5             # Overall force scaling factor
    max_force_magnitude: 2.0       # Maximum combined force magnitude
```

## Robot Physical Parameters

### URDF Configuration

Located in `src/maze_bot_description/urdf/maze_bot.urdf.xacro`:

```xml
<!-- Robot dimensions -->
<xacro:property name="base_width" value="0.40"/>     <!-- Base width (meters) -->
<xacro:property name="base_length" value="0.45"/>    <!-- Base length (meters) -->
<xacro:property name="base_height" value="0.12"/>    <!-- Base height (meters) -->

<!-- Wheel parameters -->
<xacro:property name="wheel_radius" value="0.10"/>   <!-- Wheel radius (meters) -->
<xacro:property name="wheel_width" value="0.04"/>    <!-- Wheel width (meters) -->

<!-- Mass properties -->
<xacro:property name="base_mass" value="8.0"/>       <!-- Base mass (kg) -->
<xacro:property name="wheel_mass" value="1.2"/>      <!-- Wheel mass (kg) -->
<xacro:property name="caster_mass" value="0.8"/>     <!-- Caster mass (kg) -->
```

### Gazebo Physics Parameters

Located in `src/maze_bot_description/urdf/maze_bot_gazebo.xacro`:

```xml
<!-- Contact parameters -->
<mu1>1.5</mu1>                    <!-- Primary friction coefficient -->
<mu2>1.5</mu2>                    <!-- Secondary friction coefficient -->
<kp>1000000.0</kp>                <!-- Contact stiffness -->
<kd>100.0</kd>                    <!-- Contact damping -->
<minDepth>0.001</minDepth>        <!-- Minimum contact depth -->
<maxVel>1.0</maxVel>              <!-- Maximum contact velocity -->
```

## Sensor Parameters

### LiDAR Configuration

```yaml
laser_scan:
  ros__parameters:
    # Scan parameters
    min_angle: -3.14159            # Minimum scan angle (radians)
    max_angle: 3.14159             # Maximum scan angle (radians)
    angle_increment: 0.0174533     # Angular resolution (radians)
    
    # Range parameters
    min_range: 0.1                 # Minimum detection range (meters)
    max_range: 10.0                # Maximum detection range (meters)
    
    # Quality parameters
    noise_variance: 0.01           # Sensor noise variance
    update_rate: 10.0              # Scan update frequency (Hz)
```

### Odometry Parameters

```yaml
odometry:
  ros__parameters:
    # Frame configuration
    odom_frame: "odom"             # Odometry frame name
    base_frame: "base_link"        # Robot base frame name
    
    # Noise parameters
    linear_noise: 0.01             # Linear velocity noise
    angular_noise: 0.02            # Angular velocity noise
    
    # Update rate
    publish_rate: 50.0             # Odometry publish rate (Hz)
```

## SLAM Parameters

### SLAM Toolbox Configuration

Located in `src/maze_bot_bringup/config/slam_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Map parameters
    resolution: 0.05               # Map resolution (meters/pixel)
    map_update_interval: 5.0       # Map update frequency (seconds)
    
    # Scan matching
    minimum_travel_distance: 0.5   # Minimum travel for scan matching (meters)
    minimum_travel_heading: 0.5    # Minimum rotation for scan matching (radians)
    
    # Loop closure
    do_loop_closing: true          # Enable loop closure detection
    loop_search_maximum_distance: 3.0  # Maximum distance for loop search (meters)
    
    # Optimization
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
```

## Navigation2 Parameters

### Controller Configuration

Located in `src/maze_bot_bringup/config/nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    # Update rates
    controller_frequency: 20.0     # Control loop frequency (Hz)
    
    # Velocity limits
    min_x_velocity_threshold: 0.001    # Minimum X velocity (m/s)
    min_theta_velocity_threshold: 0.001 # Minimum angular velocity (rad/s)
    
    # DWB Local Planner
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.8               # Maximum linear velocity (m/s)
      max_vel_theta: 1.0           # Maximum angular velocity (rad/s)
      acc_lim_x: 2.5               # Linear acceleration limit (m/s²)
      acc_lim_theta: 3.2           # Angular acceleration limit (rad/s²)
      
      # Trajectory sampling
      vx_samples: 20               # Linear velocity samples
      vtheta_samples: 20           # Angular velocity samples
      sim_time: 1.7                # Trajectory simulation time (seconds)
```

### Costmap Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Size and resolution
      width: 3                     # Costmap width (meters)
      height: 3                    # Costmap height (meters)
      resolution: 0.05             # Costmap resolution (meters/cell)
      
      # Update rates
      update_frequency: 5.0        # Costmap update frequency (Hz)
      publish_frequency: 2.0       # Costmap publish frequency (Hz)
      
      # Robot footprint
      robot_radius: 0.22           # Robot radius for collision checking (meters)
      
      # Inflation
      inflation_radius: 0.55       # Obstacle inflation radius (meters)
      cost_scaling_factor: 3.0     # Cost scaling for inflation
```

## Performance Tuning

### Speed vs. Safety Trade-offs

For faster navigation (less safe):
```yaml
max_linear_speed: 1.2
max_angular_speed: 1.5
wall_distance: 0.3
front_distance: 0.6
```

For safer navigation (slower):
```yaml
max_linear_speed: 0.5
max_angular_speed: 0.8
wall_distance: 0.6
front_distance: 1.0
```

### Computational Performance

For better performance on slower systems:
```yaml
controller_frequency: 10.0       # Reduce from 20.0
map_update_interval: 10.0        # Increase from 5.0
publish_frequency: 1.0           # Reduce from 2.0
vx_samples: 10                   # Reduce from 20
vtheta_samples: 10               # Reduce from 20
```

### Accuracy vs. Speed

For higher accuracy (slower):
```yaml
resolution: 0.02                 # Increase resolution
goal_tolerance: 0.2              # Tighter tolerance
sim_time: 2.5                    # Longer simulation time
```

For faster execution (less accurate):
```yaml
resolution: 0.1                  # Decrease resolution
goal_tolerance: 0.8              # Looser tolerance
sim_time: 1.0                    # Shorter simulation time
```

## Environment-Specific Tuning

### Small Mazes
```yaml
max_linear_speed: 0.6
wall_distance: 0.3
front_distance: 0.5
local_costmap_width: 2
local_costmap_height: 2
```

### Large Open Areas
```yaml
max_linear_speed: 1.0
wall_distance: 0.5
front_distance: 1.0
local_costmap_width: 5
local_costmap_height: 5
```

### Narrow Corridors
```yaml
max_linear_speed: 0.4
max_angular_speed: 0.6
wall_distance: 0.2
robot_radius: 0.18
inflation_radius: 0.3
```

## Parameter Validation

### Testing Parameter Changes

1. **Backup current parameters**:
   ```bash
   cp src/maze_bot_navigation/config/navigation_params.yaml navigation_params_backup.yaml
   ```

2. **Make incremental changes**: Modify one parameter at a time

3. **Test thoroughly**: Run stability and navigation tests

4. **Monitor performance**: Check success rates and completion times

5. **Revert if needed**: Restore backup if performance degrades

### Parameter Ranges

| Parameter | Minimum | Maximum | Recommended |
|-----------|---------|---------|-------------|
| max_linear_speed | 0.1 | 2.0 | 0.8 |
| max_angular_speed | 0.1 | 3.0 | 1.0 |
| wall_distance | 0.1 | 1.0 | 0.4 |
| goal_tolerance | 0.1 | 2.0 | 0.5 |
| controller_frequency | 5.0 | 50.0 | 20.0 |

### Common Parameter Issues

- **Robot oscillates**: Reduce PID gains (kp_wall, kd_wall)
- **Robot too slow**: Increase max_linear_speed, reduce safety distances
- **Frequent collisions**: Increase wall_distance, front_distance, robot_radius
- **Poor goal reaching**: Reduce goal_tolerance, increase attractive_gain
- **High CPU usage**: Reduce update frequencies, sample counts
