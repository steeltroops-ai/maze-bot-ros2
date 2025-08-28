# Parameter Configuration Guide

Basic parameter settings for the maze-solving robot system.

## Navigation Parameters

### Core Navigation Settings

```yaml
# Basic navigation
max_linear_speed: 1.0        # Maximum forward speed (m/s)
max_angular_speed: 1.5       # Maximum rotation speed (rad/s)
safety_distance: 0.3         # Minimum distance to obstacles (m)
goal_tolerance: 0.2          # Distance tolerance for reaching goals (m)
```

### Sensor Parameters

```yaml
# LiDAR settings
laser_scan_topic: "/scan"
min_scan_range: 0.1          # Minimum valid range (m)
max_scan_range: 10.0         # Maximum valid range (m)

# Camera settings (for vision detection)
camera_topic: "/camera/image_raw"
target_color_hue: 0          # Target color (0-179)
min_target_area: 500         # Minimum target size (pixels)
```

### Safety Parameters

```yaml
# Safety settings
emergency_stop_distance: 0.15  # Emergency stop distance (m)
collision_timeout: 5.0         # Time to wait after collision (s)
recovery_distance: 0.5         # Distance to back up during recovery (m)
```

## Algorithm Parameters

### Wall Following

```yaml
# Wall following behavior
wall_distance: 0.5           # Preferred distance from wall (m)
wall_following_speed: 0.3    # Speed while following walls (m/s)
turn_speed: 0.5              # Angular speed for turns (rad/s)
```

### Path Planning

```yaml
# Path planning settings
planning_frequency: 10.0     # Planning update rate (Hz)
path_resolution: 0.1         # Path point spacing (m)
smoothing_iterations: 3      # Number of smoothing passes
```

## Tuning Guidelines

### Speed Settings
- Increase `max_linear_speed` for faster navigation (max 2.0 m/s)
- Decrease for more precise control in tight spaces
- `max_angular_speed` affects turning agility

### Safety Settings
- Increase `safety_distance` in cluttered environments
- Decrease `emergency_stop_distance` for more aggressive navigation
- Adjust `goal_tolerance` based on required precision

### Sensor Settings
- Adjust `min_scan_range` to filter out robot body reflections
- Modify `target_color_hue` to match your target objects
- Increase `min_target_area` to reduce false detections

## Configuration Files

Parameters are stored in:
- `config/navigation_params.yaml` - Main navigation settings
- `config/robot_params.yaml` - Robot-specific parameters
- `config/sensor_params.yaml` - Sensor configuration

## Example Configurations

### Conservative (Safe but Slow)
```yaml
max_linear_speed: 0.5
safety_distance: 0.5
emergency_stop_distance: 0.3
```

### Aggressive (Fast but Risky)
```yaml
max_linear_speed: 1.5
safety_distance: 0.2
emergency_stop_distance: 0.1
```

### Precision (Accurate but Slow)
```yaml
max_linear_speed: 0.3
goal_tolerance: 0.1
path_resolution: 0.05
```
