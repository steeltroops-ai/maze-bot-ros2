#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package directories
    pkg_maze_bot_gazebo = get_package_share_directory('maze_bot_gazebo')
    pkg_maze_bot_navigation = get_package_share_directory('maze_bot_navigation')
    pkg_maze_bot_bringup = get_package_share_directory('maze_bot_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_slam = LaunchConfiguration('enable_slam')

    # Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_maze_bot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Autonomous Path Planner Node
    path_planner_node = Node(
        package='maze_bot_navigation',
        executable='autonomous_path_planner',
        name='autonomous_path_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'grid_resolution': 0.05,
                'grid_width': 20.0,
                'grid_height': 20.0,
                'robot_radius': 0.25,
                'safety_margin': 0.15,
                'max_planning_time_ms': 100.0,
                'path_smoothing_factor': 0.3,
                'use_dynamic_obstacles': True
            }
        ]
    )

    # Enhanced Navigation Controller Node
    navigation_controller_node = Node(
        package='maze_bot_navigation',
        executable='enhanced_navigation_controller',
        name='enhanced_navigation_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'max_linear_speed': 1.0,
                'max_angular_speed': 1.5,
                'lookahead_distance': 1.0,
                'emergency_stop_distance': 0.3,
                'warning_distance': 0.8,
                'curvature_speed_factor': 0.5
            }
        ]
    )

    # Vision Target Detector Node (conditional)
    vision_detector_node = Node(
        package='maze_bot_navigation',
        executable='vision_target_detector',
        name='vision_target_detector',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'target_color_lower_h': 0,
                'target_color_lower_s': 100,
                'target_color_lower_v': 100,
                'target_color_upper_h': 10,
                'target_color_upper_s': 255,
                'target_color_upper_v': 255,
                'min_contour_area': 500.0,
                'known_target_size': 0.2
            }
        ],
        condition=LaunchConfiguration('enable_vision')
    )

    # SLAM Toolbox (conditional)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_maze_bot_bringup, 'config', 'slam_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        condition=LaunchConfiguration('enable_slam')
    )

    # RViz for visualization
    rviz_config_file = os.path.join(pkg_maze_bot_bringup, 'rviz', 'autonomous_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot State Publisher (from gazebo launch)
    # Joint State Publisher (from gazebo launch)

    # Delayed start for navigation components to allow Gazebo to initialize
    delayed_navigation_start = TimerAction(
        period=5.0,
        actions=[
            path_planner_node,
            navigation_controller_node
        ]
    )

    # Delayed start for vision system
    delayed_vision_start = TimerAction(
        period=8.0,
        actions=[vision_detector_node]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'x_pose', 
            default_value='-4.0',
            description='Initial x position of robot'),
        
        DeclareLaunchArgument(
            'y_pose', 
            default_value='-4.0',
            description='Initial y position of robot'),
        
        DeclareLaunchArgument(
            'enable_vision',
            default_value='true',
            description='Enable computer vision target detection'),
        
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable SLAM mapping'),

        # Launch components
        gazebo_launch,
        slam_node,
        delayed_navigation_start,
        delayed_vision_start,
        rviz_node
    ])
