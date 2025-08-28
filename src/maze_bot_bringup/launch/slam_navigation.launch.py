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
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav_params_file = LaunchConfiguration('nav_params_file')

    # Default parameters files
    default_slam_params_file = os.path.join(pkg_maze_bot_bringup, 'config', 'slam_params.yaml')
    default_nav_params_file = os.path.join(pkg_maze_bot_navigation, 'config', 'navigation_params.yaml')

    # Gazebo launch
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

    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Navigation2 stack
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav_params_file
                }.items()
            )
        ]
    )

    # Enhanced navigation node (delayed to allow SLAM to initialize)
    enhanced_navigation_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_maze_bot_navigation, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav_params_file
                }.items()
            )
        ]
    )

    return LaunchDescription([
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
            'slam_params_file',
            default_value=default_slam_params_file,
            description='Full path to the SLAM parameters file'),
        
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=default_nav_params_file,
            description='Full path to the navigation parameters file'),

        gazebo_launch,
        slam_toolbox_node,
        nav2_launch,
        enhanced_navigation_launch
    ])
