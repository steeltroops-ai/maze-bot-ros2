#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Package directories
    pkg_maze_bot_gazebo = get_package_share_directory('maze_bot_gazebo')
    pkg_maze_bot_navigation = get_package_share_directory('maze_bot_navigation')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    params_file = LaunchConfiguration('params_file')

    # Default parameters file
    default_params_file = os.path.join(pkg_maze_bot_navigation, 'config', 'navigation_params.yaml')

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

    # Navigation launch (delayed to allow Gazebo to fully start)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_maze_bot_navigation, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file
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
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'),

        gazebo_launch,
        navigation_launch
    ])
