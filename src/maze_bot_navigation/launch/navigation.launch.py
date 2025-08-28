#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package directory
    pkg_maze_bot_navigation = get_package_share_directory('maze_bot_navigation')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Default parameters file
    default_params_file = os.path.join(pkg_maze_bot_navigation, 'config', 'navigation_params.yaml')

    # Navigation node
    navigation_node = Node(
        package='maze_bot_navigation',
        executable='maze_navigator',
        name='maze_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'),

        navigation_node
    ])
