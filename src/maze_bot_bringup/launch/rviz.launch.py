#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package directory
    pkg_maze_bot_bringup = get_package_share_directory('maze_bot_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Default RViz config file
    default_rviz_config_file = os.path.join(pkg_maze_bot_bringup, 'config', 'rviz_config.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config_file,
            description='Full path to the RViz config file to use'),

        rviz_node
    ])
