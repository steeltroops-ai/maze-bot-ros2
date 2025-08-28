#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_maze_bot_description = get_package_share_directory('maze_bot_description')
    pkg_maze_bot_gazebo = get_package_share_directory('maze_bot_gazebo')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='-4.0')

    # World file
    world = os.path.join(pkg_maze_bot_gazebo, 'worlds', 'maze_world.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world}.items(),
    )

    # URDF file path
    urdf_file = os.path.join(pkg_maze_bot_description, 'urdf', 'maze_bot_complete.urdf.xacro')
    
    # Robot State Publisher
    robot_description_config = Command(['xacro ', urdf_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_description_config
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'maze_bot',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.01'],
        output='screen'
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

        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])
