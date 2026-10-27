#!/usr/bin/env python3

"""
Launch a simulation using ArduSub SITL and Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_gz_dir = get_package_share_directory('orca_gz')

    nodes = [
        DeclareLaunchArgument('ardusub', default_value='True', description='Launch ardusub?'),
        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),
        DeclareLaunchArgument(
            'bridge',
            default_value='True',
            description='Launch SLAM bridge?',
        ),
        DeclareLaunchArgument(
            'orb',
            default_value='True',
            description='Launch ORB_SLAM3?',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='True',
            description='Run Gazebo with the GUI?',
        ),
        DeclareLaunchArgument(
            'speedup',
            default_value='1',
            description='ArduSub simulation speedup factor',
        ),
        # Launch Gazebo simulation, ROS-Gazebo bridges, and ArduSub SITL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_gz_dir, 'launch', 'gz.launch.py')),
            launch_arguments={
                'ardusub': LaunchConfiguration('ardusub'),
                'gz_gui': LaunchConfiguration('gz_gui'),
                'speedup': LaunchConfiguration('speedup'),
            }.items(),
        ),
        # Launch pose_to_path node for slam_path topic
        Node(
            package='orca_bridge',
            executable='pose_to_path.py',
            name='pose_to_path',
            output='screen',
            remappings=[
                ('pose', 'slam_pose'),
                ('path', 'slam_path'),
            ],
        ),
        # Launch ocean current generator (defaults to [0, 0, 0] current)
        Node(
            package='orca_gz',
            executable='ocean_current_generator.py',
            name='ocean_current_generator',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                }
            ],
        ),
        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '--use-sim-time',
                '--include-hidden-topics',
                '/clock',
                '/annotated_image',
                '/bridge_status',
                '/camera_info',
                '/camera_pose',
                '/ekf_pose',
                '/ekf_status_report',
                '/heartbeat',
                '/model/orca5/odometry',
                '/ocean_current',
                '/rosout',
                '/slam_delta',
                '/slam_path',
                '/slam_pose',
                '/slam_status',
                '/system_time',
                '/tf',
                '/tf_static',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                }
            ],
            arguments=['-d', os.path.join(orca_bringup_dir, 'rviz', 'sim.rviz')],
            condition=IfCondition(LaunchConfiguration('gz_gui')),
        ),
        # Bring up SLAM nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'bridge': LaunchConfiguration('bridge'),
                'orb': LaunchConfiguration('orb'),
                'mav_device': 'udpin:0.0.0.0:14551',
                'settings_file': os.path.join(orca_bringup_dir, 'param', 'sim.yaml'),
            }.items(),
        ),
    ]

    return LaunchDescription(nodes)
