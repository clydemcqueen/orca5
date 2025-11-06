#!/usr/bin/env python3

"""
Bring up SLAM nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim time?',
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
            'mav_device',
            default_value='udpin:0.0.0.0:14551',
            description='MAVLink device address',
        ),

        DeclareLaunchArgument(
            'use_vpe',
            default_value='True',
            description='Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?',
        ),

        Node(
            package='orb_slam3_ros',
            executable='orb_slam3_ros_mono',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'settings_file': os.path.join(orca_bringup_dir, 'param', 'sim.yaml'),
                'world_frame_id': 'world',
            }],
            condition=IfCondition(LaunchConfiguration('orb')),
        ),

        Node(
            package='orca_bridge',
            executable='slam_bridge.py',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mav_device': LaunchConfiguration('mav_device'),
                'use_vpe': LaunchConfiguration('use_vpe'),
            }],
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),
    ]

    return LaunchDescription(nodes)
