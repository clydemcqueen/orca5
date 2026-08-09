#!/usr/bin/env python3

"""
Helper launch for bag file replay. Does not play the bag.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    rviz_config_file = os.path.join(orca_bringup_dir, 'rviz', 'hw_replay.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fix_tf',
                default_value='False',
                description='Publish missing static transforms for older bags',
            ),
            # Launch rviz2 with hw_replay.rviz
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
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
            # Some old bags do not have static transforms, add them conditionally
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    '0',
                    '--pitch',
                    str(math.pi / 2),
                    '--yaw',
                    '0',
                    '--frame-id',
                    'base_link',
                    '--child-frame-id',
                    'camera_link',
                ],
                condition=IfCondition(LaunchConfiguration('fix_tf')),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    str(-math.pi / 2),
                    '--pitch',
                    '0',
                    '--yaw',
                    str(-math.pi / 2),
                    '--frame-id',
                    'camera_link',
                    '--child-frame-id',
                    'camera_sensor',
                ],
                condition=IfCondition(LaunchConfiguration('fix_tf')),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    str(math.pi),
                    '--pitch',
                    '0',
                    '--yaw',
                    '0',
                    '--frame-id',
                    'slam',
                    '--child-frame-id',
                    'world',
                ],
                condition=IfCondition(LaunchConfiguration('fix_tf')),
            ),
        ]
    )
