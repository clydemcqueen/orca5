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
    urdf_file_path = os.path.join(orca_bringup_dir, 'urdf', 'orca5_1.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fix_desc',
                default_value='False',
                description='Launch robot_state_publisher to publish robot_description for older bags',
            ),
            DeclareLaunchArgument(
                'fix_tf',
                default_value='False',
                description='Publish missing static transforms for older bags',
            ),
            DeclareLaunchArgument(
                'add_pose_to_path',
                default_value='False',
                description='Launch pose_to_path node?',
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
                condition=IfCondition(LaunchConfiguration('add_pose_to_path')),
            ),
            # Launch tracked_points_publisher node
            Node(
                package='orca_bridge',
                executable='tracked_points_publisher.py',
                name='tracked_points_publisher',
                output='screen',
            ),
            # Some old bags do not have static transforms, add them conditionally
            # TODO remove these fixes
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
            # Publish robot description topic for older bags. This will also publish tf_static.
            # The only duplicate is base_link -> camera_link, but the transform should be the same.
            # TODO remove this fix
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                    }
                ],
                condition=IfCondition(LaunchConfiguration('fix_desc')),
            ),
        ]
    )
