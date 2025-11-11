#!/usr/bin/env python3

"""
Launch orca5 on a topside computer and connect to a BlueROV2.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    nodes = [
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
            'use_vpe',
            default_value='True',
            description='Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?',
        ),

        # Modify this for your camera
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            parameters=[{
                'camera_name': 'sim_camera',
                'camera_info_url': 'file://' + os.path.join(orca_bringup_dir, 'config', 'sim_camera.yaml'),
                'frame_id': 'camera_sensor',
                'gscam_config': 'udpsrc port=5600 ! application/x-rtp ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert',
                'skip': 2,  # Reduce 30 fps to 10 fps
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('orb')),
        ),

        # Publish the static base_link -> camera_link transform.
        # Modify this for your vehicle
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', str(math.pi / 2),
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--include-hidden-topics',
                '/bridge_status',
                '/camera_info',
                '/camera_pose',
                '/ekf_pose',
                '/ekf_status',
                '/rosout',
                '/slam_delta',
                '/slam_pose',
                '/slam_status',
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
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['-d', os.path.join(orca_bringup_dir, 'rviz', 'hw.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Bring up SLAM nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': 'False',
                'bridge': LaunchConfiguration('bridge'),
                'orb': LaunchConfiguration('orb'),
                'mav_device': 'udpin:0.0.0.0:14550',
                'use_vpe': LaunchConfiguration('use_vpe'),
            }.items(),
        ),
    ]

    return LaunchDescription(nodes)
