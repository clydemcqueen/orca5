#!/usr/bin/env python3

"""
Bag data for camera [and imu] calibration.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = 'dwe_camera'
    gscam_config = 'udpsrc port=5600 ! application/x-rtp ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'
    skip = 2  # Reduce 30 fps to 10 fps

    nodes = [
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            parameters=[
                {
                    'camera_name': camera_name,
                    'frame_id': 'camera_sensor',
                    'gscam_config': gscam_config,
                    'skip': skip,
                    'use_sim_time': False,
                }
            ],
        ),
        Node(
            package='image_view',
            executable='image_view',
            remappings=[
                ('image', 'image_raw'),
            ],
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '/image_raw',
                '/rosout',
            ],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
