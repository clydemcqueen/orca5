#!/usr/bin/env python3

"""
Launch orca5 on a topside computer and connect to a BlueROV2.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    calib_file_arg = LaunchConfiguration('camera_calibration_file').perform(context)
    calib_filename = os.path.basename(calib_file_arg)
    if not calib_filename.endswith('.yaml'):
        calib_filename += '.yaml'

    config_file_path = os.path.join(orca_bringup_dir, 'config', calib_filename)
    param_file_path = os.path.join(orca_bringup_dir, 'param', calib_filename)
    camera_info_url = 'file://' + config_file_path

    # Modify this for your ROV
    mav_device = 'udpin:0.0.0.0:14550'
    camera_name = 'dwe_camera'
    gscam_config = 'udpsrc port=5600 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'
    skip = 2  # Reduce 30 fps to 10 fps

    return [
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            parameters=[
                {
                    'camera_name': camera_name,
                    'camera_info_url': camera_info_url,
                    'frame_id': 'camera_sensor',
                    'gscam_config': gscam_config,
                    'skip': skip,
                    'use_sim_time': False,
                }
            ],
            condition=IfCondition(LaunchConfiguration('gscam2')),
        ),
        # If we are replaying a bag, publish the camera_info message here
        Node(
            package='orca_bridge',
            executable='camera_info_publisher.py',
            output='screen',
            parameters=[
                {
                    'camera_info_url': camera_info_url,
                    'frame_id': 'camera_sensor',
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('gscam2')),
        ),
        # Publish the static base_link -> camera_link transform.
        # Modify this for your vehicle
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[
                {
                    'use_sim_time': False,
                }
            ],
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
        ),
        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '--include-hidden-topics',
                '/annotated_image',
                '/bridge_status',
                '/camera_info',
                '/camera_pose',
                '/ekf_pose',
                '/ekf_status_report',
                '/heartbeat',
                '/image_raw',
                '/rosout',
                '/slam_delta',
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
                    'use_sim_time': False,
                }
            ],
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
                'mav_device': mav_device,
                'use_vpe': LaunchConfiguration('use_vpe'),
                'settings_file': param_file_path,
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_calibration_file',
                default_value='dwe_wet_800_600.yaml',
                description='ROS2 camera calibration file in orca_bringup/config (default dwe_wet_800_600.yaml)',
            ),
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
                'gscam2',
                default_value='True',
                description='Launch gscam2?',
            ),
            DeclareLaunchArgument(
                'use_vpe',
                default_value='False',
                description='Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
