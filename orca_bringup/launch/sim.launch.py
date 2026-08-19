#!/usr/bin/env python3

"""
Launch a simulation using ArduSub SITL and Gazebo.
"""

import math
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
    sub_common_parm_file = os.path.join(orca_bringup_dir, 'config', 'sim_common.parm')
    sub_vpd_parm_file = os.path.join(orca_bringup_dir, 'config', 'sim_vpd.parm')
    sub_vpe_parm_file = os.path.join(orca_bringup_dir, 'config', 'sim_vpe.parm')
    sub_vpd_parm_files = f'{sub_common_parm_file},{sub_vpd_parm_file}'
    sub_vpe_parm_files = f'{sub_common_parm_file},{sub_vpe_parm_file}'

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
            'use_vpe',
            default_value='False',
            description='Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?',
        ),
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', os.path.join(orca_bringup_dir, 'worlds', 'pnw.world')],
            output='screen',
        ),
        # Bridge images
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            output='screen',
            name='image_bridge',
            arguments=['image_raw'],
        ),
        # Bridge other gz topics, including /clock (rviz2 decay time won't work correctly w/o this)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/orca5/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
        ),
        # Publish a camera info message, required for rviz (not used by orb_slam3_ros)
        Node(
            package='orca_bridge',
            executable='camera_info_publisher.py',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'camera_info_url': 'file://' + os.path.join(orca_bringup_dir, 'config', 'sim_camera.yaml'),
                    'frame_id': 'camera_sensor',
                }
            ],
        ),
        # Publish the static base_link -> camera_link transform.
        # This must match the transform in orca5/model.sdf (see camera_* vars in generate_model.py)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[
                {
                    'use_sim_time': True,
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
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # Bring up SLAM nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'bridge': LaunchConfiguration('bridge'),
                'orb': LaunchConfiguration('orb'),
                'mav_device': 'udpin:0.0.0.0:14551',
                'use_vpe': LaunchConfiguration('use_vpe'),
                'settings_file': os.path.join(orca_bringup_dir, 'param', 'sim.yaml'),
            }.items(),
        ),
        # Launch ArduSub w/ SIM_JSON. Make sure ardusub is on the $PATH. To use the heavy (6dof) model: specify
        # vectored_6dof as the model, AND the default params must set magic ArduSub parameter FRAME_CONFIG to 2.0.
        # Yaw is provided by Gazebo, so the start yaw value is ignored.
        ExecuteProcess(
            cmd=[
                'ardusub',
                '-S',
                '--wipe',
                '-M',
                'JSON',
                '-I0',
                '--home',
                '47.6302,-122.3982391,-0.1,0',
                '--defaults',
                sub_vpe_parm_files if LaunchConfiguration('use_vpe') else sub_vpd_parm_files,
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),
    ]

    return LaunchDescription(nodes)
