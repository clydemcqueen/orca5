#!/usr/bin/env python3

"""
Launch Gazebo simulation, ArduSub SITL, and ROS-Gazebo bridges.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_gz_dir = get_package_share_directory('orca_gz')
    default_world = os.path.join(orca_gz_dir, 'worlds', 'pnw.world')
    default_sub_parm = os.path.join(orca_gz_dir, 'config', 'sim.parm')

    nodes = [
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ardusub?',
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
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file path',
        ),
        DeclareLaunchArgument(
            'sub_parm_file',
            default_value=default_sub_parm,
            description='ArduSub default parameter file path',
        ),
        # Launch Gazebo (headless)
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-v', '3', '-r', LaunchConfiguration('world')],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gz_gui')),
        ),
        # Launch Gazebo (GUI)
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', LaunchConfiguration('world')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gz_gui')),
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
                '/ocean_current@geometry_msgs/msg/Vector3]gz.msgs.Vector3d',
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
                    'camera_info_url': 'file://' + os.path.join(orca_gz_dir, 'config', 'sim_camera.yaml'),
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
                LaunchConfiguration('sub_parm_file'),
                '--serial1',
                'udpclient:127.0.0.1:14551',
                '--speedup',
                LaunchConfiguration('speedup'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),
    ]

    return LaunchDescription(nodes)
