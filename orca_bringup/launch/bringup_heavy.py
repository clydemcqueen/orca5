#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Bring up all nodes

Use a modified navigation_launch.py that doesn't launch velocity_smoother.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    mavros_params_file = LaunchConfiguration('mavros_params_file')
    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'param', 'nav2_params.yaml')
    orca_params_file = LaunchConfiguration('orca_params_file')

    # Rewrite to add the full path
    # The rewriter will only rewrite existing keys
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('voc_file', default_value='/home/daniel/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/daniel/ros2_ws/src/orca5/orca_bringup/cfg/sim.yaml', 
                  description='Caminho para o settings .yaml'),
        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),

        DeclareLaunchArgument(
            'mavros_params_file',
            default_value=os.path.join(orca_bringup_dir, 'param', 'sim_mavros_params.yaml'),
            description='Full path to the ROS2 parameters file to use for mavros nodes',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'param', 'sim_orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Orca nodes',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        ),

        # Translate messages MAV <-> ROS
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            # mavros_node is actually many nodes, so we can't override the name
            # name='mavros_node',
            parameters=[mavros_params_file],
            condition=IfCondition(LaunchConfiguration('mavros')),
        ),

        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            namespace='orbslam3',
            output='screen',
            parameters=[{
                'voc_file': LaunchConfiguration('voc_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'rescale': True,
                'do_rectify': False,
                'ENU_publish': True,
                'tracked_points': True, 
                'parent_frame_id': 'base_link',
                'child_frame_id': 'left_camera_link',
                'frame_id': 'map',
                'use_sim_time': True,
            }],
            remappings=[
                ('camera/left','/Passive/left/image_raw'),
                ('camera/right','/Passive/right/image_raw'),
                ('pose', '/mavros/vision_pose/pose')
            ],
            condition=IfCondition(LaunchConfiguration('slam')),
        ),

        # Manage overall system (start, stop, etc.)
        Node(
            package='orca_base',
            executable='manager',
            output='screen',
            name='manager',
            parameters=[orca_params_file],
            remappings=[
                ('/camera_pose', '/orbslam3/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        # Base controller and localizer; manage external nav input, publish tf2 transforms, etc.
        Node(
            package='orca_base',
            executable='base_controller',
            output='screen',
            name='base_controller',
            parameters=[orca_params_file],
            remappings=[
                ('/camera_pose', '/orbslam3/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('base')),
        ),

        ExecuteProcess(
            cmd=['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
                 '--z', '-0.2',
                 '--frame-id', 'map',
                 '--child-frame-id', 'base_link'],
            output='screen',
        ),

        # Replacement for an URDF file: base_link->left_camera_link is static
        ExecuteProcess(
            cmd=['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
                 '--x', '0.19',
                 '--y', '0.1',
                 '--z', '-0.201',
                 '--roll', str(-math.pi /2),
                 '--yaw', str(-math.pi /2),
                 '--frame-id', 'base_link',
                 '--child-frame-id', 'left_camera_link'],
            output='screen',
        ),

        # Replacement for an URDF file: base_link->left_camera_link is static
        ExecuteProcess(
            cmd=['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
                 '--x', '0.19',
                 '--y', '-0.1',
                 '--z', '-0.201',
                 '--roll', str(-math.pi /2),
                 '--yaw', str(-math.pi /2),
                 '--frame-id', 'base_link',
                 '--child-frame-id', 'right_camera_link'],
            output='screen',
        ),

    ])
