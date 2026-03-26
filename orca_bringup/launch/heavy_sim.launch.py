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
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_bringup')

    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub_heavy.parm')
    mavros_params_file = os.path.join(orca_bringup_dir, 'param', 'sim_mavros_params.yaml')
    orca_params_file = os.path.join(orca_bringup_dir, 'param', 'sim_heavy_params.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'heavy_sim_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'inpetu_heavy.world')

    sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'camera_info', 'sim_left.ini')
    sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'camera_info', 'sim_right.ini')
    return LaunchDescription([
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB with SIM_JSON?'
        ),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

       # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub w/ SIM_JSON
        # -w: wipe eeprom
        # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        # ExecuteProcess(
        #     cmd=['ardusub', '-S', '-w', '-M', 'vectored_6dof', '--defaults', ardusub_params_file,
        #          '-I0', '--home', '33.810313,-118.39386700000001,0.0,0'],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('ardusub')),
        # ),

        # 2. Define the ArduSub process
        ExecuteProcess(
            cmd=[
                'ardusub',
                '-w',
                '-M', 'JSON',
                # '--model','vectored_6dof',
                '--defaults', ardusub_params_file,
                '-I0',
                '--home', '33.810313,-118.39386700000001,0.0,0'
            ],
            output='screen'
        ),

        # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['gz', 'sim', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '3',  '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Get images from Gazebo Sim to ROS
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['Passive/left/image_raw', 'Passive/right/image_raw'],
            output='screen',
        ),

        # Gazebo Sim doesn't publish camera info, so do that here
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'left_sim_camera',
                'frame_id': 'left_camera_link',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/Passive/left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'right_sim_camera',
                'frame_id': 'right_camera_link',
                'timer_period_ms': 50,
            }],
            remappings=[
                ('/camera_info', '/stereo_right/camera_info'),
            ],
        ),

        # Publish ground truth pose from Ignition Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/bluerov2/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            ],
            output='screen'
        ),
        # Publish IMU with remapping
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/inpetu/model/bluerov2/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen',
            remappings=[
                ('/world/inpetu/model/bluerov2/link/base_link/sensor/imu_sensor/imu', 'model/bluerov2/imu')
            ]
        ),

        # Bring up Orca and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup_heavy.py')),
            launch_arguments={
                'base': LaunchConfiguration('base'),
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'slam': LaunchConfiguration('slam'),
                'orca_params_file': orca_params_file,
            }.items(),
        ),
    ])
