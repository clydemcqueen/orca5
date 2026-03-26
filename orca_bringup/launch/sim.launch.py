#!/usr/bin/env python3

"""
Launch a simulation using ArduSub SITL and Gazebo.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    sub_common_parm_file = os.path.join(orca_bringup_dir, 'config', 'sub_common.parm')
    sub_vpd_parm_file = os.path.join(orca_bringup_dir, 'config', 'sub_vpd.parm')
    sub_vpe_parm_file = os.path.join(orca_bringup_dir, 'config', 'sub_vpe.parm')
    sub_vpd_parm_files = f'{sub_common_parm_file},{sub_vpd_parm_file}'
    sub_vpe_parm_files = f'{sub_common_parm_file},{sub_vpe_parm_file}'

    nodes = [
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ardusub?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'bridge',
            default_value='False',
            description='Launch SLAM bridge?',
        ),

        DeclareLaunchArgument(
            'orb',
            default_value='False',
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

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', os.path.join(orca_bringup_dir, 'worlds', 'inpetu.world')],
            output='screen',
        ),

        # Bridge images
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            output='screen',
            name='image_bridge',
            arguments=['left_image_raw', 'right_image_raw'],
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

        # Publish a left camera info message, required for rviz (not used by orb_slam3_ros)
        Node(
            package='orca_bridge',
            executable='camera_info_publisher.py',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + os.path.join(orca_bringup_dir, 'config', 'sim_camera.yaml'),
                'frame_id': 'left_camera_sensor',
            }],
        ),

        Node(
            package='orca_bridge',
            executable='camera_info_publisher.py',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + os.path.join(orca_bringup_dir, 'config', 'sim_camera.yaml'),
                'frame_id': 'right_camera_sensor',
            }],
        ),

        # Publish the static map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '--z', '-0.2',
                '--frame-id', 'map',
                '--child-frame-id', 'base_link',
            ],
        ),    
        # Publish the static map -> slam
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '--pitch', str(math.pi/ 2),
                '--frame-id', 'map',
                '--child-frame-id', 'slam',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '--roll', str(-math.pi/ 2),
                '--yaw', str(-math.pi/ 2),
                '--frame-id', 'map',
                '--child-frame-id', 'orbslam3',
            ],
        ),
                # Publish the static base_link -> left_camera_link transform.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '--x', '0.19',
                '--y', '0.1',
                '--z', '-0.201',
                '--roll', str(-math.pi),
                '--pitch', '0',
                '--yaw', str(-math.pi / 2),
                '--frame-id', 'base_link',
                '--child-frame-id', 'left_camera_link',
            ],
        ),

        # Publish the static base_link -> right_camera_link transform.
        # This must match the transform in orca5/model.sdf (see camera_* vars in generate_model.py)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=[
                '--x', '0.19',
                '--y', '-0.1',
                '--z', '-0.201',
                '--roll', str(-math.pi),
                '--pitch', '0',
                '--yaw', str(-math.pi / 2),
                '--frame-id', 'base_link',
                '--child-frame-id', 'right_camera_link',
            ],
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--include-hidden-topics',
                '/bridge_status',
                '/camera_info',
                '/camera_pose/scaled',
                '/ekf_pose',
                '/ekf_status',
                '/model/orca5/odometry',
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
                'use_sim_time': True,
            }],
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
            }.items(),
        ),

        # Launch ArduSub w/ SIM_JSON. Make sure ardusub is on the $PATH. To use the heavy (6dof) model: specify
        # vectored_6dof as the model, AND the default params must set magic ArduSub parameter FRAME_CONFIG to 2.0.
        # Yaw is provided by Gazebo, so the start yaw value is ignored.
        ExecuteProcess(
            cmd=['/home/daniel/ardupilot/build/sitl/bin/ardusub', '-S', '--wipe', '-M', 'JSON', '-I0', '--home', '-27.4302290,-48.443398,-0.1,0',
                 '--defaults', sub_vpe_parm_files if LaunchConfiguration('use_vpe') else sub_vpd_parm_files],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

    ]

    # teste

    return LaunchDescription(nodes)
