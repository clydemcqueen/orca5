#!/usr/bin/env python3

"""
Monocular SLAM to ArduSub bridge
"""

import math

import builtin_interfaces.msg
import geometry_msgs.msg
import orb_slam3_msgs.msg
import orca_msgs.msg
import pymavlink.dialects.v20.ardupilotmega as apm
import pymavlink.mavutil
import rclpy
import rclpy.node
import rclpy.serialization
import rclpy.time
import sensor_msgs.msg
import tf2_ros

import geometry
import slam
import sub


def stamp_to_s(stamp: builtin_interfaces.msg.Time) -> float:
    """Convert builtin_interfaces.msg.Time to seconds since the epoch"""
    return stamp.sec + stamp.nanosec / 1e9


def time_to_s(time: rclpy.time.Time) -> float:
    """Convert rclpy.time.Time to seconds since the epoch"""
    return time.nanoseconds / 1e9


def stamp_to_time(stamp: builtin_interfaces.msg.Time) -> rclpy.time.Time:
    """Convert builtin_interfaces.msg.Time to rclpy.time.Time"""
    return rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)


class MonoSlamBridge(rclpy.node.Node):
    """Monocular SLAM bridge"""

    # MAVLink connection parameters
    SOURCE_SYSTEM = 200
    SOURCE_COMPONENT = apm.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

    def __init__(self):
        super().__init__('slam_bridge')

        # Expected frame rate
        self.frame_rate = self.declare_parameter('frame_rate', 10).get_parameter_value().integer_value

        # Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?
        self.use_vpe = self.declare_parameter('use_vpe', False).get_parameter_value().bool_value

        # Max delta for position and rotation to be considered an outlier
        self.max_delta_pos = self.declare_parameter('max_delta_pos', 0.5).get_parameter_value().double_value
        self.max_delta_rot = self.declare_parameter('max_delta_rot', math.pi / 6).get_parameter_value().double_value

        # Static transform slam -> world
        self.t_slam_world = geometry.Pose()
        self.t_slam_world.set_euler(math.pi, 0, 0)

        # Static transform base_link (base) -> camera_link (link) should be available, listen for it
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.t_link_base = None

        # ArduSub state
        self.sub = sub.Sub()

        # SLAM state
        self.maps = slam.SlamMaps()
        self.tracking = False

        # Open a mavlink connection
        mav_device = self.declare_parameter('mav_device', 'udpin:0.0.0.0:14551').get_parameter_value().string_value
        self.conn = pymavlink.mavutil.mavlink_connection(
            mav_device,
            source_system=MonoSlamBridge.SOURCE_SYSTEM,
            source_component=MonoSlamBridge.SOURCE_COMPONENT)

        # Wait for a heartbeat
        self.get_logger().info(f'Waiting for heartbeat on {mav_device}...')
        self.conn.wait_heartbeat()
        self.get_logger().info('Heartbeat received from system (system %u component %u)' %
                               (self.conn.target_system, self.conn.target_component))

        # Subscriptions
        self.map_sub = self.create_subscription(sensor_msgs.msg.PointCloud2, 'map_points', self.map_callback, 10)
        self.slam_sub = self.create_subscription(orb_slam3_msgs.msg.SlamStatus, 'slam_status', self.slam_callback, 10)

        # Publishers
        self.bridge_status_pub = self.create_publisher(orca_msgs.msg.BridgeStatus, 'bridge_status', 10)
        self.ekf_pose_pub = self.create_publisher(geometry_msgs.msg.PoseStamped, 'ekf_pose', 10)
        self.ekf_status_pub = self.create_publisher(orca_msgs.msg.FilterStatus, 'ekf_status', 10)
        self.scaled_map_pub = self.create_publisher(sensor_msgs.msg.PointCloud2, 'map_points/scaled', 10)
        self.slam_delta_pub = self.create_publisher(geometry_msgs.msg.PoseStamped, 'slam_delta', 10)
        self.slam_pose_pub = self.create_publisher(geometry_msgs.msg.PoseStamped, 'slam_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Manage state on a 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Publish static transforms
        self.publish_static_transforms()

        self.get_logger().info('SLAM bridge ready')

    def publish_static_transforms(self):
        """Publish static transforms. Call this once."""

        tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()

        tf_msg.header.frame_id = 'slam'
        tf_msg.child_frame_id = 'world'
        tf_msg.transform = self.t_slam_world.to_transform_msg()
        tf_static_broadcaster.sendTransform(tf_msg)

        tf_msg.header.frame_id = 'camera_link'
        tf_msg.child_frame_id = 'camera_sensor'
        tf_msg.transform = geometry.Pose.T_FLU_OPENCV.to_transform_msg()
        tf_static_broadcaster.sendTransform(tf_msg)

    def set_ekf_sources(self, slam_tracking: bool):
        """
        Change the EKF source set to use SLAM (set 1) or no XY source (set 2).
        See the EK3_SRCn_xxxXY parameters in orca_bringup/config/sub.parm
        """
        self.conn.mav.send(apm.MAVLink_command_int_message(
            1, 1, 0, apm.MAV_CMD_SET_EKF_SOURCE_SET, 0, 0,
            1 if slam_tracking else 2, 0, 0, 0, 0, 0, 0))

    def is_outlier(self, delta_p, delta_e) -> bool:
        """Return true if the delta is too great."""

        # Calculate magnitude of position delta
        delta_pos = math.sqrt(delta_p[0] ** 2 + delta_p[1] ** 2 + delta_p[2] ** 2)

        # Ensure angles are within (-pi, pi] for comparison
        delta_roll = math.fmod(delta_e[0] + math.pi, 2 * math.pi) - math.pi
        delta_pitch = math.fmod(delta_e[1] + math.pi, 2 * math.pi) - math.pi
        delta_yaw = math.fmod(delta_e[2] + math.pi, 2 * math.pi) - math.pi

        if delta_pos > self.max_delta_pos:
            self.get_logger().warn(f'Outlier: pos_delta={delta_pos:.3f}m (max {self.max_delta_pos:.3f}m)')
            return True

        if abs(delta_roll) > self.max_delta_rot:
            self.get_logger().warn(f'Outlier: roll_delta={math.degrees(delta_roll):.3f}deg (max {math.degrees(self.max_delta_rot):.3f}deg)')
            return True

        if abs(delta_pitch) > self.max_delta_rot:
            self.get_logger().warn(f'Outlier: pitch_delta={math.degrees(delta_pitch):.3f}deg (max {math.degrees(self.max_delta_rot):.3f}deg)')
            return True

        if abs(delta_yaw) > self.max_delta_rot:
            self.get_logger().warn(f'Outlier: yaw_delta={math.degrees(delta_yaw):.3f}deg (max {math.degrees(self.max_delta_rot):.3f}deg)')
            return True

        return False

    def publish_bridge_status(self, stamp, flags):
        bridge_status_msg = orca_msgs.msg.BridgeStatus()
        bridge_status_msg.header.stamp = stamp
        bridge_status_msg.header.frame_id = 'map'
        bridge_status_msg.flags = flags
        if self.maps.current_map is not None:
            bridge_status_msg.sonar_rf = self.maps.current_map.sonar_rf
            bridge_status_msg.slam_rf = self.maps.current_map.slam_rf
            bridge_status_msg.scale = self.maps.current_map.scale
        self.bridge_status_pub.publish(bridge_status_msg)

    def slam_callback(self, msg: orb_slam3_msgs.msg.SlamStatus):

        #----------
        # Wait for everything to warm up
        #----------

        flags = 0

        # Wait for static transform to be published
        if self.t_link_base is None:
            # now = self.get_clock().now()
            msg_time = stamp_to_time(msg.header.stamp)
            if self.tf_buffer.can_transform('camera_link', 'base_link', msg_time):
                t_link_base = self.tf_buffer.lookup_transform('camera_link', 'base_link', msg_time)
                self.t_link_base = geometry.Pose.from_transform_msg(t_link_base.transform)
            else:
                self.get_logger().warn('Cannot get tf from base_link to camera_link, dropping slam message', throttle_duration_sec=1.0)
                flags |= orca_msgs.msg.BridgeStatus.WAIT_TRANSFORMS

        # Wait for sonar rangefinder readings
        if self.sub.sonar_rf_distance is None:
            self.get_logger().warn('Sonar rangefinder not available, dropping slam message', throttle_duration_sec=1.0)
            flags |= orca_msgs.msg.BridgeStatus.WAIT_SONAR_RF

        # Wait for the EKF to generate a good pose
        if self.sub.ekf_other():
            self.get_logger().warn('EKF not healthy, dropping slam message', throttle_duration_sec=1.0)
            flags |= orca_msgs.msg.BridgeStatus.WAIT_EKF

        #----------
        # Update SLAM state
        #----------

        # Wait for ORB_SLAM3 to create a map and start tracking
        if msg.tracking_state != orb_slam3_msgs.msg.SlamStatus.TRACKING_OK:
            if self.tracking:
                self.tracking = False
                self.get_logger().info('Tracking stopped')

                # The EKF will stop getting VPD messages. For a while it will rely on the IMU to generate the XY
                # position, but after ~6s it will time out and switch to const_pos mode. It appears that we can speed
                # this process up a bit by changing the EKF source set to remove the POSXY and VELXY inputs.
                self.set_ekf_sources(slam_tracking=False)
            flags |= orca_msgs.msg.BridgeStatus.WAIT_SLAM

        if flags:
            # Nothing to do
            self.publish_bridge_status(msg.header.stamp, flags)
            return

        if not self.tracking:
            self.tracking = True
            self.get_logger().info('Tracking started')

            # Start sending VPD messages. Tell the EKF to use them
            self.set_ekf_sources(slam_tracking=True)

        self.maps.update(msg, self.sub, self.get_logger())

        #----------
        # We have t_world_camera, use this to find map -> base.
        # This will give us 2 map -> base transforms: one from the EKF, one from the SLAM map
        #----------

        # We are given the pose of the camera sensor in the world frame
        t_world_camera = geometry.Pose.from_pose_msg(msg.pose)

        # Apply the current SLAM scale
        t_world_camera.apply_scale(self.maps.current_map.scale)

        # Find the pose of the camera sensor in the SLAM map frame
        t_slam_camera = self.t_slam_world.mult(t_world_camera)

        # Find the pose of camera_link (ENU) in the SLAM map frame
        t_slam_link = t_slam_camera.mult(geometry.Pose.T_OPENCV_FLU)

        # Find the pose of the base link (the ROV) in the SLAM map frame
        t_slam_base = t_slam_link.mult(self.t_link_base)

        # Calculate the delta from the previous pose
        if self.maps.current_map.t_slam_base is None:
            delta = geometry.Pose()
        else:
            delta = geometry.Pose.delta_pose(self.maps.current_map.t_slam_base, t_slam_base)

        delta_p = delta.get_position()
        delta_e = delta.get_euler()
        
        # Detect outliers
        if self.is_outlier(delta_p, delta_e):
            flags |= orca_msgs.msg.BridgeStatus.OK_OUTLIER

        # Save the current slam->base transform
        self.maps.current_map.update_pose(t_slam_base)

        # Find the pose of the base link (the ROV) in the map frame
        t_map_base = self.maps.current_map.t_map_slam.mult(t_slam_base)

        #----------
        # Send a VISION_POSITION_DELTA (VPD) or VISION_POSITION_ESTIMATE (VPE) message to ArduSub
        #----------

        if self.use_vpe:

            t_map_base_ned = t_map_base.enu_to_ned_standard()
            e_frd = t_map_base_ned.get_euler()

            self.conn.mav.vision_position_estimate_send(
                0,  # time_usec (not used)
                t_map_base_ned.p[0],
                t_map_base_ned.p[1],
                t_map_base_ned.p[2],
                e_frd[0],
                e_frd[1],
                e_frd[2]
            )

        else:

            # Convert FLU (forward, left, up) to FRD (forward, right, down)
            delta_p_frd = (delta_p[0], -delta_p[1], -delta_p[2])
            delta_e_frd = (delta_e[0], -delta_e[1], -delta_e[2])

            self.conn.mav.vision_position_delta_send(
                0,  # time_usec (not used)
                1000000 // self.frame_rate,  # delta usec
                delta_e_frd,
                delta_p_frd,
                0  # confidence (not used)
            )

        # Publish the map -> slam transform
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'slam'
        tf_msg.transform = self.maps.current_map.t_map_slam.to_transform_msg()
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publish the ROV pose as determined by ORB_SLAM3
        slam_pose_stamped_msg = geometry_msgs.msg.PoseStamped()
        slam_pose_stamped_msg.header.stamp = msg.header.stamp
        slam_pose_stamped_msg.header.frame_id = 'map'
        slam_pose_stamped_msg.pose = t_map_base.to_pose_msg()
        self.slam_pose_pub.publish(slam_pose_stamped_msg)

        # Publish the SLAM delta
        slam_delta_stamped_msg = geometry_msgs.msg.PoseStamped()
        slam_delta_stamped_msg.header.stamp = msg.header.stamp
        slam_delta_stamped_msg.header.frame_id = 'base_link'
        slam_delta_stamped_msg.pose = delta.to_pose_msg()
        self.slam_delta_pub.publish(slam_delta_stamped_msg)

        # Publish a status message
        self.publish_bridge_status(msg.header.stamp, flags)

    def map_callback(self, msg: sensor_msgs.msg.PointCloud2):
        """Scale the map and republish it."""

        if self.maps.current_map is None:
            self.get_logger().warn('No map, dropping point cloud', throttle_duration_sec=1.0)
            return

        self.scaled_map_pub.publish(slam.scale_cloud(msg, self.maps.current_map.scale))
    
    def timer_callback(self):
        """Update ArduSub state and publish the results."""

        now = self.get_clock().now()
        now_stamp = now.to_msg()
        now_s = time_to_s(now)

        # Update ArduSub state
        self.sub.update(self.conn, now_s, self.get_logger())

        # Publish the EKF status
        ekf_status_msg = orca_msgs.msg.FilterStatus()
        ekf_status_msg.header.stamp = now_stamp
        ekf_status_msg.header.frame_id = 'map'
        ekf_status_msg.flags = self.sub.ekf_status_report.flags
        ekf_status_msg.velocity_variance = self.sub.ekf_status_report.velocity_variance
        ekf_status_msg.pos_horiz_variance = self.sub.ekf_status_report.pos_horiz_variance
        ekf_status_msg.pos_vert_variance = self.sub.ekf_status_report.pos_vert_variance
        ekf_status_msg.compass_variance = self.sub.ekf_status_report.compass_variance
        ekf_status_msg.terrain_alt_variance = self.sub.ekf_status_report.terrain_alt_variance
        ekf_status_msg.airspeed_variance = self.sub.ekf_status_report.airspeed_variance
        self.ekf_status_pub.publish(ekf_status_msg)

        # Publish the ROV pose as determined by the ArduSub EKF
        t_map_base_from_ekf = self.sub.t_map_base_ned.ned_to_enu_standard()  # Swaps axes and applies 90d yaw rotation
        ekf_pose_stamped_msg = geometry_msgs.msg.PoseStamped()
        ekf_pose_stamped_msg.header.stamp = now_stamp
        ekf_pose_stamped_msg.header.frame_id = 'map'
        ekf_pose_stamped_msg.pose = t_map_base_from_ekf.to_pose_msg()
        self.ekf_pose_pub.publish(ekf_pose_stamped_msg)

        # Publish map -> base_link, note that this is from the EKF, not the SLAM pose
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = now_stamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform = t_map_base_from_ekf.to_transform_msg()
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MonoSlamBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
