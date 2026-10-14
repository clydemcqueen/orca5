"""
OrcaSimHarness: Shared test harness for Orca5 simulation end-to-end tests.

Wraps ROS 2 node subscriptions and pymavlink MAVLink connection to ArduSub SITL.
"""

import math
import os
import time
from typing import List, Optional, Tuple

import nav_msgs.msg
import orb_slam3_msgs.msg
import pymavlink.dialects.v20.ardupilotmega as apm
import pymavlink.mavutil as mavutil
import pymavlink.mavwp as mavwp
import rclpy
import rclpy.node

import orca_msgs.msg


# TODO not a fan of the name
class OrcaSimHarness:
    """Test harness managing ROS 2 node interactions and direct MAVLink connection."""

    def __init__(
        self,
        node_name: str = 'orca_sim_harness',
        mav_endpoint: str = 'tcp:127.0.0.1:5760',
        use_sim_time: bool = True,
    ):
        self.node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
        )

        self.mav_endpoint = mav_endpoint
        self.mav: Optional[mavutil.mavlink_connection] = None

        # State tracking
        self.latest_slam_status: Optional[orb_slam3_msgs.msg.SlamStatus] = None
        self.latest_ekf_status: Optional[orca_msgs.msg.EkfStatusReport] = None
        self.latest_odometry: Optional[nav_msgs.msg.Odometry] = None
        self.latest_bridge_status: Optional[orca_msgs.msg.BridgeStatus] = None

        # Odometry history
        self.odometry_samples: List[Tuple[float, float, float]] = []
        self._record_odometry: bool = False

        # ROS 2 Subscriptions
        self.slam_sub = self.node.create_subscription(
            orb_slam3_msgs.msg.SlamStatus,
            'slam_status',
            self._slam_callback,
            10,
        )
        self.ekf_sub = self.node.create_subscription(
            orca_msgs.msg.EkfStatusReport,
            'ekf_status_report',
            self._ekf_callback,
            10,
        )
        self.odom_sub = self.node.create_subscription(
            nav_msgs.msg.Odometry,
            '/model/orca5/odometry',
            self._odom_callback,
            50,
        )
        self.bridge_sub = self.node.create_subscription(
            orca_msgs.msg.BridgeStatus,
            'bridge_status',
            self._bridge_callback,
            10,
        )

    def _slam_callback(self, msg: orb_slam3_msgs.msg.SlamStatus):
        self.latest_slam_status = msg

    def _ekf_callback(self, msg: orca_msgs.msg.EkfStatusReport):
        self.latest_ekf_status = msg

    def _odom_callback(self, msg: nav_msgs.msg.Odometry):
        self.latest_odometry = msg
        if self._record_odometry:
            pos = msg.pose.pose.position
            self.odometry_samples.append((pos.x, pos.y, pos.z))

    def _bridge_callback(self, msg: orca_msgs.msg.BridgeStatus):
        self.latest_bridge_status = msg

    def spin_once(self, timeout_sec: float = 0.05):
        """Spin ROS 2 node once."""
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def connect_mavlink(self, timeout_s: float = 30.0) -> bool:
        """Connect to ArduSub SITL via MAVLink and wait for heartbeat."""
        start_time = time.time()
        self.node.get_logger().info(f'Connecting to MAVLink endpoint {self.mav_endpoint}...')

        while time.time() - start_time < timeout_s:
            try:
                if self.mav is None:
                    self.mav = mavutil.mavlink_connection(
                        self.mav_endpoint,
                        source_system=255,
                        source_component=0,
                    )
                msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if msg is not None:
                    self.node.get_logger().info(
                        f'Connected to vehicle (system={self.mav.target_system}, component={self.mav.target_component})'
                    )
                    return True
            except Exception as e:
                self.node.get_logger().warn(f'MAVLink connection attempt error: {e}')
                self.mav = None

            self.spin_once(0.1)

        raise TimeoutError(f'Failed to connect to MAVLink endpoint {self.mav_endpoint} within {timeout_s}s')

    def wait_for_slam_lock(self, timeout_s: float = 60.0) -> bool:
        """Wait for ORB_SLAM3 to report active tracking."""
        self.node.get_logger().info('Waiting for SLAM lock (tracking_state == TRACKING_OK)...')
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            self.spin_once(0.05)
            if self.latest_slam_status is not None:
                state = self.latest_slam_status.tracking_state
                if state in (
                    orb_slam3_msgs.msg.SlamStatus.TRACKING_OK,
                    orb_slam3_msgs.msg.SlamStatus.TRACKING_OK_KLT,
                ):
                    self.node.get_logger().info(f'SLAM lock acquired (state={state})')
                    return True

        curr_state = self.latest_slam_status.tracking_state if self.latest_slam_status else 'None'
        raise TimeoutError(f'SLAM lock not acquired within {timeout_s}s (last state: {curr_state})')

    def wait_for_ekf_aiding(self, timeout_s: float = 60.0) -> bool:
        """Wait for ArduSub EKF to report relative horizontal position aiding."""
        self.node.get_logger().info('Waiting for EKF relative position aiding (EKF_POS_HORIZ_REL)...')
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            self.spin_once(0.05)
            if self.latest_ekf_status is not None:
                if self.latest_ekf_status.flags & orca_msgs.msg.EkfStatusReport.EKF_POS_HORIZ_REL:
                    self.node.get_logger().info('EKF relative position aiding confirmed')
                    return True

        flags = self.latest_ekf_status.flags if self.latest_ekf_status else 'None'
        raise TimeoutError(f'EKF relative position aiding not active within {timeout_s}s (last flags: {flags})')

    def load_mission(self, filepath: str, timeout_s: float = 20.0) -> int:
        """Upload a waypoint mission from file to ArduSub using MAVLink protocol."""
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f'Mission file not found: {filepath}')

        wploader = mavwp.MAVWPLoader(
            target_system=self.mav.target_system,
            target_component=self.mav.target_component,
        )
        wploader.load(filepath)
        count = wploader.count()
        if count == 0:
            raise ValueError(f'No waypoints found in {filepath}')

        self.node.get_logger().info(f'Uploading {count} waypoints from {filepath}...')

        # Clear existing waypoints
        self.mav.waypoint_clear_all_send()
        self.mav.recv_match(type=['MISSION_ACK', 'COMMAND_ACK'], blocking=True, timeout=2.0)

        # Announce count
        self.mav.waypoint_count_send(count)

        start_time = time.time()
        while time.time() - start_time < timeout_s:
            self.spin_once(0.02)
            msg = self.mav.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                continue

            if msg.get_type() == 'MISSION_REQUEST':
                wp = wploader.wp(msg.seq)
                self.mav.mav.send(wp)
            elif msg.get_type() == 'MISSION_REQUEST_INT':
                wp = wploader.wp(msg.seq)
                wp_int = apm.MAVLink_mission_item_int_message(
                    self.mav.target_system,
                    self.mav.target_component,
                    wp.seq,
                    wp.frame,
                    wp.command,
                    wp.current,
                    wp.autocontinue,
                    wp.param1,
                    wp.param2,
                    wp.param3,
                    wp.param4,
                    int(wp.x * 1e7),
                    int(wp.y * 1e7),
                    wp.z,
                    apm.MAV_MISSION_TYPE_MISSION,
                )
                self.mav.mav.send(wp_int)
            elif msg.get_type() == 'MISSION_ACK':
                if msg.type == apm.MAV_MISSION_ACCEPTED:
                    self.node.get_logger().info(f'Mission upload accepted ({count} waypoints)')
                    return count
                else:
                    raise RuntimeError(f'Mission upload rejected by vehicle with code {msg.type}')

        raise TimeoutError(f'Mission upload timed out after {timeout_s}s')

    def arm_throttle(self, timeout_s: float = 15.0) -> bool:
        """Arm vehicle throttle."""
        self.node.get_logger().info('Arming throttle...')
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            self.spin_once(0.05)
            # Send arm command
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                apm.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0,  # 1 to arm
                0,
                0,
                0,
                0,
                0,
                0,
            )

            msg = self.mav.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=True, timeout=1.0)
            if msg:
                if msg.get_type() == 'COMMAND_ACK' and msg.command == apm.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == apm.MAV_RESULT_ACCEPTED:
                        self.node.get_logger().info('Vehicle armed (COMMAND_ACK accepted)')
                        return True
                elif msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & apm.MAV_MODE_FLAG_SAFETY_ARMED:
                        self.node.get_logger().info('Vehicle armed (HEARTBEAT confirmed)')
                        return True

        raise TimeoutError(f'Failed to arm throttle within {timeout_s}s')

    def set_mode(self, mode_name: str = 'AUTO', timeout_s: float = 15.0) -> bool:
        """Set vehicle flight mode (e.g. AUTO, MANUAL, POSHOLD)."""
        mode_mapping = mavutil.mode_mapping_sub
        inv_map = {v.upper(): k for k, v in mode_mapping.items()}
        if mode_name.upper() not in inv_map:
            raise ValueError(f'Unknown mode {mode_name}. Available: {list(inv_map.keys())}')

        mode_id = inv_map[mode_name.upper()]
        self.node.get_logger().info(f'Setting vehicle mode to {mode_name} (id={mode_id})...')

        start_time = time.time()
        while time.time() - start_time < timeout_s:
            self.spin_once(0.05)
            self.mav.set_mode(mode_id)
            msg = self.mav.recv_match(type=['HEARTBEAT', 'COMMAND_ACK'], blocking=True, timeout=1.0)
            if msg:
                if msg.get_type() == 'HEARTBEAT':
                    if msg.custom_mode == mode_id:
                        self.node.get_logger().info(f'Vehicle mode {mode_name} confirmed')
                        return True

        raise TimeoutError(f'Failed to switch mode to {mode_name} within {timeout_s}s')

    def start_dive(self, mission_filepath: Optional[str] = None) -> bool:
        """Start a dive mission: upload mission if provided, arm motors, and switch to AUTO mode."""
        if mission_filepath:
            self.load_mission(mission_filepath)
        self.arm_throttle()
        self.set_mode('AUTO')
        return True

    def wait_for_depth(
        self,
        target_depth_m: float,
        tolerance_m: float = 0.20,
        settle_s: float = 2.0,
        timeout_s: float = 60.0,
    ) -> float:
        """Wait until Gazebo ground truth reports vehicle reached target depth and settled."""
        self.node.get_logger().info(
            f'Waiting for vehicle to reach depth {target_depth_m:.2f} m (±{tolerance_m:.2f} m) and settle...'
        )
        start_time = time.time()
        reached_time: Optional[float] = None

        while time.time() - start_time < timeout_s:
            self.spin_once(0.05)
            if self.latest_odometry is not None:
                # In Gazebo ENU world, z is negative underwater
                current_depth = -self.latest_odometry.pose.pose.position.z
                vz = abs(self.latest_odometry.twist.twist.linear.z)
                if abs(current_depth - target_depth_m) <= tolerance_m and vz < 0.10:
                    if reached_time is None:
                        reached_time = time.time()
                    elif time.time() - reached_time >= settle_s:
                        self.node.get_logger().info(
                            f'Vehicle reached and settled at depth {current_depth:.2f} m (vz={vz:.3f} m/s)'
                        )
                        return current_depth
                else:
                    reached_time = None

        current_depth = -self.latest_odometry.pose.pose.position.z if self.latest_odometry else 'None'
        raise TimeoutError(
            f'Vehicle did not reach depth {target_depth_m} m within {timeout_s}s (last depth: {current_depth})'
        )

    def record_odometry_window(self, duration_s: float = 5.0) -> List[Tuple[float, float, float]]:
        """Record ground truth positions from /model/orca5/odometry for duration_s seconds."""
        self.node.get_logger().info(f'Recording odometry samples for {duration_s:.1f} s...')
        self.odometry_samples.clear()
        self._record_odometry = True

        start_time = time.time()
        while time.time() - start_time < duration_s:
            self.spin_once(0.02)

        self._record_odometry = False
        self.node.get_logger().info(f'Recorded {len(self.odometry_samples)} odometry samples')
        return list(self.odometry_samples)

    def assert_position_held(
        self,
        samples: Optional[List[Tuple[float, float, float]]] = None,
        max_deviation_m: float = 0.20,
    ) -> float:
        """Assert that position samples stayed within max_deviation_m tolerance."""
        if samples is None:
            samples = self.odometry_samples

        if len(samples) < 5:
            raise AssertionError(f'Insufficient odometry samples collected ({len(samples)}) to verify hold')

        # Calculate centroid
        mean_x = sum(p[0] for p in samples) / len(samples)
        mean_y = sum(p[1] for p in samples) / len(samples)
        mean_z = sum(p[2] for p in samples) / len(samples)

        max_dist = 0.0
        for x, y, z in samples:
            dist = math.sqrt((x - mean_x) ** 2 + (y - mean_y) ** 2 + (z - mean_z) ** 2)
            if dist > max_dist:
                max_dist = dist

        self.node.get_logger().info(
            f'Position hold max deviation: {max_dist:.4f} m (allowable: {max_deviation_m:.4f} m)'
        )
        if max_dist > max_deviation_m:
            raise AssertionError(
                f'Position hold failed: max deviation {max_dist:.4f} m exceeds tolerance {max_deviation_m:.4f} m'
            )

        return max_dist

    def close(self):
        """Clean up MAVLink and ROS node resources."""
        self._record_odometry = False
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
            self.mav = None
        if self.node:
            self.node.destroy_node()
