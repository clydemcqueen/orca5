"""
Unit tests for scale factor improvements (delta scaling, slant range correction, scale freezing, gravity-aligned map initialization).

colcon test --packages-select orca_bridge
colcon test-result --all --verbose
"""

import math
import unittest
from unittest.mock import MagicMock

from orb_slam3_msgs.msg import SlamStatus

from orca_bridge.geometry import Pose
from orca_bridge.slam import SlamMap, SlamMaps
from orca_bridge.sub import Sub


class TestScaleAndMap(unittest.TestCase):
    def setUp(self):
        self.logger = MagicMock()

    def test_slant_range_correction_flat(self):
        """When flat (roll=0, pitch=0), vertical ranges equal slant ranges."""
        t_map_slam = Pose()
        smap = SlamMap(
            map_id=1,
            sonar_rf=2.0,
            slam_rf=2.0,
            t_map_slam=t_map_slam,
            stamp_s=100.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
            scale_freeze_duration=5.0,
        )
        self.assertAlmostEqual(smap.sonar_rf, 2.0)
        self.assertAlmostEqual(smap.slam_rf, 2.0)
        self.assertAlmostEqual(smap.scale, 1.0)

    def test_slant_range_correction_tilted(self):
        """When tilted, vertical ranges are reduced by cos(pitch) * cos(roll)."""
        t_map_slam = Pose()
        roll = math.radians(10.0)
        pitch = math.radians(5.0)
        cos_tilt = math.cos(pitch) * math.cos(roll)

        smap = SlamMap(
            map_id=1,
            sonar_rf=3.0,
            slam_rf=3.0,
            t_map_slam=t_map_slam,
            stamp_s=100.0,
            roll=roll,
            pitch=pitch,
            logger=self.logger,
            scale_freeze_duration=5.0,
        )
        expected_vert = 3.0 * cos_tilt
        self.assertAlmostEqual(smap.sonar_rf, expected_vert)
        self.assertAlmostEqual(smap.slam_rf, expected_vert)
        # instant_scale is ratio of corrected vertical distances
        self.assertAlmostEqual(smap.scale, 1.0)

    def test_slant_range_extreme_tilt_rejected(self):
        """Extreme tilt (> 60 deg, cos_tilt < 0.5) is rejected."""
        t_map_slam = Pose()
        smap = SlamMap(
            map_id=1,
            sonar_rf=2.0,
            slam_rf=2.0,
            t_map_slam=t_map_slam,
            stamp_s=100.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
            scale_freeze_duration=5.0,
        )
        initial_scale = smap.scale

        # Now update with roll = 65 degrees (cos < 0.5)
        smap.update_scale(
            sonar_rf=4.0,
            slam_rf=2.0,
            stamp_s=101.0,
            roll=math.radians(65.0),
            pitch=0.0,
            logger=self.logger,
        )
        # Scale should not have updated
        self.assertEqual(smap.scale, initial_scale)

    def test_scale_freeze_after_duration(self):
        """Scale updates until scale_freeze_duration has elapsed from first valid measurement, then freezes."""
        t_map_slam = Pose()
        smap = SlamMap(
            map_id=1,
            sonar_rf=2.0,
            slam_rf=2.0,
            t_map_slam=t_map_slam,
            stamp_s=100.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
            scale_freeze_duration=5.0,
        )
        self.assertFalse(smap.scale_frozen)
        self.assertEqual(smap.first_scale_time, 100.0)

        # Update at t = 102.0s (< 5.0s elapsed): should update
        smap.update_scale(
            sonar_rf=2.2,
            slam_rf=2.0,
            stamp_s=102.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
        )
        self.assertFalse(smap.scale_frozen)
        scale_at_102 = smap.scale
        self.assertNotEqual(scale_at_102, 1.0)

        # Update at t = 105.0s (5.0s elapsed): should update and freeze
        smap.update_scale(
            sonar_rf=2.2,
            slam_rf=2.0,
            stamp_s=105.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
        )
        self.assertTrue(smap.scale_frozen)
        frozen_scale = smap.scale

        # Update at t = 108.0s: should be ignored because scale is frozen
        smap.update_scale(
            sonar_rf=2.5,
            slam_rf=2.0,
            stamp_s=108.0,
            roll=0.0,
            pitch=0.0,
            logger=self.logger,
        )
        self.assertEqual(smap.scale, frozen_scale)

    def test_scale_deltas_no_pseudo_velocity(self):
        """Scaling deltas rather than poses eliminates the pseudo-velocity artifact when scale changes."""
        # Suppose vehicle is at (10.0, 10.0, 2.0) away from map origin and stationary
        pos_prev = Pose(p=(10.0, 10.0, 2.0))
        pos_curr = Pose(p=(10.0, 10.0, 2.0))  # stationary

        # Old approach (scale pose):
        scale_prev = 1.0
        scale_curr = 1.05  # 5% scale jump
        pos_prev_scaled = Pose(p=(pos_prev.p[0] * scale_prev, pos_prev.p[1] * scale_prev, pos_prev.p[2] * scale_prev))
        pos_curr_scaled = Pose(p=(pos_curr.p[0] * scale_curr, pos_curr.p[1] * scale_curr, pos_curr.p[2] * scale_curr))
        delta_old = Pose.delta_pose(pos_prev_scaled, pos_curr_scaled)
        # Old approach has false non-zero delta despite stationary vehicle:
        old_jump = math.sqrt(sum(x**2 for x in delta_old.p))
        self.assertGreater(old_jump, 0.5)

        # New approach (scale delta):
        delta_unscaled = Pose.delta_pose(pos_prev, pos_curr)
        delta_scaled_p = (
            delta_unscaled.p[0] * scale_curr,
            delta_unscaled.p[1] * scale_curr,
            delta_unscaled.p[2] * scale_curr,
        )
        new_jump = math.sqrt(sum(x**2 for x in delta_scaled_p))
        self.assertEqual(new_jump, 0.0)

    def test_map_initialization_gravity_alignment(self):
        """Map initialization aligns t_map_slam so that initial t_map_base matches EKF pose exactly."""
        # Vehicle has non-zero roll, pitch, yaw and position in EKF
        sub_mock = MagicMock(spec=Sub)
        sub_mock.sonar_rf_distance = 2.0
        # NED pose: roll 3 deg, pitch -2 deg, yaw 45 deg, pos (1.0, 2.0, 1.5)
        pose_ned = Pose(p=(1.0, 2.0, 1.5))
        pose_ned.set_euler(math.radians(3.0), math.radians(-2.0), math.radians(45.0))
        sub_mock.t_map_base_ned = pose_ned

        t_link_base = Pose(p=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
        t_slam_world = Pose()
        t_slam_world.set_euler(math.pi, 0, 0)

        slam_maps = SlamMaps()
        msg = SlamStatus()
        msg.map_id = 42
        msg.tracking_state = SlamStatus.TRACKING_OK
        msg.header.stamp.sec = 100
        msg.header.stamp.nanosec = 0
        msg.pose.orientation.w = 1.0

        slam_maps.update(
            msg,
            sub_mock,
            t_link_base,
            t_slam_world,
            self.logger,
            scale_freeze_duration=5.0,
        )

        current_map = slam_maps.current_map
        self.assertIsNotNone(current_map)
        self.assertEqual(current_map.map_id, 42)  # type: ignore

        # Check initial t_map_base matches EKF pose in ENU standard
        t_world_cam_0 = Pose()
        t_slam_cam_0 = t_slam_world.mult(t_world_cam_0)
        t_slam_link_0 = t_slam_cam_0.mult(Pose.T_OPENCV_FLU)
        t_slam_base_0 = t_slam_link_0.mult(t_link_base)

        t_map_base_0 = current_map.t_map_slam.mult(t_slam_base_0)  # type: ignore
        expected_ekf_enu = pose_ned.ned_to_enu_standard()

        for i in range(3):
            self.assertAlmostEqual(t_map_base_0.p[i], expected_ekf_enu.p[i], places=5)
        for i in range(4):
            self.assertAlmostEqual(t_map_base_0.q[i], expected_ekf_enu.q[i], places=5)


if __name__ == '__main__':
    unittest.main()
