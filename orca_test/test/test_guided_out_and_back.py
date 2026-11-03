"""
Launch test for Orca5 simulation: dive to 1 m off seafloor, execute Guided out-and-back maneuver,
and verify return to start position, depth maintenance, and transition to POSHOLD.
"""

import math
import os
import shutil
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from orca_test import OrcaSimHarness


def _resolve_lua_script_path() -> str:
    try:
        orca_test_dir = get_package_share_directory('orca_test')
        candidate = os.path.join(orca_test_dir, 'lua_scripts', 'out_and_back.lua')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            '..',
            '..',
            'lua_scripts',
            'out_and_back.lua',
        )
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    lua_src = _resolve_lua_script_path()
    if not os.path.isfile(lua_src):
        raise FileNotFoundError(f'Lua script not found: {lua_src}')

    scripts_dir = os.path.join(os.getcwd(), 'scripts')
    os.makedirs(scripts_dir, exist_ok=True)
    shutil.copyfile(lua_src, os.path.join(scripts_dir, 'out_and_back.lua'))

    orca_bringup_dir = get_package_share_directory('orca_bringup')
    sim_launch_path = os.path.join(orca_bringup_dir, 'launch', 'sim.launch.py')

    speedup = os.environ.get('ORCA_TEST_SPEEDUP', '1')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'gz_gui': 'False',
            'speedup': speedup,
            'rviz': 'False',
            'bag': 'False',
            'enable_slam': 'False',
        }.items(),
    )

    return LaunchDescription(
        [
            sim,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGuidedOutAndBack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        staged_script = os.path.join(os.getcwd(), 'scripts', 'out_and_back.lua')
        if os.path.isfile(staged_script):
            os.remove(staged_script)

    def setUp(self):
        self.harness = OrcaSimHarness(
            node_name='test_guided_out_and_back_harness',
            mav_endpoint='tcp:127.0.0.1:5760',
            use_sim_time=True,
        )

    def tearDown(self):
        self.harness.close()

    def test_out_and_back(self):
        """End-to-end test: Dive with SLAM off, enter ALT_HOLD, enable SLAM, execute out-and-back, verify results."""
        # 1. Connect to ArduSub SITL via MAVLink
        self.harness.connect_mavlink(timeout_s=30.0)

        # 2. Dive into position with SLAM off and enter ALT_HOLD
        target_depth = 2.0
        self.harness.dive_to_depth(target_depth_m=target_depth, throttle_pwm=1350, timeout_s=60.0)

        # 3. Enable SLAM and wait for tracking lock and EKF aiding
        self.harness.set_slam_enabled(True)
        self.harness.wait_for_slam_lock(timeout_s=60.0)
        self.harness.wait_for_ekf_aiding(timeout_s=60.0)

        # 4. Wait for map scale to freeze (5s) and depth to settle in ALT_HOLD
        settle_start = time.time()
        while time.time() - settle_start < 5.5:
            self.harness.spin_once(0.05)
            self.harness._drain_mavlink()

        # Record starting depth and position from ground truth odometry
        start_pos = self.harness.latest_odometry.pose.pose.position
        start_x, start_y = start_pos.x, start_pos.y
        start_depth = -start_pos.z

        # 5. Switch to GUIDED mode to invoke out_and_back.lua
        self.harness.set_mode('GUIDED')
        self.harness.wait_for_statustext('out_and_back: GUIDED active', timeout_s=15.0)

        # 6. Wait for outbound leg to complete and verify outbound distance
        self.harness.wait_for_statustext('out_and_back: reached outbound', timeout_s=60.0)
        out_pos = self.harness.latest_odometry.pose.pose.position
        outbound_dist = math.hypot(out_pos.x - start_x, out_pos.y - start_y)
        self.assertAlmostEqual(
            outbound_dist,
            5.0,
            delta=0.60,
            msg=f'Outbound distance {outbound_dist:.2f} m deviated more than 0.60 m from target 5.0 m',
        )

        # 7. Wait for return leg to complete
        self.harness.wait_for_statustext('out_and_back: reached start', timeout_s=60.0)

        # 8. Exit GUIDED mode by switching to POSHOLD and settle for 2 seconds
        self.harness.set_mode('POSHOLD')
        settle_start = time.time()
        while time.time() - settle_start < 2.0:
            self.harness.spin_once(0.05)

        # 9. Verify vehicle switched mode to POSHOLD (custom_mode == 16)
        self.assertIsNotNone(self.harness.latest_heartbeat, 'Heartbeat not received')
        self.assertEqual(
            self.harness.latest_heartbeat.custom_mode,
            16,
            f'Vehicle custom mode was {self.harness.latest_heartbeat.custom_mode}, expected POSHOLD (16)',
        )

        # 10. Verify vehicle maintained depth within 0.30 m
        final_pos = self.harness.latest_odometry.pose.pose.position
        final_depth = -final_pos.z
        self.assertAlmostEqual(
            final_depth,
            start_depth,
            delta=0.30,
            msg=f'Final depth {final_depth:.2f} m deviated more than 0.30 m from target {start_depth:.2f} m',
        )

        # 11. Verify vehicle returned to starting position within 0.40 m tolerance
        dist_from_start = math.hypot(final_pos.x - start_x, final_pos.y - start_y)
        self.assertLessEqual(
            dist_from_start,
            0.40,
            f'Final return position ({final_pos.x:.2f}, {final_pos.y:.2f}) '
            f'is {dist_from_start:.3f} m from start ({start_x:.2f}, {start_y:.2f}), exceeding 0.40 m tolerance',
        )
