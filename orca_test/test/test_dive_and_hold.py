"""
Launch test for Orca5 simulation: dive to 1 m off seafloor and hold position for 5 seconds.
"""

import os
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
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
        }.items(),
    )

    return LaunchDescription(
        [
            sim,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDiveAndHold(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = OrcaSimHarness(
            node_name='test_dive_and_hold_harness',
            mav_endpoint='tcp:127.0.0.1:5760',
            use_sim_time=True,
        )

    def tearDown(self):
        self.harness.close()

    def test_dive_and_hold(self):
        """End-to-end test: Wait for SLAM lock & EKF aiding, dive to 1m off seafloor, verify position hold."""
        # 1. Connect to ArduSub SITL via MAVLink
        self.harness.connect_mavlink(timeout_s=30.0)

        # 2. Wait for SLAM tracking lock
        self.harness.wait_for_slam_lock(timeout_s=60.0)

        # 3. Wait for EKF relative position aiding to engage
        self.harness.wait_for_ekf_aiding(timeout_s=60.0)

        # 4. Resolve mission file path
        try:
            orca_test_dir = get_package_share_directory('orca_test')
            mission_file = os.path.join(orca_test_dir, 'missions', 'dive_and_hold.txt')
        except Exception:
            # Fallback for running test directly in source tree
            mission_file = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    '..',
                    'missions',
                    'dive_and_hold.txt',
                )
            )

        self.assertTrue(
            os.path.isfile(mission_file),
            f'Mission file does not exist: {mission_file}',
        )

        # 5. Upload mission, arm, and start dive in AUTO mode
        self.harness.start_dive(mission_file)

        # 6. Wait for vehicle to reach target depth (3.0 m, 1m off seafloor at -4.0 m)
        target_depth = 3.0
        self.harness.wait_for_depth(
            target_depth_m=target_depth,
            tolerance_m=0.20,
            settle_s=2.0,
            timeout_s=60.0,
        )

        # 7. Record odometry window during position hold
        samples = self.harness.record_odometry_window(duration_s=5.0)

        # 8. Verify vehicle held position within 0.2 m tolerance
        max_dev = self.harness.assert_position_held(samples, max_deviation_m=0.20)
        self.assertLessEqual(
            max_dev,
            0.20,
            f'ROV position hold max deviation ({max_dev:.4f} m) exceeded 0.20 m tolerance',
        )
