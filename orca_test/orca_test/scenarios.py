"""
Shared launch descriptions and end-to-end test scenarios for Orca5 simulation tests (mono and mono_imu).
"""

import math
import os
import shutil
import time
import unittest

import launch_testing.actions
import rclpy
import rclpy.parameter
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from .harness import OrcaSimHarness


def resolve_mission_path(filename: str, caller_file: str) -> str:
    """Resolve a mission file in orca_test/missions."""
    try:
        orca_test_dir = get_package_share_directory('orca_test')
        candidate = os.path.join(orca_test_dir, 'missions', filename)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    return os.path.abspath(
        os.path.join(
            os.path.dirname(caller_file),
            '..',
            'missions',
            filename,
        )
    )


def resolve_lua_script_path(filename: str, caller_file: str) -> str:
    """Resolve a Lua script file in orca_test/lua_scripts."""
    try:
        orca_test_dir = get_package_share_directory('orca_test')
        candidate = os.path.join(orca_test_dir, 'lua_scripts', filename)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    return os.path.abspath(
        os.path.join(
            os.path.dirname(caller_file),
            '..',
            '..',
            'lua_scripts',
            filename,
        )
    )


def build_sim_test_description(mono_imu: bool = False, enable_slam: bool = True) -> LaunchDescription:
    """Build a headless simulation LaunchDescription for launch_testing."""
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
            'enable_slam': 'True' if enable_slam else 'False',
            'mono_imu': 'True' if mono_imu else 'False',
        }.items(),
    )

    return LaunchDescription(
        [
            sim,
            launch_testing.actions.ReadyToTest(),
        ]
    )


def stage_out_and_back_lua(caller_file: str) -> str:
    """Copy out_and_back.lua into ./scripts so ArduSub SITL loads it on startup."""
    lua_src = resolve_lua_script_path('out_and_back.lua', caller_file)
    if not os.path.isfile(lua_src):
        raise FileNotFoundError(f'Lua script not found: {lua_src}')

    scripts_dir = os.path.join(os.getcwd(), 'scripts')
    os.makedirs(scripts_dir, exist_ok=True)
    staged_path = os.path.join(scripts_dir, 'out_and_back.lua')
    shutil.copyfile(lua_src, staged_path)
    return staged_path


def cleanup_out_and_back_lua():
    """Remove staged ./scripts/out_and_back.lua."""
    staged_script = os.path.join(os.getcwd(), 'scripts', 'out_and_back.lua')
    if os.path.isfile(staged_script):
        os.remove(staged_script)


class DiveAndHoldScenarioMixin(unittest.TestCase):
    """Shared dive-and-hold test scenario."""

    MONO_IMU: bool = False
    HARNESS_NAME: str = 'test_dive_and_hold_harness'
    CALLER_FILE: str = __file__

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = OrcaSimHarness(
            node_name=self.HARNESS_NAME,
            mav_endpoint='tcp:127.0.0.1:5760',
            use_sim_time=True,
            mono_imu=self.MONO_IMU,
        )

    def tearDown(self):
        self.harness.close()

    def run_dive_and_hold(self):
        """Wait for SLAM lock & EKF aiding, dive to 1m off seafloor, verify position hold."""
        # 1. Connect to ArduSub SITL via MAVLink
        self.harness.connect_mavlink(timeout_s=30.0)

        if self.MONO_IMU:
            self.harness.warmup_mono_imu(target_depth_m=1.5)

        # 2. Wait for SLAM tracking lock
        self.harness.wait_for_slam_lock(timeout_s=60.0)

        # 3. Wait for EKF relative position aiding to engage
        self.harness.wait_for_ekf_aiding(timeout_s=60.0)

        # 4. Resolve mission file path
        mission_file = resolve_mission_path('dive_and_hold.txt', self.CALLER_FILE)
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


class VpeHoldScenarioMixin(unittest.TestCase):
    """Shared VPD vs VPE station-keeping scenario under ocean currents."""

    MONO_IMU: bool = False
    HARNESS_NAME: str = 'test_vpe_hold_harness'
    CALLER_FILE: str = __file__

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = OrcaSimHarness(
            node_name=self.HARNESS_NAME,
            mav_endpoint='tcp:127.0.0.1:5760',
            use_sim_time=True,
            mono_imu=self.MONO_IMU,
        )

    def tearDown(self):
        self.harness.close()

    def run_vpe_station_keeping(self):
        """Compare station keeping with VPD vs VPE under ocean current disturbance."""
        # 1. Connect to ArduSub SITL via MAVLink
        self.harness.connect_mavlink(timeout_s=30.0)

        if self.MONO_IMU:
            self.harness.warmup_mono_imu(target_depth_m=1.5)

        # 2. Wait for SLAM tracking lock
        self.harness.wait_for_slam_lock(timeout_s=60.0)

        # 3. Wait for EKF relative position aiding to engage
        self.harness.wait_for_ekf_aiding(timeout_s=60.0)

        # 4. Resolve mission file path
        mission_file = resolve_mission_path('dive_and_hold.txt', self.CALLER_FILE)
        self.assertTrue(
            os.path.isfile(mission_file),
            f'Mission file does not exist: {mission_file}',
        )

        # 5. Start dive in AUTO mode to reach target depth
        self.harness.start_dive(mission_file)

        # 6. Wait for vehicle to reach target depth (3.0 m, 1 m off seafloor at -4.0 m)
        target_depth = 3.0
        self.harness.wait_for_depth(
            target_depth_m=target_depth,
            tolerance_m=0.20,
            settle_s=2.0,
            timeout_s=60.0,
        )

        # 7. Enter POSHOLD mode
        self.harness.set_mode('POSHOLD')

        # 8. Inject ocean current disturbance
        self.harness.set_node_parameters(
            'ocean_current_generator',
            [
                rclpy.parameter.Parameter(
                    'sine_freq',
                    rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                    [0.05, 0.3, 0.3],
                ),
                rclpy.parameter.Parameter(
                    'sine_amp',
                    rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                    [0.0, 0.5, 0.15],
                ),
                rclpy.parameter.Parameter(
                    'sigma',
                    rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                    [0.0, 0.15, 0.05],
                ),
            ],
        )

        # 9. Allow current to establish and vehicle to settle in POSHOLD
        settle_start = time.time()
        while time.time() - settle_start < 2.0:
            self.harness.spin_once(0.05)

        # 10. Record station keeping in VPD mode for 10 s
        samples_vpd = self.harness.record_odometry_window(duration_s=10.0)
        dev_vpd = self.harness.get_position_deviations(samples_vpd)
        self.harness.print_position_deviations('VPD Segment (10 s in currents)', dev_vpd)

        # 11. Switch slam_bridge to VPE mode
        self.harness.set_node_parameters(
            'slam_bridge',
            [
                rclpy.parameter.Parameter(
                    'use_vpe',
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                ),
            ],
        )

        # 12. Allow vehicle to settle under VPE
        settle_start = time.time()
        while time.time() - settle_start < 2.0:
            self.harness.spin_once(0.05)

        # 13. Record station keeping in VPE mode for 10 s
        samples_vpe = self.harness.record_odometry_window(duration_s=10.0)
        dev_vpe = self.harness.get_position_deviations(samples_vpe)
        self.harness.print_position_deviations('VPE Segment (10 s in currents)', dev_vpe)

        # 14. Verify hold tolerances
        max_xy_dev = 0.5
        max_z_dev_vpd = 0.5
        max_z_dev_vpe = 0.2
        self.assertLessEqual(
            dev_vpd['xy_max_dev'],
            max_xy_dev,
            f'VPD horizontal deviation ({dev_vpd["xy_max_dev"]:.4f} m) exceeded {max_xy_dev:.2f} m tolerance',
        )
        self.assertLessEqual(
            dev_vpe['xy_max_dev'],
            max_xy_dev,
            f'VPE horizontal deviation ({dev_vpe["xy_max_dev"]:.4f} m) exceeded {max_xy_dev:.2f} m tolerance',
        )
        self.assertLessEqual(
            dev_vpd['z_max_dev'],
            max_z_dev_vpd,
            f'VPD z deviation ({dev_vpd["z_max_dev"]:.4f} m) exceeded {max_z_dev_vpd:.2f} m tolerance',
        )
        self.assertLessEqual(
            dev_vpe['z_max_dev'],
            max_z_dev_vpe,
            f'VPE z deviation ({dev_vpe["z_max_dev"]:.4f} m) exceeded {max_z_dev_vpe:.2f} m tolerance',
        )


class GuidedOutAndBackScenarioMixin(unittest.TestCase):
    """Shared Guided out-and-back scenario."""

    MONO_IMU: bool = False
    HARNESS_NAME: str = 'test_guided_out_and_back_harness'

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        cleanup_out_and_back_lua()

    def setUp(self):
        self.harness = OrcaSimHarness(
            node_name=self.HARNESS_NAME,
            mav_endpoint='tcp:127.0.0.1:5760',
            use_sim_time=True,
            mono_imu=self.MONO_IMU,
        )

    def tearDown(self):
        self.harness.close()

    def run_out_and_back(self):
        """Dive with SLAM off, enter ALT_HOLD, enable SLAM, execute out-and-back, verify results."""
        # 1. Connect to ArduSub SITL via MAVLink
        self.harness.connect_mavlink(timeout_s=30.0)

        # 2. Dive into position and enable SLAM (mono_imu requires motion during initialization)
        target_depth = 2.0
        if self.MONO_IMU:
            self.harness.warmup_mono_imu(target_depth_m=target_depth, enable_slam=True)
        else:
            self.harness.dive_to_depth(target_depth_m=target_depth, throttle_pwm=1350, timeout_s=60.0)
            self.harness.set_slam_enabled(True)

        # 3. Wait for tracking lock and EKF aiding
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
