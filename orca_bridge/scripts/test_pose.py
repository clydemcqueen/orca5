"""
Unit tests for the Pose class coordinate conversion methods.

colcon test
colcon test-result --all
"""

import math
import unittest
import transforms3d
from geometry import Pose


class TestPoseCoordinateConversions(unittest.TestCase):
    """Test suite for ned_to_enu and enu_to_ned methods."""

    def setUp(self):
        """Set up common test data."""
        self.epsilon = 1e-6

    def assertPoseAlmostEqual(self, pose1, pose2, places=6):
        """Helper to assert that two poses are approximately equal."""
        for i in range(3):
            self.assertAlmostEqual(pose1.p[i], pose2.p[i], places=places,
                                   msg=f"Position component {i} differs")
        for i in range(4):
            self.assertAlmostEqual(pose1.q[i], pose2.q[i], places=places,
                                   msg=f"Quaternion component {i} differs")

    def test_ned_to_enu_frame_position_basic(self):
        """Test to_enu_frame converts NED position to ENU correctly."""
        # NED: North=1, East=2, Down=3
        pose_ned = Pose(p=(1.0, 2.0, 3.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_frame()

        # ENU: East=2, North=1, Up=-3
        expected = Pose(p=(2.0, 1.0, -3.0), q=(1.0, 0.0, 0.0, 0.0))
        self.assertPoseAlmostEqual(pose_enu, expected)

    def test_ned_to_enu_frame_quaternion_basic(self):
        """Test to_enu_frame converts NED quaternion to ENU correctly."""
        # NED with yaw rotation (rotation around Z/Down axis)
        pose_ned = Pose(p=(0.0, 0.0, 0.0), q=(0.924, 0.0, 0.0, 0.383))  # ~45 deg yaw
        pose_enu = pose_ned.ned_to_enu_frame()

        # Quaternion should be remapped: (w, x, y, z) -> (w, y, x, -z)
        expected_q = (0.924, 0.0, 0.0, -0.383)
        self.assertAlmostEqual(pose_enu.q[0], expected_q[0], places=3)
        self.assertAlmostEqual(pose_enu.q[1], expected_q[1], places=3)
        self.assertAlmostEqual(pose_enu.q[2], expected_q[2], places=3)
        self.assertAlmostEqual(pose_enu.q[3], expected_q[3], places=3)

    def test_ned_to_enu_standard_position_basic(self):
        """Test to_enu_standard converts NED position to ENU correctly."""
        pose_ned = Pose(p=(1.0, 2.0, 3.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_standard()

        # Position should be the same as to_enu_frame
        expected_p = (2.0, 1.0, -3.0)
        for i in range(3):
            self.assertAlmostEqual(pose_enu.p[i], expected_p[i])

    def test_ned_to_enu_standard_adds_90_deg_yaw(self):
        """Test to_enu_standard adds +90 degree yaw rotation."""
        # Start with identity quaternion (no rotation)
        pose_ned = Pose(p=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_standard()

        # Should have +90 deg rotation around Z axis
        # q = [cos(45°), 0, 0, sin(45°)] = [sqrt(2)/2, 0, 0, sqrt(2)/2]
        expected_q = (math.sqrt(2) * 0.5, 0.0, 0.0, math.sqrt(2) * 0.5)
        for i in range(4):
            self.assertAlmostEqual(pose_enu.q[i], expected_q[i], places=5)

    def test_enu_to_ned_frame_position_basic(self):
        """Test to_ned_frame converts ENU position to NED correctly."""
        # ENU: East=1, North=2, Up=3
        pose_enu = Pose(p=(1.0, 2.0, 3.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_ned = pose_enu.enu_to_ned_frame()

        # NED: North=2, East=1, Down=-3
        expected = Pose(p=(2.0, 1.0, -3.0), q=(1.0, 0.0, 0.0, 0.0))
        self.assertPoseAlmostEqual(pose_ned, expected)

    def test_enu_to_ned_frame_quaternion_basic(self):
        """Test to_ned_frame converts ENU quaternion to NED correctly."""
        # ENU with yaw rotation (rotation around Z/Up axis)
        pose_enu = Pose(p=(0.0, 0.0, 0.0), q=(0.924, 0.0, 0.0, 0.383))  # ~45 deg yaw
        pose_ned = pose_enu.enu_to_ned_frame()

        # Quaternion should be remapped: (w, x, y, z) -> (w, y, x, -z)
        expected_q = (0.924, 0.0, 0.0, -0.383)
        self.assertAlmostEqual(pose_ned.q[0], expected_q[0], places=3)
        self.assertAlmostEqual(pose_ned.q[1], expected_q[1], places=3)
        self.assertAlmostEqual(pose_ned.q[2], expected_q[2], places=3)
        self.assertAlmostEqual(pose_ned.q[3], expected_q[3], places=3)

    def test_enu_to_ned_standard_position_basic(self):
        """Test to_ned_standard converts ENU position to NED correctly."""
        pose_enu = Pose(p=(1.0, 2.0, 3.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_ned = pose_enu.enu_to_ned_standard()

        # Position should be the same as to_ned_frame
        expected_p = (2.0, 1.0, -3.0)
        for i in range(3):
            self.assertAlmostEqual(pose_ned.p[i], expected_p[i])

    def test_enu_to_ned_standard_subtracts_90_deg_yaw(self):
        """Test to_ned_standard subtracts -90 degree yaw rotation."""
        # Start with identity quaternion (no rotation)
        pose_enu = Pose(p=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_ned = pose_enu.enu_to_ned_standard()

        # After -90 deg yaw rotation in ENU, then axis swap to NED
        # Final result should be identity after full conversion
        expected_q = (math.sqrt(2) * 0.5, 0.0, 0.0, math.sqrt(2) * 0.5)
        for i in range(4):
            self.assertAlmostEqual(pose_ned.q[i], expected_q[i], places=5)

    def test_enu_ned_frame_roundtrip(self):
        """Test that to_enu_frame and to_ned_frame are inverse operations."""
        original = Pose(p=(1.0, 2.0, 3.0), q=(0.924, 0.1, 0.2, 0.3))
        roundtrip = original.ned_to_enu_frame().enu_to_ned_frame()

        self.assertPoseAlmostEqual(original, roundtrip)

    def test_ned_enu_frame_roundtrip(self):
        """Test that to_ned_frame and to_enu_frame are inverse operations."""
        original = Pose(p=(5.0, -2.0, 10.0), q=(0.8, 0.3, 0.4, 0.3))
        roundtrip = original.enu_to_ned_frame().ned_to_enu_frame()

        self.assertPoseAlmostEqual(original, roundtrip)

    def test_enu_ned_standard_roundtrip(self):
        """Test roundtrip conversion using standard methods."""
        original = Pose(p=(1.0, 2.0, 3.0), q=(0.924, 0.1, 0.2, 0.3))
        roundtrip = original.ned_to_enu_standard().enu_to_ned_standard()

        self.assertPoseAlmostEqual(original, roundtrip)

    def test_ned_enu_standard_roundtrip(self):
        """Test roundtrip conversion using standard methods."""
        original = Pose(p=(5.0, -2.0, 10.0), q=(0.8, 0.3, 0.4, 0.3))
        roundtrip = original.enu_to_ned_standard().ned_to_enu_standard()

        self.assertPoseAlmostEqual(original, roundtrip)

    def test_ned_to_enu_frame_with_roll_pitch(self):
        """Test to_enu_frame with non-zero roll and pitch."""
        # Create pose with roll and pitch in NED
        roll, pitch, yaw = 0.1, 0.2, 0.0
        q_ned = transforms3d.euler.euler2quat(roll, pitch, yaw)
        pose_ned = Pose(p=(1.0, 2.0, 3.0), q=q_ned)

        pose_enu = pose_ned.ned_to_enu_frame()

        # Verify position transformation
        self.assertAlmostEqual(pose_enu.p[0], 2.0)
        self.assertAlmostEqual(pose_enu.p[1], 1.0)
        self.assertAlmostEqual(pose_enu.p[2], -3.0)

        # Quaternion should be remapped but roundtrip should work
        pose_back = pose_enu.enu_to_ned_frame()
        self.assertPoseAlmostEqual(pose_ned, pose_back)

    def test_enu_to_ned_frame_with_roll_pitch(self):
        """Test to_ned_frame with non-zero roll and pitch."""
        # Create pose with roll and pitch in ENU
        roll, pitch, yaw = 0.1, 0.2, 0.0
        q_enu = transforms3d.euler.euler2quat(roll, pitch, yaw)
        pose_enu = Pose(p=(1.0, 2.0, 3.0), q=q_enu)

        pose_ned = pose_enu.enu_to_ned_frame()

        # Verify position transformation
        self.assertAlmostEqual(pose_ned.p[0], 2.0)
        self.assertAlmostEqual(pose_ned.p[1], 1.0)
        self.assertAlmostEqual(pose_ned.p[2], -3.0)

        # Roundtrip should work
        pose_back = pose_ned.ned_to_enu_frame()
        self.assertPoseAlmostEqual(pose_enu, pose_back)

    def test_ned_to_enu_standard_yaw_alignment(self):
        """Test that to_enu_standard properly aligns coordinate frames."""
        # In NED: North is "forward" (X+)
        # In ENU: East is "forward" (X+)
        # Standard conversion should align these by adding 90° yaw

        # Pose pointing North in NED (identity orientation)
        pose_ned = Pose(p=(10.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_standard()

        # After conversion, should be pointing East in ENU
        # Position: North=10 in NED -> East=0, North=10 in ENU
        self.assertAlmostEqual(pose_enu.p[0], 0.0)
        self.assertAlmostEqual(pose_enu.p[1], 10.0)
        self.assertAlmostEqual(pose_enu.p[2], 0.0)

    def test_enu_to_ned_standard_yaw_alignment(self):
        """Test that to_ned_standard properly aligns coordinate frames."""
        # In ENU: East is "forward" (X+)
        # In NED: North is "forward" (X+)
        # Standard conversion should align these by subtracting 90° yaw

        # Pose pointing East in ENU (identity orientation)
        pose_enu = Pose(p=(10.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_ned = pose_enu.enu_to_ned_standard()

        # Position: East=10 in ENU -> North=0, East=10 in NED
        self.assertAlmostEqual(pose_ned.p[0], 0.0)
        self.assertAlmostEqual(pose_ned.p[1], 10.0)
        self.assertAlmostEqual(pose_ned.p[2], 0.0)

    def test_ned_to_enu_frame_zero_position(self):
        """Test to_enu_frame at the origin."""
        pose_ned = Pose(p=(0.0, 0.0, 0.0), q=(0.707, 0.0, 0.707, 0.0))
        pose_enu = pose_ned.ned_to_enu_frame()

        self.assertAlmostEqual(pose_enu.p[0], 0.0)
        self.assertAlmostEqual(pose_enu.p[1], 0.0)
        self.assertAlmostEqual(pose_enu.p[2], 0.0)

    def test_enu_to_ned_frame_zero_position(self):
        """Test to_ned_frame at the origin."""
        pose_enu = Pose(p=(0.0, 0.0, 0.0), q=(0.707, 0.0, 0.707, 0.0))
        pose_ned = pose_enu.enu_to_ned_frame()

        self.assertAlmostEqual(pose_ned.p[0], 0.0)
        self.assertAlmostEqual(pose_ned.p[1], 0.0)
        self.assertAlmostEqual(pose_ned.p[2], 0.0)

    def test_ned_to_enu_frame_negative_altitude(self):
        """Test to_enu_frame with negative altitude."""
        # NED: Down=-5 means 5 meters above surface
        pose_ned = Pose(p=(0.0, 0.0, -5.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_frame()

        # ENU: Up should be +5 (above surface)
        self.assertAlmostEqual(pose_enu.p[2], 5.0)

    def test_enu_to_ned_frame_negative_altitude(self):
        """Test to_ned_frame with negative altitude."""
        # ENU: Up=-5 means 5 meters below surface
        pose_enu = Pose(p=(0.0, 0.0, -5.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_ned = pose_enu.enu_to_ned_frame()

        # NED: Down should be +5 (below surface)
        self.assertAlmostEqual(pose_ned.p[2], 5.0)

    def test_quaternion_normalization_preserved(self):
        """Test that quaternion normalization is preserved through conversions."""
        # Create normalized quaternion
        q = (0.5, 0.5, 0.5, 0.5)
        pose_ned = Pose(p=(1.0, 2.0, 3.0), q=q)

        # Check NED quaternion is normalized
        q_norm: float = sum(x ** 2 for x in pose_ned.q)
        self.assertAlmostEqual(q_norm, 1.0)

        # Convert to ENU and check normalization
        pose_enu = pose_ned.ned_to_enu_frame()
        q_norm_enu:float = sum(x ** 2 for x in pose_enu.q)
        self.assertAlmostEqual(q_norm_enu, 1.0)

        # Convert to standard and check normalization
        pose_enu_std = pose_ned.ned_to_enu_standard()
        q_norm_enu_std:float = sum(x ** 2 for x in pose_enu_std.q)
        self.assertAlmostEqual(q_norm_enu_std, 1.0)

    def test_large_position_values(self):
        """Test conversions with large position values."""
        pose_ned = Pose(p=(1000.0, -2000.0, 500.0), q=(1.0, 0.0, 0.0, 0.0))
        pose_enu = pose_ned.ned_to_enu_frame()

        self.assertAlmostEqual(pose_enu.p[0], -2000.0)
        self.assertAlmostEqual(pose_enu.p[1], 1000.0)
        self.assertAlmostEqual(pose_enu.p[2], -500.0)

        # Roundtrip
        pose_back = pose_enu.enu_to_ned_frame()
        self.assertPoseAlmostEqual(pose_ned, pose_back)

    def test_standard_frame_static_transforms(self):
        """Test T_STANDARD_FRAME and T_FRAME_STANDARD static transforms."""
        pose_enu_standard = Pose(p=(1.0, 2.0, 3.0), q=(1.0, 0.0, 0.0, 0.0))

        pose_enu_frame = pose_enu_standard.enu_to_ned_standard().ned_to_enu_frame()
        pose_enu_frame_method2 = pose_enu_standard.mult(Pose.T_STANDARD_FRAME)
        self.assertPoseAlmostEqual(pose_enu_frame, pose_enu_frame_method2)

        pose_enu_standard = pose_enu_frame.enu_to_ned_frame().ned_to_enu_standard()
        pose_enu_standard_method2 = pose_enu_frame.mult(Pose.T_FRAME_STANDARD)
        self.assertPoseAlmostEqual(pose_enu_standard, pose_enu_standard_method2)

if __name__ == '__main__':
    unittest.main()