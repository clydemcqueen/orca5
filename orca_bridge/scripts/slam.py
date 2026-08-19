import math

import geometry
import numpy as np
import orb_slam3_msgs.msg
import sensor_msgs.msg
import sub
import transforms3d
from sensor_msgs_py import point_cloud2


def scale_cloud(msg: sensor_msgs.msg.PointCloud2, scale: float):
    """Scale a point cloud."""

    cloud = point_cloud2.read_points_numpy(msg)
    cloud *= scale
    return point_cloud2.create_cloud_xyz32(msg.header, cloud)


def rf_distance(msg: orb_slam3_msgs.msg.SlamStatus) -> float:
    """Emulate a sonar rangefinder."""

    half_beam_angle_d = 20.0  # Beam angle of 40d
    max_tan_theta_sq = math.tan(math.radians(half_beam_angle_d)) ** 2
    min_points = 15

    if msg.tracked_points.width * msg.tracked_points.height < min_points:
        return 0.0

    points = point_cloud2.read_points_numpy(msg.tracked_points, field_names=['x', 'y', 'z'], skip_nans=True)
    if points.shape[0] < min_points:
        return 0.0

    # Get the pose of the world in the camera frame
    t_camera_world = geometry.Pose.from_pose_msg(msg.pose).inverse()

    # Move the tracked points from the world frame to the camera frame
    r_camera_world = transforms3d.quaternions.quat2mat(t_camera_world.q)
    p_camera_world = np.asarray(t_camera_world.p)
    points_camera = points @ r_camera_world.T + p_camera_world

    # Reject z values that are outside the beam
    xy_dist_sq = points_camera[:, 0] ** 2 + points_camera[:, 1] ** 2
    max_xy_dist_sq = (points_camera[:, 2] ** 2) * max_tan_theta_sq
    z_in_beam = points_camera[xy_dist_sq <= max_xy_dist_sq, 2]

    if z_in_beam.shape[0] < min_points:
        # print(f'Too few points inside the sonar beam: {z_in_beam.shape[0]} < {min_points}')
        return 0.0

    # The median is robust to outliers
    return float(np.percentile(z_in_beam, 50.0))


class LowPassFilter:
    """A simple low-pass filter implementation using exponential moving average."""

    def __init__(self, alpha: float):
        """
        Initialize the low-pass filter.

        Args:
            alpha: Smoothing factor between 0 and 1.
                   Higher values (closer to 1) = less smoothing, faster response.
                   Lower values (closer to 0) = more smoothing, slower response.
                   Typical values: 0.1 to 0.5
        """
        if not 0 < alpha <= 1:
            raise ValueError('Alpha must be between 0 (exclusive) and 1 (inclusive)')

        self.alpha = alpha
        self.value = None

    def update(self, measurement: float) -> float:
        if self.value is None:
            # Initialize with the first measurement
            self.value = measurement
        else:
            # Apply exponential moving average: y[n] = α * x[n] + (1 - α) * y[n-1]
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value

        return self.value

    def get_value(self) -> float | None:
        return self.value

    def reset(self):
        self.value = None


class SlamMap:
    """A single SLAM map"""

    def __init__(self, map_id: int, sonar_rf: float, slam_rf: float, t_map_base: geometry.Pose, logger):
        # The map id from ORB_SLAM3
        self.map_id = map_id

        # Rangefinder readings for logging
        self.sonar_rf = sonar_rf
        self.slam_rf = slam_rf

        # Scale factor for this map. Default to 1.0 until we get a good reading, then use a low pass filter
        self.scale = 1.0
        self.scale_filter = LowPassFilter(0.1)
        self.update_scale(sonar_rf, slam_rf, logger)

        # Initialize map -> slam from map -> base
        self.t_map_slam = t_map_base

        # The latest pose of the base link in the slam frame
        self.t_slam_base: geometry.Pose | None = None

    def update_scale(self, sonar_rf: float, slam_rf: float, logger):
        self.sonar_rf = sonar_rf
        self.slam_rf = slam_rf

        # Hard limits
        if slam_rf < 0.1 or slam_rf > 10.0:
            # logger.warn(f'slam_rf too low/high, dropping: {slam_rf:.3f}m')
            return
        if sonar_rf < 0.1 or sonar_rf > 10.0:
            # logger.warn(f'sonar_rf too low/high, dropping: {sonar_rf:.3f}m')
            return

        # Try to prevent sonar reflections
        instant_scale = sonar_rf / slam_rf
        if self.scale_filter.value is not None:
            expected_sonar_rf = slam_rf * self.scale
            if abs(sonar_rf - expected_sonar_rf) > 0.3 * expected_sonar_rf:
                logger.warn(f'sonar_rf jumped, dropping: {sonar_rf:.3f}m')
                return

        self.scale = self.scale_filter.update(instant_scale)

    def update_pose(self, t_slam_base: geometry.Pose):
        self.t_slam_base = t_slam_base


class SlamMaps:
    """A set of SLAM maps."""

    def __init__(self):
        # The set of maps that we've seen
        self.maps: dict[int, SlamMap] = {}

        # The current map
        self.current_map: SlamMap | None = None

    def update(self, msg: orb_slam3_msgs.msg.SlamStatus, sub: sub.Sub, logger):
        # Only trust the map data if we are tracking
        if msg.tracking_state != orb_slam3_msgs.msg.SlamStatus.TRACKING_OK:
            logger.error('Not tracking, do not update')
            return

        sonar_rf_distance = sub.sonar_rf_distance
        if sonar_rf_distance is None:
            logger.error('sonar_rf_distance is None in SlamMaps.update')
            return

        slam_rf_distance = rf_distance(msg)

        if self.current_map is None or msg.map_id not in self.maps:
            logger.info(f'Create map {msg.map_id}')
            t_map_base = sub.t_map_base_ned.ned_to_enu_frame()  # Use axes-only conversion
            self.maps[msg.map_id] = SlamMap(msg.map_id, sonar_rf_distance, slam_rf_distance, t_map_base, logger)
            self.current_map = self.maps[msg.map_id]
        else:
            if self.current_map.map_id != msg.map_id:
                # This never happens: ORB_SLAM3 creates a new map and then merges in the old map(s).
                # Sadly, we lose the old pose and scale information.
                logger.error(f'Switch to map {msg.map_id} -- WTF')
                self.current_map = self.maps[msg.map_id]
            self.current_map.update_scale(sonar_rf_distance, slam_rf_distance, logger)
