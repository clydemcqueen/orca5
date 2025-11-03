"""
A simple Pose class for representing poses in ENU or NED coordinates.

Quaternions follow the transforms3d convention: [w, x, y, z], where w is the real part.
"""

import math

import geometry_msgs.msg
import numpy as np
import transforms3d


class Pose:
    """Simple pose object, using 2 tuples"""

    # Static transform camera_sensor (OpenCV) -> camera_link (FLU)
    T_OPENCV_FLU = None
    
    # Static transform camera_link (FLU) -> camera_sensor (OpenCV)
    T_FLU_OPENCV = None

    def __init__(self, p: tuple[float, float, float] = (0., 0., 0.), q: tuple[float, float, float, float] = (1., 0., 0., 0.)):
        self.p: tuple[float, float, float] = p
        self.q: tuple[float, float, float, float] = q

    @staticmethod
    def from_pose_msg(pose_msg: geometry_msgs.msg.Pose):
        return Pose(
            (pose_msg.position.x, pose_msg.position.y, pose_msg.position.z),
            (pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z))

    @staticmethod
    def from_transform_msg(tf_msg: geometry_msgs.msg.Transform):
        return Pose(
            (tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z),
            (tf_msg.rotation.w, tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z))

    @staticmethod
    def delta_pose(pose1: 'Pose', pose2: 'Pose'):
        return pose1.inverse().mult(pose2)

    def set_position(self, x: float, y: float, z: float):
        self.p = (x, y, z)

    def set_altitude(self, z: float):
        self.p = (self.p[0], self.p[1], z)

    def get_position(self):
        return self.p

    def set_quaternion(self, w: float, x: float, y: float, z: float):
        self.q = (w, x, y, z)

    def get_quaternion(self):
        return self.q

    def set_euler(self, roll: float, pitch: float, yaw: float):
        self.q = transforms3d.euler.euler2quat(roll, pitch, yaw)

    def get_euler(self) -> (float, float, float):
        return transforms3d.euler.quat2euler(self.q)

    def ned_to_enu_frame(self):
        """Convert NED to ENU by swapping axes only."""
        
        # Rearrange position axes
        p_enu = (self.p[1], self.p[0], -self.p[2])

        # Rearrange quaternion axes
        q_enu = (self.q[0], self.q[2], self.q[1], -self.q[3])

        return Pose(p_enu, q_enu)

    def ned_to_enu_standard(self):
        """Convert NED to ENU by swapping axes and rotating by 90 degrees around Z axis."""
        
        # Rearrange position axes
        p_enu = (self.p[1], self.p[0], -self.p[2])

        # Rearrange quaternion axes
        q_aux = (self.q[0], self.q[2], self.q[1], -self.q[3])

        # Rotate yaw by 90 degrees
        q_rz90 = [math.sqrt(2) * 0.5, 0, 0, math.sqrt(2) * 0.5]
        q_enu = transforms3d.quaternions.qmult(q_aux, q_rz90)

        return Pose(p_enu, q_enu)

    def enu_to_ned_frame(self):
        """Convert ENU to NED by swapping axes only."""
        
        return self.ned_to_enu_frame()  # Symmetric transformation

    def enu_to_ned_standard(self):
        """Convert ENU to NED by swapping axes and rotating by -90 degrees around Z axis."""
        
        # Rearrange position axes
        p_ned = (self.p[1], self.p[0], -self.p[2])

        # Rotate yaw by -90 degrees
        q_rz90_inv = (math.sqrt(2) * 0.5, 0, 0, -math.sqrt(2) * 0.5)
        q_aux = transforms3d.quaternions.qmult(self.q, q_rz90_inv)

        # Rearrange quaternion axes
        q_ned = (q_aux[0], q_aux[2], q_aux[1], -q_aux[3])

        return Pose(p_ned, q_ned)

    def apply_scale(self, scale):
        self.p = (self.p[0] * scale, self.p[1] * scale, self.p[2] * scale)

    def mult(self, rhs):
        tf_self = transforms3d.affines.compose(self.p, transforms3d.quaternions.quat2mat(self.q), np.ones(3))
        tf_rhs = transforms3d.affines.compose(rhs.p, transforms3d.quaternions.quat2mat(rhs.q), np.ones(3))
        tf_product = tf_self @ tf_rhs
        T, R, _, _ = transforms3d.affines.decompose(tf_product)
        return Pose(T, transforms3d.quaternions.mat2quat(R))

    def inverse(self):
        q_inv = transforms3d.quaternions.qinverse(self.q)
        p_inv = -transforms3d.quaternions.rotate_vector(self.p, q_inv)
        return Pose(p_inv, q_inv)

    def to_pose_msg(self) -> geometry_msgs.msg.Pose:
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position.x = self.p[0]
        pose_msg.position.y = self.p[1]
        pose_msg.position.z = self.p[2]
        pose_msg.orientation.w = self.q[0]
        pose_msg.orientation.x = self.q[1]
        pose_msg.orientation.y = self.q[2]
        pose_msg.orientation.z = self.q[3]
        return pose_msg

    def to_transform_msg(self) -> geometry_msgs.msg.Transform:
        tf_msg = geometry_msgs.msg.Transform()
        tf_msg.translation.x = self.p[0]
        tf_msg.translation.y = self.p[1]
        tf_msg.translation.z = self.p[2]
        tf_msg.rotation.w = self.q[0]
        tf_msg.rotation.x = self.q[1]
        tf_msg.rotation.y = self.q[2]
        tf_msg.rotation.z = self.q[3]
        return tf_msg

    def to_matrix(self) -> np.ndarray:
        return transforms3d.affines.compose(self.p, transforms3d.quaternions.quat2mat(self.q), np.ones(3))

    def __str__(self):
        p = self.p
        e = self.get_euler()
        return f"p=[{p[0] :8.3f}, {p[1] :8.3f}, {p[2] :8.3f}], e=[{e[0] :8.3f}, {e[1] :8.3f}, {e[2] :8.3f}]"


# Initialize static transforms
Pose.T_OPENCV_FLU = Pose()
Pose.T_OPENCV_FLU.set_euler(0, -math.pi / 2, math.pi / 2)
Pose.T_FLU_OPENCV = Pose.T_OPENCV_FLU.inverse()
