#!/usr/bin/env python3

"""
Subscribe to a PoseStamped topic, accumulate poses, and publish a Path message
"""

import rclpy
import rclpy.node
import rclpy.qos
import geometry_msgs.msg
import nav_msgs.msg

class PoseToPathNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('pose_to_path')

        self.declare_parameter('max_poses', 1000)
        self.declare_parameter('skip', 5)

        self.max_poses = self.get_parameter('max_poses').value
        self.skip = self.get_parameter('skip').value

        self.num_poses = 0  # Number of poses seen so far
        self.path_msg = nav_msgs.msg.Path()

        self.path_pub = self.create_publisher(nav_msgs.msg.Path, 'path', 10)

        # /ap/... topics are published best-effort
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.pose_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, 'pose', self.pose_callback, qos_profile)

    def pose_callback(self, msg: geometry_msgs.msg.PoseStamped):
        if len(self.path_msg.poses) > self.max_poses:
            # Too many poses, start over
            self.path_msg.poses.clear()
            self.num_poses = 0

        self.num_poses += 1
        if self.num_poses % (self.skip + 1) == 0:
            self.path_msg.header = msg.header
            self.path_msg.header.frame_id = 'map'
            self.path_msg.poses.append(msg)
            self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    pose_to_path_node = PoseToPathNode()
    rclpy.spin(pose_to_path_node)
    pose_to_path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()