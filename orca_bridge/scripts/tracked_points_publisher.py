#!/usr/bin/env python3

import rclpy
from orb_slam3_msgs.msg import SlamStatus
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class TrackedPointsPublisher(Node):
    def __init__(self):
        super().__init__('tracked_points_publisher')

        self.subscription = self.create_subscription(SlamStatus, '/slam_status', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, '/tracked_points', 10)

    def listener_callback(self, msg: SlamStatus):
        # Publish the tracked_points PointCloud2.
        # Zero out the timestamp so RViz/TF uses the latest available transform,
        # avoiding ExtrapolationExceptions caused by SLAM processing latency in recorded bags.
        points_msg = msg.tracked_points
        points_msg.header.stamp.sec = 0
        points_msg.header.stamp.nanosec = 0
        self.publisher_.publish(points_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackedPointsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in tracked_points_publisher: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
