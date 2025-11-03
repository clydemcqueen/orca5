#!/usr/bin/env python3

import rclpy
import rclpy.node
import sensor_msgs.msg
import yaml


class CameraInfoPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        self.declare_parameter('camera_info_url', '')
        self.camera_info_url_ = self.get_parameter('camera_info_url').get_parameter_value().string_value

        self.declare_parameter('frame_id', 'camera_sensor')
        self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value

        if not self.camera_info_url_:
            self.get_logger().error('Missing camera_info_url parameter')
            return

        if not self.camera_info_url_.endswith('.yaml'):
            self.get_logger().error('Must be a yaml file')
            return

        # Strip file:// prefix if present
        if self.camera_info_url_.startswith('file://'):
            self.camera_info_url_ = self.camera_info_url_[7:]

        self.camera_info_msg_ = self.parse_yaml(self.camera_info_url_)
        if self.camera_info_msg_ is None:
            return

        self.get_logger().info(f'Loaded camera info from {self.camera_info_url_}')
        self.publisher_ = self.create_publisher(sensor_msgs.msg.CameraInfo, 'camera_info', 10)
        self.timer_ = self.create_timer(0.05, self.timer_callback)

    def parse_yaml(self, filename):
        try:
            with open(filename, 'r') as stream:
                calib_data = yaml.safe_load(stream)
        except FileNotFoundError:
            self.get_logger().error(f'Camera info file not found: {filename}')
            return None
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return None

        cam_info = sensor_msgs.msg.CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.k = calib_data['camera_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        cam_info.d = calib_data['distortion_coefficients']['data']
        cam_info.r = calib_data['rectification_matrix']['data']
        cam_info.p = calib_data['projection_matrix']['data']

        return cam_info

    def timer_callback(self):
        self.camera_info_msg_.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_msg_.header.frame_id = self.frame_id_
        self.publisher_.publish(self.camera_info_msg_)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()