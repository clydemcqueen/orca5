#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from example_interfaces.srv import Trigger  # Replace with the actual service type you need
from pynput import keyboard
from mavros_msgs.srv import CommandBool


class KeyListenerNode(Node):
    def __init__(self):
        super().__init__('key_listener')
        self.publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.channels = [1500] * 18  # Initialize RC channels with neutral values

        self.default_values = {
            1 - 1: 1500,  # Default for channel pitch
            2 - 1: 1500,  # Default for channel roll
            4 - 1: 1500,  # Default for channel yaw
            3 - 1: 1500,  # Default for channel throttle
            5 - 1: 1500,  # Default for channel forward
            6 - 1: 1500,  # Default for channel lateral
        }

        self.key_to_channel_map = {
            keyboard.KeyCode(char='8'): 1 - 1,
            keyboard.KeyCode(char='2'): 1 - 1,
            keyboard.KeyCode(char='4'): 2 - 1,
            keyboard.KeyCode(char='6'): 2 - 1,
            keyboard.KeyCode(char='w'): 5 - 1,
            keyboard.KeyCode(char='s'): 5 - 1,
            keyboard.KeyCode(char='a'): 6 - 1,
            keyboard.KeyCode(char='d'): 6 - 1,
            keyboard.Key.up: 3 - 1,
            keyboard.Key.down: 3 - 1,
            keyboard.Key.left: 4 - 1,
            keyboard.Key.right: 4 - 1,
        }

        self.listener = None

    def on_press(self, key):
        if key in self.key_to_channel_map:
            channel = self.key_to_channel_map[key]
            change = 1 if key in (
                keyboard.Key.up, keyboard.Key.right, keyboard.KeyCode(char='w'), keyboard.KeyCode(char='d'),
                keyboard.KeyCode(char='2'), keyboard.KeyCode(char='6')) else -1
            self.channels[channel] += change * 10  # Change value by 10 units
            self.channels[channel] = max(1100, min(1900, self.channels[channel]))  # Limit range
            self.publish_channels()

    def on_release(self, key):
        if key in self.key_to_channel_map:
            channel = self.key_to_channel_map[key]
            self.channels[channel] = self.default_values[channel]  # Reset to default
            self.publish_channels()
        if key == keyboard.Key.esc:
            return False

    def publish_channels(self):
        msg = OverrideRCIn()
        msg.channels = self.channels
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published RC Override: {self.channels}')

    def start_listener(self):
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()


def main(args=None):
    rclpy.init(args=args)
    node = KeyListenerNode()

    client = node.create_client(CommandBool, '/mavros/cmd/arming')  # Replace with the actual service name
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = CommandBool.Request()
    request.value = True  # Set the request value accordingly

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        if future.result().success:
            node.get_logger().info('Service call succeeded, starting key listener.')
            node.start_listener()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info('Keyboard Interrupt, closing node')
                request.value = False
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future)
                pass
            finally:
                if node.listener:
                    node.listener.stop()
                node.destroy_node()
                rclpy.shutdown()
        else:
            node.get_logger().error('Service call failed: %s' % future.result().result)
    else:
        node.get_logger().error('Service call failed')


if __name__ == '__main__':
    main()
