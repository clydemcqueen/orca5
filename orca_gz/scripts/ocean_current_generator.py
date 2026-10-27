#!/usr/bin/env python3

"""
Publish ocean current velocity vectors (/ocean_current) in the Gazebo world frame (ENU: x=East, y=North, z=Up).

Each axis supports a combination of:
  1. Constant mean velocity (mean)
  2. Sinusoidal periodic wave/swell (sine_amp * sin(2 * pi * sine_freq * t))
  3. 1st-order Gauss-Markov / Ornstein-Uhlenbeck turbulence process (mu, sigma)

All parameters can be updated dynamically at runtime via `ros2 param set`.
"""

import math
import random
from typing import List

import geometry_msgs.msg
import rclpy
import rclpy.node
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.executors import ExternalShutdownException


class OceanCurrentGenerator(rclpy.node.Node):
    def __init__(self):
        super().__init__('ocean_current_generator')

        # Update rate in Hz (configured at startup)
        self.rate_hz: float = float(self.declare_parameter('rate_hz', 10.0).value)
        if self.rate_hz <= 0.0:
            self.rate_hz = 10.0
        self.dt: float = 1.0 / self.rate_hz

        # Random seed (-1 = non-deterministic system seed)
        self.seed: int = int(self.declare_parameter('seed', -1).value)
        self.rng = random.Random(self.seed if self.seed >= 0 else None)

        # Per-axis [x, y, z] parameters in the ENU world frame
        self.mean: List[float] = list(self.declare_parameter('mean', [0.0, 0.0, 0.0]).value)
        self.mu: List[float] = list(self.declare_parameter('mu', [0.5, 0.5, 0.5]).value)
        self.sigma: List[float] = list(self.declare_parameter('sigma', [0.0, 0.0, 0.0]).value)
        self.sine_amp: List[float] = list(self.declare_parameter('sine_amp', [0.0, 0.0, 0.0]).value)
        self.sine_freq: List[float] = list(self.declare_parameter('sine_freq', [0.05, 0.05, 0.2]).value)

        # Zero-mean Gauss-Markov state per axis
        self.gm_state: List[float] = [0.0, 0.0, 0.0]

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher = self.create_publisher(geometry_msgs.msg.Vector3, '/ocean_current', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'Ocean current generator started at {self.rate_hz:.1f} Hz '
            f'(mean={self.mean}, sigma={self.sigma}, sine_amp={self.sine_amp})'
        )

    def parameters_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name in ('mean', 'mu', 'sigma', 'sine_amp', 'sine_freq'):
                val = list(param.value)
                if len(val) != 3:
                    return SetParametersResult(
                        successful=False,
                        reason=f'Parameter {param.name} must be a 3-element array [x, y, z]',
                    )
                setattr(self, param.name, [float(v) for v in val])
                if param.name == 'sigma':
                    for i in range(3):
                        if self.sigma[i] == 0.0:
                            self.gm_state[i] = 0.0
            elif param.name == 'seed':
                self.seed = int(param.value)
                self.rng = random.Random(self.seed if self.seed >= 0 else None)
                self.gm_state = [0.0, 0.0, 0.0]

        return SetParametersResult(successful=True)

    def step_axis(self, i: int, time_sec: float) -> float:
        # 1st-order Gauss-Markov process around 0
        if self.sigma[i] > 0.0:
            decay = max(0.0, 1.0 - self.mu[i] * self.dt)
            self.gm_state[i] = decay * self.gm_state[i] + self.sigma[i] * math.sqrt(self.dt) * self.rng.gauss(0.0, 1.0)
        else:
            self.gm_state[i] = 0.0

        # Sinusoidal component
        sine = 0.0
        if self.sine_amp[i] != 0.0 and self.sine_freq[i] != 0.0:
            sine = self.sine_amp[i] * math.sin(2.0 * math.pi * self.sine_freq[i] * time_sec)

        return self.mean[i] + sine + self.gm_state[i]

    def timer_callback(self):
        time_sec = self.get_clock().now().nanoseconds / 1e9

        msg = geometry_msgs.msg.Vector3(
            x=self.step_axis(0, time_sec),
            y=self.step_axis(1, time_sec),
            z=self.step_axis(2, time_sec),
        )
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OceanCurrentGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
