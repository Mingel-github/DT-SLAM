#!/usr/bin/env python3
"""Move the Gazebo test box uniformly between two world-axis limits."""

import argparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class BoxAxisPatrol(Node):
    def __init__(self, axis: str, lower: float, upper: float, speed: float):
        super().__init__('dt_slam_box_axis_patrol')
        if lower >= upper:
            raise ValueError('lower must be smaller than upper')
        if speed <= 0.0:
            raise ValueError('speed must be positive')

        self.axis = axis
        self.lower = lower
        self.upper = upper
        self.speed = speed
        self.direction = 1.0
        self.last_position = None
        self.publisher = self.create_publisher(Twist, '/box_cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/box_odom', self._on_odom, 10
        )
        self.get_logger().info(
            f'Uniform box patrol: axis={axis}, range=[{lower:.2f}, {upper:.2f}], '
            f'speed={speed:.2f} m/s'
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        coordinate = position.x if self.axis == 'x' else position.y
        self.last_position = coordinate

        if coordinate >= self.upper and self.direction > 0.0:
            self.direction = -1.0
            self.get_logger().info(f'Upper limit reached at {coordinate:.3f}')
        elif coordinate <= self.lower and self.direction < 0.0:
            self.direction = 1.0
            self.get_logger().info(f'Lower limit reached at {coordinate:.3f}')

        command = Twist()
        # The formal box spawns at yaw=0, so local X is world X.
        command.linear.x = self.speed * self.direction
        self.publisher.publish(command)

    def stop(self) -> None:
        self.publisher.publish(Twist())


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--axis', choices=('x', 'y'), default='x')
    parser.add_argument('--lower', type=float, required=True)
    parser.add_argument('--upper', type=float, required=True)
    parser.add_argument('--speed', type=float, default=0.5)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    rclpy.init()
    node = BoxAxisPatrol(
        arguments.axis,
        arguments.lower,
        arguments.upper,
        arguments.speed,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
