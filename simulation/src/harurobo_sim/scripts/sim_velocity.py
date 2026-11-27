#!/usr/bin/env python3
"""Forward fresh planar velocity commands to Gazebo."""

from math import hypot, isfinite
from time import monotonic

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


MAX_LINEAR_SPEED = 0.25
MAX_ANGULAR_SPEED = 0.50


class SimVelocity(Node):
    def __init__(self):
        super().__init__('sim_velocity')
        self.command = Twist()
        self.received_at = float('-inf')
        self.output = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.create_subscription(TwistStamped, '/cmd_vel/manual', self.receive, 1)
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, message):
        twist = message.twist
        planar = (twist.linear.x, twist.linear.y, twist.angular.z)
        self.command = Twist()
        if message.header.frame_id in ('', 'base_link') and all(map(isfinite, planar)):
            x, y, yaw = planar
            speed = hypot(x, y)
            scale = MAX_LINEAR_SPEED / max(speed, MAX_LINEAR_SPEED)
            self.command.linear.x = x * scale
            self.command.linear.y = y * scale
            self.command.angular.z = max(-MAX_ANGULAR_SPEED, min(yaw, MAX_ANGULAR_SPEED))
        self.received_at = monotonic()

    def publish(self):
        self.output.publish(self.command if monotonic() - self.received_at < 0.30 else Twist())


def main():
    rclpy.init()
    node = SimVelocity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
