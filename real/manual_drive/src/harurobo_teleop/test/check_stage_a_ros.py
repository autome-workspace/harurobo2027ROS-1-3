"""Smoke test: run with the teleop node alone on an isolated ROS domain."""
import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    rclpy.init()
    node = rclpy.create_node('check_stage_a_ros')
    pub = node.create_publisher(Joy, '/joy', qos_profile_sensor_data)
    observed = {'armed': None, 'twist': None}
    node.create_subscription(Bool, '/manual/armed', lambda msg: observed.update(armed=msg.data), 1)
    node.create_subscription(TwistStamped, '/cmd_vel/manual',
                             lambda msg: observed.update(twist=msg), 1)

    def joy(x=0.0, y=0.0, yaw=0.0, pressed=False):
        msg = Joy()
        msg.axes = [y, x, yaw, 0.0, 0.0, 0.0]
        msg.buttons = [0] * 15
        msg.buttons[9] = int(pressed)
        return msg

    def spin(duration, msg=None):
        end = time.monotonic() + duration
        next_pub = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if msg is not None and now >= next_pub:
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                next_pub = now + 0.02
            rclpy.spin_once(node, timeout_sec=0.005)

    def expect(armed, vx=0.0, vy=0.0, wz=0.0):
        check(observed['armed'] is armed, f'armed: expected {armed}, got {observed["armed"]}')
        msg = observed['twist']
        check(msg is not None and msg.header.frame_id == 'base_link', 'missing base_link command')
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        check(all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(values, (vx, vy, wz))),
              f'velocity: expected {(vx, vy, wz)}, got {values}')

    try:
        deadline = time.monotonic() + 8.0
        while not pub.get_subscription_count() or observed['twist'] is None or observed['armed'] is None:
            check(time.monotonic() < deadline, 'teleop discovery timed out')
            spin(0.05)
        expect(False)
        spin(0.2, joy(pressed=True))
        expect(False)
        spin(0.2, joy())
        spin(0.2, joy(pressed=True))
        expect(True)
        spin(0.2, joy(1.0, -1.0, 1.0, True))
        expect(True, 0.25 / math.sqrt(2.0), -0.25 / math.sqrt(2.0), 0.5)
        spin(0.2, joy(1.0))
        expect(False)
        spin(0.2, joy(1.0, pressed=True))
        expect(False)
        spin(0.2, joy())
        spin(0.2, joy(pressed=True))
        spin(0.2, joy(1.0, pressed=True))
        expect(True, 0.25)
        start = time.monotonic()
        while observed['armed'] or observed['twist'].twist.linear.x != 0.0:
            check(time.monotonic() - start < 0.25, 'no stop within 250 ms of ending Joy stream')
            rclpy.spin_once(node, timeout_sec=0.005)
        print(f'Joy stream stop: {(time.monotonic() - start) * 1000:.0f} ms after test stream ended')
        spin(0.2, joy(1.0, pressed=True))
        expect(False)
        spin(0.2, joy())
        spin(0.2, joy(pressed=True))
        expect(True)
        spin(0.2, Joy())
        expect(False)
        print('PASS: startup, L1, mapping, diagonal cap, timeout, re-arm and malformed Joy')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
