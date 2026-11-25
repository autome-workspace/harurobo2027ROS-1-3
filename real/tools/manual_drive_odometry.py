#!/usr/bin/env python3
"""Publish planar odometry from passive DM3520 feedback and the DM IMU."""

import math
from pathlib import Path
import socket
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('manual_drive_odometry')
        interface = self.declare_parameter('can_interface', 'can0').value
        default_repo = str(Path(__file__).resolve().parents[1] / 'manual_drive')
        repo = Path(self.declare_parameter('drive_repo', default_repo).value)
        sys.path.insert(0, str(repo / 'src/harurobo_base/test'))
        from bench_zero_enable import MOTORS, feedback_state
        from bench_single_pulse import velocity
        from manual_drive import FORWARD_METRES_PER_RAD, LEFT_METRES_PER_RAD
        from probe_dm3520_readonly import CAN_FRAME

        self.motors = tuple(MOTORS)
        self.feedback_state = feedback_state
        self.velocity = velocity
        self.frame = CAN_FRAME
        self.forward_scale = FORWARD_METRES_PER_RAD
        self.left_scale = LEFT_METRES_PER_RAD
        self.can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b''.join(struct.pack('=II', 0x10 + motor, 0xC00007FF) for motor in self.motors)
        self.can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        self.can.bind((interface,))
        self.can.setblocking(False)

        self.wheels = {}
        self.last_update = None
        self.imu = None
        self.initial_yaw = None
        self.x = self.y = 0.
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        imu_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data', self.on_imu, imu_qos)
        self.create_timer(.02, self.update)

    def on_imu(self, msg):
        q = msg.orientation
        gyro = msg.angular_velocity.z
        if (msg.header.frame_id != 'imu_link' or not .98 <= math.hypot(q.x, q.y, q.z, q.w) <= 1.02
                or not math.isfinite(gyro) or msg.orientation_covariance[0] < 0
                or msg.angular_velocity_covariance[0] < 0):
            self.imu = None
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if self.initial_yaw is None:
            self.initial_yaw = yaw
        self.imu = time.monotonic(), math.remainder(yaw - self.initial_yaw, math.tau), gyro

    def update(self):
        now = time.monotonic()
        dt = 0. if self.last_update is None else now - self.last_update
        self.last_update = now
        try:
            for _ in range(256):
                try:
                    packet = self.can.recv(self.frame.size)
                except BlockingIOError:
                    break
                motor = (self.frame.unpack(packet)[0] & 0x7ff) - 0x10
                if motor not in self.motors:
                    continue
                state = self.feedback_state(packet, motor)
                if state is not None:
                    self.wheels[motor] = (now, state, self.velocity(packet, 200.))
            else:
                self.wheels.clear()
                return
        except (OSError, RuntimeError) as error:
            self.wheels.clear()
            self.get_logger().warn(f'CAN receive failed: {error}', throttle_duration_sec=2.)
            return

        if self.imu is None or now - self.imu[0] > .15:
            self.get_logger().warn('Waiting for fresh IMU data', throttle_duration_sec=2.)
            return
        _, yaw, gyro = self.imu
        vx = vy = 0.
        if len(self.wheels) == 4 and all(now - t < .10 and state == 1
                                         for t, state, _ in self.wheels.values()):
            a, b, c, d = (self.wheels[m][2] for m in self.motors)
            vx = self.forward_scale * (a + b - c - d) / 4
            vy = self.left_scale * (a - b - c + d) / 4
            if dt <= .10:
                self.x += dt * (math.cos(yaw)*vx - math.sin(yaw)*vy)
                self.y += dt * (math.sin(yaw)*vx + math.cos(yaw)*vy)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = gyro
        self.publisher.publish(msg)
        tf = TransformStamped(header=msg.header, child_frame_id=msg.child_frame_id)
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = None
    try:
        node = WheelOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.can.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
