"""Isolated synthetic ROS test. No physical serial or CAN access."""
import math
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('check_heading_preview')
    imu_pub = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    cmd_pub = node.create_publisher(TwistStamped, '/cmd_vel/manual', 1)
    received = []
    node.create_subscription(TwistStamped, '/cmd_vel/manual/heading_preview', received.append, 1)

    def phase(vx=0., yaw=0., manual=0., imu=True, valid=True, old=False, duration=.3):
        end = time.monotonic()+duration
        next_pub = 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                now = node.get_clock().now().to_msg()
                command = TwistStamped()
                command.header.stamp = now
                command.header.frame_id = 'base_link'
                command.twist.linear.x = vx
                command.twist.angular.z = manual
                cmd_pub.publish(command)
                if imu:
                    sample = Imu()
                    sample.header.stamp = now
                    if old:
                        sample.header.stamp.sec -= 1
                    sample.header.frame_id = 'imu_link'
                    sample.orientation.z = math.sin(yaw/2) if valid else 0.
                    sample.orientation.w = math.cos(yaw/2) if valid else 0.
                    imu_pub.publish(sample)
                next_pub = time.monotonic()+.02
            rclpy.spin_once(node, timeout_sec=.005)

    def expect(vx=0., wz=0.):
        assert received, 'missing preview messages'
        output = received[-1]
        assert output.header.frame_id == 'base_link'
        assert math.isclose(output.twist.linear.x, vx, abs_tol=1e-6), output
        assert math.isclose(output.twist.angular.z, wz, abs_tol=1e-6), output

    end=time.monotonic()+5
    while not imu_pub.get_subscription_count() or not cmd_pub.get_subscription_count() or not received:
        assert time.monotonic() < end, 'discovery timeout'
        rclpy.spin_once(node,timeout_sec=.02)
    try:
        phase(); expect()
        phase(vx=.1,duration=.45); expect(.1,0.)
        phase(vx=.1,yaw=.2); expect(.1,-.1)
        phase(vx=.1,yaw=1.); expect(.1,-.2)
        phase(vx=.1,yaw=1.,manual=.3); expect(.1,.3)
        phase(yaw=1.); expect()
        phase(vx=.1,yaw=1.,duration=.45); expect(.1,0.)
        phase(vx=.1,imu=False); expect()
        phase(vx=.1,yaw=1.); expect()  # Fresh samples alone cannot clear the fault.
        phase(yaw=1.); expect()
        phase(vx=.1,yaw=1.,duration=.45); expect(.1,0.)
        phase(vx=.1,valid=False); expect()
        phase(); expect()
        phase(vx=.1,old=True); expect()
        print('PASS: capture, correction sign/cap, manual priority, neutral reset, stale IMU latch, quaternion/stamp rejection')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
