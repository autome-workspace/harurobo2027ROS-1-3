"""Isolated ROS+PTY integration test. Does not open physical serial or CAN."""
import binascii
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def packet(rid, values):
    data = b'\x55\xaa\x01' + bytes([rid]) + struct.pack('<fff', *values)
    return data + struct.pack('<H', binascii.crc_hqx(data, 0xffff)) + b'\x0a'


def main():
    rclpy.init()
    node = rclpy.create_node('check_imu_pty')
    samples, statuses = [], []
    node.create_subscription(Imu, '/imu/data', samples.append, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/diagnostics', statuses.append, 1)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait(predicate, limit=5):
        end = time.monotonic() + limit
        while not predicate():
            assert time.monotonic() < end, 'PTY test timed out'
            spin(.02)

    def last_status():
        return statuses[-1].status[0].level if statuses else None

    def create_pty(link):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        os.close(slave)
        if link.is_symlink():
            link.unlink()
        link.symlink_to(path)
        return master

    with tempfile.TemporaryDirectory(prefix='imu-pty.', dir='log') as work:
        link = Path(work).absolute() / 'device'
        master = create_pty(link)
        executable = Path('install/harurobo_imu/lib/harurobo_imu/imu_receiver').absolute()
        with open('log/stage-d-imu-pty-node.log', 'w') as log:
            child = subprocess.Popen([str(executable), '--ros-args', '-p', f'port:={link}'],
                                     stdout=log, stderr=subprocess.STDOUT)
            try:
                wait(lambda: last_status() == DiagnosticStatus.STALE)
                data = (packet(1, (0, 0, 9.81)) + packet(2, (0, 0, .25))
                        + packet(3, (0, 0, 90)))
                # Fragment serial bytes independently of protocol packets.
                for byte in data:
                    os.write(master, bytes([byte]))
                wait(lambda: len(samples) >= 1)
                sample = samples[-1]
                assert sample.header.frame_id == 'imu_link'
                assert abs(sample.angular_velocity.z - .25) < 1e-6
                assert abs(sample.linear_acceleration.z - 9.81) < 1e-5
                assert abs(sample.orientation.z - math.sqrt(.5)) < 1e-6
                assert abs(sample.orientation.w - math.sqrt(.5)) < 1e-6
                spin(.2)
                count = len(samples)
                assert count == 1, 'stale data republished'
                wait(lambda: last_status() == DiagnosticStatus.STALE)
                corrupt = bytearray(packet(2, (0, 0, .5)))
                corrupt[5] ^= 0x10
                os.write(master, packet(1, (0, 0, 9.81)) + corrupt + packet(3, (0, 0, 180)))
                spin(.15)
                assert len(samples) == count, 'bad CRC accepted or old gyro reused'
                os.write(master, packet(1, (0, 0, 9.81)) + packet(2, (0, 0, .5)))
                spin(.02)
                os.close(master)
                master = None
                wait(lambda: last_status() == DiagnosticStatus.ERROR)
                master = create_pty(link)
                wait(lambda: last_status() == DiagnosticStatus.STALE)
                os.write(master, packet(3, (0, 0, 180)))
                spin(.1)
                assert len(samples) == count, 'pre-disconnect sample reused'
                os.write(master, data)
                wait(lambda: len(samples) == count + 1)
                print('PASS: fragmented frames, SI/quaternion, CRC rejection, stale stop, disconnect/reconnect cache reset')
            finally:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
                if master is not None:
                    os.close(master)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
