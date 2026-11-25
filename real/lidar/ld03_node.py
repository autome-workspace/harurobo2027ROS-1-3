#!/usr/bin/env python3
"""Minimal LD03 / LDROBOT 0x54 0x2c UART to ROS 2 LaserScan bridge.

For initial visualization and validation, not safety-critical navigation.
"""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial


# LDROBOT CRC-8: polynomial 0x4D, init 0, non-reflected.
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x4D if crc & 0x80 else 0)) & 0xFF
    return crc


class LD03Node(Node):
    def __init__(self):
        super().__init__('ld03_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('clockwise', False)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.clockwise = self.get_parameter('clockwise').value
        self.ser = serial.Serial(port, baud, timeout=0)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.buf = bytearray()
        self.ranges = [math.inf] * 360
        self.intensities = [0.0] * 360
        self.last_start = None
        self.scan_start = time.monotonic()
        self.count = 0
        self.good_packets = 0
        self.bad_packets = 0
        self.create_timer(0.005, self.poll)
        self.create_timer(5.0, self.log_status)
        self.get_logger().info(f'Listening on {port} @ {baud}, /scan (LaserScan)')

    def log_status(self):
        self.get_logger().info(
            f'valid packets={self.good_packets}, CRC errors={self.bad_packets}, scans published={self.count}'
        )

    def poll(self):
        available = self.ser.in_waiting
        if available:
            self.buf.extend(self.ser.read(min(available, 4096)))
        while len(self.buf) >= 47:
            pos = self.buf.find(b'\x54\x2c')
            if pos == -1:
                self.buf[:] = self.buf[-1:]
                return
            if pos:
                del self.buf[:pos]
            if len(self.buf) < 47:
                return
            packet = bytes(self.buf[:47])
            if crc8(packet[:46]) != packet[46]:
                self.bad_packets += 1
                del self.buf[0]
                continue
            del self.buf[:47]
            self.good_packets += 1
            self.decode(packet)

    def decode(self, packet):
        start = struct.unpack_from('<H', packet, 4)[0] / 100.0
        end = struct.unpack_from('<H', packet, 42)[0] / 100.0
        if self.last_start is not None and start < self.last_start - 180.0:
            self.publish_scan()
            self.ranges = [math.inf] * 360
            self.intensities = [0.0] * 360
            self.scan_start = time.monotonic()
        self.last_start = start
        diff = (end - start) % 360.0
        if diff > 45.0:  # reject implausibly large inter-point angles
            return
        for i in range(12):
            distance_mm = struct.unpack_from('<H', packet, 6 + i * 3)[0]
            intensity = packet[8 + i * 3]
            if not distance_mm:
                continue
            angle = (start + diff * i / 11.0) % 360.0
            if self.clockwise:
                angle = (-angle) % 360.0
            index = int(round(angle)) % 360
            distance_m = distance_mm / 1000.0
            if 0.05 <= distance_m <= 12.0:
                if distance_m < self.ranges[index]:
                    self.ranges[index] = distance_m
                    self.intensities[index] = float(intensity)

    def publish_scan(self):
        elapsed = time.monotonic() - self.scan_start
        if elapsed <= 0 or not any(math.isfinite(x) for x in self.ranges):
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / 360.0
        msg.angle_max = msg.angle_min + 359 * msg.angle_increment
        msg.scan_time = elapsed
        msg.time_increment = elapsed / 360.0  # approximate: points are not acquired uniformly
        msg.range_min = 0.05
        msg.range_max = 12.0
        msg.ranges = self.ranges.copy()
        msg.intensities = self.intensities.copy()
        self.pub.publish(msg)
        self.count += 1

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = LD03Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
