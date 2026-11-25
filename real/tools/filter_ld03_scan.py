#!/usr/bin/env python3
"""Filter LD03 returns and publish scans whose acquisition time has an odom TF."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def filter_ranges(ranges, minimum, maximum, full_circle):
    def valid(value):
        return math.isfinite(value) and minimum <= value <= maximum

    count = len(ranges)
    filtered = []
    for i, value in enumerate(ranges):
        if not valid(value):
            filtered.append(math.inf)
            continue
        # Preserve edges and small structures with any nearby consistent return.
        tolerance = .12 + .03 * value
        neighbors = (i + offset for offset in (-2, -1, 1, 2))
        if any(valid(ranges[j % count]) and abs(ranges[j % count] - value) <= tolerance
               for j in neighbors if full_circle or 0 <= j < count):
            filtered.append(value)
        else:
            filtered.append(math.inf)
    return filtered


def crop_forward(scan, half_angle, center=0.):
    step = scan.angle_increment
    if not math.isfinite(step) or step <= 0:
        return None
    bearings = [(math.remainder(scan.angle_min + i * step - center, math.tau), i)
                for i in range(len(scan.ranges))]
    selected = sorted((angle, i) for angle, i in bearings if abs(angle) <= half_angle + 1e-6)
    if len(selected) < 3:
        return None
    first_angle = selected[0][0]
    if any(abs(angle - (first_angle + j * step)) > .01 * step
           for j, (angle, _) in enumerate(selected)):
        return None
    indices = [i for _, i in selected]
    if len(scan.intensities) == len(scan.ranges):
        scan.intensities = [scan.intensities[i] for i in indices]
    else:
        scan.intensities = []
    scan.ranges = [scan.ranges[i] for i in indices]
    scan.angle_min = first_angle
    scan.angle_max = first_angle + (len(indices) - 1) * step
    # Only a contiguous sector has regularly spaced acquisition times.
    contiguous = indices[-1] - indices[0] == len(indices) - 1
    if not contiguous:
        scan.time_increment = 0.
    return indices[0] if contiguous else 0


class ScanFilter(Node):
    def __init__(self):
        super().__init__('ld03_scan_filter')
        self.minimum = self.declare_parameter('min_range', .14).value
        width = self.declare_parameter('forward_fov_deg', 60.).value
        center = self.declare_parameter('forward_center_deg', 180.).value
        if not math.isfinite(center):
            raise ValueError('forward_center_deg must be finite')
        self.center = math.radians(math.remainder(center, 360.))
        if not math.isfinite(width) or not 0 < width <= 180:
            raise ValueError('forward_fov_deg must be in (0, 180]')
        self.half_angle = math.radians(width / 2)
        if not math.isfinite(self.minimum) or self.minimum < 0:
            raise ValueError('min_range must be finite and nonnegative')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(.01, self.publish_ready)
        self.pending = None

    def on_scan(self, scan):
        if len(scan.ranges) < 5 or not .01 < scan.scan_time < .5:
            return
        stamp = Time.from_msg(scan.header.stamp) - Duration(seconds=float(scan.scan_time))
        full_circle = abs(scan.angle_increment) * len(scan.ranges) >= math.tau - .025
        minimum = max(scan.range_min, self.minimum)
        scan.ranges = filter_ranges(scan.ranges, minimum, scan.range_max, full_circle)
        scan.range_min = minimum
        first_index = crop_forward(scan, self.half_angle, self.center)
        if first_index is not None:
            stamp += Duration(seconds=first_index * float(scan.time_increment))
            scan.header.stamp = stamp.to_msg()
            scan.header.frame_id = 'laser_forward'
            self.pending = (stamp, scan)

    def publish_ready(self):
        if self.pending is None:
            return
        stamp, scan = self.pending
        if self.get_clock().now().nanoseconds - stamp.nanoseconds > 500_000_000:
            self.pending = None
            self.get_logger().warning('No odom TF for scan acquisition time', throttle_duration_sec=2.)
            return
        if self.tf.can_transform('odom', scan.header.frame_id, stamp):
            self.publisher.publish(scan)
            self.pending = None


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
