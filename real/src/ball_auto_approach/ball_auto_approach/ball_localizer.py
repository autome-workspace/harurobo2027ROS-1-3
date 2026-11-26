import math
from collections import deque

import numpy as np
from ball_approach_interfaces.msg import BallDetection, BallEstimate
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .common import LATEST, run, seconds


def transform_parts(stamped):
    t, q = stamped.transform.translation, stamped.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return rotation, np.array([t.x, t.y, t.z])


def scan_clusters(points, valid, gap, max_missing_bins):
    ids = np.flatnonzero(valid)
    if len(ids) == 0:
        return []
    cuts = np.flatnonzero((np.diff(ids) > max_missing_bins + 1)
                          | (np.linalg.norm(np.diff(points[ids], axis=0), axis=1) > gap))
    return list(np.split(ids, cuts + 1))


def circle_center(points, radius):
    closest = points[np.argmin(np.linalg.norm(points, axis=1))]
    center = closest * (1 + radius / np.linalg.norm(closest))
    for _ in range(8):
        diff = center - points
        lengths = np.maximum(np.linalg.norm(diff, axis=1), 1e-6)
        delta = np.linalg.lstsq(diff / lengths[:, None], radius - lengths, rcond=None)[0]
        if np.linalg.norm(delta) > radius:
            return None
        center += delta
    return center


def merge_adjacent_scan(scan, previous):
    """Fill empty bins from one recent revolution of the same scanner."""
    ranges = np.asarray(scan.ranges)
    present = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if (previous is None or scan.header.frame_id != previous.header.frame_id
            or len(previous.ranges) != len(ranges) or scan.angle_min != previous.angle_min
            or scan.angle_increment != previous.angle_increment
            or not 0 < seconds(scan.header.stamp) - seconds(previous.header.stamp) <= .105):
        return ranges, present
    return np.where(present, ranges, previous.ranges), present


class BallLocalizer(Node):
    def __init__(self):
        super().__init__('ball_localizer')
        self.declare_parameters('', [
            ('association_mode', 'calibrated'),
            ('track_max_step', .025), ('track_max_gap', .20),
            ('empirical_limits', [0.0]), ('empirical_coefficients', [0.0]),
            ('empirical_pixel_error', [14.0, 14.0, 20.0]),
            ('base_frame', 'base_link'), ('ball_radius', .030), ('ball_center_z', .045),
            ('max_stamp_skew', .10), ('scan_stamp_is_end', True),
            ('cluster_gap', .035), ('max_missing_bins', 2), ('min_points', 4), ('fit_error', .009),
            ('angular_gate', .07), ('size_ratio_limit', 1.6), ('max_range', 2.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        self.history = deque(maxlen=12)
        self.previous_scan = None
        self.lock = None
        self.track_failed = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.estimates = self.create_publisher(BallEstimate, '/ball/estimate', LATEST)
        self.markers = self.create_publisher(Marker, '/ball/marker', 10)
        self.create_subscription(BallDetection, '/ball/detection', lambda msg: self.history.append(msg), LATEST)
        self.create_subscription(BallEstimate, '/ball/tracking_lock', self.on_lock, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, LATEST)

    def on_scan(self, scan):
        previous, self.previous_scan = self.previous_scan, scan
        now = seconds(self.get_clock().now().to_msg())
        stamp = seconds(scan.header.stamp)
        duration = scan.scan_time
        reference = stamp + (-.5 if self.p('scan_stamp_is_end') else .5) * duration
        t = Time(nanoseconds=round(max(0., reference) * 1e9)) if math.isfinite(reference) else Time()
        estimate = BallEstimate(scan_range_min=float(scan.range_min), ball_radius=self.p('ball_radius'))
        estimate.header.stamp = t.to_msg()
        estimate.header.frame_id = self.p('base_frame')
        estimate.tracking = self.lock is not None
        fresh_scan = (stamp > 0 and math.isfinite(reference)
                      and 0 <= duration <= .3 and -.05 <= now - reference < .3)
        try:
            if estimate.tracking:
                estimate.image_stamp = self.lock.image_stamp
                gap = reference - seconds(self.lock.header.stamp)
                if gap <= 0:
                    return
                if fresh_scan and not self.track_failed and gap <= self.p('track_max_gap'):
                    self.match(scan, None, t, estimate, previous)
                if estimate.valid:
                    self.lock = estimate
                else:
                    self.track_failed = True
            elif fresh_scan and self.history:
                detection = min(self.history, key=lambda item: abs(seconds(item.header.stamp) - reference))
                estimate.image_stamp = detection.header.stamp
                if (self.history[-1].detected and detection.detected
                        and abs(seconds(detection.header.stamp) - reference) <= self.p('max_stamp_skew')):
                    self.match(scan, detection, t, estimate, previous)
        except (TransformException, ValueError, np.linalg.LinAlgError) as error:
            self.track_failed = estimate.tracking
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
        self.estimates.publish(estimate)
        marker = Marker(header=estimate.header, ns='ball', id=0, type=Marker.SPHERE)
        marker.action = Marker.ADD if estimate.valid else Marker.DELETE
        if estimate.valid:
            marker.pose.position = estimate.center
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 2 * estimate.ball_radius
            marker.color.g, marker.color.a = 1.0, .85
            marker.lifetime.nanosec = 300_000_000
        self.markers.publish(marker)

    def on_lock(self, message):
        self.lock = None
        self.track_failed = False
        if message.valid and message.header.frame_id == self.p('base_frame'):
            self.lock = message

    def matches_track(self, center, distance):
        previous = self.lock.center
        step = self.p('track_max_step')
        if (math.hypot(center[0] - previous.x, center[1] - previous.y) > step
                or abs(distance - self.lock.surface_distance) > step):
            return False
        return self.p('association_mode') != 'empirical' or self.in_empirical_range(center, distance)

    def match(self, scan, detection, t, estimate, previous):
        base = estimate.header.frame_id
        laser_rot, laser_origin = transform_parts(self.tf.lookup_transform(base, scan.header.frame_id, t))
        mode = self.p('association_mode')
        camera = None
        if mode == 'calibrated' and detection is not None:
            if detection.ray.z <= 0 or detection.angular_radius <= 0:
                return
            camera = transform_parts(self.tf.lookup_transform(
                base, detection.header.frame_id, Time.from_msg(detection.header.stamp)))
        elif mode not in ('calibrated', 'empirical'):
            return
        radius = estimate.ball_radius
        z = self.p('ball_center_z')
        if laser_rot[2, 2] < .998 or not .0 < radius <= .5:
            return
        offset = z - laser_origin[2]
        if abs(offset) >= .8 * radius:
            return
        section = math.sqrt(radius**2 - offset**2)
        ranges, present = merge_adjacent_scan(scan, previous if mode == 'empirical' else None)
        if len(ranges) < 5 or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
            return
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max) & (ranges > 0)
        r = np.where(valid, ranges, 0.)
        planar = np.column_stack((r * np.cos(angles), r * np.sin(angles)))
        points = np.column_stack((planar, np.zeros(len(r)))) @ laser_rot.T + laser_origin

        # Rotate a near-360-degree scan to avoid splitting a target at the scan seam.
        if abs(abs(scan.angle_increment) * len(ranges) - math.tau) < .03:
            anchor = int(np.argmin(np.where(valid, ranges, np.inf)))
            order = np.roll(np.arange(len(ranges)), len(ranges) // 2 - anchor)
            planar, points, valid, r, present = (array[order] for array in (planar, points, valid, r, present))

        candidates = []
        for ids in scan_clusters(planar, valid, self.p('cluster_gap'), self.p('max_missing_bins')):
            if (len(ids) < self.p('min_points') or np.count_nonzero(present[ids]) < 2
                    or np.min(r[ids]) > self.p('max_range')):
                continue
            samples = planar[ids]
            span = np.linalg.norm(samples[0] - samples[-1])
            if not .7 * section <= span <= 2.1 * section:
                continue
            center_2d = circle_center(samples, section)
            if center_2d is None:
                continue
            lengths = np.linalg.norm(samples - center_2d, axis=1)
            distance = float(np.linalg.norm(center_2d) - section)
            if (np.max(abs(lengths - section)) > self.p('fit_error')
                    or abs(distance - np.min(r[ids])) > self.p('fit_error')
                    or distance < scan.range_min):
                continue
            center = laser_rot @ np.array([*center_2d, 0.]) + laser_origin
            center[2] = z
            if center[0] <= 0:
                continue
            matched = (self.matches_track(center, distance) if estimate.tracking
                       else self.matches_image(detection, center, distance, radius, camera))
            if matched:
                candidates.append((center, distance, ids))
        if len(candidates) != 1:
            return
        center, distance, ids = candidates[0]
        estimate.center.x, estimate.center.y, estimate.center.z = map(float, center)
        estimate.surface_distance = distance
        valid[ids] = False
        estimate.obstacles = [Point32(x=float(x), y=float(y), z=float(z)) for x, y, z in points[valid]]
        estimate.valid = True

    def in_empirical_range(self, center, distance):
        limits = self.p('empirical_limits')
        angle = math.atan2(center[1], center[0])
        return len(limits) == 4 and limits[0] <= angle <= limits[1] and limits[2] <= distance <= limits[3]

    def matches_image(self, detection, center, distance, radius, camera):
        if camera is None:
            coefficients = self.p('empirical_coefficients')
            if len(coefficients) != 9 or not self.in_empirical_range(center, distance):
                return False
            angle = math.atan2(center[1], center[0])
            # Each row predicts image u, v or diameter from bearing and inverse depth.
            expected = np.asarray(coefficients).reshape((3, 3)) @ [1., angle, 1. / distance]
            tolerance = self.p('empirical_pixel_error')
            error = np.abs(expected - [detection.u, detection.v, detection.diameter])
            return len(tolerance) == 3 and bool(np.isfinite(error).all() and np.all(error <= tolerance))

        camera_rot, camera_origin = camera
        optical = camera_rot.T @ (center - camera_origin)
        depth = np.linalg.norm(optical)
        ray = np.array([detection.ray.x, detection.ray.y, detection.ray.z])
        if optical[2] <= 0 or depth <= radius or not np.isfinite(ray).all():
            return False
        ray_length = np.linalg.norm(ray)
        if ray_length <= 0:
            return False
        cosine = np.dot(ray, optical) / (ray_length * depth)
        angle = math.acos(float(np.clip(cosine, -1., 1.)))
        ratio = math.asin(radius / depth) / detection.angular_radius
        limit = self.p('size_ratio_limit')
        return angle <= self.p('angular_gate') and 1 / limit <= ratio <= limit


def main():
    run(BallLocalizer)
