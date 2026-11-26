import math

import cv2
import numpy as np
from ball_approach_interfaces.msg import BallDetection
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .common import LATEST, run


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.declare_parameters('', [
            ('hsv_lower', [174, 70, 60]), ('hsv_upper', [12, 255, 255]),
            ('min_area', 80.0), ('min_circularity', 0.70), ('min_solidity', 0.75), ('debug', True),
        ])
        self.add_on_set_parameters_callback(self.validate)
        self.info = None
        self.bridge = CvBridge()
        self.detections = self.create_publisher(BallDetection, '/ball/detection', LATEST)
        self.debug = self.create_publisher(Image, '/ball/debug_image', LATEST)
        self.mask = self.create_publisher(Image, '/ball/mask', LATEST)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, LATEST)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, LATEST)

    def validate(self, updates):
        for update in updates:
            if update.name in ('hsv_lower', 'hsv_upper'):
                if (len(update.value) != 3
                        or any(not 0 <= v <= limit for v, limit in zip(update.value, (179, 255, 255)))):
                    return SetParametersResult(successful=False, reason='HSV limits: H 0..179, S/V 0..255')
        return SetParametersResult(successful=True)

    def on_info(self, info):
        self.info = info

    def on_image(self, image):
        param = lambda name: self.get_parameter(name).value
        detection = BallDetection(header=image.header)
        try:
            frame = self.bridge.imgmsg_to_cv2(image, 'bgr8').copy()
        except CvBridgeError:
            self.detections.publish(detection)
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low, high = param('hsv_lower'), param('hsv_upper')
        if low[0] <= high[0]:
            mask = cv2.inRange(hsv, tuple(low), tuple(high))
        else:
            mask = cv2.inRange(hsv, tuple(low), (179, *high[1:]))
            mask |= cv2.inRange(hsv, (0, *low[1:]), tuple(high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        label = 'Ball not detected'
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < param('min_area'):
                break
            # Specular highlights can cut into the edge of a colored ball.
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            perimeter = cv2.arcLength(hull, True)
            if (perimeter <= 0 or hull_area <= 0 or area / hull_area < param('min_solidity')
                    or 4 * math.pi * hull_area / perimeter**2 < param('min_circularity')):
                continue
            (u, v), radius = cv2.minEnclosingCircle(hull)
            if min(u, image.width - 1 - u, v, image.height - 1 - v) <= radius:
                continue
            detection.detected = True
            detection.u, detection.v = u, v
            detection.area, detection.diameter = area, 2 * radius
            cv2.drawContours(frame, [hull], -1, (0, 255, 0), 2)
            cv2.circle(frame, (round(u), round(v)), 4, (0, 0, 255), -1)
            ray = self.camera_ray(image, u, v, radius)
            if ray is not None:
                detection.ray.x, detection.ray.y = map(float, ray[0])
                detection.ray.z = 1.0
                detection.angular_radius = math.atan(float(np.linalg.norm(ray[1] - ray[0])))
                yaw = -math.degrees(math.atan(detection.ray.x))
                label = f'left {yaw:+.1f} deg  area {area:.0f} px2  diameter {2 * radius:.0f} px'
            else:
                label = f'Ball detected: center ({u:.0f}, {v:.0f}), diameter {2 * radius:.0f} px'
            break
        color = (0, 255, 0) if detection.detected else (180, 180, 180)
        cv2.putText(frame, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)
        self.detections.publish(detection)
        if param('debug'):
            for publisher, data, encoding in ((self.debug, frame, 'bgr8'), (self.mask, mask, 'mono8')):
                msg = self.bridge.cv2_to_imgmsg(data, encoding)
                msg.header = image.header
                publisher.publish(msg)

    def camera_ray(self, image, u, v, radius):
        info = self.info
        if (info is None or info.width != image.width or info.height != image.height
                or info.header.frame_id != image.header.frame_id or min(info.k[0], info.k[4]) <= 0):
            return None
        points = np.array([[[u, v]], [[u + radius, v]]], dtype=np.float64)
        matrix = np.asarray(info.k).reshape((3, 3))
        distortion = np.asarray(info.d)
        try:
            if info.distortion_model == 'equidistant':
                rays = cv2.fisheye.undistortPoints(points, matrix, distortion)
            elif info.distortion_model in ('plumb_bob', 'rational_polynomial', ''):
                rays = cv2.undistortPoints(points, matrix, distortion)
            else:
                return None
        except cv2.error:
            return None
        return rays[:, 0, :] if np.isfinite(rays).all() else None


def main():
    run(BallDetector)
