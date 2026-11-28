"""Publish the calibrated sphere detector to the existing approach interface."""
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from ball_approach_interfaces.msg import BallDetection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .ball_detector import CameraTracker, annotate_candidates
from .camera_model import CameraModel
from .storage import load_valid


class ApproachDetector(Node):
    def __init__(self):
        super().__init__('calibrated_approach_detector')
        self.declare_parameter('extrinsic_file', '')
        data, self.camera = load_valid(self.get_parameter('extrinsic_file').value)
        self.settings = data['settings']
        self.bridge = CvBridge()
        self.tracker = CameraTracker()
        self.compatible = False
        self.image = None
        self.image_at = 0.
        self.signature = None
        self.publisher = self.create_publisher(BallDetection, '/camera_lidar/ball_detection', 1)
        self.debug = self.create_publisher(Image, '/camera_lidar/approach_image', 1)
        self.create_subscription(CameraInfo, self.settings['info_topic'], self.info_received, qos_profile_sensor_data)
        self.create_subscription(Image, self.settings['image_topic'], self.image_received, qos_profile_sensor_data)
        # Keep enough history for nearest-image/scan matching in the existing localizer.
        self.create_timer(1 / max(15., self.settings['processing_hz']), self.tick)

    def info_received(self, msg):
        try:
            if self.settings['intrinsics_file']:
                self.compatible = ((msg.width, msg.height) == (self.camera.width, self.camera.height)
                                   and msg.header.frame_id == self.settings['camera_frame'])
            else:
                self.compatible = (CameraModel.from_info(msg, self.settings['image_rectified']).as_dict()
                                   == self.camera.as_dict() and msg.header.frame_id == self.settings['camera_frame'])
        except ValueError:
            self.compatible = False

    def image_received(self, msg):
        self.image, self.image_at = msg, time.monotonic()

    def tick(self):
        msg = self.image
        output = BallDetection()
        if msg is not None:
            output.header = msg.header
        age = ((self.get_clock().now().nanoseconds / 1e9 -
                (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)) if msg else float('inf'))
        if (msg is None or not self.compatible or time.monotonic() - self.image_at > .3
                or not -.05 <= age < .3 or msg.header.frame_id != self.settings['camera_frame']
                or (msg.width, msg.height) != (self.camera.width, self.camera.height)):
            self.tracker = CameraTracker()
            self.publisher.publish(output)
            return
        signature = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.signature == signature:
            return
        self.signature = signature
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            selected, candidates = self.tracker.detect(image, self.camera, self.settings)
            if selected:
                output.detected = True
                output.u, output.v = selected['uv']
                output.area, output.diameter = selected['area'], 2 * selected['radius']
                output.angular_radius = selected['angular_radius']
                output.ray.x, output.ray.y, output.ray.z = map(float, selected['ray'])
            annotate_candidates(image, candidates, selected)
            preview = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            preview.header = msg.header
            self.debug.publish(preview)
        except (ValueError, CvBridgeError, cv2.error) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)
            output.detected = False
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = ApproachDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
