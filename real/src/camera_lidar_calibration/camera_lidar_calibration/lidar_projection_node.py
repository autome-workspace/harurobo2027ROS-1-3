from collections import deque
import time
import signal

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Float32

from .ball_detector import find_balls
from .calibration_node import CalibrationNode
from .camera_model import CameraModel
from .lidar_ball_detector import detect_candidates
from .projection import overlay
from .storage import load_valid


class ProjectionNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_projection')
        self.declare_parameter('extrinsic_file', '')
        data, self.camera = load_valid(self.get_parameter('extrinsic_file').value)
        self.settings = data['settings']
        self.result = data['result']
        self.bridge = CvBridge()
        self.images = deque(maxlen=16)
        self.scan = None
        self.scan_received = self.last_image = self.last_output = 0.
        self.compatible = True
        self.previous = None
        self.signature = None
        self.image_pub = self.create_publisher(Image, '/camera_lidar/overlay', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera_lidar/point_cloud', 1)
        self.position_pub = self.create_publisher(PointStamped, '/ball/position', 1)
        self.distance_pub = self.create_publisher(Float32, '/ball/distance', 1)
        self.detected_pub = self.create_publisher(Bool, '/ball/detected', 1)
        self.create_subscription(Image, self.settings['image_topic'], self.image_received, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.settings['info_topic'], self.info_received, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.settings['scan_topic'], self.scan_received_callback, qos_profile_sensor_data)
        self.create_timer(1 / self.settings['processing_hz'], self.tick)

    # Sharing synchronization semantics also preserves the LD03 end-of-scan correction.
    ready_observation = CalibrationNode.ready_observation

    def image_received(self, msg):
        self.images.append(msg)
        self.last_image = time.monotonic()

    def scan_received_callback(self, msg):
        self.scan = msg
        self.scan_received = time.monotonic()

    def info_received(self, msg):
        try:
            if self.settings['intrinsics_file']:
                self.compatible = ((msg.width, msg.height) == (self.camera.width, self.camera.height)
                                   and msg.header.frame_id == self.settings['camera_frame'])
                return
            camera = CameraModel.from_info(msg, self.settings['image_rectified'])
            self.compatible = (camera.as_dict() == self.camera.as_dict()
                               and msg.header.frame_id == self.settings['camera_frame'])
            if not self.compatible:
                self.get_logger().error('CameraInfoが保存較正と一致しません。投影を停止します。', throttle_duration_sec=2.)
        except ValueError:
            self.compatible = False

    def lost(self):
        self.detected_pub.publish(Bool(data=False))
        self.distance_pub.publish(Float32(data=float('nan')))
        self.previous = None

    def tick(self):
        try:
            ready, _ = self.ready_observation()
            if ready is None:
                self.lost()
                return
            image_msg, scan, reference, _ = ready
            signature = (scan.header.stamp.sec, scan.header.stamp.nanosec)
            if signature == self.signature:
                if time.monotonic() - self.last_output > self.settings['stale_timeout']:
                    self.lost()
                return
            self.signature = signature
            image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            detection = find_balls(image, self.camera, self.settings)
            points, candidates = detect_candidates(scan, self.settings)
            preview, selected = overlay(image, points, candidates, detection, self.result, self.camera,
                                        self.settings, self.previous)
            output = self.bridge.cv2_to_imgmsg(preview, 'bgr8')
            output.header = image_msg.header
            self.image_pub.publish(output)
            self.cloud_pub.publish(create_cloud_xyz32(scan.header, np.column_stack((points, np.zeros(len(points))))))
            if selected is None:
                self.lost()
            else:
                position = PointStamped()
                position.header.frame_id = self.settings['lidar_frame']
                # Position corresponds to the scan reference time.
                from rclpy.time import Time
                position.header.stamp = Time(nanoseconds=round(reference * 1e9)).to_msg()
                position.point.x, position.point.y, position.point.z = selected['center']
                self.position_pub.publish(position)
                self.distance_pub.publish(Float32(data=selected['surface_distance']))
                self.detected_pub.publish(Bool(data=True))
                self.previous = np.array(selected['center'])
            self.last_output = time.monotonic()
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)
            self.lost()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    def request_stop(signum, frame):
        stopping[0] = True
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, request_stop)
    node = None
    try:
        node = ProjectionNode()
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    except (ValueError, OSError, KeyError) as error:
        print(f'投影を開始できません。{error}', flush=True)
        return 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
