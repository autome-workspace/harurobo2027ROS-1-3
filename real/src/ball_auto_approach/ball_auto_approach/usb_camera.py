import cv2
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .common import LATEST, run


class UsbCamera(Node):
    def __init__(self):
        super().__init__('usb_camera')
        self.declare_parameters('', [
            ('device', '/dev/video0'), ('width', 640), ('height', 480),
            ('fps', 30.0), ('frame', 'camera_optical_frame'), ('calibration', ''),
        ])
        param = lambda name: self.get_parameter(name).value
        self.frame = param('frame')
        self.width, self.height = param('width'), param('height')
        self.calibration = None
        if param('calibration'):
            with open(param('calibration'), encoding='utf-8') as stream:
                self.calibration = yaml.safe_load(stream)
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, '/camera/image_raw', LATEST)
        self.infos = self.create_publisher(CameraInfo, '/camera/camera_info', LATEST)
        self.capture = cv2.VideoCapture(param('device'), cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"Cannot open camera {param('device')}")
        for field, value in (
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
            (cv2.CAP_PROP_FRAME_WIDTH, self.width), (cv2.CAP_PROP_FRAME_HEIGHT, self.height),
            (cv2.CAP_PROP_FPS, param('fps')), (cv2.CAP_PROP_BUFFERSIZE, 1),
        ):
            self.capture.set(field, value)
        self.create_timer(1.0 / param('fps'), self.read)

    def read(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('Camera frame unavailable', throttle_duration_sec=2.0)
            return
        image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame
        info = CameraInfo(header=image.header, width=image.width, height=image.height)
        cal = self.calibration
        if cal and (cal['image_width'], cal['image_height']) == (image.width, image.height):
            info.distortion_model = cal['distortion_model']
            info.k = cal['camera_matrix']['data']
            info.d = cal['distortion_coefficients']['data']
            info.r = cal['rectification_matrix']['data']
            info.p = cal['projection_matrix']['data']
        self.infos.publish(info)
        self.images.publish(image)

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main():
    run(UsbCamera)
