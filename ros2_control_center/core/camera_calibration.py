"""ROS image calibration worker. JSON lines on stdin/stdout keep Qt independent."""
import base64
import json
import select
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def emit(**message):
    print(json.dumps(message), flush=True)


class Calibration:
    def __init__(self, columns, rows, square_mm):
        self.board = (columns, rows)
        self.square_m = square_mm / 1000
        self.size = None
        self.samples = []
        self.descriptors = []
        self.corners = None
        self.descriptor = None
        self.stamp = 0.
        self.model = None

    def detect(self, frame):
        size = (frame.shape[1], frame.shape[0])
        if self.size is not None and size != self.size:
            raise ValueError('映像サイズが変わりました。採取をやり直してください。')
        self.size = size
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, self.board)
        self.corners = corners if found else None
        self.descriptor = None
        self.stamp = time.monotonic()
        if found:
            points = corners.reshape(-1, 2) / np.array(size)
            # Include normalized grid shape so tilted views are distinguished.
            self.descriptor = points
            cv2.drawChessboardCorners(frame, self.board, corners, found)
        return found

    def capture(self):
        if self.corners is None or time.monotonic() - self.stamp > 1.5:
            raise ValueError('チェッカーボードが検出されていません。')
        if len(self.samples) >= 40:
            raise ValueError('採取上限の40枚に達しました。')
        if any(np.sqrt(np.mean((self.descriptor - old) ** 2)) < .025 for old in self.descriptors):
            raise ValueError('既存の画像と近い姿勢です。位置・距離・傾きを変えてください。')
        self.samples.append(self.corners.copy())
        self.descriptors.append(self.descriptor.copy())
        self.model = None

    def solve(self):
        if len(self.samples) < 12:
            raise ValueError('異なる位置・角度で12枚以上採取してください。')
        points = np.zeros((self.board[0] * self.board[1], 3), np.float32)
        points[:, :2] = np.mgrid[:self.board[0], :self.board[1]].T.reshape(-1, 2) * self.square_m
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(
            [points.copy() for _ in self.samples], self.samples, self.size, None, None)
        if not np.isfinite(rms) or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
            raise ValueError('較正値を計算できませんでした。')
        if rms > 1.0:
            raise ValueError(f'再投影誤差が大きいため保存できません（{rms:.3f} px）。採取をやり直してください。')
        projection = np.column_stack((matrix, np.zeros(3)))
        self.model = {
            'image_width': self.size[0], 'image_height': self.size[1],
            'camera_name': 'control_center_camera', 'distortion_model': 'plumb_bob',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.ravel().tolist()},
            'distortion_coefficients': {'rows': 1, 'cols': distortion.size, 'data': distortion.ravel().tolist()},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4, 'data': projection.ravel().tolist()},
        }
        return rms


def main():
    columns, rows = map(int, sys.argv[1:3])
    square_mm = float(sys.argv[3])
    if not (3 <= columns <= 20 and 3 <= rows <= 20 and 1 <= square_mm <= 200):
        raise ValueError('較正ボードの値が範囲外です。')
    calibration = Calibration(columns, rows, square_mm)
    rclpy.init(args=[])
    node = rclpy.create_node('control_center_camera_calibration')
    bridge = CvBridge()
    latest = [None]
    last_frame = 0.

    def receive(msg):
        latest[0] = msg

    subscription = node.create_subscription(Image, '/camera/image_raw', receive, qos_profile_sensor_data)
    emit(status='カメラ映像を待っています。')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                try:
                    command = json.loads(line)
                    if command['action'] == 'capture':
                        calibration.capture()
                        emit(status=f'{len(calibration.samples)}枚を採取しました。', count=len(calibration.samples))
                    elif command['action'] == 'solve':
                        rms = calibration.solve()
                        emit(status=f'較正完了・再投影誤差 {rms:.3f} px', result=calibration.model)
                except (ValueError, KeyError, cv2.error) as error:
                    emit(error=str(error))
            if latest[0] is None or time.monotonic() - last_frame < .4:
                continue
            msg, latest[0] = latest[0], None
            try:
                # Reject delayed frames to avoid saving a stale board pose.
                age = node.get_clock().now().nanoseconds / 1e9 - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
                if not -.1 <= age < 1.:
                    emit(error='カメラ映像の時刻が古いため採取できません。')
                    calibration.corners = None
                    continue
                frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                found = calibration.detect(frame)
                preview = cv2.resize(frame, (480, round(frame.shape[0] * 480 / frame.shape[1])))
                ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    emit(image=base64.b64encode(encoded).decode(),
                         found=found, size=list(calibration.size))
            except (ValueError, cv2.error) as error:
                emit(error=str(error))
            last_frame = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
