from collections import deque
import copy
import json
from pathlib import Path
import subprocess
import sys
import time
import signal

import yaml

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from .ball_detector import CameraTracker, annotate_candidates, make_mask, pick_hsv
from .camera_model import CameraModel
from .extrinsic_solver import solve, validate_result
from .lidar_ball_detector import detect_candidates
from .projection import overlay
from .samples import SampleCollector
from .settings import atomic_yaml, defaults, user_directory, validate
from .storage import bundle, read_bundle, save_bundle


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_calibration')
        settings = defaults()
        local = user_directory() / 'settings.yaml'
        load_error = ''
        if local.exists():
            try:
                local_settings = yaml.safe_load(local.read_text()) or {}
                settings.update(local_settings)
                validate(settings)
            except (ValueError, TypeError, OSError, yaml.YAMLError) as error:
                settings = defaults()
                load_error = f'保存設定を読み込めません。設定画面で修正してください。{error}'
        self.declare_parameters('', list(settings.items()))
        self.settings = validate({key: self.get_parameter(key).value for key in settings})
        self.bridge = CvBridge()
        self.status_pub = self.create_publisher(String, '/camera_lidar/calibration/status', 1)
        self.preview_pub = self.create_publisher(Image, '/camera_lidar/calibration/preview', 1)
        self.create_subscription(String, '/camera_lidar/calibration/command', self.command, 10)
        self.sources = []
        self.tf_process = None
        self.last_saved = None
        self.show_mask = False
        self.tf_status = ''
        self.create_subscription(String, '/camera_lidar/tf_status', self.tf_status_received, 1)
        self.result = None
        self.message = load_error or 'センサを待っています。'
        self.last_image = 0.
        self.latest_status = {}
        self.configure_sources()
        self.timer = self.create_timer(1 / self.settings['processing_hz'], self.tick)

    def configure_sources(self):
        for subscription in self.sources:
            self.destroy_subscription(subscription)
        self.images = deque(maxlen=16)
        self.scan = None
        self.scan_received = 0.
        self.last_image = 0.
        self.last_processed = None
        self.camera = None
        self.compatible = True
        self.collector = SampleCollector(self.settings)
        self.selected_center = None
        self.manual_selection = False
        self.camera_tracker = CameraTracker()
        if self.settings['intrinsics_file']:
            try:
                self.camera = CameraModel.from_yaml(self.settings['intrinsics_file'], self.settings['image_rectified'])
            except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
                self.message = f'内部較正YAMLを読み込めません。{error}'
        self.sources = [
            self.create_subscription(Image, self.settings['image_topic'], self.image_received, qos_profile_sensor_data),
            self.create_subscription(CameraInfo, self.settings['info_topic'], self.info_received, qos_profile_sensor_data),
            self.create_subscription(LaserScan, self.settings['scan_topic'], self.scan_received_callback, qos_profile_sensor_data),
        ]

    def image_received(self, msg):
        self.images.append(msg)
        self.last_image = time.monotonic()

    def info_received(self, msg):
        try:
            if msg.header.frame_id != self.settings['camera_frame']:
                raise ValueError(f"CameraInfoフレームが設定と異なります（{msg.header.frame_id}）。")
            if self.settings['intrinsics_file']:
                if self.camera is None or (msg.width, msg.height) != (self.camera.width, self.camera.height):
                    raise ValueError('内部較正YAMLとCameraInfoの解像度が一致しません。')
                self.compatible = True
                return
            camera = CameraModel.from_info(msg, self.settings['image_rectified'])
            if self.camera and self.camera.as_dict() != camera.as_dict() and self.collector.samples:
                self.compatible = False
                raise ValueError('内部較正が変わりました。保存後にサンプルを全削除して取り直してください。')
            self.camera = camera
            self.compatible = True
        except (ValueError, cv2.error) as error:
            self.compatible = False
            if self.result is not None:
                self.result['valid'] = False
                self.result['warnings'] = ['内部較正またはCameraInfoが一致しません。']
            if self.tf_active():
                self.stop_tf()
                self.tf_status = '内部較正の変化でTF配信を停止しました。受信側の再起動が必要です。'
            self.message = str(error)

    def scan_received_callback(self, msg):
        self.scan = msg
        self.scan_received = time.monotonic()

    def tf_status_received(self, msg):
        try:
            self.tf_status = json.loads(msg.data)['message']
        except (ValueError, KeyError):
            pass

    def tf_active(self):
        return self.tf_process is not None and self.tf_process.poll() is None

    def command(self, msg):
        try:
            request = json.loads(msg.data)
            action = request['action']
            if self.tf_active() and action in ('configure', 'capture', 'delete', 'clear', 'solve', 'load', 'auto', 'tune_stationarity', 'pick_color', 'sample_preset', 'lidar_sector', 'lidar_min_points'):
                raise ValueError('先に「TF配信停止」を押してください。取付変更後は受信側のTFキャッシュも再起動が必要です。')
            if action == 'capture':
                sample = self.collector.capture(request.get('role', 'auto'))
                self.invalidate_result(sample['role'])
                self.message = f"サンプル {sample['id']} を取得しました。"
            elif action == 'auto':
                self.collector.auto = bool(request['enabled'])
            elif action == 'tune_stationarity':
                allowed = ('stationary_duration', 'stationary_position', 'stationary_pixels')
                changes = request['values']
                if set(changes) != set(allowed):
                    raise ValueError('静止条件の項目が無効です。')
                settings = validate(self.settings | changes)
                atomic_yaml(user_directory() / 'settings.yaml', settings)
                self.settings = settings
                self.collector.settings = settings
                self.collector.history.clear()
                self.collector.latest = None
                self.message = '静止条件を保存しました。取得済みサンプルは保持しています。'
            elif action == 'show_mask':
                self.show_mask = bool(request['enabled'])
            elif action == 'pick_color':
                if not self.images or time.monotonic() - self.last_image > self.settings['stale_timeout']:
                    raise ValueError('新しいカメラ映像を待ってください。')
                if self.collector.samples and not request.get('clear_samples'):
                    raise ValueError('色認識条件の変更前にサンプルを保存・全削除してください。')
                image = self.bridge.imgmsg_to_cv2(self.images[-1], 'bgr8')
                lower, upper = pick_hsv(image, *request['uv'])
                settings = validate(self.settings | {'hsv_lower': lower, 'hsv_upper': upper})
                atomic_yaml(user_directory() / 'settings.yaml', settings)
                self.settings = settings
                self.collector = SampleCollector(settings)
                self.camera_tracker.reset()
                self.result = None
                self.export_color_parameters()
                self.message = f'ボールの色を設定しました。HSV下限 {lower} / 上限 {upper}'
            elif action == 'select_camera':
                if request.get('uv') is None:
                    self.camera_tracker.reset()
                else:
                    self.camera_tracker.select(request['uv'], self.latest_status.get('camera_candidates', []))
                self.collector.latest = None
                self.collector.history.clear()
                self.message = '画像のボール選択を更新しました。'
            elif action == 'select':
                value = request.get('center')
                self.selected_center = np.asarray(value, float) if value is not None else None
                if self.selected_center is not None and (self.selected_center.shape != (3,) or not np.isfinite(self.selected_center).all()):
                    raise ValueError('選択位置が無効です。')
                self.manual_selection = value is not None
                self.collector.observe(None)
                self.collector.history.clear()
            elif action == 'delete':
                self.collector.delete(request['ids'])
                self.result = None
            elif action == 'clear':
                self.collector.samples.clear()
                self.collector.observe(None)
                self.collector.history.clear()
                self.result = None
                self.camera = None
                self.compatible = True
            elif action in ('lidar_sector', 'lidar_min_points'):
                changes = request['values']
                allowed = ({'lidar_sector_enabled', 'lidar_sector_center_deg', 'lidar_sector_width_deg'}
                           if action == 'lidar_sector' else {'min_points'})
                if set(changes) != allowed:
                    raise ValueError('LiDAR設定の項目が無効です。')
                settings = validate(self.settings | changes)
                atomic_yaml(user_directory() / 'settings.yaml', settings)
                self.settings = settings
                self.collector.settings = settings
                self.collector.auto = False
                self.collector.history.clear()
                self.collector.latest = None
                self.selected_center = None
                self.manual_selection = False
                self.result = None
                self.message = 'LiDAR設定を保存しました。取得済みサンプルは保持しています。再較正後に結果を保存してください。'
                if settings['min_points'] == 3:
                    self.message += ' 3点判定は誤認識に注意し、画像と位置を照合してください。'
            elif action == 'sample_preset':
                presets = {'five': (4, 1, 5, 5), 'standard': (12, 4, 25, 5)}
                if request.get('preset') not in presets:
                    raise ValueError('サンプル数の設定が無効です。')
                values = presets[request['preset']]
                settings = validate(self.settings | dict(zip(
                    ('min_train_samples', 'min_validation_samples', 'target_samples', 'validation_every'), values)))
                atomic_yaml(user_directory() / 'settings.yaml', settings)
                self.settings = settings
                self.collector.settings = settings
                self.collector.auto = False
                self.result = None
                self.message = ('5点モード：推定4点＋独立検証1点。少数点では誤認識に弱いため、'
                                '四隅に広げて取得し、別位置でも投影を確認してください。'
                                if request['preset'] == 'five' else '標準モードに戻しました。')
                self.message += ' 取得済みサンプルと用途は保持しています。'
            elif action == 'solve':
                if self.camera is None:
                    raise ValueError('カメラ内部較正がありません。')
                self.collector.auto = False
                self.result = None
                self.result = solve(self.collector.samples, self.camera, self.settings)
                self.message = '較正成功' if self.result['valid'] else '警告 ' + ' / '.join(self.result['warnings'])
            elif action == 'validate':
                if self.result is None:
                    raise ValueError('先に外部較正を実行してください。')
                self.result = solve(self.collector.samples, self.camera, self.settings)
                validation = validate_result(self.result, self.collector.samples, self.camera, self.settings)
                self.result['independent_validation'] = validation
                if not validation['valid']:
                    self.result['valid'] = False
                self.message = '独立検証 ' + json.dumps(validation, ensure_ascii=False)
            elif action == 'save':
                path = str(Path(request['path']).expanduser().resolve())
                settings = self.settings | {'extrinsic_file': path}
                save_bundle(path, bundle(settings, self.camera, self.collector.samples, self.result))
                self.last_saved = path
                self.settings = settings
                try:
                    atomic_yaml(user_directory() / 'settings.yaml', settings)
                    self.message = f'保存しました {path}'
                except OSError as error:
                    self.message = f'結果は保存しましたが、既定の投影ファイル設定を保存できません。{error}'
            elif action == 'load':
                data, camera = read_bundle(request['path'])
                self.settings = data['settings']
                self.configure_sources()
                self.timer.timer_period_ns = round(1e9 / self.settings['processing_hz'])
                self.camera = camera
                self.collector.samples = data['samples']
                self.collector.next_id = max((sample['id'] for sample in data['samples']), default=0) + 1
                self.result = data['result']
                self.last_saved = str(Path(request['path']).expanduser())
                self.message = '保存データを読み込みました。センサなしでも再計算できます。'
            elif action == 'configure':
                settings = validate(request['settings'])
                if self.collector.samples and not request.get('clear_samples'):
                    raise ValueError('設定変更にはサンプルの全削除確認が必要です。')
                atomic_yaml(user_directory() / 'settings.yaml', settings)
                self.settings = settings
                self.result = None
                self.configure_sources()
                self.timer.timer_period_ns = round(1e9 / settings['processing_hz'])
                self.export_color_parameters()
                self.message = '設定を保存しました。'
            elif action == 'apply_tf':
                if not self.compatible:
                    raise ValueError('現在のカメラ内部較正が保存結果と一致しません。')
                if not self.result or not self.result['valid'] or not self.last_saved:
                    raise ValueError('有効な較正結果を保存してから適用してください。')
                save_bundle(self.last_saved, bundle(self.settings, self.camera, self.collector.samples, self.result))
                if self.tf_active():
                    raise ValueError('TFは配信中です。')
                self.tf_status = 'TF競合を確認しています。'
                self.tf_process = subprocess.Popen([
                    sys.executable, '-m', 'camera_lidar_calibration.calibration_tf_publisher',
                    '--ros-args', '-p', 'extrinsic_file:=' + json.dumps(self.last_saved)])
                self.message = 'TF競合を監査しています。配信開始・拒否は状態表示で確認してください。'
            elif action == 'stop_tf':
                self.stop_tf()
                self.message = 'TF配信停止。既存の受信側に残った静的TFは、受信ノード再起動で消去してください。'
            else:
                raise ValueError('未対応の操作です。')
        except (ValueError, KeyError, TypeError, OSError, cv2.error, yaml.YAMLError) as error:
            self.message = str(error)
            self.get_logger().warning(self.message)
        self.publish_status()

    def export_color_parameters(self):
        parameters = {key: self.settings[key] for key in ('hsv_lower', 'hsv_upper')}
        try:
            atomic_yaml(user_directory() / 'ball_detector.yaml',
                        {'ball_detector': {'ros__parameters': parameters}})
        except OSError as error:
            self.get_logger().warning(f'既存認識ノード用の色設定を保存できません。{error}')

    def invalidate_result(self, role):
        # Additional held-out observations may validate the same fit without retraining.
        if role == 'train':
            self.result = None
        elif self.result is not None:
            self.result['valid'] = False
            self.result['warnings'] = ['新しい検証点で再較正・検証してください。']

    def ready_observation(self):
        now = time.monotonic()
        if self.images and (self.camera is None or not self.compatible):
            image = self.images[-1]
            return None, f'内部較正待ち（{image.width}×{image.height}用の内部較正が必要です）'
        if (not self.images or self.scan is None
                or now - self.last_image > self.settings['stale_timeout']
                or now - self.scan_received > self.settings['stale_timeout']):
            return None, 'センサまたは内部較正を待っています。'
        scan = self.scan
        if scan.header.frame_id != self.settings['lidar_frame']:
            return None, f'LiDARフレームが異なります（{scan.header.frame_id}）。'
        if not (0 < scan.scan_time <= .3):
            return None, 'スキャンの一周時間が無効です。'
        sign = -.5 if self.settings['scan_stamp_is_end'] else .5
        reference = stamp_seconds(scan.header.stamp) + sign * scan.scan_time
        image = min(self.images, key=lambda msg: abs(stamp_seconds(msg.header.stamp) - reference))
        delta = abs(stamp_seconds(image.header.stamp) - reference)
        ros_now = self.get_clock().now().nanoseconds / 1e9
        if (delta > self.settings['sync_tolerance'] or
                not -.05 <= ros_now - reference < self.settings['stale_timeout'] or
                not -.05 <= ros_now - stamp_seconds(image.header.stamp) < self.settings['stale_timeout']):
            return None, '時刻同期またはデータ鮮度の条件を満たしていません。'
        if (image.width, image.height) != (self.camera.width, self.camera.height):
            return None, '画像サイズと内部較正が一致しません。'
        if image.header.frame_id != self.settings['camera_frame']:
            return None, '画像の光学フレーム名が一致しません。'
        return (image, scan, reference, delta), ''

    def select_lidar(self, candidates):
        selected = None
        if self.manual_selection:
            if self.selected_center is not None and candidates:
                closest = min(candidates, key=lambda item: np.linalg.norm(np.array(item['center']) - self.selected_center))
                if np.linalg.norm(np.array(closest['center']) - self.selected_center) <= self.settings['track_step']:
                    selected = closest
                    self.selected_center = np.array(closest['center'])
            if selected is None:
                self.selected_center = None
        elif len(candidates) == 1:
            selected = candidates[0]
        return selected

    def lidar_preview(self):
        if self.scan is None or time.monotonic() - self.scan_received > self.settings['stale_timeout']:
            return [], [], None, 'データ待ち'
        scan = self.scan
        if scan.header.frame_id != self.settings['lidar_frame']:
            return [], [], None, 'フレーム名の不一致'
        if not 0 < scan.scan_time <= .3:
            return [], [], None, '走査時間が無効'
        sign = -.5 if self.settings['scan_stamp_is_end'] else .5
        reference = stamp_seconds(scan.header.stamp) + sign * scan.scan_time
        age = self.get_clock().now().nanoseconds / 1e9 - reference
        if not -.05 <= age < self.settings['stale_timeout']:
            return [], [], None, '受信時刻が古い'
        points, candidates = detect_candidates(scan, self.settings)
        return points.tolist(), candidates, self.select_lidar(candidates), f'受信中（{len(points)}点）'

    def tick(self):
        if self.tf_process is not None and self.tf_process.poll() is not None:
            code = self.tf_process.returncode
            if code:
                self.tf_status += f' TF配信は終了しました（終了コード {code}）。'
            self.tf_process = None
        try:
            ready, reason = self.ready_observation()
            if ready is None:
                self.collector.observe(None)
                self.latest_status = {'sensor_status': reason, 'points': [], 'candidates': [],
                                      'camera_detected': False, 'lidar_detected': False, 'sync_delta': None}
                points, candidates, selected, lidar_status = self.lidar_preview()
                self.latest_status.update(
                    points=points[::max(1, len(points) // 720)], candidates=candidates, selected=selected,
                    lidar_detected=selected is not None, lidar_status=lidar_status)
                # Each sensor remains visible while the other input is unavailable.
                if self.images and time.monotonic() - self.last_image < self.settings['stale_timeout']:
                    msg = self.images[-1]
                    image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
                    camera = (self.camera if self.compatible and self.camera is not None
                              and (msg.width, msg.height) == (self.camera.width, self.camera.height) else None)
                    detection, camera_candidates = self.camera_tracker.detect(image, camera, self.settings)
                    count = len(camera_candidates)
                    self.latest_status.update(camera_detected=detection is not None, camera_count=count,
                                              camera_candidates=camera_candidates, detection=detection)
                    if self.show_mask:
                        image = cv2.cvtColor(make_mask(image, self.settings), cv2.COLOR_GRAY2BGR)
                    annotate_candidates(image, camera_candidates, detection)
                    preview = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                    preview.header = msg.header
                    self.preview_pub.publish(preview)
                self.publish_status()
                return
            image_msg, scan, reference, delta = ready
            signature = (scan.header.stamp.sec, scan.header.stamp.nanosec)
            if signature == self.last_processed:
                self.publish_status()
                return
            self.last_processed = signature
            image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8').copy()
            detection, camera_candidates = self.camera_tracker.detect(image, self.camera, self.settings)
            camera_count = len(camera_candidates)
            points, candidates = detect_candidates(scan, self.settings)
            selected = self.select_lidar(candidates)
            observation = None
            if detection is not None and selected is not None:
                observation = {
                    'xyz': selected['center'], 'uv': detection['uv'], 'camera': detection,
                    'lidar': selected, 'image_stamp': stamp_seconds(image_msg.header.stamp),
                    'scan_stamp': reference, 'sync_delta': delta,
                }
            self.collector.observe(observation)
            if self.collector.auto:
                try:
                    sample = self.collector.capture()
                    self.invalidate_result(sample['role'])
                    self.message = f"自動取得 {sample['id']} / {self.settings['target_samples']}"
                    if len(self.collector.samples) >= self.settings['target_samples']:
                        self.collector.auto = False
                except ValueError:
                    pass
            display = cv2.cvtColor(make_mask(image, self.settings), cv2.COLOR_GRAY2BGR) if self.show_mask else image
            preview, _ = overlay(display, points, candidates, detection, self.result, self.camera, self.settings)
            annotate_candidates(preview, camera_candidates, detection)
            msg = self.bridge.cv2_to_imgmsg(preview, 'bgr8')
            msg.header = image_msg.header
            self.preview_pub.publish(msg)
            self.latest_status = {
                'sensor_status': '同期済み', 'points': points[::max(1, len(points) // 720)].tolist(),
                'candidates': candidates, 'selected': selected,
                'camera_detected': detection is not None, 'camera_count': camera_count, 'camera_candidates': camera_candidates,
                'lidar_detected': selected is not None, 'lidar_status': f'受信中（{len(points)}点）',
                'sync_delta': delta,
                'detection': detection,
            }
            self.publish_status()
        except (ValueError, TypeError, cv2.error) as error:
            self.collector.observe(None)
            self.message = str(error)
            self.latest_status = {'sensor_status': str(error), 'camera_detected': False, 'points': [], 'candidates': []}
            self.publish_status()

    def publish_status(self):
        rows = [{'id': sample['id'], 'role': sample['role'], 'xyz': sample['xyz'], 'uv': sample['uv']}
                for sample in self.collector.samples]
        stability = self.collector.stability()
        if self.latest_status.get('sensor_status') != '同期済み':
            stability['reason'] = self.latest_status.get('sensor_status', 'センサ待ち')
        elif not self.latest_status.get('lidar_detected'):
            stability['reason'] = ('LiDARの球候補がありません' if not self.latest_status.get('candidates')
                                   else 'LiDARの球候補をクリックして選択してください')
        elif not self.latest_status.get('camera_detected'):
            stability['reason'] = 'カメラの球認識待ち'
        state = self.latest_status | {
            'message': self.message, 'settings': self.settings, 'samples': rows,
            'result': self.result, 'stationary': self.collector.stationary(), 'stability': stability,
            'auto': self.collector.auto, 'tf_process_running': self.tf_active(), 'tf_status': self.tf_status,
        }
        self.status_pub.publish(String(data=json.dumps(state, ensure_ascii=False, allow_nan=False)))

    def stop_tf(self):
        if self.tf_active():
            try:
                self.tf_process.terminate()
            except ProcessLookupError:
                self.tf_process = None
                return
            try:
                self.tf_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.tf_process.kill()
                self.tf_process.wait()
        self.tf_process = None
        self.tf_status = 'TF配信は停止しています。'

    def destroy_node(self):
        self.stop_tf()
        return super().destroy_node()


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
        node = CalibrationNode()
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
