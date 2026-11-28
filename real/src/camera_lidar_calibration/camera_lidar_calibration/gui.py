import json
from pathlib import Path
from queue import Empty, Full, Queue
import subprocess
import sys
import signal
import time

import cv2
from PySide6.QtCore import QThread, QTimer, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from .settings import defaults, user_directory, validate


class RosWorker(QThread):
    def __init__(self):
        super().__init__()
        self.commands = Queue(maxsize=64)
        self.latest_state = None
        self.latest_image = None
        self.latest_image_size = None
        self.received = 0.
        self.image_received = 0.
        self.error = ''

    def run(self):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        node = None
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('camera_lidar_calibration_gui')
            publisher = node.create_publisher(String, '/camera_lidar/calibration/command', 10)
            bridge = CvBridge()
            def state(msg):
                self.latest_state = json.loads(msg.data)
                self.received = time.monotonic()
            def image(msg):
                frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                scale = min(1., 800 / frame.shape[1])
                frame = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)))
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.latest_image_size = (msg.width, msg.height)
                    self.latest_image = encoded.tobytes()
                    self.image_received = time.monotonic()
            node.create_subscription(String, '/camera_lidar/calibration/status', state, 1)
            node.create_subscription(Image, '/camera_lidar/calibration/preview', image, qos_profile_sensor_data)
            while rclpy.ok() and not self.isInterruptionRequested():
                for _ in range(10):
                    try:
                        command = self.commands.get_nowait()
                    except Empty:
                        break
                    publisher.publish(String(data=json.dumps(command)))
                rclpy.spin_once(node, timeout_sec=.03)
        except Exception as error:
            self.error = str(error)
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()



class CameraView(QLabel):
    def __init__(self, clicked, selected):
        super().__init__('カメラ映像待ち')
        self.clicked = clicked
        self.selected = selected
        self.source_size = None
        self.picking = False

    def mousePressEvent(self, event):
        pixmap = self.pixmap()
        if self.source_size is None or pixmap is None or pixmap.isNull():
            return
        x = event.position().x() - (self.width() - pixmap.width()) / 2
        y = event.position().y() - (self.height() - pixmap.height()) / 2
        if 0 <= x < pixmap.width() and 0 <= y < pixmap.height():
            u = x * self.source_size[0] / pixmap.width()
            v = y * self.source_size[1] / pixmap.height()
            if self.picking:
                self.picking = False
                self.setCursor(Qt.CursorShape.ArrowCursor)
                self.clicked(u, v)
            else:
                self.selected(u, v)


class LidarView(QWidget):
    def __init__(self, command):
        super().__init__()
        self.command = command
        self.state = {}
        self.setMinimumSize(320, 260)
        self.extent = 2.

    def pixel(self, point):
        scale = min(self.width(), self.height()) / (2 * self.extent)
        return self.width() / 2 - point[1] * scale, self.height() / 2 - point[0] * scale

    def paintEvent(self, event):
        from PySide6.QtCore import QPointF
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#0b141f'))
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(QColor('#34465b'), 1))
        center = self.pixel([0, 0])
        painter.drawLine(0, round(center[1]), self.width(), round(center[1]))
        painter.drawLine(round(center[0]), 0, round(center[0]), self.height())
        painter.setPen(QColor('#dce6f5'))
        painter.drawText(round(center[0]) + 5, round(center[1]) + 16, 'LiDAR / X↑ Y←')
        painter.setPen(QPen(QColor('#8a9fb4'), 2))
        for point in self.state.get('points', []):
            painter.drawPoint(QPointF(*self.pixel(point)))
        scale = min(self.width(), self.height()) / (2 * self.extent)
        for candidate in self.state.get('candidates', []):
            painter.setPen(QPen(QColor('#f5c86b'), 2))
            xy = self.pixel(candidate['center'])
            radius = candidate['radius'] * scale
            painter.drawEllipse(QPointF(*xy), radius, radius)
            for point in candidate['points']:
                painter.drawPoint(QPointF(*self.pixel(point)))
            painter.drawText(round(xy[0]) + 6, round(xy[1]) - 6,
                             f"{candidate['scan_surface_distance']:.3f} m")
        for sample in self.state.get('samples', []):
            color = '#68dba9' if sample['role'] == 'train' else '#f2a4e1'
            painter.setPen(QPen(QColor(color), 2))
            xy = self.pixel(sample['xyz'])
            painter.drawEllipse(QPointF(*xy), 4, 4)
        selected = self.state.get('selected')
        if selected:
            painter.setPen(QPen(QColor('#42dbff'), 3))
            painter.drawEllipse(QPointF(*self.pixel(selected['center'])), 8, 8)

    def mousePressEvent(self, event):
        candidates = self.state.get('candidates', [])
        if not candidates:
            return
        def distance(candidate):
            x, y = self.pixel(candidate['center'])
            return (event.position().x() - x)**2 + (event.position().y() - y)**2
        closest = min(candidates, key=distance)
        if distance(closest) < 25**2:
            self.command('select', center=closest['center'])


LABELS = {
    'image_topic': '画像トピック', 'info_topic': 'CameraInfoトピック', 'scan_topic': 'LiDARトピック',
    'camera_frame': 'カメラ光学フレーム', 'lidar_frame': 'LiDARフレーム',
    'intrinsics_file': '内部較正YAML（空欄でCameraInfo）', 'image_rectified': '入力画像は歪み補正済み',
    'ball_diameter': 'ボール直径 m', 'center_height_offset': '球中心−走査面の高さ m',
    'hsv_lower': 'HSV下限 [H,S,V]', 'hsv_upper': 'HSV上限 [H,S,V]',
    'image_dominance_ratio': '優先候補の面積比（0で無効）',
    'min_area': '最小輪郭面積 px²', 'min_circularity': '最小円形度',
    'min_solidity': '最小充実度', 'center_mode': '画像中心 sphere / contour',
    'min_range': '最小認識距離 m', 'max_range': '最大認識距離 m',
    'lidar_sector_enabled': 'LiDAR角度フィルターを有効化',
    'lidar_sector_center_deg': 'LiDAR前方の角度 deg（LiDAR座標）',
    'lidar_sector_width_deg': 'LiDAR使用角度幅 deg',
    'cluster_gap': 'クラスタ間隔 m', 'max_missing_bins': '許容欠測ビン数',
    'min_points': 'クラスタ最小点数', 'max_fit_rms': '円フィット最大RMS m',
    'min_arc_span': '最小弧幅 / 断面半径', 'scan_stamp_is_end': 'LiDAR時刻は走査終端',
    'sync_tolerance': '時刻同期許容差 s', 'stale_timeout': 'データ鮮度上限 s',
    'stationary_duration': '静止判定時間 s', 'stationary_position': '静止位置許容差 m',
    'stationary_pixels': '静止画像許容差 px', 'sample_interval': '最小取得間隔 s',
    'duplicate_distance': '重複位置の距離 m', 'target_samples': '目標サンプル数',
    'max_samples': '保持サンプル上限', 'validation_every': '何個ごとに検証用を取得',
    'min_train_samples': '学習サンプル最小数', 'min_validation_samples': '検証サンプル最小数',
    'min_axis_span': '各軸の最小分布幅 m', 'min_hull_area': '最小分布面積 m²',
    'min_pixel_span': '画像上の最小分布幅 px', 'ransac_pixels': '外れ値除去閾値 px',
    'max_rms_pixels': '再投影RMS上限 px', 'max_error_pixels': '再投影誤差上限 px',
    'ambiguity_pixels': '複数解の識別誤差 px', 'max_translation': 'センサ間距離上限 m',
    'max_condition': '条件数上限', 'association_pixels': '球の投影照合許容差 px',
    'track_step': '追跡位置の最大変化 m', 'processing_hz': '処理周波数 Hz',
    'extrinsic_file': '外部較正ファイル（投影用）',
}


class SettingsDialog(QDialog):
    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.setWindowTitle('外部較正の設定')
        self.resize(650, 650)
        layout = QVBoxLayout(self)
        scroll = QScrollArea(widgetResizable=True)
        body = QWidget()
        form = QFormLayout(body)
        scroll.setWidget(body)
        layout.addWidget(scroll)
        self.inputs = {}
        self.settings = settings
        for key, value in settings.items():
            if type(value) is bool:
                widget = QCheckBox()
                widget.setChecked(value)
            elif type(value) in (float, int):
                widget = QDoubleSpinBox(minimum=-100000, maximum=1000000, decimals=6)
                widget.setValue(value)
            else:
                widget = QLineEdit(value if isinstance(value, str) else json.dumps(value))
            form.addRow(LABELS.get(key, key), widget)
            self.inputs[key] = widget
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.check)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def values(self):
        output = {}
        for key, widget in self.inputs.items():
            original = self.settings[key]
            if type(original) is bool:
                value = widget.isChecked()
            elif type(original) in (float, int):
                value = type(original)(widget.value())
            else:
                value = widget.text() if isinstance(original, str) else json.loads(widget.text())
            output[key] = value
        return validate(output)

    def check(self):
        try:
            self.values()
            self.accept()
        except (ValueError, TypeError) as error:
            QMessageBox.warning(self, '設定エラー', str(error))


class CalibrationWindow(QMainWindow):
    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self.state = {}
        self.sample_signature = None
        self.internal_process = None
        self.mask_enabled = False
        self.setWindowTitle('カメラ・LiDAR 外部較正')
        self.resize(1320, 900)
        self.setMinimumSize(850, 650)
        self.setStyleSheet("""
            QWidget { background:#111b28; color:#e2edf7; font-size:14px; }
            QPushButton,QLineEdit,QDoubleSpinBox { background:#24364b; padding:7px; border:1px solid #49617a; border-radius:5px; }
            QPushButton:disabled { color:#718399; }
            QTableWidget,QPlainTextEdit { background:#0b141f; }
        """)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        self.status = QLabel('較正ノードを待っています。')
        self.status.setWordWrap(True)
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.status)
        self.message = QLabel('')
        self.message.setWordWrap(True)
        self.message.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.message)
        self.quality = QLabel('較正 未実行')
        self.quality.setWordWrap(True)
        self.quality.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.quality)
        views = QSplitter()
        self.image = CameraView(self.pick_color, lambda u, v: self.send('select_camera', uv=[u, v]))
        self.image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image.setMinimumSize(320, 220)
        views.addWidget(self.image)
        self.lidar = LidarView(self.send)
        views.addWidget(self.lidar)
        layout.addWidget(views, 2)
        image_hint = QLabel('画像上のボールをクリックすると追跡対象を選べます。黄色は候補、緑は認識中です。')
        image_hint.setWordWrap(True)
        layout.addWidget(image_hint)
        self.validation = QCheckBox('次の手動取得は検証用（未選択時は5個ごと等、設定の割合で自動振分）')
        layout.addWidget(self.validation)
        presets = QHBoxLayout()
        self.five_button = QPushButton('5点モード（推定4＋検証1）')
        self.five_button.clicked.connect(lambda: self.send('sample_preset', preset='five'))
        presets.addWidget(self.five_button)
        standard_button = QPushButton('標準モード（推定12以上＋検証4以上）')
        standard_button.clicked.connect(lambda: self.send('sample_preset', preset='standard'))
        presets.addWidget(standard_button)
        layout.addLayout(presets)
        self.sample_guidance = QLabel('')
        self.sample_guidance.setWordWrap(True)
        layout.addWidget(self.sample_guidance)
        sector = QHBoxLayout()
        self.sector_values = None
        self.sector_enabled = QCheckBox('LiDAR角度制限')
        self.sector_center = QDoubleSpinBox(minimum=-180, maximum=180, decimals=2, suffix=' °')
        self.sector_width = QDoubleSpinBox(minimum=1, maximum=360, decimals=1, value=180, suffix=' °')
        sector.addWidget(self.sector_enabled)
        for label, control in (('前方の角度', self.sector_center), ('使用幅', self.sector_width)):
            sector.addWidget(QLabel(label))
            sector.addWidget(control)
        self.sector_button = QPushButton('角度範囲を適用')
        self.sector_button.clicked.connect(self.apply_sector)
        self.sector_button.setToolTip('LiDARのX軸が0°、Y軸が90°です。元の/scanと取得済みサンプルは保持します。')
        sector.addWidget(self.sector_button)
        layout.addLayout(sector)
        sparse = QHBoxLayout()
        self.min_points_value = None
        self.min_points_input = QDoubleSpinBox(minimum=3, maximum=1000, decimals=0, value=4)
        sparse.addWidget(QLabel('LiDAR候補の最低点数'))
        sparse.addWidget(self.min_points_input)
        self.min_points_button = QPushButton('最低点数を適用')
        self.min_points_button.clicked.connect(lambda: self.send(
            'lidar_min_points', values={'min_points': int(self.min_points_input.value())}))
        sparse.addWidget(self.min_points_button)
        note = QLabel('3点は疎な球向け。誤認識に注意し、画像と位置を確認してください。')
        note.setWordWrap(True)
        sparse.addWidget(note)
        layout.addLayout(sparse)
        tuning = QHBoxLayout()
        self.stability_values = None
        self.stability_duration = QDoubleSpinBox(minimum=.1, maximum=5, decimals=2, value=.4, suffix=' s')
        self.stability_position = QDoubleSpinBox(minimum=1, maximum=50, decimals=1, value=10, suffix=' mm')
        self.stability_pixels = QDoubleSpinBox(minimum=.5, maximum=30, decimals=1, value=4, suffix=' px')
        for name, control in (('静止時間', self.stability_duration),
                              ('LiDAR許容幅', self.stability_position), ('画像許容幅', self.stability_pixels)):
            tuning.addWidget(QLabel(name))
            tuning.addWidget(control)
        self.tune_button = QPushButton('静止条件を適用')
        self.tune_button.clicked.connect(self.tune_stationarity)
        self.tune_button.setToolTip('取得済みサンプルを保持して静止条件だけ変更します。')
        tuning.addWidget(self.tune_button)
        layout.addLayout(tuning)
        self.stability_status = QLabel('')
        self.stability_status.setWordWrap(True)
        layout.addWidget(self.stability_status)
        actions = [
            ('内部較正', self.internal), ('撮影範囲・解像度', self.capture_settings),
            ('ボールの色を選ぶ', self.start_color_pick), ('色マスク表示', self.toggle_mask),
            ('設定', self.configure), ('サンプル取得', self.capture),
            ('自動取得 開始/停止', self.automatic), ('LiDAR自動選択', lambda: self.send('select', center=None)),
            ('最後を削除', self.delete_last), ('選択を削除', self.delete_selected), ('全削除', self.clear),
            ('外部較正実行', lambda: self.send('solve')), ('独立点で検証', lambda: self.send('validate')),
            ('保存', self.save), ('読み込み', self.load), ('TFへ反映', self.apply_tf),
            ('TF配信停止', lambda: self.send('stop_tf')),
        ]
        grid = QGridLayout()
        self.buttons = {}
        for index, (name, callback) in enumerate(actions):
            button = QPushButton(name)
            button.clicked.connect(callback)
            self.buttons[name] = button
            grid.addWidget(button, index // 5, index % 5)
        layout.addLayout(grid)
        lower = QSplitter()
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(['ID', '用途', 'X m', 'Y m', 'Z m', 'u px', 'v px', '誤差 px'])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        lower.addWidget(self.table)
        self.result = QPlainTextEdit(readOnly=True)
        lower.addWidget(self.result)
        layout.addWidget(lower, 1)
        self.timer = QTimer(self, interval=100)
        self.timer.timeout.connect(self.update_state)
        self.timer.start()

    def send(self, action, **values):
        try:
            self.worker.commands.put_nowait({'action': action, **values})
        except Full:
            self.message.setText('操作の送信待ちです。少し待ってください。')

    def tune_stationarity(self):
        self.send('tune_stationarity', values={
            'stationary_duration': self.stability_duration.value(),
            'stationary_position': self.stability_position.value() / 1000,
            'stationary_pixels': self.stability_pixels.value(),
        })

    def apply_sector(self):
        self.send('lidar_sector', values={
            'lidar_sector_enabled': self.sector_enabled.isChecked(),
            'lidar_sector_center_deg': self.sector_center.value(),
            'lidar_sector_width_deg': self.sector_width.value(),
        })

    def capture(self):
        self.send('capture', role='validation' if self.validation.isChecked() else 'auto')

    def automatic(self):
        self.send('auto', enabled=not self.state.get('auto', False))

    def delete_last(self):
        samples = self.state.get('samples', [])
        if samples:
            self.send('delete', ids=[samples[-1]['id']])

    def delete_selected(self):
        rows = {index.row() for index in self.table.selectedIndexes()}
        self.send('delete', ids=[self.state['samples'][row]['id'] for row in rows])

    def clear(self):
        if QMessageBox.question(self, '全削除', '取得サンプルと較正結果を全削除しますか？') == QMessageBox.StandardButton.Yes:
            self.send('clear')

    def configure(self):
        dialog = SettingsDialog(self.state.get('settings', defaults()), self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            if self.state.get('samples') and QMessageBox.question(
                    self, '設定変更', '設定を変更して取得済みサンプルを全削除しますか？'
            ) != QMessageBox.StandardButton.Yes:
                return
            self.send('configure', settings=dialog.values(), clear_samples=True)

    def save(self):
        path, _ = QFileDialog.getSaveFileName(self, '結果と全サンプルを保存',
                                             str(user_directory() / 'extrinsic.yaml'), 'YAML (*.yaml)')
        if path:
            self.send('save', path=path)

    def load(self):
        path, _ = QFileDialog.getOpenFileName(self, '結果と全サンプルを読み込み', str(user_directory()), 'YAML (*.yaml)')
        if path:
            self.send('load', path=path)

    def apply_tf(self):
        if QMessageBox.question(self, 'TFへの反映',
                                '保存した有効な較正をTFへ反映しますか？ 既存の同じchildのTFがある場合は拒否します。'
        ) == QMessageBox.StandardButton.Yes:
            self.send('apply_tf')

    def internal(self):
        self.open_internal(False)

    def capture_settings(self):
        self.open_internal(True)

    def open_internal(self, capture):
        if self.internal_process is None or self.internal_process.poll() is not None:
            command = [sys.executable, '-m', 'camera_lidar_calibration.internal_gui']
            if capture:
                command.append('--capture-settings')
            self.internal_process = subprocess.Popen(command)

    def start_color_pick(self):
        if self.image.picking:
            self.image.picking = False
            self.image.setCursor(Qt.CursorShape.ArrowCursor)
            return
        if self.state.get('samples') and QMessageBox.question(
                self, '色の変更', '認識条件を変更すると、取得済みサンプルと結果を全削除します。先に保存しましたか？'
        ) != QMessageBox.StandardButton.Yes:
            return
        self.send('auto', enabled=False)
        self.image.picking = True
        self.image.setCursor(Qt.CursorShape.CrossCursor)

    def pick_color(self, u, v):
        self.send('pick_color', uv=[u, v], clear_samples=True)

    def toggle_mask(self):
        self.mask_enabled = not self.mask_enabled
        self.buttons['色マスク表示'].setText('元画像に戻す' if self.mask_enabled else '色マスク表示')
        self.send('show_mask', enabled=self.mask_enabled)

    def update_state(self):
        connected = time.monotonic() - self.worker.received < 2 and self.worker.latest_state is not None
        for name, button in self.buttons.items():
            button.setEnabled(connected or name in ('内部較正', '撮影範囲・解像度'))
        self.buttons['TFへ反映'].setEnabled(connected and bool((self.state.get('result') or {}).get('valid')))
        self.buttons['サンプル取得'].setEnabled(connected and self.state.get('stationary', False))
        self.tune_button.setEnabled(connected and not self.state.get('tf_process_running', False))
        self.sector_button.setEnabled(connected and not self.state.get('tf_process_running', False))
        self.min_points_button.setEnabled(connected and not self.state.get('tf_process_running', False))
        state = self.worker.latest_state
        if state is not None:
            self.state = state
            if self.min_points_value != state['settings']['min_points']:
                self.min_points_input.setValue(state['settings']['min_points'])
                self.min_points_value = state['settings']['min_points']
            current_sector = tuple(state['settings'][key] for key in
                                   ('lidar_sector_enabled', 'lidar_sector_center_deg', 'lidar_sector_width_deg'))
            if self.sector_values != current_sector:
                self.sector_enabled.setChecked(current_sector[0])
                self.sector_center.setValue(current_sector[1])
                self.sector_width.setValue(current_sector[2])
                self.sector_values = current_sector
            current_stability = tuple(state['settings'][key] for key in
                                      ('stationary_duration', 'stationary_position', 'stationary_pixels'))
            if self.stability_values != current_stability:
                self.stability_duration.setValue(state['settings']['stationary_duration'])
                self.stability_position.setValue(state['settings']['stationary_position'] * 1000)
                self.stability_pixels.setValue(state['settings']['stationary_pixels'])
                self.stability_values = current_stability
            stability = state.get('stability', {})
            position_spread = stability.get('position_spread')
            pixel_spread = stability.get('pixel_spread')
            widths = ''
            if position_spread is not None and pixel_spread is not None:
                widths = f" / 揺れ幅 LiDAR {position_spread * 1000:.1f} mm・画像 {pixel_spread:.1f} px"
            self.stability_status.setText(
                f"{stability.get('reason', '静止確認待ち')} / "
                f"{stability.get('count', 0)} 観測・{stability.get('duration', 0):.2f} s{widths}")
            delta = state.get('sync_delta')
            delta_text = f'{delta * 1000:.0f} ms' if delta is not None else '—'
            camera_status = '認識' if state.get('camera_detected') else '未認識'
            if not state.get('camera_detected') and state.get('camera_count', 0) > 1:
                camera_status = f"対象選択待ち（{state['camera_count']}候補）"
            if state.get('detection') and not state['detection'].get('geometry_valid', True):
                camera_status += '・内部較正待ち'
            self.status.setText(
                f"カメラ {camera_status} / "
                f"LiDAR {state.get('lidar_status', '受信待ち')}・"
                f"{'球認識' if state.get('lidar_detected') else str(len(state.get('candidates', []))) + '候補'} / "
                f"時刻差 {delta_text} / {'静止' if state.get('stationary') else '静止待ち'} / "
                f"{len(state.get('samples', []))} サンプル / "
                f"{'自動取得中' if state.get('auto') else '手動取得'}\n"
                f"{state.get('sensor_status', '')}")
            if state['settings']['lidar_sector_enabled']:
                self.status.setText(self.status.text() +
                    f" / LiDAR範囲 {state['settings']['lidar_sector_center_deg']:.1f}°中心・幅{state['settings']['lidar_sector_width_deg']:.1f}°")
            message = ('映像内のボールの色が付いている部分をクリックしてください。'
                       if self.image.picking else state.get('message', ''))
            self.message.setText(message + '\n' + state.get('tf_status', ''))
            self.lidar.state = state
            self.lidar.extent = state['settings']['max_range']
            self.lidar.update()
            samples = state.get('samples', [])
            settings = state['settings']
            train_count = sum(sample['role'] == 'train' for sample in samples)
            validation_count = len(samples) - train_count
            self.sample_guidance.setText(
                f"推定用 {train_count}/{settings['min_train_samples']} 点以上・"
                f"検証用 {validation_count}/{settings['min_validation_samples']} 点以上。" +
                ('少数点モード：手前左右・奥左右を推定用、別位置を検証用に。誤認識への余裕が小さく、追加検証を推奨。'
                 if settings['min_train_samples'] < 12 or settings['min_validation_samples'] < 4 else ''))
            result_errors = {entry['id']: entry['error_pixels'] for entry in
                             (state.get('result') or {}).get('sample_errors', [])}
            signature = [(sample['id'], sample['role'], result_errors.get(sample['id'])) for sample in samples]
            if signature != self.sample_signature:
                self.sample_signature = signature
                self.table.setRowCount(len(samples))
                for row, sample in enumerate(samples):
                    values = [sample['id'], sample['role'], *sample['xyz'], *sample['uv'], result_errors.get(sample['id'], '—')]
                    for column, value in enumerate(values):
                        self.table.setItem(row, column, QTableWidgetItem(
                            f'{value:.4f}' if isinstance(value, float) else str(value)))
            result = state.get('result')
            if result is None:
                self.quality.setText('較正 未実行')
                self.quality.setStyleSheet('color:#a5b8cc')
            elif result['valid']:
                self.quality.setText(
                    f"較正 有効 · 学習RMS {result['train_rms']:.2f} px · "
                    f"検証RMS {result['validation_rms']:.2f} px · "
                    f"採用 {result['used_samples']} / 除外 {result['excluded_samples']} / "
                    f"検証 {result['validation_samples']} 点")
                self.quality.setStyleSheet('color:#68dba9')
            else:
                self.quality.setText('較正 無効 · ' + ' / '.join(result.get('warnings', [])))
                self.quality.setStyleSheet('color:#f5c86b')
            text = json.dumps(result, ensure_ascii=False, indent=2) if result else '較正結果待ち'
            if self.result.toPlainText() != text:
                self.result.setPlainText(text)
        if time.monotonic() - self.worker.received > 2:
            self.status.setText('較正ノードとの接続待ち ' + self.worker.error)
            self.lidar.state = {'samples': self.state.get('samples', [])}
            self.lidar.update()
        image = self.worker.latest_image
        if image is not None and time.monotonic() - self.worker.image_received < 1.5:
            self.image.source_size = getattr(self.worker, 'latest_image_size', None)
            pixmap = QPixmap()
            pixmap.loadFromData(image)
            self.image.setPixmap(pixmap.scaled(self.image.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                               Qt.TransformationMode.SmoothTransformation))
        else:
            self.image.clear()
            self.image.setText('カメラ映像待ち・センサ接続を確認してください。')

    def closeEvent(self, event):
        self.worker.requestInterruption()
        if self.worker.isRunning():
            QTimer.singleShot(100, self.close)
            event.ignore()
            return
        if self.internal_process is not None and self.internal_process.poll() is None:
            self.internal_process.terminate()
        event.accept()


def run():
    app = QApplication([sys.argv[0]])
    worker = RosWorker()
    window = CalibrationWindow(worker)
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, lambda signum, frame: window.close())
    worker.start()
    window.show()
    return app.exec()
