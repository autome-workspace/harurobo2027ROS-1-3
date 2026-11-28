import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox, QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QListWidget, QMainWindow,
    QMessageBox, QPushButton, QScrollArea, QStackedWidget, QVBoxLayout, QWidget,
)

from core.config_manager import MODES, USER_CONFIG
from core.device_monitor import devices, inventory
from ui.log_viewer import LogView

COLORS = {'ok': '#70dbac', 'muted': '#9baec3', 'warning': '#f4c576', 'error': '#ff8c91'}


def label(text, name=None):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    if name:
        widget.setObjectName(name)
    return widget


class MainWindow(QMainWindow):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle('ROS 2 Control Center')
        self.resize(1170, 780)
        self.setMinimumSize(800, 540)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        sidebar = QFrame(objectName='sidebar')
        sidebar.setFixedWidth(172)
        layout.addWidget(sidebar)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(16, 22, 16, 16)
        side.addWidget(label('ROS 2\nControl Center', 'brand'))
        self.navigation = QListWidget()
        self.navigation.addItems(['ダッシュボード', 'プログラム管理', 'トピック監視',
                                  'デバイス管理', 'パラメータ設定', 'ログ', '設定'])
        side.addWidget(self.navigation)
        self.pages = QStackedWidget()
        layout.addWidget(self.pages, 1)
        self.navigation.currentRowChanged.connect(self.pages.setCurrentIndex)

        dashboard = self.page('ダッシュボード')
        toolbar = QHBoxLayout()
        dashboard.addLayout(toolbar)
        self.mode = QComboBox()
        for key, name in MODES.items():
            self.mode.addItem(name, key)
        self.mode.currentIndexChanged.connect(self.select_mode)
        toolbar.addWidget(self.mode)
        self.start_all = QPushButton('全システム起動', objectName='primary')
        self.start_all.clicked.connect(self.start_default)
        toolbar.addWidget(self.start_all)
        self.stop_all = QPushButton('全システム停止')
        self.stop_all.clicked.connect(lambda: controller.stop())
        toolbar.addWidget(self.stop_all)
        toolbar.addStretch()
        self.robot_stop = QPushButton('ロボット停止', objectName='stop')
        self.robot_stop.setToolTip('GUI管理下のプログラムを停止します。物理非常停止装置も用意してください。')
        self.robot_stop.clicked.connect(lambda: controller.stop())
        toolbar.addWidget(self.robot_stop)
        can_tools = QHBoxLayout()
        self.can_status = label('')
        self.can_status.setWordWrap(True)
        can_tools.addWidget(self.can_status, 1)
        self.can_enable = QPushButton('モーターCANを有効化')
        self.can_enable.setToolTip('管理者認証後、can0を1 Mbps・ONE-SHOTで有効化します。')
        self.can_enable.clicked.connect(controller.activate_can)
        can_tools.addWidget(self.can_enable)
        dashboard.addLayout(can_tools)
        encoder_can_tools = QHBoxLayout()
        self.encoder_can_status = label('')
        self.encoder_can_status.setWordWrap(True)
        encoder_can_tools.addWidget(self.encoder_can_status, 1)
        self.encoder_can_enable = QPushButton('エンコーダーCANを有効化')
        self.encoder_can_enable.clicked.connect(controller.activate_encoder_can)
        encoder_can_tools.addWidget(self.encoder_can_enable)
        self.encoder_read_start = QPushButton('エンコーダー読取開始')
        self.encoder_read_start.clicked.connect(self.toggle_encoder_reader)
        encoder_can_tools.addWidget(self.encoder_read_start)
        dashboard.addLayout(encoder_can_tools)
        self.calibration_button = QPushButton('カメラ較正')
        self.calibration_button.clicked.connect(self.open_calibration)
        can_tools.addWidget(self.calibration_button)
        self.approach_settings_button = QPushButton('自動接近設定')
        self.approach_settings_button.clicked.connect(self.open_approach_settings)
        can_tools.addWidget(self.approach_settings_button)
        self.distance_settings_button=QPushButton('指定距離移動設定')
        self.distance_settings_button.clicked.connect(self.open_distance_settings)
        can_tools.addWidget(self.distance_settings_button)
        self.calibration_dialog = None
        self.summary = label('')
        self.summary.setWordWrap(True)
        dashboard.addWidget(self.summary)
        self.controller_help = label('')
        self.controller_help.setWordWrap(True)
        dashboard.addWidget(self.controller_help)
        self.approach_status = label('')
        self.approach_status.setWordWrap(True)
        dashboard.addWidget(self.approach_status)
        self.encoder_status = label('')
        self.encoder_status.setWordWrap(True)
        self.encoder_status.setToolTip('角度は1回転内、相対回転数は受信開始位置が0です。規格位置は1回転65536で、内部の生カウントとは異なります。')
        dashboard.addWidget(self.encoder_status)
        self.motor_automatic = QCheckBox('足回り駆動で自動接近を有効にする（L1保持＋△）')
        self.motor_automatic.setToolTip('OFFでは手動のみ。ONでは足回り起動時にカメラ・LiDARと自動接近処理も起動します。停止中に切り替えてください。')
        self.motor_automatic.toggled.connect(self.change_motor_automatic)
        dashboard.addWidget(self.motor_automatic)
        self.dashboard_rows = self.program_grid(dashboard)
        dashboard.addWidget(label('リアルタイムログ', 'section'))
        self.recent = label('')
        self.recent.setWordWrap(True)
        dashboard.addWidget(self.recent)
        self.latest = []

        programs = self.page('プログラム管理')
        self.program_rows = self.program_grid(programs, details=True)
        topics = self.page('トピック監視')
        self.graph = label('')
        self.graph.setWordWrap(True)
        topics.addWidget(self.graph)
        topics.addStretch()
        device_page = self.page('デバイス管理')
        self.device_status = label('')
        self.device_status.setWordWrap(True)
        self.last_device_scan = 0.0
        self.detected = {}
        device_page.addWidget(self.device_status)
        self.device_encoder_status = label('')
        self.device_encoder_status.setWordWrap(True)
        device_page.addWidget(self.device_encoder_status)
        device_page.addStretch()
        params = self.page('パラメータ設定')
        params.addWidget(label('稼働中の ROS パラメータ編集は Phase 2 で対応します。'))
        params.addStretch()
        logs = self.page('ログ')
        self.logs = LogView(controller.config.programs)
        logs.addWidget(self.logs)
        settings = self.page('設定')
        local_settings = label(f'PC 固有の設定ファイル\n{USER_CONFIG}\n\n変更は次回起動時に反映します。')
        local_settings.setWordWrap(True)
        settings.addWidget(local_settings)
        camera_settings = QPushButton('カメラ撮影設定')
        camera_settings.clicked.connect(self.open_camera_settings)
        settings.addWidget(camera_settings)
        settings.addStretch()
        self.navigation.setCurrentRow(0)
        controller.refresh.connect(self.refresh)
        controller.encoder_changed.connect(self.update_encoder_display)
        controller.log.connect(self.record)
        self.refresh()

    def page(self, title):
        scroll = QScrollArea(widgetResizable=True)
        self.pages.addWidget(scroll)
        body = QWidget()
        scroll.setWidget(body)
        layout = QVBoxLayout(body)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)
        layout.addWidget(label(title, 'heading'))
        return layout

    def program_grid(self, parent, details=False):
        frame = QFrame(objectName='panel')
        parent.addWidget(frame)
        layout = QGridLayout(frame)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setColumnStretch(0, 1)
        rows = {}
        for index, (key, row) in enumerate(self.controller.config.programs.items()):
            name = label(row['name'])
            name.setWordWrap(True)
            name.setToolTip(row.get('disabled', ''))
            status = label('')
            start = QPushButton('起動')
            stop = QPushButton('停止')
            start.clicked.connect(lambda checked=False, item=key: self.start_program(item))
            stop.clicked.connect(lambda checked=False, item=key: self.controller.stop(item))
            widgets = [name, status, start, stop]
            for col, widget in enumerate(widgets):
                layout.addWidget(widget, 2 * index, col)
            if details:
                command = row.get('command', row.get('disabled', ''))
                description = label(' '.join(command) if isinstance(command, list) else command, 'command')
                description.setWordWrap(True)
                layout.addWidget(description, 2 * index + 1, 0, 1, 4)
                widgets.append(description)
            rows[key] = widgets
        return rows

    def toggle_encoder_reader(self):
        c = self.controller
        if 'encoders' in c.processes.jobs or 'encoders' in c.pending:
            c.stop('encoders')
        else:
            c.start(['encoders'])

    def open_camera_settings(self):
        from ui.camera_settings import CameraSettingsDialog
        CameraSettingsDialog(self.controller.config, self).exec()

    def open_distance_settings(self):
        from ui.encoder_distance_settings import EncoderDistanceSettings
        EncoderDistanceSettings(self.controller,self).exec()

    def open_approach_settings(self):
        from ui.approach_settings import ApproachSettingsDialog
        dialog = ApproachSettingsDialog(self.controller, self)
        dialog.exec()

    def change_motor_automatic(self, enabled):
        self.controller.set_motor_automatic(enabled)
        self.refresh()

    def open_calibration(self):
        from ui.camera_calibration import CameraCalibrationDialog
        if self.calibration_dialog is None or not self.calibration_dialog.isVisible():
            self.calibration_dialog = CameraCalibrationDialog(self.controller, self)
        self.calibration_dialog.show()
        self.calibration_dialog.raise_()

    def select_mode(self):
        mode = self.mode.currentData()
        if not self.controller.set_mode(mode):
            self.mode.setCurrentIndex(self.mode.findData(self.controller.mode))
        self.refresh()

    def start_default(self):
        mode = self.controller.mode
        self.controller.start([key for key, row in self.controller.config.programs.items()
                               if mode in row['modes'] and row.get('autostart')])

    def start_program(self, key):
        row = self.controller.config.programs[key]
        if row.get('explicit') and QMessageBox.question(
            self, '起動の確認', row.get('confirmation', 'プログラムを起動しますか？')
        ) != QMessageBox.StandardButton.Yes:
            return
        self.controller.start([key], explicit=True)

    def record(self, key, level, line):
        self.logs.record(key, level, line)
        if line:
            self.latest = [*self.latest[-3:], self.logs.records[-1][2][:220]]

    def update_encoder_display(self):
        c = self.controller
        mode = c.mode
        self.encoder_status.setVisible(mode == 'real')
        self.device_encoder_status.setVisible(mode == 'real')
        if c.encoder_available():
            data = c.encoders
            lines = [f"AMT102 エンコーダー（読み取り専用・{data.get('interface', '接続先待ち')}）"]
            if not data.get('can_connected'):
                lines.append('CANの有効化待ち：' + data.get('error', ''))
            for item in data.get('channels', []):
                if not item.get('fresh'):
                    lines.append(f"CH {item['channel']}：受信途絶（以前の値）")
                    continue
                lines.append(f"CH {item['channel']}：{item['angle_deg']:.4f}° / "
                             f"相対 {item['relative_turns']:+.6f}回転 / {item['rx_hz']:.0f} Hz / "
                             + item.get('fault_text', '異常履歴あり（詳細確認中）' if item['fault'] else '受信正常')
                             + (' / 時刻同期あり' if item['synced'] else ' / 時刻同期なし'))
            if not data.get('channels'):
                lines.append('位置データ待ち。2台は異なるchに設定してください。')
            text = '\n'.join(lines)
        else:
            text = ('エンコーダー表示データ受信待ち（読取プログラム稼働中）' if 'encoders' in c.processes.jobs
                    else 'エンコーダー読取が停止しています。「エンコーダー読取開始」を押してください。')
        self.encoder_status.setText(text)
        self.device_encoder_status.setText(text)

    def refresh(self):
        c = self.controller
        mode = c.mode
        self.controller_help.setVisible(mode in ('real', 'simulation'))
        self.update_encoder_display()
        self.controller_help.setText(
            ('PS4 操縦（Gazebo）を起動してください。' if mode == 'simulation'
             else 'PS4 コントローラーを起動し、受信を確認してから足回り駆動を起動してください。') +
            '\n左スティックで前後・左右、右スティック横で旋回。'
            '中立でL1を離してから押し直し、L1を押している間だけ操作できます。'
            '\nL1を離すと停止します。入力途絶後も中立・L1の押し直しが必要です。')
        self.mode.setEnabled(not c.busy() and not c.closing and
                             not (self.calibration_dialog and self.calibration_dialog.isVisible()))
        if mode=='real' and c.config.variables.get('MOTOR_ENCODER_DISTANCE'):
            self.controller_help.setText(self.controller_help.text()+'\nエンコーダー指定距離移動ON：L1保持＋十字キーで開始、○・L1解除で中止。')
        self.distance_settings_button.setVisible(mode == 'real')
        self.distance_settings_button.setEnabled('motor' not in c.processes.jobs and not c.pending and not c.shutdown and not c.closing)
        self.approach_settings_button.setVisible(mode == 'real')
        self.motor_automatic.setVisible(mode == 'real')
        self.motor_automatic.setEnabled(not c.closing and not c.pending and not c.shutdown
                                        and 'motor' not in c.processes.jobs and not c.external('motor'))
        self.motor_automatic.blockSignals(True)
        self.motor_automatic.setChecked(bool(c.config.variables.get('MOTOR_AUTO_APPROACH')))
        self.motor_automatic.blockSignals(False)
        self.approach_settings_button.setEnabled(
            not c.closing and not c.pending and not c.shutdown and not c.can.running()
            and 'motor' not in c.processes.jobs)
        self.approach_status.setVisible(mode == 'real')
        if 'motor' in c.processes.jobs and c.config.variables.get('MOTOR_AUTO_APPROACH') and c.monitor_available():
            state = c.approach.get('state', '状態待ち')
            code, _, reason = state.partition(' ')
            reason = {
                'swept footprint blocked': '機体周囲に障害物を検出',
                'camera ball lost or stale': 'カメラのボール認識が途絶',
                'LiDAR target lost or ambiguous': 'LiDARの対象を見失った、または候補が複数',
                'missing or stale LiDAR track': 'LiDARの追跡データが途絶',
                'outside verified approach range': '確認済み距離の範囲外',
                'camera and LiDAR acquisition failed': 'カメラとLiDARの位置照合に失敗',
            }.get(reason, reason)
            names = {'MANUAL': '△で接近開始', 'DETECTING': '対象照合中',
                     'CAMERA_ALIGN': 'カメラ位置合わせ中', 'ALIGNING': '横位置合わせ中',
                     'APPROACHING': '接近中', 'COMPLETE': '到達', 'ABORT': '中止'}
            estimate = c.approach.get('estimate', {})
            position = (f"表面距離 {estimate['distance'] * 1000:.0f} mm / "
                        f"横ずれ {estimate['lateral'] * 1000:.0f} mm"
                        if estimate.get('valid') else 'ボールの位置未確認')
            drive = '足回り駆動・自動接近'
            self.approach_status.setText(
                f"{drive} · {names.get(code, code)} {reason}\n{position}\n"
                'L1を保持して△で開始、○・スティック操作・L1解除で中止')
        else:
            self.approach_status.setText('自動接近は足回り駆動に統合しました。有効にする場合は設定を保存し、上の切り替えをONにしてください。')
        self.calibration_button.setVisible(mode == 'real')
        self.calibration_button.setEnabled(
            not c.closing and not c.pending and not c.shutdown and not c.can.running()
            and c.monitor_available() and 'motor' not in c.processes.jobs
            and not c.external('motor'))
        self.can_status.setVisible(mode == 'real')
        self.can_enable.setVisible(mode == 'real')
        self.can_status.setText('DM3519：' + c.can.description())
        self.encoder_can_status.setVisible(mode == 'real')
        self.encoder_can_enable.setVisible(mode == 'real')
        self.encoder_read_start.setVisible(mode == 'real')
        encoder_running = 'encoders' in c.processes.jobs or 'encoders' in c.pending
        encoder_job = c.processes.jobs.get('encoders')
        self.encoder_read_start.setText('エンコーダー読取停止' if encoder_running else 'エンコーダー読取開始')
        self.encoder_read_start.setEnabled(mode == 'real' and not c.closing and not c.shutdown
            and ((encoder_running and not (encoder_job and encoder_job.stopping))
                 or (not encoder_running and c.monitor_available() and not c.external('encoders') and not c.pending)))
        self.encoder_can_status.setText('AMT102：' + c.encoder_can.description())
        self.encoder_can_enable.setEnabled(mode == 'real' and c.encoder_can.can_activate()
                                          and not c.closing and not c.shutdown)
        self.can_enable.setEnabled(
            mode == 'real' and c.can.can_activate() and not c.closing and not c.shutdown
            and c.monitor_available() and not c.external('motor')
            and 'motor' not in c.processes.jobs and 'motor' not in c.pending)
        startable = c.monitor_available() or mode == 'demo'
        batch = [key for key, row in c.config.programs.items()
                 if mode in row['modes'] and row.get('autostart')]
        ready = startable and not (c.pending or c.shutdown or c.closing or c.can.running() or c.encoder_can.running())
        self.start_all.setEnabled(ready and any(key not in c.processes.jobs and not c.external(key) for key in batch))
        self.start_all.setToolTip('一括起動対象がありません。' if not batch else '')
        self.stop_all.setEnabled(c.busy())
        for rows in (self.dashboard_rows, self.program_rows):
            for key, widgets in rows.items():
                row = c.config.programs[key]
                for widget in widgets:
                    widget.setVisible(mode in row['modes'])
                status, color = c.status(key)
                widgets[1].setText(status)
                widgets[1].setStyleSheet(f'color: {COLORS[color]}')
                owned = key in c.processes.jobs
                can_ready = not row.get('requires_can') or c.can.drive_ready()
                widgets[2].setEnabled(ready and can_ready and not (owned or row.get('disabled') or c.external(key)))
                widgets[2].setToolTip('CANを有効化し、UPを確認してください。' if not can_ready else '')
                widgets[3].setEnabled((owned or key in c.pending) and key not in c.shutdown)
                if len(widgets) > 4:
                    command = row['command']
                    widgets[4].setText(' '.join(command) if isinstance(command, list) else command)
        if time.monotonic() - self.last_device_scan >= 2:
            self.last_device_scan = time.monotonic()
            try:
                self.detected = devices()
                details = inventory()
            except OSError:
                details = []
            self.device_status.setText('\n'.join(
                f'{name}   ' + ('未評価' if state is None else 'OS 検出' if state else '未検出')
                for name, state in self.detected.items()
            ) + '\n\n通信状態は各プログラムの配信状態とログを確認してください。' +
                '\n\n' + '\n'.join(details))
        found = '、'.join(name for name, state in self.detected.items() if state) or '検出なし'
        connection = 'ROS 監視中' if c.monitor_available() else 'ROS 監視待ち'
        latest_error = next((line for _, level, line in reversed(self.logs.records)
                             if level in ('WARN', 'ERROR')), '')
        self.summary.setText(f'{connection}  ·  Domain {c.config.domains[mode]}\nOS 検出  {found}' +
                             (f'\n{latest_error[:180]}' if latest_error else ''))
        self.graph.setText('直近の配信を確認したトピック\n' + ('\n'.join(sorted(c.live)) if c.monitor_available() and c.live else '—') +
                           '\n\n検出ノード\n' + ('\n'.join(sorted(c.nodes)) if c.monitor_available() and c.nodes else '—'))
        self.recent.setText('\n'.join(self.latest))
        self.logs.flush()
        if c.closing and not c.processes.jobs and c.can.idle() and c.encoder_can.idle():
            self.close()

    def closeEvent(self, event):
        if self.calibration_dialog and self.calibration_dialog.isVisible():
            self.calibration_dialog.close()
            if self.calibration_dialog.isVisible():
                event.ignore()
                return
        self.controller.closing = True
        self.controller.can.close()
        self.controller.encoder_can.close()
        if self.controller.processes.jobs or not self.controller.can.idle() or not self.controller.encoder_can.idle():
            self.controller.stop()
            event.ignore()
            return
        event.accept()
