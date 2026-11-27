import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QListWidget, QMainWindow,
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
        self.can_enable = QPushButton('CANを有効化')
        self.can_enable.setToolTip('管理者認証後、can0を1 Mbps・ONE-SHOTで有効化します。')
        self.can_enable.clicked.connect(controller.activate_can)
        can_tools.addWidget(self.can_enable)
        dashboard.addLayout(can_tools)
        self.calibration_button = QPushButton('カメラ較正')
        self.calibration_button.clicked.connect(self.open_calibration)
        can_tools.addWidget(self.calibration_button)
        self.approach_settings_button = QPushButton('自動接近設定')
        self.approach_settings_button.clicked.connect(self.open_approach_settings)
        can_tools.addWidget(self.approach_settings_button)
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

    def open_camera_settings(self):
        from ui.camera_settings import CameraSettingsDialog
        CameraSettingsDialog(self.controller.config, self).exec()

    def open_approach_settings(self):
        from ui.approach_settings import ApproachSettingsDialog
        dialog = ApproachSettingsDialog(self.controller, self)
        dialog.exec()

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
            self, '起動の確認', row.get('confirmation', 'ボール自動接近の非駆動プレビューを起動しますか？')
        ) != QMessageBox.StandardButton.Yes:
            return
        self.controller.start([key], explicit=True)

    def record(self, key, level, line):
        self.logs.record(key, level, line)
        if line:
            self.latest = [*self.latest[-3:], self.logs.records[-1][2][:220]]

    def refresh(self):
        c = self.controller
        mode = c.mode
        self.controller_help.setVisible(mode in ('real', 'simulation'))
        self.controller_help.setText(
            ('PS4 操縦（Gazebo）を起動してください。' if mode == 'simulation'
             else 'PS4 コントローラーを起動し、受信を確認してから足回り駆動を起動してください。') +
            '\n左スティックで前後・左右、右スティック横で旋回。'
            '中立でL1を離してから押し直し、L1を押している間だけ操作できます。'
            '\nL1を離すと停止します。入力途絶後も中立・L1の押し直しが必要です。')
        self.mode.setEnabled(not c.busy() and not c.closing and
                             not (self.calibration_dialog and self.calibration_dialog.isVisible()))
        self.approach_settings_button.setVisible(mode == 'real')
        self.approach_settings_button.setEnabled(
            not c.closing and not c.pending and not c.shutdown and not c.can.running()
            and not any(key in c.processes.jobs for key in ('motor', 'approach', 'auto_drive')))
        self.approach_status.setVisible(mode == 'real')
        if 'approach' in c.processes.jobs and c.monitor_available():
            state = c.approach.get('state', '状態待ち')
            code, _, reason = state.partition(' ')
            names = {'MANUAL': '△で接近開始', 'DETECTING': '対象照合中',
                     'CAMERA_ALIGN': 'カメラ位置合わせ中', 'ALIGNING': '横位置合わせ中',
                     'APPROACHING': '接近中', 'COMPLETE': '到達', 'ABORT': '中止'}
            estimate = c.approach.get('estimate', {})
            position = (f"表面距離 {estimate['distance'] * 1000:.0f} mm / "
                        f"横ずれ {estimate['lateral'] * 1000:.0f} mm"
                        if estimate.get('valid') else 'ボールの位置未確認')
            drive = '実機駆動' if 'auto_drive' in c.processes.jobs else '非駆動確認'
            self.approach_status.setText(
                f"{drive} · {names.get(code, code)} {reason}\n{position}\n"
                'L1を保持して△で開始、○・スティック操作・L1解除で中止')
        else:
            self.approach_status.setText('自動接近設定を保存し、まず非駆動確認を起動してください。')
        self.calibration_button.setVisible(mode == 'real')
        self.calibration_button.setEnabled(
            not c.closing and not c.pending and not c.shutdown and not c.can.running()
            and c.monitor_available() and not any(key in c.processes.jobs for key in ('motor', 'auto_drive'))
            and not c.external('motor'))
        self.can_status.setVisible(mode == 'real')
        self.can_enable.setVisible(mode == 'real')
        self.can_status.setText(c.can.description())
        self.can_enable.setEnabled(
            mode == 'real' and c.can.can_activate() and not c.closing and not c.shutdown
            and c.monitor_available() and not c.external('motor')
            and 'motor' not in c.processes.jobs and 'motor' not in c.pending)
        startable = c.monitor_available() or mode == 'demo'
        batch = [key for key, row in c.config.programs.items()
                 if mode in row['modes'] and row.get('autostart')]
        ready = startable and not (c.pending or c.shutdown or c.closing or c.can.running())
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
                widgets[2].setEnabled(ready and not (owned or row.get('disabled') or c.external(key)))
                widgets[3].setEnabled((owned or key in c.pending) and key not in c.shutdown)
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
        if c.closing and not c.processes.jobs and c.can.idle():
            self.close()

    def closeEvent(self, event):
        if self.calibration_dialog and self.calibration_dialog.isVisible():
            self.calibration_dialog.close()
            if self.calibration_dialog.isVisible():
                event.ignore()
                return
        self.controller.closing = True
        self.controller.can.close()
        if self.controller.processes.jobs or not self.controller.can.idle():
            self.controller.stop()
            event.ignore()
            return
        event.accept()
