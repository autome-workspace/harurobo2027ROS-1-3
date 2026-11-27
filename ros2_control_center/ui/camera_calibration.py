import base64
import json
from pathlib import Path

from PySide6.QtCore import QProcess, QTimer, Qt
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QDialog, QDoubleSpinBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QSpinBox, QVBoxLayout,
)
import yaml

from core.config_manager import APP, USER_CONFIG


class CameraCalibrationDialog(QDialog):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setWindowTitle('カメラ較正')
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.resize(700, 640)
        self.result = None
        self.buffer = b''
        self.closing = False
        layout = QVBoxLayout(self)
        instructions = QLabel(
            'チェッカーボードを用意し、内側の交点数と1マスの実測寸法を入力してください。\n'
            'ボード全体を映し、画面の中央・四隅、距離、上下左右の傾きを変えて12枚以上採取します。\n'
            '較正中はカメラと機体の取り付けを固定してください。')
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        form = QFormLayout()
        self.columns = QSpinBox(minimum=3, maximum=20, value=9)
        self.rows = QSpinBox(minimum=3, maximum=20, value=6)
        self.square = QDoubleSpinBox(minimum=1, maximum=200, value=25, suffix=' mm')
        form.addRow('横の内側交点数', self.columns)
        form.addRow('縦の内側交点数', self.rows)
        form.addRow('1マスの実測寸法', self.square)
        layout.addLayout(form)
        self.preview = QLabel('「映像を開始」で接続中のカメラを使用します。')
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(480, 280)
        layout.addWidget(self.preview, 1)
        self.status = QLabel('入力値は例です。使用するボードに合わせて変更してください。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.start = QPushButton('映像を開始')
        self.capture = QPushButton('画像を採取')
        self.solve = QPushButton('較正を計算')
        self.save = QPushButton('保存して次回カメラ起動に適用')
        self.stop = QPushButton('停止・採取をリセット')
        tools = QHBoxLayout()
        for button in (self.start, self.capture, self.solve, self.stop):
            tools.addWidget(button)
        layout.addLayout(tools)
        layout.addWidget(self.save)
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.read)
        self.process.readyReadStandardError.connect(self.read_error)
        self.process.finished.connect(self.worker_finished)
        self.process.errorOccurred.connect(self.failed)
        self.start.clicked.connect(self.begin)
        self.capture.clicked.connect(lambda: self.command('capture'))
        self.solve.clicked.connect(lambda: self.command('solve'))
        self.stop.clicked.connect(self.process.terminate)
        self.save.clicked.connect(self.save_result)
        self.reset()

    def reset(self):
        self.start.setEnabled(True)
        for control in (self.capture, self.solve, self.save, self.stop):
            control.setEnabled(False)
        for control in (self.columns, self.rows, self.square):
            control.setEnabled(True)

    def begin(self):
        if self.controller.mode != 'real' or self.controller.closing:
            return
        self.result = None
        self.buffer = b''
        self.save.setEnabled(False)
        if 'camera' not in self.controller.processes.jobs and not self.controller.external('camera'):
            self.controller.start(['camera'])
        try:
            argv = self.controller.config.wrapped([
                '/usr/bin/python3', str(APP / 'core/camera_calibration.py'),
                str(self.columns.value()), str(self.rows.value()), str(self.square.value()),
            ])
        except (OSError, ValueError) as error:
            self.status.setText(str(error))
            return
        from PySide6.QtCore import QProcessEnvironment
        environment = QProcessEnvironment()
        for key, value in self.controller.config.environment('real').items():
            environment.insert(key, value)
        self.process.setProcessEnvironment(environment)
        self.process.start(argv[0], argv[1:])
        self.start.setEnabled(False)
        self.stop.setEnabled(True)
        for control in (self.columns, self.rows, self.square):
            control.setEnabled(False)
        self.status.setText('カメラ映像を待っています。')

    def command(self, action):
        if self.process.state() == QProcess.ProcessState.Running:
            self.process.write((json.dumps({'action': action}) + '\n').encode())
        if action == 'capture':
            self.result = None
            self.save.setEnabled(False)
        if action == 'solve':
            self.status.setText('較正を計算しています。')

    def read(self):
        self.buffer += bytes(self.process.readAllStandardOutput())
        lines = self.buffer.split(b'\n')
        self.buffer = lines.pop()
        for line in lines:
            try:
                message = json.loads(line)
                if 'image' in message:
                    pixmap = QPixmap()
                    pixmap.loadFromData(base64.b64decode(message['image']))
                    self.preview.setPixmap(pixmap)
                    self.capture.setEnabled(message['found'])
                if 'count' in message:
                    self.solve.setEnabled(message['count'] >= 12)
                if 'result' in message:
                    self.result = message['result']
                    self.save.setEnabled(True)
                if 'status' in message or 'error' in message:
                    self.status.setText(message.get('error', message.get('status')))
            except (ValueError, TypeError):
                self.status.setText('較正プロセスの出力を読み取れませんでした。')

    def read_error(self):
        text = bytes(self.process.readAllStandardError()).decode(errors='replace').strip()
        if text:
            self.status.setText(text[-600:])

    def failed(self, error):
        self.status.setText(self.process.errorString())
        if self.process.state() == QProcess.ProcessState.NotRunning:
            self.reset()

    def worker_finished(self, code, exit_status):
        self.reset()
        if self.closing:
            QTimer.singleShot(0, self.close)
        elif code == 0:
            self.status.setText('較正処理を停止しました。')
        else:
            self.status.setText('較正処理が終了しました。再開する場合は「映像を開始」を押してください。')

    def save_result(self):
        if self.result is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, 'カメラ較正を保存', str(USER_CONFIG.parent /
                f"camera_calibration_{self.result['image_width']}x{self.result['image_height']}.yaml"), 'YAML (*.yaml)')
        if not path:
            return
        try:
            target = Path(path)
            target.parent.mkdir(parents=True, exist_ok=True)
            # Qt asks for overwrite confirmation when selecting an existing file.
            target.write_text(yaml.safe_dump(self.result, sort_keys=False))
            profiles = dict(self.controller.config.variables.get('CAMERA_CALIBRATIONS', {}))
            profiles[f"{self.result['image_width']}x{self.result['image_height']}"] = str(target.resolve())
            self.controller.config.save_variables({
                'CAMERA_CALIBRATION': str(target.resolve()), 'CAMERA_CALIBRATIONS': profiles,
            })
            self.status.setText('保存しました。カメラを停止して再起動すると反映されます。'
                                ' カメラとLiDARの位置関係は別途設定が必要です。')
        except (OSError, ValueError, yaml.YAMLError) as error:
            QMessageBox.warning(self, '保存エラー', str(error))

    def closeEvent(self, event):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.closing = True
            self.process.terminate()
            event.ignore()
            return
        super().closeEvent(event)
