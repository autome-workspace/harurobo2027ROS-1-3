from pathlib import Path
import re
import shutil

from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QHBoxLayout,
    QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout,
)
import yaml


class CameraSettingsDialog(QDialog):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        config.refresh_variables()
        self.setWindowTitle('カメラ撮影設定')
        self.resize(650, 300)
        self.profiles = dict(config.variables.get('CAMERA_CALIBRATIONS', {}))
        current_file = config.variables.get('CAMERA_CALIBRATION')
        if current_file and Path(current_file).is_file():
            metadata = yaml.safe_load(Path(current_file).read_text())
            if 'image_width' in metadata and 'image_height' in metadata:
                self.profiles[f"{metadata['image_width']}x{metadata['image_height']}"] = current_file
        layout = QVBoxLayout(self)
        help_text = QLabel('解像度・縦横比によって撮影範囲が変わります。保存後にカメラを停止・再起動してください。\n'
                           '解像度が変わった場合は、その解像度用の内部較正が必要です。以前の較正ファイルも保持します。')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        form = QFormLayout()
        self.mode = QComboBox()
        width = int(config.variables.get('CAMERA_WIDTH', 640))
        height = int(config.variables.get('CAMERA_HEIGHT', 480))
        self.mode.addItem(f'{width} × {height}', f'{width}x{height}')
        form.addRow('撮影解像度', self.mode)
        self.calibration = QLineEdit()
        row = QHBoxLayout()
        row.addWidget(self.calibration)
        browse = QPushButton('較正YAMLを選択')
        browse.clicked.connect(self.browse)
        row.addWidget(browse)
        form.addRow('対応する内部較正', row)
        layout.addLayout(form)
        self.status = QLabel('')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.mode_changed()
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.probe = QProcess(self)
        self.probe.finished.connect(self.formats)
        self.probe.errorOccurred.connect(lambda error: self.status.setText('対応解像度を取得できません。機器の接続を確認してください。'))
        program = shutil.which('v4l2-ctl')
        if program and config.variables.get('CAMERA_PORT'):
            self.probe.start(program, ['--device=' + config.variables['CAMERA_PORT'], '--list-formats-ext'])
            self.timeout = QTimer(self, interval=2000, singleShot=True)
            self.timeout.timeout.connect(self.probe.kill)
            self.probe.finished.connect(self.timeout.stop)
            self.timeout.start()

    def mode_changed(self):
        width, height = map(int, self.mode.currentData().split('x'))
        path = self.profiles.get(f'{width}x{height}', '')
        self.calibration.setText(path)
        self.status.setText('保存済みの内部較正を使用します。' if path else
                            'この解像度の内部較正は未設定です。映像確認後に内部較正を実行してください。')

    def formats(self, code, status):
        output = bytes(self.probe.readAllStandardOutput()).decode(errors='replace')
        current = tuple(map(int, self.mode.currentData().split('x')))
        modes = sorted({(int(width), int(height)) for width, height in
                        re.findall(r'Size: Discrete (\d+)x(\d+)', output)} | {current},
                       key=lambda size: size[0] * size[1])
        self.mode.blockSignals(True)
        self.mode.clear()
        for size in modes:
            self.mode.addItem(f'{size[0]} × {size[1]}', f'{size[0]}x{size[1]}')
        self.mode.setCurrentIndex(self.mode.findData(f'{current[0]}x{current[1]}'))
        self.mode.blockSignals(False)

    def browse(self):
        path, _ = QFileDialog.getOpenFileName(self, '内部較正YAML', '', 'YAML (*.yaml)')
        if path:
            self.calibration.setText(path)

    def save(self):
        try:
            width, height = map(int, self.mode.currentData().split('x'))
            path = self.calibration.text().strip()
            if path:
                metadata = yaml.safe_load(Path(path).expanduser().read_text())
                if (metadata['image_width'], metadata['image_height']) != (width, height):
                    raise ValueError('内部較正の画像サイズと撮影解像度が一致しません。')
                path = str(Path(path).expanduser().resolve())
                self.profiles[f'{width}x{height}'] = path
            self.config.save_variables({
                'CAMERA_WIDTH': width, 'CAMERA_HEIGHT': height,
                'CAMERA_CALIBRATION': path, 'CAMERA_CALIBRATIONS': self.profiles,
            })
            self.accept()
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            QMessageBox.warning(self, '保存エラー', str(error))

    def closeEvent(self, event):
        if self.probe.state() != QProcess.ProcessState.NotRunning:
            self.probe.kill()
        super().closeEvent(event)
