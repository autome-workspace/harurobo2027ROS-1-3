import math
import os
from pathlib import Path

from PySide6.QtWidgets import (
    QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QLabel, QMessageBox, QVBoxLayout,
    QLineEdit, QPushButton, QFileDialog,
)
import yaml

from core.config_manager import ROOT
from core.approach_config import apply_calibration, load_valid, observed_minimum


class ApproachSettingsDialog(QDialog):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setWindowTitle('自動接近設定')
        self.config = controller.config
        local = yaml.safe_load(self.config.path.read_text()) if self.config.path.exists() else {}
        self.geometry = (local or {}).get('robot_geometry', {})
        layout = QVBoxLayout(self)
        help_text = QLabel(
            '機体中心基準の位置とカメラ較正を使用します。\n'
            'LiDARの向きはRVizで正面・左を確認してください。初期表示は以前の測定値です。\n'
            '検証済み最小距離は、実際のボールで安定した測距を確認した最短距離を入力します。\n'
            '0のままでは自動接近付きの足回り駆動は起動できません。手動のみなら自動接近をOFFにしてください。')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        form = QFormLayout()
        self.extrinsic = QLineEdit()
        self.extrinsic.setPlaceholderText('空欄なら従来の手入力位置を使用')
        folder = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'camera-lidar-calibration'
        if (folder / 'settings.yaml').is_file():
            recent = yaml.safe_load((folder / 'settings.yaml').read_text()) or {}
            self.extrinsic.setText(recent.get('extrinsic_file', ''))
        form.addRow('保存済み外部較正YAML', self.extrinsic)
        choose = QPushButton('外部較正ファイルを選ぶ')
        choose.clicked.connect(self.choose_extrinsic)
        form.addRow('', choose)
        self.calibration_info = QLabel('')
        self.calibration_info.setWordWrap(True)
        form.addRow('', self.calibration_info)
        self.target = QDoubleSpinBox(minimum=100, maximum=2000, value=350, suffix=' mm')
        self.minimum = QDoubleSpinBox(minimum=0, maximum=2000, value=0, suffix=' mm')
        old_yaw = self.geometry.get('lidar_orientation_rad')
        self.yaw = QDoubleSpinBox(minimum=-180, maximum=180, decimals=3,
                                 value=math.degrees(old_yaw[2]) if old_yaw else -179.882,
                                 suffix=' °')
        form.addRow('目標表面距離', self.target)
        form.addRow('検証済み最小表面距離', self.minimum)
        form.addRow('LiDARのヨー角（左向きが正）', self.yaw)
        layout.addLayout(form)
        current = self.config.variables.get('APPROACH_SETTINGS')
        if current and Path(current).is_file():
            settings = yaml.safe_load(Path(current).read_text())
            parameters = yaml.safe_load(Path(settings['parameters_file']).read_text())
            p = parameters['approach_controller']['ros__parameters']
            self.target.setValue(p['target_distance'] * 1000)
            self.minimum.setValue(p['validated_min_range'] * 1000)
            self.yaw.setValue(math.degrees(settings['geometry']['lidar_orientation_rad'][2]))
            if settings.get('extrinsic_file'):
                self.extrinsic.setText(settings['extrinsic_file'])
        self.extrinsic.editingFinished.connect(self.inspect_extrinsic)
        self.inspect_extrinsic()
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def choose_extrinsic(self):
        path, _ = QFileDialog.getOpenFileName(self, '外部較正結果', self.extrinsic.text(), 'YAML (*.yaml *.yml)')
        if path:
            self.extrinsic.setText(path)
            self.inspect_extrinsic()

    def inspect_extrinsic(self):
        path = self.extrinsic.text().strip()
        if not path:
            self.calibration_info.setText('従来の手入力位置を使用します。')
            return
        try:
            data, _ = load_valid(path)
            minimum = observed_minimum(data)
            self.calibration_info.setText(
                f"較正有効：検証RMS {data['result']['validation_rms']:.2f}px。"
                f"取得済み球表面距離の最小値 {minimum*1000:.0f}mm。\n"
                '較正済み接近は最大30mm/s。較正GUI・画像投影で認識を確認してから、足回りの自動接近をONにしてください。')
            if self.minimum.value() == 0 and minimum < self.target.value()/1000 - .01:
                # Round upward so the guard never permits an unobserved closer distance.
                self.minimum.setValue(math.ceil(minimum * 1000))
        except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
            self.calibration_info.setText('外部較正を利用できません：' + str(error))

    def save(self):
        try:
            if 'motor' in self.controller.processes.jobs:
                raise ValueError('自動接近と実機駆動を停止してから保存してください。')
            required = ('lidar_position_m', 'camera_position_m', 'camera_pitch_rad',
                        'ball_center_height_m', 'ball_diameter_m', 'body_length_m', 'body_width_m')
            if any(self.geometry.get(key) is None for key in required):
                raise ValueError('センサー位置、カメラ角度、車体寸法、ボール中心の高さが未設定です。')
            if not self.config.variables.get('CAMERA_CALIBRATION'):
                raise ValueError('先にカメラ較正を保存してください。')
            if self.minimum.value() > self.target.value() - 10:
                raise ValueError('目標距離の許容幅下端まで検証できる最小距離を指定してください。')
            geometry = self.geometry | {'lidar_orientation_rad': [0., 0., math.radians(self.yaw.value())]}
            parameters = yaml.safe_load((ROOT / 'real/src/ball_auto_approach/config/params.yaml').read_text())
            parameters['ball_localizer']['ros__parameters'].update(
                ball_center_z=geometry['ball_center_height_m'], ball_radius=geometry['ball_diameter_m'] / 2)
            parameters['approach_controller']['ros__parameters'].update(
                target_distance=self.target.value() / 1000,
                validated_min_range=self.minimum.value() / 1000,
                half_length=geometry['body_length_m'] / 2,
                half_width=geometry['body_width_m'] / 2,
                auto_enabled=True)
            extrinsic = self.extrinsic.text().strip()
            if extrinsic:
                data = apply_calibration(parameters, geometry, extrinsic)
                minimum = observed_minimum(data)
                if 0 < self.minimum.value()/1000 < minimum - 1e-9:
                    raise ValueError(f'保存較正で確認した最短距離は{math.ceil(minimum*1000)}mmです。'
                                     'それより近い距離は追加確認・再較正してください。')
            folder = self.config.path.parent
            folder.mkdir(parents=True, exist_ok=True)
            parameters_path = folder / 'approach_parameters.yaml'
            settings_path = folder / 'approach_settings.yaml'
            parameters_path.write_text(yaml.safe_dump(parameters, sort_keys=False))
            settings_path.write_text(yaml.safe_dump(
                {'geometry': geometry, 'parameters_file': str(parameters_path),
                 **({'extrinsic_file': str(Path(extrinsic).expanduser().resolve())} if extrinsic else {})}, sort_keys=False))
            self.config.save_variables({'APPROACH_SETTINGS': str(settings_path)})
            self.accept()
        except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
            QMessageBox.warning(self, '自動接近設定', str(error))
