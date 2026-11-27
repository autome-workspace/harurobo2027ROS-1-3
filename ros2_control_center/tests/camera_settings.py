"""Verify capture profiles without opening camera hardware."""
import os
from pathlib import Path
import sys
import tempfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication
import yaml

from core.config_manager import Config
from ui.camera_settings import CameraSettingsDialog

app = QApplication([])
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'local.yaml'
    calibration = Path(directory) / 'camera640.yaml'
    calibration.write_text('image_width: 640\nimage_height: 480\n')
    path.write_text(yaml.safe_dump({'variables': {'CAMERA_CALIBRATION': str(calibration)}}))
    config = Config(path)
    dialog = CameraSettingsDialog(config)
    dialog.mode.addItem('1280 × 720', '1280x720')
    dialog.mode.setCurrentIndex(dialog.mode.findData('1280x720'))
    assert not dialog.calibration.text()
    dialog.save()
    assert config.variables['CAMERA_WIDTH'] == 1280
    assert config.variables['CAMERA_CALIBRATIONS']['640x480'] == str(calibration)
    assert calibration.exists()
    second = Config(path)
    second.save_variables({'CAMERA_WIDTH': 640, 'CAMERA_HEIGHT': 480, 'CAMERA_PORT': '/dev/null'})
    argv = config.command('camera')
    assert 'width:=640' in argv and 'height:=480' in argv
    dialog.close()
    app.processEvents()
print('PASS: per-resolution calibration preservation and cross-process setting reload')
