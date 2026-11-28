"""Exercise GUI persistence and mutually exclusive drive modes without ROS/CAN."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import yaml
from PySide6.QtWidgets import QApplication, QMessageBox
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.config_manager import Config
from core.encoder_distance import DEFAULTS
from ui.encoder_distance_settings import EncoderDistanceSettings

app=QApplication([])
warnings=[]
QMessageBox.warning=lambda *args:warnings.append(args[-1])
with tempfile.TemporaryDirectory() as directory:
    config=Config(Path(directory)/'local.yaml')
    c=SimpleNamespace(config=config,processes=SimpleNamespace(jobs={}),pending=[],shutdown=False)
    dialog=EncoderDistanceSettings(c)
    dialog.enabled.setChecked(True)
    dialog.save()
    assert warnings and not dialog.path.exists()
    dialog.signs['forward_sign'].setCurrentIndex(1)
    dialog.signs['left_sign'].setCurrentIndex(2)
    dialog.distance.setValue(150)
    dialog.save()
    assert yaml.safe_load(dialog.path.read_text())==DEFAULTS|{'distance_m':.15,'forward_sign':1,'left_sign':-1}
    assert config.variables['MOTOR_ENCODER_DISTANCE'] and not config.variables['MOTOR_AUTO_APPROACH']
    assert config.order(['motor'],'real')==['joy','encoders','imu','motor']
    assert '--encoder-distance' in config.command('motor')
    restored=EncoderDistanceSettings(c)
    assert restored.enabled.isChecked() and restored.distance.value()==150
    c.processes.jobs['motor']=object()
    restored.distance.setValue(200)
    restored.save()
    assert yaml.safe_load(dialog.path.read_text())['distance_m']==.15
print('PASS: sign confirmation, distance persistence, launch dependencies and running-settings lock; no motors')
