import os
from queue import Queue
import time
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pytest
try:
    import PySide6
except ImportError:
    HAVE_QT = False
else:
    HAVE_QT = True
    from PySide6.QtWidgets import QApplication
    from PySide6.QtTest import QTest
    from PySide6.QtCore import Qt, QPoint
    from PySide6.QtGui import QPixmap
    from camera_lidar_calibration.gui import CalibrationWindow, SettingsDialog
from camera_lidar_calibration.settings import defaults


@pytest.mark.skipif(not HAVE_QT, reason='GUI test runs separately in the existing PySide6 virtual environment')
def test_gui_commands_settings_and_disconnect():
    app = QApplication.instance() or QApplication([])
    settings = defaults()
    state = {
        'settings': settings, 'message': '', 'sensor_status': '同期済み',
        'camera_detected': True, 'lidar_detected': True, 'stationary': True, 'auto': False,
        'sync_delta': .02,
        'samples': [{'id': 1, 'role': 'train', 'xyz': [.5,.1,0.], 'uv': [320.,240.]}],
        'candidates': [], 'points': [[.5,.1]], 'result': None,
    }
    worker = SimpleNamespace(commands=Queue(), latest_state=state, latest_image=None,
                             received=time.monotonic(), image_received=0., error='',
                             requestInterruption=lambda:None, isRunning=lambda:False)
    window = CalibrationWindow(worker)
    window.show()
    window.update_state()
    window.update_state()
    assert window.table.rowCount() == 1
    assert window.buttons['サンプル取得'].isEnabled()
    assert not window.buttons['TFへ反映'].isEnabled()
    window.stability_duration.setValue(.4)
    window.stability_position.setValue(10.)
    window.stability_pixels.setValue(4.)
    window.tune_stationarity()
    assert worker.commands.get_nowait() == {
        'action':'tune_stationarity',
        'values':{'stationary_duration':.4,'stationary_position':.01,'stationary_pixels':4.}}
    window.capture()
    assert worker.commands.get_nowait() == {'action':'capture','role':'auto'}
    window.validation.setChecked(True)
    window.capture()
    assert worker.commands.get_nowait()['role'] == 'validation'
    window.delete_last()
    assert worker.commands.get_nowait() == {'action':'delete','ids':[1]}
    window.image.source_size=(1280,720)
    window.image.setPixmap(QPixmap(320,180))
    window.image.picking=True
    QTest.mouseClick(window.image,Qt.MouseButton.LeftButton,
                     pos=QPoint(window.image.width()//2,window.image.height()//2))
    picked=worker.commands.get_nowait()
    assert picked['action']=='pick_color' and picked['clear_samples']
    assert abs(picked['uv'][0]-640)<5 and abs(picked['uv'][1]-360)<5
    QTest.mouseClick(window.image,Qt.MouseButton.LeftButton,
                     pos=QPoint(window.image.width()//2,window.image.height()//2))
    assert worker.commands.get_nowait()['action']=='select_camera'

    candidate = {'center':[.5,.1,0.], 'radius':.03, 'points':[[.48,.1]],
                 'scan_surface_distance':.47}
    state['candidates']=[candidate]
    window.update_state()
    app.processEvents()
    x,y=window.lidar.pixel(candidate['center'])
    QTest.mouseClick(window.lidar,Qt.MouseButton.LeftButton,pos=QPoint(round(x),round(y)))
    assert worker.commands.get_nowait() == {'action':'select','center':[.5,.1,0.]}
    dialog = SettingsDialog(settings)
    dialog.inputs['center_height_offset'].setValue(-.015)
    assert dialog.values()['center_height_offset'] == -.015
    assert window.grab().width() > 0
    worker.received = 0.
    window.update_state()
    assert not window.buttons['サンプル取得'].isEnabled()
    assert not window.lidar.state.get('points')
    window.close()
    app.processEvents()
