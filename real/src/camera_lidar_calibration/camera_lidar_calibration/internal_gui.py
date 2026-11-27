"""Reuse the Control Center's existing 9x6 / 25 mm intrinsic calibration GUI."""
import os
from pathlib import Path
import sys
import signal

from .calibration_gui import ensure_qt


def main():
    ensure_qt(__package__ + '.internal_gui')
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication, QMessageBox
    candidates = [parent / 'ros2_control_center' for parent in Path(__file__).resolve().parents]
    detected = next((path for path in candidates if (path / 'main.py').is_file()), Path('ros2_control_center'))
    directory = Path(os.environ.get('CONTROL_CENTER_DIR', str(detected)))
    if not (directory / 'core/camera_calibration.py').is_file():
        app = QApplication([])
        QMessageBox.warning(None, '内部較正',
                            '既存Control Centerが見つかりません。CONTROL_CENTER_DIRに設置先を指定してください。')
        return 1
    sys.path.insert(0, str(directory))
    from core.config_manager import Config
    from core.controller import Controller
    from ui.camera_calibration import CameraCalibrationDialog
    app = QApplication([sys.argv[0]])
    if '--capture-settings' in sys.argv:
        from ui.camera_settings import CameraSettingsDialog
        CameraSettingsDialog(Config()).exec()
        return 0
    app.setQuitOnLastWindowClosed(False)
    controller = Controller(Config())
    controller.mode = 'real'
    dialog = CameraCalibrationDialog(controller)
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, lambda signum, frame: dialog.close())
    dialog.show()
    def cleanup():
        if not dialog.isVisible():
            controller.closing = True
            controller.stop()
            controller.can.close()
            if not controller.processes.jobs and controller.can.idle():
                app.quit()
    timer = QTimer(interval=100)
    timer.timeout.connect(cleanup)
    timer.start()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
