import fcntl
import os
from pathlib import Path
import signal
import sys

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication, QMessageBox
import yaml

from core.config_manager import APP, Config
from core.controller import Controller
from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName('ROS 2 Control Center')
    app.setDesktopFileName('ros2-control-center')
    app.setWindowIcon(QIcon(str(APP / 'resources/icon.svg')))
    app.setStyleSheet((APP / 'resources/dark.qss').read_text())
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'ros2-control-center'
    try:
        cache.mkdir(parents=True, exist_ok=True)
        instance_lock = (cache / 'instance.lock').open('a')
        fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        controller = Controller(Config())
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        message = 'すでに起動しています。' if isinstance(error, BlockingIOError) else str(error)
        QMessageBox.critical(None, '起動エラー', message)
        return 1
    with instance_lock:
        window = MainWindow(controller)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda signum, frame: window.close())
        window.show()
        return app.exec()


if __name__ == '__main__':
    sys.exit(main())
