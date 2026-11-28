import argparse
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
    parser = argparse.ArgumentParser(description='ROS 2 Control Center')
    parser.add_argument('--encoders', action='store_true', help='実機モードでエンコーダーの読取を開始')
    parser.add_argument('--encoder-rviz', action='store_true', help='エンコーダーの読取とRViz表示を開始')
    options, qt_args = parser.parse_known_args()
    app = QApplication([sys.argv[0], *qt_args])
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
        if options.encoders or options.encoder_rviz:
            window.mode.setCurrentIndex(window.mode.findData('real'))
            controller.encoder_read_requested = True
            controller.encoder_view_requested = options.encoder_rviz
        window.show()
        return app.exec()


if __name__ == '__main__':
    sys.exit(main())
