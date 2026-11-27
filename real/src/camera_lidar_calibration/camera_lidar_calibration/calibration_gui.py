import importlib.util
import os
from pathlib import Path
import sys


def ensure_qt(module=None):
    if importlib.util.find_spec('PySide6') is not None:
        return
    directory = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
    interpreter = Path(os.environ.get('CAMERA_LIDAR_GUI_PYTHON',
                                      str(directory / 'ros2-control-center/venv/bin/python')))
    if interpreter.is_file() and Path(sys.executable) != interpreter:
        os.execv(str(interpreter), [str(interpreter), '-m', module or __package__ + '.calibration_gui', *sys.argv[1:]])
    raise RuntimeError('PySide6がありません。既存Control Centerのinstall.shでユーザー用仮想環境を準備してください。')


def main():
    ensure_qt()
    from .gui import run
    return run()


if __name__ == '__main__':
    raise SystemExit(main())
