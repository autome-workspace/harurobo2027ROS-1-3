"""Hardware-free integration checks for the GUI, ROS graph and process ownership."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication

from core.config_manager import Config, ROOT
from core.controller import Controller
from ui.main_window import MainWindow


def wait(app, condition, description, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if condition():
            return
        time.sleep(0.025)
    raise AssertionError(description)


def main(local_path):
    app = QApplication([])
    config = Config(local_path)
    # Keep the test publisher separate from an open desktop application.
    config.domains['demo'] = next(domain for domain in range(98, 101)
                                  if domain not in config.domains.values())
    assert 'approach' not in config.programs and 'auto_drive' not in config.programs
    assert config.order(['motor'], 'real') == ['joy', 'motor']
    controller = Controller(config)
    window = MainWindow(controller)
    window.show()
    try:
        wait(app, controller.monitor_available, 'ROS graph did not initialize')
        window.start_all.click()
        wait(app, lambda: 'demo_echo' in controller.processes.jobs, 'dependency did not become ready')
        wait(app, lambda: any(key == 'demo_echo' and 'data: demo' in line
                             for key, _, line in window.logs.records), 'subscriber output missing')
        assert controller.status('demo_source')[1] == 'ok'
        wait(app, lambda: window.dashboard_rows['demo_echo'][1].text() == '受信確認済み',
             'dashboard did not confirm reception')
        sender = controller.processes.jobs['demo_source'].process
        sender.send_signal(signal.SIGSTOP)
        try:
            wait(app, lambda: window.dashboard_rows['demo_echo'][1].text() == '受信途絶',
                 'dashboard did not report interrupted reception')
        finally:
            sender.send_signal(signal.SIGCONT)
        wait(app, lambda: window.dashboard_rows['demo_echo'][1].text() == '受信確認済み',
             'dashboard did not confirm resumed reception')
        assert not controller.set_mode('real')
        window.stop_all.click()
        wait(app, lambda: not controller.busy(), 'stop did not drain the process groups')
        wait(app, lambda: '/control_center_demo' not in controller.live, 'stale topic')
        external = subprocess.Popen(config.command('demo_source'), cwd=ROOT,
                                    env=config.environment('demo'), start_new_session=True,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            wait(app, lambda: controller.external('demo_source'), 'external publisher missing')
            wait(app, lambda: not window.dashboard_rows['demo_source'][2].isEnabled(),
                 'external publisher did not disable Start')
            window.robot_stop.click()
            app.processEvents()
            assert external.poll() is None
        finally:
            external.terminate()
            external.wait(timeout=10)
        if '--gazebo' in sys.argv:
            config.programs['gazebo']['command'].append('headless:=true')
            window.mode.setCurrentIndex(window.mode.findData('simulation'))
            wait(app, controller.monitor_available, 'simulation graph did not initialize')
            wait(app, window.start_all.isEnabled, 'simulation start button did not enable')
            window.start_all.click()
            wait(app, lambda: controller.status('gazebo')[1] == 'ok',
                 'Gazebo did not produce clock, odometry and scan', timeout=45)
            window.start_program('joy_sim')
            wait(app, lambda: 'joy_sim' in controller.processes.jobs,
                 'simulation controller did not start')
            wait(app, lambda: '/harurobo_teleop' in controller.nodes,
                 'simulation teleop node missing')
            window.stop_all.click()
            wait(app, lambda: not controller.busy(), 'Gazebo/controller process groups did not stop')
    except Exception:
        for key, _, line in list(window.logs.records):
            if key == 'demo_source' and 'publishing' in line:
                continue
            print(line, file=sys.stderr)
        raise
    finally:
        window.close()
        wait(app, lambda: not controller.processes.jobs, 'close left a managed process')
    print('Smoke checks passed')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as directory:
        main(Path(directory)/'local.yaml')
