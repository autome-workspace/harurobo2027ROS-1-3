"""Check CAN presentation and authentication outcomes without changing hardware."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PySide6.QtWidgets import QApplication

from core.config_manager import Config
from core.controller import Controller
from ui.main_window import MainWindow


app = QApplication([])


def wait(condition):
    deadline = time.monotonic() + 5
    while not condition():
        app.processEvents()
        if time.monotonic() >= deadline:
            raise AssertionError('timeout')
        time.sleep(.01)


controller = Controller(Config())
controller.timer.stop()
controller.processes.timer.stop()
wait(lambda: controller.can.probe.state() == controller.can.probe.ProcessState.NotRunning)
window = MainWindow(controller)
controller.mode = 'real'
controller.snapshot = time.monotonic()
controller.live = {'/diagnostics'}
controller.started['can'] = time.monotonic()
controller.processes.jobs['can'] = SimpleNamespace(stopping=0)
name = 'harurobo_base: SocketCAN monitor'
logs = []
controller.log.connect(lambda key, level, line: logs.append((level, line)))

with tempfile.TemporaryDirectory() as directory:
    folder = Path(directory)
    state = folder / 'link.json'
    link = {'flags': [], 'linkinfo': {'info_kind': 'can', 'info_data': {'state': 'STOPPED'}}}
    state.write_text(json.dumps([link]))
    ip = folder / 'ip'
    ip.write_text('#!/usr/bin/python3\nfrom pathlib import Path\nprint(Path(' +
                  repr(str(state)) + ').read_text())\n')
    ip.chmod(0o755)
    pkexec = folder / 'pkexec'
    # This fake authentication tool only edits its own test fixture.
    pkexec.write_text('''#!/usr/bin/python3
import json
from pathlib import Path
import sys
assert sys.argv[1:] == ['--disable-internal-agent', IP, 'link', 'set',
    'dev', 'can0', 'up', 'type', 'can', 'bitrate', '1000000',
    'one-shot', 'on', 'restart-ms', '0']
path = Path(STATE)
link = json.loads(path.read_text())[0]
link['flags'] = ['UP']
link['linkinfo']['info_data'] = {'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 1000000}}
path.write_text(json.dumps([link]))
'''.replace('IP', repr(str(ip))).replace('STATE', repr(str(state))))
    pkexec.chmod(0o755)
    controller.can.timer.stop()
    controller.can.ip = str(ip)
    controller.can.pkexec = str(pkexec)
    controller.can.inspect()
    wait(lambda: bool(controller.can.link) and not controller.can.is_up())
    window.refresh()
    assert window.can_enable.isEnabled()
    controller.diagnostics[name] = (2, 'SocketCAN unavailable')
    assert controller.status('can')[1] == 'error'
    window.can_enable.click()
    wait(lambda: controller.can.is_up() and not controller.can.running())
    controller.diagnostics[name] = (3, 'No recent CAN data frames')
    assert controller.status('can') == ('起動中・受信待ち', 'ok')
    window.refresh()
    assert not window.can_enable.isEnabled()
    controller.diagnostics[name] = (2, 'SocketCAN bus-off observed (latched)')
    assert controller.status('can')[1] == 'error'
    link['flags'] = []
    state.write_text(json.dumps([link]))
    controller.can.inspect()
    wait(lambda: not controller.can.is_up())
    pkexec.write_text('#!/bin/sh\nexit 126\n')
    controller.can.activate()
    wait(lambda: not controller.can.running())
    assert any(level == 'WARN' and 'キャンセル' in line for level, line in logs)
    assert not controller.can.is_up()
controller.processes.jobs.clear()
controller.can.close()
wait(controller.can.idle)
window.close()
print('PASS: CAN waiting/error display, GUI activation and authentication cancellation')
