import json
import time
from pathlib import Path

import yaml

from PySide6.QtCore import QObject, QTimer, Signal

from core.can_manager import CanManager
from core.config_manager import APP
from core.process_manager import ProcessManager

MONITOR = '_monitor'


class Controller(QObject):
    refresh = Signal()
    log = Signal(str, str, str)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.can = CanManager(self)
        self.can.log.connect(self.log)
        self.can.changed.connect(self.refresh)
        self.mode = 'demo'
        self.processes = ProcessManager(self)
        self.processes.output.connect(self.output)
        self.processes.exited.connect(self.exited)
        self.nodes = set()
        self.live = set()
        self.diagnostics = {}
        self.approach = {}
        self.snapshot = 0.0
        self.monitor_retry = 0.0
        self.started = {}
        self.last_expected_output = {}
        self.failures = {}
        self.pending = []
        self.shutdown = []
        self.closing = False
        self.timer = QTimer(self, interval=250)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    def monitor_available(self):
        return time.monotonic() - self.snapshot < 2 and self.snapshot > 0

    def output(self, key, stream, line):
        if key == MONITOR and self.processes.jobs[key].stopping:
            return
        if key == MONITOR and stream == 'stdout':
            try:
                data = json.loads(line)
                self.nodes = set(data['nodes'])
                self.live = set(data['live'])
                self.diagnostics = data['diagnostics']
                self.approach = data.get('approach', {})
                self.snapshot = time.monotonic()
                return
            except (ValueError, KeyError, TypeError):
                pass
        row = self.config.programs.get(key, {})
        expected = row.get('expected_stdout')
        if stream == 'stdout' and expected and (
            line.startswith(expected) if row.get('stdout_prefix') else line == expected
        ):
            self.last_expected_output[key] = time.monotonic()
        if not line:
            return
        self.log.emit(key, 'ERROR' if 'ERROR' in line or 'FATAL' in line else
                      'WARN' if stream == 'stderr' or 'WARN' in line else 'INFO', line)

    def exited(self, key, code, stopping):
        if key == MONITOR:
            self.snapshot = 0
            self.monitor_retry = time.monotonic() + 3
        else:
            if not stopping and (code or any(key in self.config.programs[item].get('depends_on', [])
                                             for item in self.pending)):
                self.failures[key] = f'終了コード {code}'
            self.started.pop(key, None)
            self.last_expected_output.pop(key, None)
        self.log.emit(key, 'INFO' if stopping or code == 0 else 'ERROR', f'終了コード {code}')

    def set_mode(self, mode):
        if self.busy():
            return False
        self.mode = mode
        self.failures.clear()
        self.nodes.clear()
        self.live.clear()
        self.diagnostics.clear()
        self.approach.clear()
        self.snapshot = 0
        self.monitor_retry = 0
        self.processes.stop(MONITOR)
        return True

    def busy(self):
        return bool(self.can.running() or self.pending or self.shutdown or (self.processes.jobs.keys() - {MONITOR}))

    def external(self, key):
        row = self.config.programs[key]
        if not self.monitor_available():
            return False
        if nodes := row.get('nodes'):
            return bool(self.nodes.intersection(nodes))
        return bool(self.live.intersection(row.get('expected_topics', [])))

    def status(self, key):
        row = self.config.programs[key]
        if row.get('disabled'):
            return ('外部稼働（管理外）', 'warning') if self.external(key) else ('ロック中', 'muted')
        job = self.processes.jobs.get(key)
        if job:
            if job.stopping or key in self.shutdown:
                return '停止処理中', 'warning'
            if row.get('expected_stdout'):
                received = self.last_expected_output.get(key)
                now = time.monotonic()
                if received is None:
                    return ('受信待ち' if now - self.started[key] < 20 else '受信未確認'), 'warning'
                if row.get('stdout_prefix'):
                    return '駆動セッション稼働', 'ok'
                if now - received >= 3:
                    return '受信途絶', 'warning'
                return '受信確認済み', 'ok'
            topics = set(row.get('expected_topics', []))
            if not topics:
                return 'プロセス稼働中', 'ok'
            if self.monitor_available() and topics <= self.live:
                if name := row.get('diagnostic_name'):
                    level, message = self.diagnostics.get(name, (3, '診断結果待ち'))
                    if (level == 3 and message == 'No recent CAN data frames'
                            and name == 'harurobo_base: SocketCAN monitor' and self.can.is_up()
                            and self.can.link.get('linkinfo', {}).get('info_data', {}).get('state') == 'ERROR-ACTIVE'):
                        return '起動中・受信待ち', 'ok'
                    if level:
                        return message, 'error' if level == 2 else 'warning'
                if key == 'approach' and not self.approach.get('estimate', {}).get('valid'):
                    return 'ボール位置の照合待ち', 'warning'
                return '配信確認済み', 'ok'
            if time.monotonic() - self.started[key] < 20:
                return '配信待ち', 'warning'
            return '配信未確認', 'warning'
        if key in self.failures:
            return self.failures[key], 'error'
        if key in self.pending:
            return '起動待ち', 'warning'
        if self.external(key):
            return '外部稼働（管理外）', 'warning'
        return '停止中', 'muted'

    def activate_can(self):
        if (self.mode == 'real' and not self.closing and not self.shutdown
                and 'motor' not in self.processes.jobs and 'motor' not in self.pending
                and self.monitor_available() and not self.external('motor')):
            self.can.activate()

    def start(self, ids, explicit=False):
        if self.pending or self.shutdown or self.closing or self.can.running():
            return
        try:
            order = self.config.order(ids, self.mode)
            jobs = self.processes.jobs
            pending = [key for key in order if key not in jobs]
            for key in pending:
                if any(item in jobs or self.external(item)
                       for item in self.config.programs[key].get('conflicts_with', [])):
                    raise ValueError('手動駆動と自動接近の実機駆動は、停止してから切り替えてください。')
                if self.config.programs[key].get('explicit') and (not explicit or key not in ids):
                    raise ValueError('明示起動が必要なプログラムが含まれています。')
                self.config.command(key)
            self.pending = pending
            for key in pending:
                self.failures.pop(key, None)
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.log.emit('システム', 'ERROR', str(error))

    def stop(self, key=None):
        self.pending.clear()
        selected = set(self.processes.jobs) - {MONITOR}
        if key is not None:
            affected = {key}
            for item in selected:
                if key in self.config.order([item], self.mode):
                    affected.add(item)
            selected &= affected
        self.shutdown = list(dict.fromkeys([
            *self.shutdown, *(item for item in reversed(self.processes.jobs) if item in selected)
        ]))

    def tick(self):
        jobs = self.processes.jobs
        now = time.monotonic()
        if self.closing and not self.busy():
            self.processes.stop(MONITOR)
        if not self.closing and MONITOR not in jobs and now >= self.monitor_retry:
            self.monitor_retry = now + 3
            rows = (row for row in self.config.programs.values() if self.mode in row['modes'])
            topics = sorted({topic for row in rows for topic in row.get('expected_topics', [])})
            overlay = ['install/setup.bash'] if self.mode == 'real' and (APP.parent / 'install/setup.bash').is_file() else []
            try:
                self.processes.start(MONITOR, self.config.wrapped([
                    '/usr/bin/python3', str(APP / 'core/ros_monitor.py'), *topics
                ], overlay), self.config.environment(self.mode))
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
                self.log.emit(MONITOR, 'WARN', str(error))
        while self.shutdown and self.shutdown[0] not in jobs:
            self.shutdown.pop(0)
        if self.shutdown:
            self.processes.stop(self.shutdown[0])
        for key in ('auto_drive',):
            if key in jobs and 'approach' not in jobs:
                self.processes.stop(key)
        if self.pending:
            self.advance()
        self.refresh.emit()

    def advance(self):
        key = self.pending[0]
        row = self.config.programs[key]
        dependencies = row.get('depends_on', [])
        try:
            if key in ('joy', 'joy_sim') and self.external('motor'):
                raise ValueError('管理外の足回り駆動を検出したため、コントローラーを起動できません。')
            if self.external(key):
                raise ValueError('同じノードまたは監視対象の配信を管理外で検出しました。')
            if self.mode != 'demo' and not self.monitor_available():
                raise ValueError('ROS グラフの監視開始を待ってから起動してください。')
            for dep in dependencies:
                if dep not in self.processes.jobs:
                    raise ValueError(f'依存プログラム {dep} が停止しました。')
                if self.status(dep)[1] != 'ok':
                    if time.monotonic() - self.started[dep] < 25:
                        return
                    raise ValueError(f'依存プログラム {dep} の配信待ちがタイムアウトしました。')
            if key == 'auto_drive':
                settings = yaml.safe_load(Path(self.config.variables['APPROACH_SETTINGS']).read_text())
                parameters = yaml.safe_load(Path(settings['parameters_file']).read_text())
                p = parameters['approach_controller']['ros__parameters']
                if not (p['auto_enabled'] and 0 < p['validated_min_range'] <= p['target_distance'] - p['distance_tolerance'] + 1e-9):
                    raise ValueError('自動接近設定で、実測した最小距離を保存してください。')
                if not self.approach.get('estimate', {}).get('valid'):
                    raise ValueError('ボール位置の照合を確認してから実機駆動を開始してください。')
                if self.approach.get('state', '').split(' ', 1)[0] != 'MANUAL':
                    raise ValueError('L1を離して自動接近を解除してから実機駆動を開始してください。')
            self.processes.start(key, self.config.command(key), self.config.environment(self.mode))
            self.started[key] = time.monotonic()
            self.pending.pop(0)
            self.log.emit(key, 'INFO', '起動しました。')
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.failures[key] = '起動中止'
            self.log.emit(key, 'ERROR', str(error))
            self.pending.clear()
