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
    encoder_changed = Signal()

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.can = CanManager(self)
        self.can.log.connect(self.log)
        self.can.changed.connect(self.refresh)
        self.encoder_can = CanManager(self,
            interface=config.variables.get('ENCODER_CAN_INTERFACE', 'slcan0'),
            serial_port=config.variables.get('ENCODER_CAN_PORT', ''))
        self.encoder_can.log.connect(self.log)
        self.encoder_can.changed.connect(self.refresh)
        self.mode = 'demo'
        self.processes = ProcessManager(self)
        self.processes.output.connect(self.output)
        self.processes.exited.connect(self.exited)
        self.nodes = set()
        self.live = set()
        self.diagnostics = {}
        self.approach = {}
        self.encoders = {}
        self.encoder_received_at = 0.
        self.encoder_read_requested = False
        self.encoder_view_requested = False
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

    def encoder_available(self):
        return bool(self.encoders) and 0 <= time.monotonic()-self.encoder_received_at < .6

    def update_encoders(self, data):
        self.encoders = data.get('encoders', {})
        self.encoder_received_at = data.get('encoder_received_at', time.monotonic()) if self.encoders else 0.
        self.encoder_changed.emit()

    def output(self, key, stream, line):
        if key == MONITOR and self.processes.jobs[key].stopping:
            return
        if key == MONITOR and stream == 'stdout':
            try:
                data = json.loads(line)
                if data.get('kind') == 'encoders':
                    self.update_encoders(data)
                    return
                self.nodes = set(data['nodes'])
                self.live = set(data['live'])
                self.diagnostics = data['diagnostics']
                self.approach = data.get('approach', {})
                self.update_encoders(data)
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
        self.encoders.clear()
        self.encoder_received_at = 0.
        self.encoder_read_requested = False
        self.encoder_view_requested = False
        self.snapshot = 0
        self.monitor_retry = 0
        self.processes.stop(MONITOR)
        return True

    def busy(self):
        return bool(self.can.running() or self.encoder_can.running() or self.pending or self.shutdown or (self.processes.jobs.keys() - {MONITOR}))

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
                if key == 'encoders':
                    if not self.encoders:
                        return 'エンコーダー状態待ち', 'warning'
                    if not self.encoders.get('can_connected'):
                        return 'CANの有効化待ち', 'warning'
                    channels = [c for c in self.encoders.get('channels', []) if c.get('fresh')]
                    expected = self.encoders.get('expected_devices', 2)
                    if len(channels) < expected:
                        return f'{len(channels)}/{expected}台受信・chと接続を確認', 'warning'
                    if any(c.get('fault_level', 'warning' if c.get('fault') else 'ok') == 'warning' for c in channels):
                        return f'{len(channels)}台受信・異常履歴あり', 'warning'
                    if any(c.get('fault') for c in channels):
                        return f'{len(channels)}台受信中（再起動履歴のみ）', 'ok'
                    return f'{len(channels)}台受信中', 'ok'
                if name := row.get('diagnostic_name'):
                    level, message = self.diagnostics.get(name, (3, '診断結果待ち'))
                    if (level == 3 and message == 'No recent CAN data frames'
                            and name == 'harurobo_base: SocketCAN monitor' and self.can.is_up()
                            and self.can.link.get('linkinfo', {}).get('info_data', {}).get('state') == 'ERROR-ACTIVE'):
                        return '起動中・受信待ち', 'ok'
                    if level:
                        return message, 'error' if level == 2 else 'warning'
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

    def activate_encoder_can(self):
        if self.mode == 'real' and not self.closing and not self.shutdown:
            self.encoder_read_requested = True
            self.encoder_can.activate()

    def set_motor_automatic(self, enabled):
        if (self.closing or self.pending or self.shutdown or 'motor' in self.processes.jobs
                or self.external('motor')):
            return False
        try:
            self.config.save_variables({'MOTOR_AUTO_APPROACH': bool(enabled),
                                        **({'MOTOR_ENCODER_DISTANCE': False} if enabled else {})})
        except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
            self.log.emit('システム', 'ERROR', f'自動接近設定を保存できません。{error}')
            return False
        self.snapshot = 0.
        self.monitor_retry = 0.
        self.processes.stop(MONITOR)
        return True

    def start(self, ids, explicit=False):
        if self.pending or self.shutdown or self.closing or self.can.running() or self.encoder_can.running():
            return
        try:
            order = self.config.order(ids, self.mode)
            jobs = self.processes.jobs
            pending = [key for key in order if key not in jobs]
            if any(self.config.programs[key].get('requires_can') for key in pending) and not self.can.drive_ready():
                raise ValueError('CANがDOWN・未検出・未確認のため足回りを起動できません。CANを有効化してから起動してください。')
            for key in pending:
                if any(item in jobs or self.external(item)
                       for item in self.config.programs[key].get('conflicts_with', [])):
                    raise ValueError('競合するプログラムを停止してから起動してください。')
                if self.config.programs[key].get('explicit') and (not explicit or key not in ids):
                    raise ValueError('明示起動が必要なプログラムが含まれています。')
                self.config.command(key)
            self.pending = pending
            for key in pending:
                self.failures.pop(key, None)
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.log.emit('システム', 'ERROR', str(error))

    def stop(self, key=None):
        if key is None or key == 'encoders':
            self.encoder_read_requested = False
        if key is None or key in ('encoders', 'encoder_view'):
            self.encoder_view_requested = False
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
        if 'motor' in jobs and not jobs['motor'].stopping:
            if (not self.can.drive_ready() or
                    any(dep not in jobs for dep in self.config.programs['motor'].get('depends_on', []))):
                self.log.emit('motor', 'WARN', 'CANまたは依存プログラムが利用できないため、足回りを停止します。')
                self.stop('motor')
        if self.pending:
            self.advance()
        if (self.encoder_read_requested and self.mode == 'real' and self.monitor_available()
                and self.encoder_can.is_up() and not self.closing and not self.pending and not self.shutdown):
            self.encoder_read_requested = False
            if 'encoders' not in jobs and not self.external('encoders'):
                self.start(['encoders'])
        if (self.encoder_view_requested and self.mode == 'real' and self.monitor_available()
                and not self.closing and not self.pending and not self.shutdown):
            self.encoder_view_requested = False
            self.start(['encoder_view'])
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
                if row.get('dependency_readiness') == 'process' and not self.processes.jobs[dep].stopping:
                    continue
                if self.status(dep)[1] != 'ok':
                    if time.monotonic() - self.started[dep] < 25:
                        return
                    raise ValueError(f'依存プログラム {dep} の配信待ちがタイムアウトしました。')
            if row.get('requires_can') and not self.can.drive_ready():
                raise ValueError('起動待ち中にCANがDOWNまたは未確認になったため、足回りの起動を中止しました。')
            if key == 'motor' and self.config.variables.get('MOTOR_AUTO_APPROACH'):
                settings = yaml.safe_load(Path(self.config.variables['APPROACH_SETTINGS']).read_text())
                parameters = yaml.safe_load(Path(settings['parameters_file']).read_text())
                p = parameters['approach_controller']['ros__parameters']
                if not (p['auto_enabled'] and 0 < p['validated_min_range'] <= p['target_distance'] - p['distance_tolerance'] + 1e-9):
                    raise ValueError('自動接近設定で、実測した最小距離を保存してください。')
            self.processes.start(key, self.config.command(key), self.config.environment(self.mode))
            self.started[key] = time.monotonic()
            self.pending.pop(0)
            self.log.emit(key, 'INFO', '起動しました。')
        except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.failures[key] = '起動中止'
            self.log.emit(key, 'ERROR', str(error))
            self.pending.clear()
