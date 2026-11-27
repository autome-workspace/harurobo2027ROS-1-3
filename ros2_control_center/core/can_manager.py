"""Asynchronous SocketCAN inspection and authenticated interface activation."""
import json
import shutil

from PySide6.QtCore import QObject, QProcess, QTimer, Signal


class CanManager(QObject):
    log = Signal(str, str, str)
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.link = {}
        self.ip = shutil.which('ip')
        self.pkexec = shutil.which('pkexec')
        self.probe = QProcess(self)
        self.probe.finished.connect(self.inspected)
        self.probe.errorOccurred.connect(self.inspect_failed)
        self.activation = QProcess(self)
        self.activation.finished.connect(self.activated)
        self.activation.errorOccurred.connect(self.activate_failed)
        self.timer = QTimer(self, interval=2000)
        self.timer.timeout.connect(self.inspect)
        self.timer.start()
        self.inspect()

    def running(self):
        return self.activation.state() != QProcess.ProcessState.NotRunning

    def is_up(self):
        return 'UP' in self.link.get('flags', [])

    def can_activate(self):
        return bool(self.ip and self.pkexec and self.link and not self.is_up() and not self.running())

    def description(self):
        if self.running():
            return 'CAN 有効化中（認証を確認してください）'
        if not self.link:
            return 'can0 未検出'
        if not self.is_up():
            return 'can0 は DOWN（有効化が必要）'
        data = self.link.get('linkinfo', {}).get('info_data', {})
        bitrate = data.get('bittiming', {}).get('bitrate', '不明')
        return f"can0 は UP · {bitrate} bps · {data.get('state', '状態不明')}"

    def inspect(self):
        if self.ip and self.probe.state() == QProcess.ProcessState.NotRunning:
            self.probe.start(self.ip, ['-j', '-details', 'link', 'show', 'dev', 'can0'])

    def inspected(self, code, status):
        try:
            rows = json.loads(bytes(self.probe.readAllStandardOutput()))
            link = rows[0] if code == 0 and len(rows) == 1 else {}
            self.link = link if link.get('linkinfo', {}).get('info_kind') == 'can' else {}
        except (ValueError, IndexError, TypeError, AttributeError):
            self.link = {}
        self.probe.readAllStandardError()
        self.changed.emit()

    def inspect_failed(self, error):
        self.link = {}
        self.changed.emit()

    def activate(self):
        if not self.can_activate():
            return
        self.log.emit('CAN 設定', 'INFO', 'CAN有効化の管理者認証を待っています。')
        self.activation.start(self.pkexec, [
            '--disable-internal-agent', self.ip, 'link', 'set', 'dev', 'can0', 'up',
            'type', 'can', 'bitrate', '1000000', 'one-shot', 'on', 'restart-ms', '0',
        ])
        self.changed.emit()

    def activated(self, code, status):
        output = bytes(self.activation.readAllStandardError()).decode(errors='replace').strip()
        if status == QProcess.ExitStatus.NormalExit and code == 0:
            self.log.emit('CAN 設定', 'INFO', 'can0 を有効化しました（1 Mbps・ONE-SHOT）。')
        elif code in (126, 127):
            self.log.emit('CAN 設定', 'WARN', 'CAN有効化の認証がキャンセルされたか、許可されませんでした。')
        else:
            self.log.emit('CAN 設定', 'ERROR', f'CAN有効化に失敗しました（終了コード {code}）。{output}')
        self.inspect()
        self.changed.emit()

    def activate_failed(self, error):
        self.log.emit('CAN 設定', 'ERROR', f'認証処理を開始できません。{self.activation.errorString()}')
        self.changed.emit()

    def close(self):
        self.timer.stop()
        for process in (self.probe, self.activation):
            if process.state() != QProcess.ProcessState.NotRunning:
                process.kill()

    def idle(self):
        return all(process.state() == QProcess.ProcessState.NotRunning
                   for process in (self.probe, self.activation))
