"""Asynchronous SocketCAN inspection and authenticated interface activation."""
import json
import shutil
import time
from pathlib import Path
import re

from PySide6.QtCore import QObject, QProcess, QTimer, Signal


class CanManager(QObject):
    log = Signal(str, str, str)
    changed = Signal()

    def __init__(self, parent=None, *, interface='can0', serial_port=''):
        super().__init__(parent)
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
            raise ValueError('CANインターフェース名が無効です。')
        self.interface = interface
        self.serial_port = serial_port
        self.slcand = shutil.which('slcand')
        self.pending_setup = False
        self.stage = ''
        self.closed = False
        self.link = {}
        self.inspected_at = 0.
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
        return self.pending_setup or self.activation.state() != QProcess.ProcessState.NotRunning

    def is_up(self):
        return 'UP' in self.link.get('flags', [])

    def drive_ready(self):
        return (self.is_up() and time.monotonic() - self.inspected_at < 5
                and self.link.get('linkinfo', {}).get('info_data', {}).get('state') == 'ERROR-ACTIVE')

    def can_activate(self):
        available = self.link or (self.slcand and self.serial_port and Path(self.serial_port).exists())
        return bool(self.ip and self.pkexec and available and not self.is_up() and not self.running() and not self.closed)

    def description(self):
        if self.running():
            return 'CAN 有効化中（認証を確認してください）'
        if not self.link:
            return (f'{self.interface} 未作成（USB接続済み・有効化が必要）'
                    if self.serial_port and Path(self.serial_port).exists() else f'{self.interface} 未検出')
        if not self.is_up():
            return f'{self.interface} は DOWN（有効化が必要）'
        data = self.link.get('linkinfo', {}).get('info_data', {})
        bitrate = data.get('bittiming', {}).get('bitrate', '不明')
        if self.serial_port:
            return f"{self.interface} は UP · SLCAN · {data.get('state', '状態不明')}"
        return f"{self.interface} は UP · {bitrate} bps · {data.get('state', '状態不明')}"

    def inspect(self):
        if not self.closed and self.ip and self.probe.state() == QProcess.ProcessState.NotRunning:
            self.probe.start(self.ip, ['-j', '-details', 'link', 'show', 'dev', self.interface])

    def inspected(self, code, status):
        self.inspected_at = time.monotonic()
        try:
            rows = json.loads(bytes(self.probe.readAllStandardOutput()))
            link = rows[0] if code == 0 and len(rows) == 1 else {}
            self.link = link if (link.get('linkinfo', {}).get('info_kind') == 'can' or link.get('link_type') == 'can') else {}
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
        self.pending_setup = True
        if not self.link and self.serial_port:
            self.stage = 'attach'
            self.activation.start(self.pkexec, ['--disable-internal-agent', self.slcand,
                '-o', '-c', '-s8', self.serial_port, self.interface])
        else:
            self.start_link()
        self.changed.emit()

    def start_link(self):
        if self.closed or not self.pending_setup:
            return
        self.stage = 'up'
        arguments = ['--disable-internal-agent', self.ip, 'link', 'set', 'dev', self.interface, 'up']
        if not self.serial_port:
            arguments += ['type', 'can', 'bitrate', '1000000', 'one-shot', 'on', 'restart-ms', '0']
        self.activation.start(self.pkexec, arguments)

    def activated(self, code, status):
        output = bytes(self.activation.readAllStandardError()).decode(errors='replace').strip()
        if status == QProcess.ExitStatus.NormalExit and code == 0 and self.stage == 'attach':
            QTimer.singleShot(300, self.start_link)
            return
        self.pending_setup = False
        if status == QProcess.ExitStatus.NormalExit and code == 0:
            self.log.emit('CAN 設定', 'INFO', f'{self.interface} を有効化しました。')
        elif code in (126, 127):
            self.log.emit('CAN 設定', 'WARN', 'CAN有効化の認証がキャンセルされたか、許可されませんでした。')
        else:
            self.log.emit('CAN 設定', 'ERROR', f'CAN有効化に失敗しました（終了コード {code}）。{output}')
        self.inspect()
        self.changed.emit()

    def activate_failed(self, error):
        self.pending_setup = False
        self.log.emit('CAN 設定', 'ERROR', f'認証処理を開始できません。{self.activation.errorString()}')
        self.changed.emit()

    def close(self):
        self.closed = True
        self.pending_setup = False
        self.timer.stop()
        for process in (self.probe, self.activation):
            if process.state() != QProcess.ProcessState.NotRunning:
                process.kill()

    def idle(self):
        return not self.pending_setup and all(process.state() == QProcess.ProcessState.NotRunning
                   for process in (self.probe, self.activation))
