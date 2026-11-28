from dataclasses import dataclass, field
import os
import signal
import subprocess
import time

from PySide6.QtCore import QObject, QTimer, Signal

from core.config_manager import ROOT


@dataclass
class Job:
    process: subprocess.Popen
    stopping: int = 0
    deadline: float = 0
    buffers: dict = field(default_factory=dict)


class ProcessManager(QObject):
    output = Signal(str, str, str)
    exited = Signal(str, int, bool)
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.jobs = {}
        self.timer = QTimer(self, interval=50)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    def start(self, key, argv, env):
        if key in self.jobs:
            return
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True)
        self.jobs[key] = Job(process)
        for pipe in (process.stdout, process.stderr):
            os.set_blocking(pipe.fileno(), False)
        self.changed.emit()

    @staticmethod
    def send(job, value):
        try:
            os.killpg(job.process.pid, value)
        except ProcessLookupError:
            return False
        return True

    def stop(self, key):
        job = self.jobs.get(key)
        if job and not job.stopping:
            job.stopping = 1
            job.deadline = time.monotonic() + 8
            self.send(job, signal.SIGINT)
            self.changed.emit()

    def read(self, key, job, stream, pipe):
        if pipe.closed:
            return
        # Structured monitor snapshots include all channels; log lines stay small.
        limit = 65536 if key == '_monitor' else 4096
        try:
            data = os.read(pipe.fileno(), 65536 if key == '_monitor' else 16384)
        except BlockingIOError:
            return
        pending = job.buffers.get(stream, b'') + data
        lines = pending.split(b'\n')
        tail = lines.pop()
        if not data:
            pipe.close()
        if tail and (not data or len(tail) >= limit):
            lines.append(tail)
            tail = b''
        job.buffers[stream] = tail
        for line in lines:
            self.output.emit(key, stream, line[:limit].decode(errors='replace').rstrip())

    def poll(self):
        now = time.monotonic()
        for key, job in list(self.jobs.items()):
            process = job.process
            self.read(key, job, 'stdout', process.stdout)
            self.read(key, job, 'stderr', process.stderr)
            code = process.poll()
            group_live = self.send(job, 0)
            if code is not None and not group_live and process.stdout.closed and process.stderr.closed:
                del self.jobs[key]
                self.exited.emit(key, code, bool(job.stopping))
                self.changed.emit()
                continue
            if code is not None and group_live and not job.stopping:
                self.stop(key)
            if job.stopping and group_live and now >= job.deadline:
                self.send(job, signal.SIGTERM if job.stopping == 1 else signal.SIGKILL)
                job.stopping += 1
                job.deadline = now + 2
