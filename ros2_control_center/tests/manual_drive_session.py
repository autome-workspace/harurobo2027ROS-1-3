"""Exercise real-drive supervision with fake children, without opening CAN."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

CORE = Path(__file__).resolve().parents[1] / 'core'


def wait_for(condition):
    deadline = time.monotonic() + 8
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError('timeout')
        time.sleep(.025)


def run(kill_supervisor):
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        (folder / 'watchdog_stop.py').write_text("""
from pathlib import Path
import os
import time
folder = Path(__file__).parent
(folder / 'watchdog.pid').write_text(str(os.getpid()))
print('WATCHDOG: waiting for the first heartbeat', flush=True)
while not (folder / 'done').exists():
    time.sleep(.025)
""")
        (folder / 'manual_drive.py').write_text("""
from pathlib import Path
import os
import signal
import time
folder = Path(__file__).parent
def stop(signum, frame):
    (folder / 'done').touch()
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
(folder / 'drive.pid').write_text(str(os.getpid()))
while True:
    time.sleep(.025)
""")
        code = (
            'import sys; from pathlib import Path; '
            f'sys.path.insert(0, {str(CORE)!r}); '
            'import manual_drive_session as session; '
            f'session.SCRIPTS = Path({directory!r}); '
            'sys.exit(session.main())'
        )
        process = subprocess.Popen([sys.executable, '-c', code],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            wait_for(lambda: (folder / 'drive.pid').exists())
            if kill_supervisor:
                process.kill()
            else:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
            wait_for(lambda: (folder / 'done').exists())
            if not kill_supervisor:
                assert process.returncode == 0
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            # Only the fake children created in this test are cleanup targets.
            for path in folder.glob('*.pid'):
                try:
                    os.kill(int(path.read_text()), signal.SIGTERM)
                except ProcessLookupError:
                    pass


run(False)
run(True)
print('PASS: graceful stop and driver stop on supervisor death, without CAN')
