"""Supervise the existing real drive and its independent stop watchdog."""
import argparse
import os
from pathlib import Path
import ctypes
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'real/manual_drive/src/harurobo_base/test'
stopping = False


def request_stop(signum, frame):
    global stopping
    stopping = True


def stop_if_parent_dies():
    # This supervisor is single-threaded. Linux delivers SIGINT to the driver
    # even when the GUI escalates to killing the supervisor.
    parent = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'prctl')
    if os.getppid() != parent or parent == 1:
        os.kill(os.getpid(), signal.SIGINT)


def main(automatic=False, encoder_distance=False):
    if encoder_distance:
        from encoder_distance import validate
        import yaml
        path = Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))/'ros2-control-center/encoder_distance.yaml'
        validate(yaml.safe_load(path.read_text()))
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, request_stop)
    parent = os.getppid()
    log_dir = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'ros2-control-center'
    log_dir.mkdir(parents=True, exist_ok=True)
    writer = tempfile.NamedTemporaryFile(prefix='watchdog-', suffix='.log', dir=log_dir, delete=False)
    reader = open(writer.name, 'rb')
    print(f'INFO: 独立ウォッチドッグログ {writer.name}', flush=True)
    watchdog = subprocess.Popen(
        [sys.executable, '-u', str(SCRIPTS / 'watchdog_stop.py')],
        start_new_session=True, stdout=writer, stderr=subprocess.STDOUT,
    )
    drive = None
    clean = False
    try:
        # The watchdog binds its exclusive UDP port before allowing CAN access.
        deadline = time.monotonic() + 5
        ready = False
        while not stopping and watchdog.poll() is None and time.monotonic() < deadline:
            line = reader.readline()
            if line:
                print(line.decode(errors='replace'), end='', flush=True)
                if b'waiting for the first heartbeat' in line:
                    ready = True
                    break
            time.sleep(.05)
        if not ready:
            reader.seek(0)
            details = reader.read().decode(errors='replace')
            if 'Address already in use' in details:
                raise RuntimeError('停止ウォッチドッグ用ポート55901が使用中です。'
                                   '以前の停止監視が残っている可能性があります。'
                                   '駆動と停止監視の状態を確認してください。')
            raise RuntimeError('停止ウォッチドッグを開始できませんでした。')
        if stopping or os.getppid() != parent:
            return 1
        command = ([sys.executable, '-u', str(Path(__file__).with_name('approach_drive.py'))]
                   if automatic else [sys.executable, '-u', str(SCRIPTS / 'manual_drive.py'),
                                      '--execute-powered-robot-estop-ready'])
        if encoder_distance:
            command = [sys.executable, '-u', str(Path(__file__).with_name('encoder_distance_drive.py'))]
        drive = subprocess.Popen(
            command,
            start_new_session=True, preexec_fn=stop_if_parent_dies,
        )
        while not stopping and os.getppid() == parent and drive.poll() is None:
            if watchdog.poll() is not None:
                raise RuntimeError('停止ウォッチドッグが終了しました。駆動を停止します。')
            data = reader.read()
            if data:
                print(data.decode(errors='replace'), end='', flush=True)
            time.sleep(.05)
    finally:
        if drive is not None:
            if drive.poll() is None:
                drive.send_signal(signal.SIGINT)
            try:
                drive.wait(timeout=5)
                try:
                    clean = watchdog.wait(timeout=1) == 0
                except subprocess.TimeoutExpired:
                    clean = False
            except subprocess.TimeoutExpired:
                # Killing the stuck drive stops its heartbeats. Leave the independent
                # watchdog running to keep sending zero and disable frames.
                drive.kill()
                drive.wait()
                print('ERROR: 駆動停止がタイムアウトしました。物理非常停止を操作してください。', flush=True)
        if drive is None:
            watchdog.terminate()
            watchdog.wait(timeout=2)
        if drive is not None and not clean:
            print(f'ERROR: 停止を確認できません。物理非常停止を操作してください。'
                  f' 独立ウォッチドッグ PID={watchdog.pid} を保持します。', flush=True)
        reader.close()
        writer.close()
    return 0 if clean else 1


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--automatic', action='store_true')
        parser.add_argument('--encoder-distance', action='store_true')
        args = parser.parse_args()
        if args.automatic and args.encoder_distance:
            raise RuntimeError('自動接近とエンコーダー指定移動は同時に有効化できません。')
        sys.exit(main(args.automatic, args.encoder_distance))
    except (OSError, RuntimeError, ValueError) as error:
        print(f'ERROR: {error}', flush=True)
        sys.exit(1)
