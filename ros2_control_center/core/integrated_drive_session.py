"""One managed drive session owns manual control and optional ball approach."""
from pathlib import Path
import sys

import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from approach_session import description as approach_description


def description(path, drive_command=None, failures=None):
    settings = yaml.safe_load(Path(path).read_text())
    parameters = yaml.safe_load(Path(settings['parameters_file']).read_text())
    control = parameters['approach_controller']['ros__parameters']
    if not (control['auto_enabled'] and 0 < control['validated_min_range'] <=
            control['target_distance'] - control['distance_tolerance'] + 1e-9):
        raise ValueError('自動接近設定で、目標距離と確認済み最小距離を保存してください。')
    # Validate the complete approach configuration before creating any motor process.
    approach = approach_description(path)
    drive = ExecuteProcess(
        cmd=drive_command or [sys.executable, '-u', str(Path(__file__).with_name('manual_drive_session.py')), '--automatic'],
        output='screen', output_format='{line}')
    def process_exited(event, context):
        if not context.is_shutdown and event.returncode and failures is not None:
            failures.append(event.returncode)
        return [EmitEvent(event=Shutdown(reason='足回りまたは自動接近のプロセスが終了しました。'))]
    shutdown = RegisterEventHandler(OnProcessExit(on_exit=process_exited))
    return LaunchDescription([shutdown, *approach.entities, drive])


def main():
    service = LaunchService()
    failures = []
    service.include_launch_description(description(sys.argv[1], failures=failures))
    result = service.run()
    return 1 if failures else result


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        print(f'ERROR: 足回り駆動を開始できません。{error}', flush=True)
        sys.exit(1)
