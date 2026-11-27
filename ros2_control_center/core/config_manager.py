import os
from pathlib import Path
import shlex
import json
import tempfile

import yaml

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parent
USER_CONFIG = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'ros2-control-center' / 'local.yaml'
MODES = {'demo': 'デモ', 'simulation': 'シミュレーション', 'real': '実機'}


class Config:
    def __init__(self, path=None):
        shared = yaml.safe_load((APP / 'config/programs.yaml').read_text())
        local_path = path or USER_CONFIG
        self.path = local_path
        local = yaml.safe_load(local_path.read_text()) if local_path.exists() else {}
        local = local or {}
        settings = shared['settings']
        self.shared_variables = settings['variables']
        self.variables = self.shared_variables | local.get('variables', {})
        self.variables_revision = self.path.stat().st_mtime_ns if self.path.exists() else None
        self.domains = settings['domains'] | local.get('domains', {})
        if set(self.domains) != set(MODES) or any(
            type(domain) is not int or not 0 <= domain <= 101 for domain in self.domains.values()
        ) or len(set(self.domains.values())) != len(MODES) or self.domains['simulation'] != 87:
            raise ValueError('ROS ドメインは互いに異なる 0–101 の整数にしてください。Gazebo は 87 固定です。')
        self.setup = settings['setup'] + local.get('setup', [])
        rows = shared['programs']
        self.programs = {row['id']: row for row in rows}
        if len(self.programs) != len(rows):
            raise ValueError('プログラム ID が重複しています。')
        for key, row in self.programs.items():
            if row.get('disabled'):
                continue
            if not row.get('command') or not set(row['modes']) <= set(MODES):
                raise ValueError(f'{key}: 起動コマンドまたはモードが無効です。')
            for mode in row['modes']:
                self.order([key], mode)

    def save_variables(self, changes):
        local = yaml.safe_load(self.path.read_text()) if self.path.exists() else {}
        local = local or {}
        local['variables'] = local.get('variables', {}) | changes
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            try:
                yaml.safe_dump(local, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
                temporary.replace(self.path)
            finally:
                temporary.unlink(missing_ok=True)
        self.variables = self.shared_variables | local['variables']
        self.variables_revision = self.path.stat().st_mtime_ns

    def order(self, ids, mode):
        ordered = []

        def visit(key, visiting):
            if key in visiting:
                raise ValueError(f'{key}: 循環依存があります。')
            if key in ordered:
                return
            row = self.programs.get(key)
            if not row or mode not in row['modes'] or row.get('disabled'):
                raise ValueError(f'{key}: このモードでは起動できません。')
            for dependency in row.get('depends_on', []):
                visit(dependency, visiting | {key})
            ordered.append(key)

        for key in ids:
            visit(key, set())
        return ordered

    def environment(self, mode):
        env = os.environ.copy()
        env.pop('ROS_LOCALHOST_ONLY', None)
        return env | {
            'ROS_DOMAIN_ID': str(self.domains[mode]),
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'ROS_STATIC_PEERS': '',
            'PYTHONUNBUFFERED': '1',
        }

    def wrapped(self, args, extra_setup=()):
        files = [ROOT / Path(path).expanduser() for path in [*self.setup, *extra_setup]]
        for path in files:
            if not path.is_file():
                raise ValueError(f'環境設定ファイルがありません: {path}')
        return [
            '/bin/bash', '-c',
            'while [[ $1 != -- ]]; do source "$1" || exit; shift; done; shift; exec "$@"',
            'control-center', *map(str, files), '--', *args,
        ]

    def refresh_variables(self):
        revision = self.path.stat().st_mtime_ns if self.path.exists() else None
        if revision != self.variables_revision:
            local = yaml.safe_load(self.path.read_text()) if self.path.exists() else {}
            self.variables = self.shared_variables | (local or {}).get('variables', {})
            self.variables_revision = revision

    def command(self, key):
        self.refresh_variables()
        row = self.programs[key]
        values = {}
        if device := row.get('requires_device'):
            path = Path(str(self.variables.get(device, ''))).expanduser()
            if not path.is_absolute() or not path.is_char_device():
                raise ValueError(f'{device} に接続中のデバイスのパスを設定してください。')
            values[device] = str(path)
        if variable := row.get('requires_file'):
            path = Path(str(self.variables.get(variable, ''))).expanduser()
            if not path.is_absolute() or not path.is_file():
                raise ValueError('先に「自動接近設定」でセンサー位置と距離を保存してください。')
            values[variable] = str(path)
        command = row['command']
        args = shlex.split(command) if isinstance(command, str) else command
        for name, value in values.items():
            args = [arg.replace('{' + name + '}', value) for arg in args]
        for parameter, variable in row.get('parameters', {}).items():
            args = [*args, '-p', f'{parameter}:={json.dumps(self.variables[variable])}']
        if variable := row.get('optional_params_file'):
            if value := self.variables.get(variable):
                path = Path(value).expanduser()
                if not path.is_file():
                    raise ValueError(f'{variable} のファイルがありません。')
                args = [*args, '--params-file', str(path)]
        for parameter, variable in row.get('optional_file_parameters', {}).items():
            if value := self.variables.get(variable):
                path = Path(value).expanduser()
                if not path.is_file():
                    raise ValueError(f'{variable} のファイルがありません。')
                args = [*args, '-p', f'{parameter}:={json.dumps(str(path.resolve()))}']
        return self.wrapped(args, row.get('setup', []))
