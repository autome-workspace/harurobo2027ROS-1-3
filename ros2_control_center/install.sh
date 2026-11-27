#!/usr/bin/env bash
set -euo pipefail
app_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
data_dir=${XDG_DATA_HOME:-"$HOME/.local/share"}
runtime="$data_dir/ros2-control-center"
applications="$data_dir/applications"
desktop_file="$applications/ros2-control-center.desktop"
icon_dir="$data_dir/icons/hicolor/scalable/apps"
mkdir -p "$runtime" "$applications" "$icon_dir"
if [[ ! -x "$runtime/venv/bin/python" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$runtime/venv"
fi
if ! "$runtime/venv/bin/python" -c 'import PySide6.QtWidgets, yaml' 2>/dev/null; then
    "$runtime/venv/bin/python" -m pip install -r "$app_dir/requirements.txt"
fi
/usr/bin/python3 - "$runtime" "$app_dir" "$desktop_file" <<'PY'
from pathlib import Path
import shlex
import sys

runtime, app, desktop_file = map(Path, sys.argv[1:])
launcher = runtime / 'launch'
launcher.write_text('#!/bin/sh\nexec ' + shlex.join([
    str(runtime / 'venv/bin/python'), str(app / 'main.py')
]) + ' "$@"\n')
launcher.chmod(0o755)

def desktop_escape(path):
    value = str(path)
    value = ''.join('\\' + char if char in '\\"$`' else char for char in value)
    return '"' + value.replace('\\', '\\\\').replace('%', '%%') + '"'

template = (app / 'ros2-control-center.desktop').read_text()
desktop_file.write_text(template.replace('Exec=ros2-control-center',
                                         'Exec=' + desktop_escape(launcher)))
PY
install -m644 "$app_dir/resources/icon.svg" "$icon_dir/ros2-control-center.svg"
if [[ ${1:-} == --desktop ]]; then
    desktop_dir=$(xdg-user-dir DESKTOP)
    mkdir -p "$desktop_dir"
    install -m755 "$desktop_file" "$desktop_dir/ros2-control-center.desktop"
    if command -v gio >/dev/null; then
        gio set "$desktop_dir/ros2-control-center.desktop" metadata::trusted true || true
    fi
fi
if command -v update-desktop-database >/dev/null; then
    update-desktop-database "$applications"
fi
printf '登録しました: %s\n' "$desktop_file"
