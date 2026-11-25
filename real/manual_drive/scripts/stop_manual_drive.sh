#!/usr/bin/env bash
# Gracefully stop the manual drive stack: manual_drive (SIGINT) -> watchdog
# exits via the done handshake; joy is stopped last.
set -uo pipefail
drive_pid=$(pgrep -f "^python3 -u src/harurobo_base/test/manual_drive.py" | head -n 1 || true)
if [ -z "${drive_pid}" ]; then
  echo "manual_drive is not running."
else
  echo "Stopping manual_drive (pid ${drive_pid}) with SIGINT..."
  kill -INT "${drive_pid}"
  for _ in $(seq 1 10); do
    pgrep -f "^python3 -u src/harurobo_base/test/manual_drive.py" >/dev/null || break
    sleep 1
  done
  if pgrep -f "^python3 -u src/harurobo_base/test/manual_drive.py" >/dev/null; then
    echo "WARNING: manual_drive did not exit; motors may still need the estop/power cut."
    exit 1
  fi
fi
watchdog_pid=$(pgrep -f "^python3 -u src/harurobo_base/test/watchdog_stop.py" | head -n 1 || true)
if [ -n "${watchdog_pid}" ]; then
  sleep 2
  kill -INT "${watchdog_pid}" 2>/dev/null || true
  sleep 1
  kill -9 "${watchdog_pid}" 2>/dev/null || true
fi
pkill -INT -f "game_controller_nod[e]" 2>/dev/null || true
sleep 2
pkill -9 -f "game_controller_nod[e]" 2>/dev/null || true
echo "Stack stopped. Turn drive power OFF."
