#!/usr/bin/env bash
# Start the continuous manual drive stack: joy + watchdog + manual_drive.
# Safe stop: scripts/stop_manual_drive.sh (SIGINT only; never SIGKILL).
set -eo pipefail  # no -u: ROS setup.bash uses unbound vars
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "== checking existing stack =="
if pgrep -f '^python3 -u src/harurobo_base/test/manual_drive[.]py([[:space:]]|$)' >/dev/null; then
  echo "manual_drive is already running:"
  pgrep -af "manual_driv[e]|watchdog_sto[p]|game_controller_nod[e]" || true
  exit 0
fi

echo "== checking can0 =="
if ! ip -details link show dev can0 2>/dev/null | grep -q "ONE-SHOT> state ERROR-ACTIVE"; then
  echo "can0 is not in the required state (UP, ERROR-ACTIVE, ONE-SHOT, 1 Mbit/s)."
  echo "Run on this PC:"
  echo "  sudo ip link set dev can0 down"
  echo "  sudo ip link set dev can0 up type can bitrate 1000000 one-shot on restart-ms 0"
  exit 1
fi

echo "== starting joy (node name MUST match the params section) =="
setsid nohup ros2 run joy game_controller_node --ros-args \
  -r __node:=game_controller \
  --params-file install/harurobo_teleop/share/harurobo_teleop/config/stage_a.yaml \
  > log/startup-joy.log 2>&1 < /dev/null &
sleep 6
if ! timeout 6 ros2 topic echo /joy --once >/dev/null 2>&1; then
  echo "WARNING: no live /joy data. If the DS4 was just replugged, press the PS button once."
fi

echo "== starting watchdog =="
setsid nohup python3 -u src/harurobo_base/test/watchdog_stop.py \
  > log/startup-watchdog.log 2>&1 < /dev/null &
sleep 1

echo "== starting manual_drive =="
setsid nohup python3 -u src/harurobo_base/test/manual_drive.py \
  --execute-powered-robot-estop-ready \
  > log/startup-drive.log 2>&1 < /dev/null &
sleep 4
tail -n 1 log/startup-drive.log

echo
echo "Stack is up. L1=deadman (release to stop). Re-arm: neutral + release L1, then press."
echo "Stop with: scripts/stop_manual_drive.sh"
pgrep -af "manual_driv[e]|watchdog_sto[p]|game_controller_nod[e]" || true
