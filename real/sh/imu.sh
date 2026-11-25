#!/bin/bash
set -e

workspace_dir="$(cd -- "$(dirname -- "$(realpath -- "${BASH_SOURCE[0]}")")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"

exec ros2 launch dm_imu dm_rviz.launch.py "$@"
