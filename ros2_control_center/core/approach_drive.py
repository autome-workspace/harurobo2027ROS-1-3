"""Use the existing freshness-checked Twist drive on a dedicated command topic."""
from pathlib import Path
import sys

scripts = Path(__file__).resolve().parents[2] / 'real/manual_drive/src/harurobo_base/test'
sys.path.insert(0, str(scripts))
from manual_drive import run

if __name__ == '__main__':
    # run() initializes rclpy and consumes these ROS remapping arguments.
    sys.argv = [sys.argv[0], '--ros-args', '-r', '/cmd_vel:=/control_center/approach_cmd_vel']
    run(use_twist=True)
