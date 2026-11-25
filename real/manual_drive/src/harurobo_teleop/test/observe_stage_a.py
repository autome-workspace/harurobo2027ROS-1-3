"""Log DS4 acceptance-test transitions; timestamps use Linux monotonic time."""
import json
import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node('observe_stage_a')
    previous = {}
    last_joy = None
    silent = False

    def log(event, **fields):
        print(json.dumps(dict(t=round(time.monotonic(), 6), event=event, **fields)), flush=True)

    def transition(event, value):
        if previous.get(event) != value:
            previous[event] = value
            log(event, value=value, last_joy=last_joy)

    def on_joy(msg):
        nonlocal last_joy, silent
        last_joy = time.monotonic()
        if silent:
            log('joy_resumed')
            silent = False
        transition('joy', {'axes': [round(x, 1) for x in msg.axes[:3]],
                           'l1': msg.buttons[9] if len(msg.buttons) > 9 else None})

    def on_cmd(msg):
        values = msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z
        transition('cmd', [round(value, 3) for value in values])
        if math.hypot(*values[:2]) > 0.250001:
            log('linear_limit_exceeded', values=values)

    def check_silence():
        nonlocal silent
        if last_joy is not None and time.monotonic() - last_joy >= 0.10 and not silent:
            silent = True
            log('joy_silent', last_joy=last_joy)

    node.create_subscription(Joy, '/joy', on_joy, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, '/cmd_vel/manual', on_cmd, 1)
    node.create_subscription(Bool, '/manual/armed', lambda msg: transition('armed', msg.data), 1)
    node.create_timer(0.01, check_silence)
    log('capture_started')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
