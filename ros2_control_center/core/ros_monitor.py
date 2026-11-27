"""Run in a separate interpreter with the selected ROS environment."""
import json
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    wanted = set(sys.argv[1:])
    rclpy.init(args=[])
    node = rclpy.create_node('control_center_monitor')
    subscriptions = {}
    received = {}
    diagnostics = {}
    last_snapshot = 0.0
    details = {}

    def mark(topic, message):
        now = time.monotonic()
        received[topic] = now
        if topic == '/ball/state':
            details['state'] = (now, message.data)
        if topic == '/ball/estimate':
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            details['estimate'] = (now, {
                'valid': bool(message.valid and -.05 <= age < .3),
                'distance': message.surface_distance, 'lateral': message.center.y,
                'tracking': message.tracking,
            })
        if topic == '/diagnostics':
            for status in message.status:
                diagnostics[status.name] = (now, int.from_bytes(status.level), status.message)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_snapshot < 0.5:
                continue
            topics = dict(node.get_topic_names_and_types())
            for topic in wanted & topics.keys():
                types = topics[topic]
                if len(types) != 1 or topic in subscriptions:
                    continue
                try:
                    message = get_message(types[0])
                except (ImportError, AttributeError, ValueError):
                    continue
                subscriptions[topic] = node.create_subscription(
                    message, topic, lambda msg, name=topic: mark(name, msg), qos_profile_sensor_data)
            nodes = [namespace.rstrip('/') + '/' + name
                     for name, namespace in node.get_node_names_and_namespaces()]
            diagnostics = {name: entry for name, entry in diagnostics.items() if now - entry[0] < 3}
            snapshot = {
                'nodes': nodes,
                'approach': {name: value for name, (stamp, value) in details.items() if now - stamp < .6},
                'live': [topic for topic, stamp in received.items()
                         if now - stamp < 3 and node.count_publishers(topic)],
                'diagnostics': {name: entry[1:] for name, entry in diagnostics.items()},
            }
            print(json.dumps(snapshot), flush=True)
            last_snapshot = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
