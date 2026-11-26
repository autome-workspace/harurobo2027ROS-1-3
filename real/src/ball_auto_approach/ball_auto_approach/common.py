import signal

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def run(node_type):
    # Finish the current callback and leave the ROS context up for a final zero.
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    node = None
    try:
        node = node_type()
        executor.add_node(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.1)
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
