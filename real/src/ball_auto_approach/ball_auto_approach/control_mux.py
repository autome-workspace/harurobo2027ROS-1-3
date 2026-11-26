from geometry_msgs.msg import Twist


class ControlMux:
    """One output owner. A terminal cycle always sends zero before returning to manual."""

    def __init__(self, node):
        self.manual = node.create_publisher(Twist, '/cmd_vel_manual', 1)
        self.auto = node.create_publisher(Twist, '/cmd_vel_auto', 1)
        self.output = node.create_publisher(Twist, '/cmd_vel', 1)

    def publish(self, manual, auto, automatic, inhibit):
        self.manual.publish(manual)
        self.auto.publish(auto)
        selected = auto if automatic else manual
        self.output.publish(Twist() if inhibit else selected)
