"""Visual rotation indicators in a display-only frame, independent of robot TF."""
import json
import math
import signal
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'amt102_view'
NAMESPACES = ('ring', 'angle', 'value')


def markers(state, now_ns):
    output = MarkerArray()
    active = set()
    for row in state.get('channels', []):
        channel = row['channel']
        age = (now_ns-int(row.get('ros_stamp_ns',0)))/1e9
        angle = float(row['angle_deg'])
        turns = float(row['relative_turns'])
        if (not state.get('can_connected') or not row.get('fresh') or type(channel) is not int
                or not 0<=channel<=7 or not -.05<=age<.5
                or not math.isfinite(angle) or not math.isfinite(turns)):
            continue
        active.add(channel)
        # Layout coordinates are only for this display, not sensor mount positions.
        x,y = (channel%4-1.5)*1.4, -(channel//4)*1.5
        def marker(namespace, kind):
            m=Marker()
            m.header.frame_id=FRAME
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(now_ns,10**9)
            m.ns=namespace;m.id=channel;m.type=kind;m.action=Marker.ADD
            m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.w=1.
            m.color.a=1.;m.lifetime=Duration(seconds=.6).to_msg()
            return m
        ring=marker('ring',Marker.LINE_STRIP)
        ring.scale.x=.012;ring.color.r=ring.color.g=ring.color.b=.4
        ring.points=[Point(x=.5*math.cos(i*math.tau/64),y=.5*math.sin(i*math.tau/64),z=0.) for i in range(65)]
        arrow=marker('angle',Marker.ARROW)
        arrow.pose.orientation.z=math.sin(math.radians(angle)/2)
        arrow.pose.orientation.w=math.cos(math.radians(angle)/2)
        arrow.scale.x=.5;arrow.scale.y=.06;arrow.scale.z=.08
        if row.get('fault_level')=='warning':
            arrow.color.r=1.;arrow.color.g=.65;arrow.color.b=.1
        elif channel%2:
            arrow.color.r=.1;arrow.color.g=.8;arrow.color.b=1.
        else:
            arrow.color.r=.3;arrow.color.g=1.;arrow.color.b=.4
        label=marker('value',Marker.TEXT_VIEW_FACING)
        label.pose.position.y-=.7;label.pose.position.z=.08
        label.scale.z=.13;label.color.r=label.color.g=label.color.b=1.
        label.text=f'CH {channel}\n{angle:.2f} deg\n{turns:+.4f} rev'
        output.markers.extend([ring,arrow,label])
    return output,active


def deletes(channels):
    output=[]
    for channel in sorted(channels):
        for namespace in NAMESPACES:
            m=Marker();m.header.frame_id=FRAME;m.ns=namespace;m.id=channel;m.action=Marker.DELETE
            output.append(m)
    return output


class EncoderVisualizer(Node):
    def __init__(self):
        super().__init__('amt102_encoder_view')
        self.active=set()
        self.received_at=0.
        self.publisher=self.create_publisher(MarkerArray,'/encoders/markers',1)
        self.display_tf=StaticTransformBroadcaster(self)
        reference=TransformStamped()
        reference.header.stamp=self.get_clock().now().to_msg()
        reference.header.frame_id='amt102_visualization'
        reference.child_frame_id=FRAME
        reference.transform.rotation.w=1.
        self.display_tf.sendTransform(reference)
        self.create_subscription(String,'/encoders/state',self.received,qos_profile_sensor_data)
        self.create_timer(.1,self.expire)

    def received(self,msg):
        try:
            state=json.loads(msg.data)
            output,current=markers(state,self.get_clock().now().nanoseconds)
            output.markers.extend(deletes(self.active-current))
            self.publisher.publish(output)
            self.active=current
            self.received_at=time.monotonic()
        except (ValueError,TypeError,KeyError,AttributeError) as error:
            self.get_logger().warning(f'エンコーダー表示データを読めません：{error}',throttle_duration_sec=2.)

    def expire(self):
        if self.active and time.monotonic()-self.received_at>=.5:
            self.publisher.publish(MarkerArray(markers=deletes(self.active)))
            self.active.clear()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stopping.__setitem__(0,True))
    node=EncoderVisualizer()
    try:
        while rclpy.ok() and not stopping[0]:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.publisher.publish(MarkerArray(markers=deletes(node.active)))
        node.destroy_node();rclpy.try_shutdown()
