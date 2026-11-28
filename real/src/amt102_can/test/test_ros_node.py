import json
import math
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from amt102_can.encoder_node import EncoderNode


def test_two_channel_publication_stale_and_disconnect(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','91')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    rclpy.init(args=[])
    node=EncoderNode()
    for timer in node.timers:timer.cancel()
    # No real socket and no CAN transmission in this test.
    node.receiver.socket=object()
    monkeypatch.setattr(node.receiver,'is_up',lambda:True)
    monkeypatch.setattr(node.receiver,'close',lambda:setattr(node.receiver,'socket',None))
    frames=[(0x360,bytes.fromhex('00 00 00 00 00 01')),
            (0x361,bytes.fromhex('01 00 80 00 80 02'))]
    monkeypatch.setattr(node.receiver,'receive',lambda:frames.copy())
    queries=[]
    monkeypatch.setattr(node.receiver,'request_fault',lambda data:queries.append(data))
    status=[];joints={0:[],1:[]}
    node.status=type('Capture',(),{'publish':lambda _,m:status.append(json.loads(m.data))})()
    node.joints={ch:type('Capture',(),{'publish':lambda _,m,ch=ch:joints[ch].append(m)})() for ch in (0,1)}
    try:
        node.poll();node.publish()
        assert status[-1]['received_channels']==2
        assert status[-1]['channels'][1]['fault']
        assert status[-1]['channels'][1]['angle_deg']==180
        assert len(joints[0])==len(joints[1])==1
        assert queries and queries[-1][0]==1
        seq=queries[-1][-1]
        frames.append((0x488,bytes([1,0,5,0,0,0,1,seq])))
        node.poll();node.publish()
        assert status[-1]['channels'][1]['fault_level']=='info'
        assert status[-1]['channels'][1]['fault_detail']==1
        frames.pop()
        old_stamp=joints[0][-1].header.stamp
        count=len(joints[0])
        node.publish()
        assert len(joints[0])==count  # no fresh timestamp on old values
        frames[0]=(0x360,bytes.fromhex('00 00 40 00 00 03'))
        node.poll();node.publish()
        assert math.isclose(joints[0][-1].position[0],math.pi/2)
        monkeypatch.setattr(node.receiver,'is_up',lambda:False)
        node.publish()
        assert not status[-1]['can_connected'] and not any(c['fresh'] for c in status[-1]['channels'])
        assert status[-1]['channels'][0]['rx_hz']==0
        assert len(joints[0])==count+1
    finally:
        node.destroy_node();rclpy.try_shutdown()
