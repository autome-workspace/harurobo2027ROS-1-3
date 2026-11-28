import math
from amt102_can.encoder_visualizer import markers,deletes
from visualization_msgs.msg import Marker


def test_separate_rotations_and_stale_data_removal():
    now=100_000_000_000
    state={'can_connected':True,'channels':[
        {'channel':1,'fresh':True,'ros_stamp_ns':now,'angle_deg':90.,'relative_turns':.25},
        {'channel':2,'fresh':True,'ros_stamp_ns':now,'angle_deg':180.,'relative_turns':.5}]}
    output,active=markers(state,now)
    assert active=={1,2}
    arrows={m.id:m for m in output.markers if m.ns=='angle'}
    assert math.isclose(arrows[1].pose.orientation.z,math.sin(math.pi/4))
    assert math.isclose(arrows[1].pose.orientation.w,math.cos(math.pi/4))
    assert math.isclose(arrows[2].pose.orientation.z,1.)
    assert arrows[1].pose.position.x!=arrows[2].pose.position.x
    assert all(m.header.frame_id=='amt102_view' and m.lifetime.nanosec==600_000_000 for m in output.markers)
    assert not markers(state,now+600_000_000)[1]
    state['can_connected']=False
    assert not markers(state,now)[1]
    removed=deletes(active)
    assert len(removed)==6 and all(m.action==Marker.DELETE for m in removed)
