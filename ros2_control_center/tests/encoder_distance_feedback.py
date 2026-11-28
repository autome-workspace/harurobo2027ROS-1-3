"""Verify feedback interlocks with a fake driver: no ROS node or CAN opened."""
import os
from pathlib import Path
import runpy
import sys
import tempfile
import threading
from types import SimpleNamespace,ModuleType
import yaml

core=Path(__file__).resolve().parents[1]/'core'
sys.path.insert(0,str(core))
fake=ModuleType('manual_drive')
class Base:
    def finish(self,reason):
        self.reason=reason;self.move=self.request=None
        self.gate.armed_at=None;self.gate.wheels=(0.,)*4
fake.DpadDrive=Base
fake.run=lambda:None
fake.mix=lambda *args:args
fake.FORWARD_METRES_PER_RAD=.110;fake.LEFT_METRES_PER_RAD=.097;fake.WHEEL_CAP=.5
sys.modules['manual_drive']=fake
with tempfile.TemporaryDirectory() as directory:
    os.environ['XDG_CONFIG_HOME']=directory
    settings=Path(directory)/'ros2-control-center/encoder_distance.yaml'
    settings.parent.mkdir()
    settings.write_text(yaml.safe_dump({'distance_m':.1,'forward_sign':1,'left_sign':1}))
    cls=runpy.run_path(str(core/'encoder_distance_drive.py'))['EncoderDrive']
    def instance():
        obj=cls.__new__(cls)
        obj.cancelled=False;obj.move=None;obj.request=(1,0);obj.imu=(10.,0.,0.)
        obj.gate=SimpleNamespace(armed_at=9.,wheels=(0.,)*4)
        obj.lock=threading.Lock();obj.received=10.
        obj.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
        obj.data={'stream_id':'first','boot_notifications':0,'can_connected':True,'channels':[
            {'channel':ch,'relative_counts':0,'segment':1,'ros_stamp_ns':10_000_000_000,'fresh':True,'fault':False}
            for ch in (1,2)]}
        return obj
    o=instance();assert o.command(10.)[0]==0 and o.move is not None
    assert o.command(10.01)[0]>0
    o.data['stream_id']='restarted'
    assert o.command(10.)==(0.,)*4 and 'restarted' in o.reason
    for change in (
        lambda o:setattr(o,'received',9.),
        lambda o:o.data['channels'][0].update(ros_stamp_ns=9_000_000_000),
        lambda o:o.data['channels'][0].update(fault=True),
        lambda o:o.data['channels'][0].update(relative_counts='invalid'),
        lambda o:o.data.update(can_connected=False),
    ):
        o=instance();change(o)
        assert o.command(10.)==(0.,)*4 and o.gate.armed_at is None
    o=instance();o.data['channels'][0].update(fault=True,fault_detail=1,fault_level='info')
    assert o.command(10.)[0]==0 and o.move is not None
    assert o.command(10.01)[0]>0
print('PASS: feedback freshness, malformed data, faults, CAN disconnect and reader restart interlocks; no motors')
