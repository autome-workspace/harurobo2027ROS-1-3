"""Known encoder displacements and aborts; never open CAN or motor processes."""
from pathlib import Path
import sys,math,copy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.encoder_distance import DistanceMove,validate,CIRCUMFERENCE
p={'distance_m':.1,'forward_sign':1,'left_sign':-1}
def rows():return {1:{'relative_counts':0,'segment':1},2:{'relative_counts':0,'segment':1}}
r=rows();m=DistanceMove((1,0),0.,0.,r,p)
v,reason=m.command(r,0.,0.,.1);assert not reason and 0<v[0]<=.25
r[2]['relative_counts']=round(.1/CIRCUMFERENCE*65536)
v,reason=m.command(r,0.,0.,.5);assert v is None and reason.startswith('complete')
r=rows();m=DistanceMove((0,1),0.,0.,r,p)
r[1]['relative_counts']=-round(.1/CIRCUMFERENCE*65536)
assert m.command(r,0.,0.,.5)[1].startswith('complete')
for mutate,heading,now,text in [
    (lambda r:r[1].update(segment=2),0.,.1,'reset'),
    (lambda r:r[2].update(relative_counts=-3000),0.,.1,'wrong direction'),
    (lambda r:None,.04,.1,'heading'),
    (lambda r:None,0.,1.1,'stall')]:
 r=rows();m=DistanceMove((1,0),0.,0.,r,p);mutate(r)
 assert text in m.command(r,heading,0.,now)[1]
try:validate(p|{'forward_sign':0})
except ValueError:pass
else:raise AssertionError('unconfirmed sign accepted')
for direction in ((1,0),(-1,0),(0,1),(0,-1)):
 dx,dy=direction
 for cross,limit,abort in ((.024,.05,False),(.04,.05,False),(-.04,.05,False),(.06,.05,True),(.04,.025,True)):
  r=rows();settings={'distance_m':.5,'forward_sign':-1,'left_sign':1,'cross_track_limit_m':limit}
  m=DistanceMove(direction,0.,0.,r,settings)
  # Transform along=20mm and signed lateral displacement to CH2=X / CH1=Y.
  x=dx*.02-dy*cross;y=dy*.02+dx*cross
  r[2]['relative_counts']=round(-x/CIRCUMFERENCE*65536)
  r[1]['relative_counts']=round(y/CIRCUMFERENCE*65536)
  command,reason=m.command(r,0.,0.,.1)
  assert (reason is not None)==abort,(direction,cross,limit,reason)
  if abort:assert '横ずれ超過' in reason and 'wrong direction' not in reason
 r=rows();m=DistanceMove(direction,0.,0.,r,settings|{'cross_track_limit_m':.1})
 r[2]['relative_counts']=round(dx*.006/CIRCUMFERENCE*65536)
 r[1]['relative_counts']=round(-dy*.006/CIRCUMFERENCE*65536)
 assert '逆方向移動' in m.command(r,0.,0.,.1)[1]
print('PASS: all four directions accept lateral drift within configured limit; cross-track and reverse stops remain distinct')
print('PASS: 50mm direct wheels, X=CH2/Y=CH1, distance completion, sign, reset, heading and stall aborts; no motors')
