"""Quantized 2-D plant with wheel lag, cross-coupling and side disturbances."""
from pathlib import Path
import math
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.encoder_distance import DistanceMove,CIRCUMFERENCE

def encode(x,y):
    return {1:{'relative_counts':round(y/CIRCUMFERENCE*65536),'segment':1},
            2:{'relative_counts':round(-x/CIRCUMFERENCE*65536),'segment':1}}

for direction in ((1,0),(-1,0),(0,1),(0,-1)):
 dx,dy=direction
 for bias in (-.15,.15):
  for impulse in (0.,.02):
   m=DistanceMove(direction,0.,0.,encode(0.,0.),{'distance_m':.5,'forward_sign':-1,'left_sign':1})
   x=y=vx=vy=0.;previous=(0.,0.);correcting=False
   for i in range(1,2500):
    if i==80:x-=dy*impulse;y+=dx*impulse
    command,reason=m.command(encode(x,y),0.,0.,i*.01)
    if reason:break
    ux,uy,_=command
    assert math.hypot(ux,uy)<=.25+1e-9
    assert math.hypot(ux-previous[0],uy-previous[1])<=.004+1e-9
    lateral=-dy*ux+dx*uy;cross=-dy*x+dx*y
    assert abs(lateral)<=.08+1e-9
    correcting=correcting or lateral*cross<0
    previous=(ux,uy)
    along=dx*ux+dy*uy
    vx+=.01/.11*(ux-dy*bias*along-vx)
    vy+=.01/.11*(uy+dx*bias*along-vy)
    x+=vx*.01;y+=vy*.01
   else:raise AssertionError('failed to converge')
   assert reason.startswith('complete'),(direction,bias,impulse,reason)
   assert correcting and abs(-dy*x+dx*y)<=.0051
   for _ in range(100):
    vx+=.01/.11*(-vx);vy+=.01/.11*(-vy);x+=vx*.01;y+=vy*.01
   assert abs(dx*x+dy*y-.5)<.01 and abs(-dy*x+dx*y)<.01

 # When forward distance is reached, finish lateral alignment before completion.
 m=DistanceMove(direction,0.,0.,encode(0.,0.),{'distance_m':.5,'forward_sign':-1,'left_sign':1})
 x=dx*.5-dy*.02;y=dy*.5+dx*.02
 command,reason=m.command(encode(x,y),0.,0.,.1)
 assert reason is None and -dy*command[0]+dx*command[1]<0
 # Hard deviation limit remains active despite enabled correction.
 x=dx*.1-dy*.06;y=dy*.1+dx*.06
 assert '横ずれ超過' in m.command(encode(x,y),0.,0.,.2)[1]

print('PASS: 16 four-direction lateral-PD simulations with drift and impulses, vector speed/acceleration limits, final alignment and hard deviation stop; no motors')
