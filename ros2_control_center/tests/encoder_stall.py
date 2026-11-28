"""Regression for terminal stiction; preserve true-stall and timeout interlocks."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.encoder_distance import DistanceMove,CIRCUMFERENCE,validate

def rows():return {c:{'relative_counts':0,'segment':1} for c in (1,2)}
base={'distance_m':.5,'forward_sign':-1,'left_sign':1}
for direction in ((1,0),(-1,0),(0,1),(0,-1)):
 for minimum in (0.,.03):
  r=rows();m=DistanceMove(direction,0.,0.,r,base|{'min_speed_mps':minimum})
  travel=velocity=previous=0.
  for tick in range(1,6000):
   t=tick*.01
   r[2]['relative_counts']=round(-direction[0]*travel/CIRCUMFERENCE*65536)
   r[1]['relative_counts']=round(direction[1]*travel/CIRCUMFERENCE*65536)
   command,reason=m.command(r,0.,0.,t)
   if reason:break
   speed=command[0]*direction[0]+command[1]*direction[1]
   assert 0<=speed<=.25 and abs(speed-previous)<=.004+1e-9
   previous=speed
   # Hypothetical plant: commands under 25mm/s cannot overcome friction.
   velocity+=.01/(.1+.01)*((speed if speed>=.025 else 0.)-velocity)
   travel+=velocity*.01
  else:raise AssertionError('did not finish')
  if minimum:
   assert reason.startswith('complete'),reason
   assert abs(travel-.5)<.01
  else:
   assert 'encoder stall' in reason and 'remaining=' in reason,reason

r=rows();m=DistanceMove((1,0),0.,0.,r,base)
for i in range(1,103):
 command,reason=m.command(r,0.,0.,i*.01)
 if reason:break
assert 'encoder stall' in reason  # Blocked/disconnected wheel must not creep forever.

r=rows();m=DistanceMove((1,0),0.,0.,r,base)
for i in range(1,570):
 t=i*.1;r[2]['relative_counts']=round(-.002*t/CIRCUMFERENCE*65536)
 command,reason=m.command(r,0.,0.,t)
 if reason:break
assert 'time limit' in reason and 'encoder stall' not in reason,reason
try:validate(base|{'max_speed_mps':.03,'min_speed_mps':.04})
except ValueError:pass
else:raise AssertionError('minimum above maximum accepted')
print('PASS: terminal-stiction reproduced and prevented in four directions; blocked wheel still stops; timeout distinguished; no motors')
