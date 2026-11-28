"""Replay controller frames through the actual distance-move L1 gate; no CAN."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'real/manual_drive/src/harurobo_base/test'))
from manual_drive import FourContinuous,DpadDrive

def armed_move():
    gate=FourContinuous();drive=DpadDrive(gate)
    axes=[0.]*3;buttons=[0]*15
    drive.on_joy(axes,buttons,0.)
    buttons[9]=1;drive.on_joy(axes,buttons,.01)
    buttons[11]=1;drive.on_joy(axes,buttons,.02)
    assert drive.request==(1,0)
    drive.move=object();drive.request=None
    return gate,drive,axes,buttons

gate,drive,axes,buttons=armed_move()
buttons[9]=0;drive.on_joy(axes,buttons,.03)
assert gate.armed_at is not None and not drive.cancelled
buttons[9]=1;drive.on_joy(axes,buttons,.04)
assert gate.armed_at is not None and not drive.cancelled
buttons[9]=0
for t in (.05,.08,.11,.14):
    drive.on_joy(axes,buttons,t)
    assert gate.armed_at is not None and not drive.cancelled
drive.on_joy(axes,buttons,.16)
assert gate.armed_at is None and drive.cancelled and 'L1 released' in drive.cancel_reason
assert drive.command(.16)==(0.,)*4 and drive.move is None

gate,drive,axes,buttons=armed_move()
gate.expire(.18)
assert gate.armed_at is None and 'timeout' in gate.last_stop_reason
assert drive.command(.18)==(0.,)*4 and drive.move is None
drive.on_joy(axes,buttons,.19)
assert gate.armed_at is None  # Held L1 must not restart after timeout.

for event,text in [('stick','stick moved'),('dpad','multiple D-pad'),('invalid','invalid controller')]:
    gate,drive,axes,buttons=armed_move()
    if event=='stick':axes[0]=.2
    elif event=='dpad':buttons[12]=1
    else:axes[0]=float('nan')
    drive.on_joy(axes,buttons,.03)
    assert gate.armed_at is None and drive.cancelled and text in drive.cancel_reason
print('PASS: transient L1 release preserved, confirmed release/150ms timeout stop, re-arm required, stick/multiple-key/invalid-input cancellation; no CAN')
