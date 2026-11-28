"""SLCAN setup uses only the dedicated encoder adapter, with fake tools."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6.QtWidgets import QApplication
from core.can_manager import CanManager
app=QApplication([])

def wait(test):
    end=time.monotonic()+5
    while not test():
        app.processEvents()
        if time.monotonic()>end:raise AssertionError('timeout')
        time.sleep(.01)

with tempfile.TemporaryDirectory() as folder:
    folder=Path(folder);state=folder/'state.json';state.write_text('[]');record=folder/'calls.json';record.write_text('[]')
    ip=folder/'ip';ip.write_text('#!/usr/bin/python3\nfrom pathlib import Path\nprint(Path('+repr(str(state))+').read_text())\n');ip.chmod(0o755)
    slcand=folder/'slcand';slcand.touch()
    pkexec=folder/'pkexec';pkexec.write_text('''#!/usr/bin/python3
from pathlib import Path
import json,sys
args=sys.argv[1:]
assert args.pop(0)=='--disable-internal-agent'
assert 'can0' not in args
path=Path(STATE);rec=Path(RECORD);calls=json.loads(rec.read_text())
if args[0]==SLCAND:
    assert args[1:]==['-o','-c','-s8','/dev/null','slcan0']
    calls.append('attach')
    links=[{'ifname':'slcan0','flags':[],'link_type':'can','linkinfo':{'info_kind':'can','info_data':{'state':'STOPPED'}}}]
else:
    assert args==[IP,'link','set','dev','slcan0','up']
    calls.append('up');links=json.loads(path.read_text());links[0]['flags']=['UP'];links[0]['linkinfo']['info_data']['state']='ERROR-ACTIVE'
path.write_text(json.dumps(links));rec.write_text(json.dumps(calls))
'''.replace('STATE',repr(str(state))).replace('RECORD',repr(str(record))).replace('SLCAND',repr(str(slcand))).replace('IP',repr(str(ip))));pkexec.chmod(0o755)
    manager=CanManager(interface='slcan0',serial_port='/dev/null')
    manager.timer.stop();wait(lambda:manager.probe.state()==manager.probe.ProcessState.NotRunning)
    manager.ip=str(ip);manager.slcand=str(slcand);manager.pkexec=str(pkexec)
    manager.inspect();wait(lambda:not manager.link and manager.probe.state()==manager.probe.ProcessState.NotRunning)
    assert manager.can_activate();manager.activate()
    wait(lambda:manager.is_up() and not manager.running())
    assert json.loads(record.read_text())==['attach','up']
    links=json.loads(state.read_text());links[0]['flags']=[];state.write_text(json.dumps(links))
    manager.inspect();wait(lambda:not manager.is_up())
    manager.activate();wait(lambda:manager.is_up() and not manager.running())
    assert json.loads(record.read_text())==['attach','up','up']
    manager.close();wait(manager.idle)
print('PASS: encoder SLCAN attachment and activation, no motor can0 commands, no duplicate daemon on re-enable')
