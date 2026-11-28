"""Check the unified GUI and launch shutdown using fake drive processes only."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import yaml
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
APP = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP))
from PySide6.QtWidgets import QApplication
from core.config_manager import Config
from core.controller import Controller
from ui.main_window import MainWindow

app=QApplication([])
with tempfile.TemporaryDirectory() as directory:
    folder=Path(directory)
    parameters=yaml.safe_load((APP.parent/'real/src/ball_auto_approach/config/params.yaml').read_text())
    parameters['approach_controller']['ros__parameters'].update(auto_enabled=True,validated_min_range=.05)
    params=folder/'parameters.yaml';params.write_text(yaml.safe_dump(parameters))
    settings=folder/'approach.yaml'
    settings.write_text(yaml.safe_dump({'parameters_file':str(params),'geometry':{
        'lidar_position_m':[.18,0.,.08],'lidar_orientation_rad':[0.,0.,0.],
        'camera_position_m':[.18,0.,.3],'camera_pitch_rad':.4}}))
    local=folder/'local.yaml'
    local.write_text(yaml.safe_dump({'variables':{'CAMERA_PORT':'/dev/null','LIDAR_PORT':'/dev/null',
                                                 'APPROACH_SETTINGS':str(settings)}}))
    config=Config(local)
    assert config.order(['motor'],'real')==['joy','motor']
    assert config.order(['rviz_real'],'real')==['lidar','rviz_real']
    assert config.order(['rviz_sim'],'simulation')==['gazebo','rviz_sim']
    assert 'approach' not in config.programs and 'auto_drive' not in config.programs
    assert 'integrated_drive_session.py' not in ' '.join(config.command('motor'))
    controller=Controller(config)
    controller.timer.stop();controller.processes.timer.stop()
    controller.mode='real';controller.snapshot=time.monotonic()
    window=MainWindow(controller)
    window.motor_automatic.setChecked(True)
    assert config.order(['motor'],'real')==['joy','camera','lidar','motor']
    assert 'integrated_drive_session.py' in ' '.join(config.command('motor'))
    assert config.programs['motor']['requires_can']
    controller.processes.jobs['motor']=SimpleNamespace(stopping=0)
    controller.started['motor']=time.monotonic()
    window.refresh()
    assert not window.motor_automatic.isEnabled()
    assert not controller.set_motor_automatic(False)
    controller.processes.jobs.clear()
    window.motor_automatic.setChecked(False)
    assert config.order(['motor'],'real')==['joy','motor']
    assert not yaml.safe_load(local.read_text())['variables']['MOTOR_AUTO_APPROACH']
    assert 'approach' not in window.dashboard_rows and 'auto_drive' not in window.dashboard_rows
    controller.can.close();controller.encoder_can.close();window.close()
    while not controller.can.idle():app.processEvents()
    fake=folder/'fake_drive.py'
    fake.write_text("import signal,time,sys\nsignal.signal(signal.SIGINT,lambda *args:sys.exit(0))\nprint('READY: continuous drive. fake',flush=True)\ntime.sleep(30)\n")
    for critical_failure in (False,True,2):
        code=f'''
import sys
sys.path.insert(0,{str(APP/'core')!r})
from integrated_drive_session import description
from launch import LaunchService
from launch.actions import ExecuteProcess
command=[sys.executable,{str(fake)!r}] if {critical_failure!r} else [sys.executable,'-c',"print('READY: continuous drive. fake',flush=True);import time;time.sleep(1)"]
failures=[]
d=description({str(settings)!r},drive_command=command,failures=failures)
if {critical_failure!r}:d.add_action(ExecuteProcess(cmd=[sys.executable,'-c','import time,sys;time.sleep(1);sys.exit({int(critical_failure==2)})']))
s=LaunchService();s.include_launch_description(d);rc=s.run();sys.exit(1 if failures else rc)
'''
        env=os.environ|{'ROS_DOMAIN_ID':'95','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
        run=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,env=env,timeout=15)
        assert run.returncode==(1 if critical_failure==2 else 0),run.stdout+run.stderr
        assert any(line.startswith('READY: continuous drive.') for line in run.stdout.splitlines()),run.stdout
    rviz=yaml.safe_load((APP/'config/control_center.rviz').read_text())['Visualization Manager']
    scan=next(d for d in rviz['Displays'] if d['Class']=='rviz_default_plugins/LaserScan')
    assert rviz['Global Options']['Fixed Frame']=='laser'
    assert scan['Enabled'] and scan['Topic']['Value']=='/scan'
    assert scan['Topic']['Reliability Policy']=='Best Effort'
print('PASS: one drive entry, persisted auto toggle, running-mode lock, fake drive/critical exit shutdown, direct LaserScan RViz config')
