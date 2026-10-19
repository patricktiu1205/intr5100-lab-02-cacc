"""Load each standalone submission in a copy of the actual TA GUI environment.

Runs Qt offscreen, with synthetic telemetry and in-memory command sinks only.
No vehicle clients are constructed and no network connections are opened.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SHARED = ROOT.parent/'Shared Environment'

CHILD = r'''
import json, math, sys, time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
from PyQt6.QtWidgets import QApplication, QMessageBox
import student_controller2 as submission
from ta_experiment2_gui import Experiment2GUI
from vehicle_io import VehicleState
from idm_controller import IDMController

def xy(route,s):
    s %= route.loop_length
    i = max(i for i,p in enumerate(route.points) if p.s<=s)
    p,q=route.points[i],route.points[(i+1)%len(route.points)]
    a=(s-p.s)/route.segment_length(i)
    return p.x+a*(q.x-p.x), p.y+a*(q.y-p.y)

class Sink:
    def __init__(self): self.commands=[]
    def set_velocity(self,v):
        assert isinstance(v,int) and 0<=v<=100
        self.commands.append(v)
    def close(self): pass

app=QApplication([])
messages=[]
with patch.object(QMessageBox,'warning',side_effect=lambda *a: messages.append(str(a[-1]))), \
     patch.object(QMessageBox,'information',return_value=None):
    w=Experiment2GUI(submission.student_control)
    w.ui_timer.stop()
    assert w.route_valid, messages
    assert len(w.control_sections)==3
    assert Path(submission.__file__).resolve().parent==Path(__file__).resolve().parent
    w.idm_controller=IDMController(w._idm_params())
    w.command_clients={r:Sink() for r in w.roles}
    states={}
    w.state_clients={r:SimpleNamespace(latest=lambda r=r:states[r], stop=lambda:None) for r in w.roles}
    w.recording=True
    w.run_start_wall=time.time()
    w.last_control_mono=None
    sec=w.control_sections[-1]
    clock=[100.]
    def telemetry(offsets,seq):
        for role,offset in zip(w.roles,offsets):
            x,y=xy(w.route,offset)
            states[role]=VehicleState(role,'synthetic',time.time(),seq,
                dict(x=x,y=y,heading=0.,velocity=40.,center_distance=1.))
    with patch('time.monotonic',side_effect=lambda:clock[0]):
        for n in range(30):
            clock[0]=100+n*.2
            telemetry([sec.start_s+1900+n*5,sec.start_s+1200+n*5,sec.start_s+500+n*5],n)
            w._record_states()
            w._control_step()
        assert w.recorder.command_rows[-1]['mode']=='student_straight'
        assert w.recorder.command_rows[-2]['mode']=='idm_straight'
        # Leader on a different straight: middle fallback, student still active.
        clock[0]+=1.
        telemetry([w.control_sections[0].start_s+300,sec.start_s+1200,sec.start_s+500],31)
        w._record_states();w._control_step()
        assert w.recorder.command_rows[-2]['mode']=='leader_follow_noncontrol'
        assert w.recorder.command_rows[-1]['mode']=='student_straight'
        # Student on another straight: actual TA bypass behavior.
        clock[0]+=1.
        telemetry([sec.start_s+1900,sec.start_s+1200,w.control_sections[0].start_s+300],32)
        w._record_states();w._control_step()
        assert w.recorder.command_rows[-1]['mode']=='leader_follow_noncontrol'
        # Re-entry and range-sensor stop through actual TA safety shield.
        clock[0]+=1.
        telemetry([sec.start_s+1900,sec.start_s+1200,sec.start_s+500],33)
        states['student'].data['center_distance']=.1
        w._record_states();w._control_step()
        row=w.recorder.command_rows[-1]
        assert row['mode']=='student_straight' and row['v_cmd']==0 and row['emergency']==1
        assert row['reason']=='center_sensor_emergency'
        # New run reuses the imported module, resetting history/controller state.
        clock[0]+=1.
        w.recorder.reset()
        telemetry([sec.start_s+1900,sec.start_s+1200,sec.start_s+500],34)
        w._record_states();w._control_step()
        assert w.recorder.command_rows[-1]['mode']=='student_straight'
        assert submission._controller.history_len==3
    assert 'student_fallback' not in [r['mode'] for r in w.recorder.command_rows]
    assert not messages,messages
    result=dict(scale=submission.GAMMA,commands=len(w.command_clients['student'].commands),
        route_valid=w.route_valid,checks=['TA import','Qt construction','real route load',
        'IDM/student active','middle fallback','student bypass','re-entry',
        'sensor shield','new-run reset'],module_path=str(submission.__file__))
    w.recording=False
    w.close()
    print(json.dumps(result))
'''


def fingerprints():
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SHARED.iterdir() if p.is_file()}


def main():
    before=fingerprints()
    results=[]
    scratch=ROOT/'.dropin_tmp'
    for version in ('unscaled','scaled_2p8'):
        folder=scratch/version
        folder.mkdir(parents=True,exist_ok=True)
        for p in SHARED.iterdir():
            if p.is_file(): shutil.copy2(p,folder/p.name)
        shutil.copy2(ROOT/'controllers'/version/'student_controller2.py',folder/'student_controller2.py')
        runner=folder/'_check_dropin.py'
        runner.write_text(CHILD,encoding='utf-8')
        env=os.environ.copy()
        env['QT_QPA_PLATFORM']='offscreen'
        env['MPLCONFIGDIR']=str(ROOT/'.mplconfig')
        run=subprocess.run([sys.executable,'-I',str(runner)],cwd=folder,env=env,
                           capture_output=True,text=True,timeout=60)
        if run.returncode:
            raise RuntimeError(f'{version}: {run.stdout}\n{run.stderr}')
        result=json.loads(run.stdout.strip().splitlines()[-1])
        result['version']=version
        results.append(result)
        print(json.dumps(result))
    assert before==fingerprints(),'Original Shared Environment changed'
    (ROOT/'simulation_results/dropin_checks.json').write_text(
        json.dumps(dict(shared_environment_unchanged=True,results=results),indent=2),encoding='utf-8')


if __name__=='__main__': main()
