"""Read-only diagnosis of the first physical run; does not change controllers."""
from pathlib import Path
from zipfile import ZipFile
from collections import Counter
from unittest.mock import patch
import csv
import io
import importlib.util
import json
import os
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
sys.path.insert(0,str(ROOT/'environment'))
from vehicle_io import VehicleState


def main():
    raw=ROOT/'data/raw/group1_deployment_error_20261020_135916'
    read=lambda name:list(csv.DictReader(io.StringIO((raw/name).read_text(encoding='utf-8-sig'))))
    commands=read('commands.csv');trajectory=read('trajectory.csv')
    metadata=json.loads((raw/'metadata.json').read_text(encoding='utf-8'))
    out=ROOT/'diagnostics/experiment2_20261020_135916';out.mkdir(parents=True,exist_ok=True)
    student=[r for r in commands if r['role']=='student']
    active=[r for r in student if r['mode']=='student_straight']
    bypass=[r for r in student if r['mode']=='leader_follow_noncontrol']
    # Approximate input reconstruction: latest logged packet received before command log time.
    # Callback input snapshots and exact callback-entry time are not stored by the TA logger.
    spec=importlib.util.spec_from_file_location('pid_diagnostic_replay',ROOT/'controllers/pid/scale_2p8/student_controller2.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ordered=sorted(trajectory,key=lambda r:float(r['recv_time']))
    pointer=0;latest={};history=[];replay=[]
    for row in student:
        wall=float(row['time']);t=float(row['t_from_start'])
        while pointer<len(ordered) and float(ordered[pointer]['recv_time'])<=wall:
            packet=ordered[pointer];pointer+=1
            role=packet['role']
            latest[role]=VehicleState(role,packet['host'],float(packet['recv_time']),int(packet['seq']),json.loads(packet['raw_json']))
            hist=dict(packet)
            for field in ('t_from_start','velocity','x','y','heading','s'):hist[field]=float(hist[field])
            history.append(hist)
        if row['mode']!='student_straight':continue
        with patch('time.time',return_value=wall),patch('time.monotonic',return_value=t):
            predicted=module.student_control(latest['leader'],latest['middle'],latest['student'],history)
        replay.append(dict(t=t,logged_desired=float(row['desired_velocity']),
            replay_desired=predicted,spacing=float(row['spacing']),
            max_state_age=max(wall-s.recv_time for s in latest.values())))
    with (out/'approximate_pid_replay.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(replay[0]));w.writeheader();w.writerows(replay)
    motion={}
    for role in ('leader','middle','student'):
        rows=sorted([r for r in trajectory if r['role']==role],key=lambda r:float(r['recv_time']))
        t=np.array([float(r['t_from_start']) for r in rows])
        s=np.unwrap(np.array([float(r['s']) for r in rows])*2*np.pi/metadata['route_loop_length'])*metadata['route_loop_length']/(2*np.pi)
        v=np.array([float(r['velocity']) for r in rows])
        motion[role]=(t,s,v)
    # Ratios on non-overlapping, near-constant reported-speed windows, not instantaneous differences.
    ratios={}
    for role,(t,s,v) in motion.items():
        bins=[]
        for start in np.arange(35,160,5):
            idx=np.flatnonzero((t>=start)&(t<start+5))
            if len(idx)<10 or np.ptp(v[idx])>2 or np.mean(v[idx])<30:continue
            coord_speed=(s[idx[-1]]-s[idx[0]])/(t[idx[-1]]-t[idx[0]])
            bins.append(coord_speed/np.mean(v[idx]))
        ratios[role]=dict(windows=len(bins),median=float(np.median(bins)),
            minimum=float(min(bins)),maximum=float(max(bins)))
    summary=dict(duration=float(student[-1]['t_from_start']),commands_per_role=len(student),
        active_calls=len(active),bypass_calls=len(bypass),active_desired_values=sorted(set(float(r['desired_velocity']) for r in active)),
        active_first=float(active[0]['t_from_start']),active_last=float(active[-1]['t_from_start']),
        first_bypass_time=float(bypass[0]['t_from_start']),first_bypass_spacing=float(bypass[0]['spacing']),
        initial_spacing=float(student[0]['spacing']),final_spacing=float(student[-1]['spacing']),
        student_spacing_emergencies=sum(r['emergency']=='1' for r in student),
        dt_median=float(np.median([float(r['dt']) for r in student])),
        dt_max=float(max(float(r['dt']) for r in student)),
        student_stationary_s_range_first_30=float(np.ptp(motion['student'][1][(motion['student'][0]>=0)&(motion['student'][0]<30)])),
        student_reported_speed_max_first_30=float(max(motion['student'][2][(motion['student'][0]>=0)&(motion['student'][0]<30)])),
        steady_speed_ratios=ratios,replay_desired_range=[min(r['replay_desired'] for r in replay),max(r['replay_desired'] for r in replay)],
        replay_max_state_age=max(r['max_state_age'] for r in replay))
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
    ct=np.array([float(r['t_from_start']) for r in student])
    axes[0].plot(ct,[float(r['spacing']) for r in student],label='Logged student bumper gap')
    axes[0].axhline(80,color='red',ls=':',label='TA emergency threshold')
    axes[0].set_ylabel('Gap (coordinate units)');axes[0].legend()
    axes[1].plot(ct,[float(r['v_cmd']) for r in student],label='Logged student command (pre integer rounding)')
    axes[1].step(ct,[round(float(r['v_cmd'])) for r in student],where='post',label='Integer sent by supplied sender',alpha=.7)
    axes[1].set_ylabel('Speed command');axes[1].legend()
    for role,(t,s,v) in motion.items():
        axes[2].plot(t,v,label=role)
    axes[2].set_ylabel('Reported velocity');axes[2].legend(ncol=3)
    axes[3].plot([r['t'] for r in replay],[r['replay_desired'] for r in replay],label='Approximate replay: current PID 2.8 file')
    axes[3].plot([r['t'] for r in replay],[r['logged_desired'] for r in replay],label='Logged active-controller return')
    axes[3].set_ylabel('Desired command');axes[3].legend();axes[3].set_xlabel('Time from run start (s)')
    for ax in axes:
        ax.axvspan(0,float(bypass[0]['t_from_start']),alpha=.08,color='red')
        ax.grid(alpha=.2)
    fig.suptitle('Diagnostic run: active control returns 0.09; bypass restores motion\nReplay is approximate, not proof of deployed-source identity')
    fig.tight_layout();fig.savefig(out/'diagnosis.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
