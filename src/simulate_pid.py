"""Offline longitudinal sensitivity study; no network or physical vehicle access."""
from __future__ import annotations
import csv
import json
import math
import sys
import os
from pathlib import Path
from types import SimpleNamespace
from collections import deque
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
sys.path.insert(0,str(ROOT/'environment' if (ROOT/'environment').exists()
                      else ROOT.parent/'Shared Environment'))
from idm_controller import IDMController, IDMParams
from experiment_common import LeaderPerturbationManager
from pid_controller import PredictivePID, load_geometry


class StraightRoute:
    """Long straight isolates longitudinal control without section transitions."""
    loop_length = 1.e7
    def project_xy(self,x,y):
        return SimpleNamespace(s=x)
    def forward_gap(self,a,b):
        return (a-b)%self.loop_length


class StraightSection:
    def contains(self,s,length):
        return True


def xy(route,s):
    if isinstance(route,StraightRoute):
        return s,0.
    s %= route.loop_length
    i = max(0,np.searchsorted([p.s for p in route.points],s,side='right')-1)
    p,q = route.points[i],route.points[(i+1)%len(route.points)]
    ratio = (s-p.s)/route.segment_length(i)
    return p.x+ratio*(q.x-p.x),p.y+ratio*(q.y-p.y)


def run(scale=1.,plant_scale=1.,prediction='idm',lag=.5,delay=.0,
        noise=0.,route_case=False,initial_gap=420.,dt=.2,duration=100.,hard_brake=False,
        plant_beta=None):
    route,sections = load_geometry() if route_case else (StraightRoute(),[StraightSection()])
    ctrl = PredictivePID(scale,scale,route,sections,prediction)
    # Simulator uses TA IDMController, not the predictor's replica.
    idm = IDMController(IDMParams())
    manager = LeaderPerturbationManager()
    if route_case:
        manager.reset(sections,40.)
    x = np.array([4500.,4500.-700.,4500.-700.-250.-initial_gap])
    v = np.full(3,40.*plant_scale)
    command = np.full(3,40.)
    applied = command.copy()
    pending = deque()
    history, rows = [],[]
    rng = np.random.default_rng(5100)
    plant_beta = plant_scale if plant_beta is None else plant_beta
    emergency_count = 0
    forecast_checks=[]
    for n in range(round(duration/dt)):
        t = n*dt
        if route_case:
            command[0],_ = manager.step(now_mono=t,leader_s=x[0]%route.loop_length,
                observed_velocity=v[0]/plant_scale,base_speed=40.,loop_length=route.loop_length)
        else:
            # Includes held cruise, acceleration and braking; not the route schedule.
            command[0] = (40 if t<10 else 40+15*min((t-10)/8,1) if t<30
                          else 55-30*min((t-30)/8,1) if t<50
                          else 25+15*min((t-50)/8,1) if t<70 else 40)
            if hard_brake:
                command[0] = 40. if t<20 or t>=40 else 0.
        ss = x%route.loop_length
        obs=[]
        for i,role in enumerate(('leader','middle','student')):
            px,py = xy(route,ss[i])
            px += rng.normal(0,noise)
            py += rng.normal(0,noise)
            vr = max(0,v[i]/plant_scale+rng.normal(0,noise*.03))
            state = SimpleNamespace(x=px,y=py,heading=0.,velocity=vr)
            obs.append(state)
            history.append(dict(role=role,t_from_start=t,x=px,y=py,velocity=vr,s=ss[i]))
        gap01 = x[0]-x[1]-250
        gap12 = x[1]-x[2]-250
        middle_active = ctrl.same(ss[0],ss[1])
        if middle_active:
            desired = idm.next_velocity(obs[1].velocity,obs[0].velocity,gap01,dt,v_max=70.)
        else:
            desired = obs[0].velocity
            idm.last_cmd_v = desired
        command[1] = round(0 if gap01<80 else np.clip(desired,0,100))
        active = ctrl.same(ss[1],ss[2])
        if active:
            command[2] = ctrl.step(*obs,history,now=t)
            d=ctrl.diagnostics
            forecast_checks.append((n+round(ctrl.horizon/dt),d.get('predicted_middle',v[1])))
        else:
            command[2] = obs[0].velocity
            d={}
        emergency = gap12<80
        emergency_count += int(emergency)
        command[2] = round(0 if emergency else np.clip(command[2],0,100))
        rows.append(dict(t=t,v0=v[0],v1=v[1],v2=v[2],gap=gap12,
            error=gap12-ctrl.standstill-ctrl.headway*v[2],u2=command[2],
            active=int(active),emergency=int(emergency),protective=int(d.get('protective',False))))
        pending.append((t+delay,command.copy()))
        while pending and pending[0][0] <= t+1e-9:
            _,applied = pending.popleft()
        # Heterogeneous plant lag and acceleration limits differ from predictor.
        tau = np.array([lag*.8,lag*1.2,lag])
        acc = np.clip((plant_beta*applied-v)/tau,-60*plant_beta,35*plant_beta)
        nv = np.maximum(0,v+acc*dt)
        x += .5*(v+nv)*dt
        v=nv
    a=np.array([[r[k] for k in ('v0','v1','v2','gap','error','active')] for r in rows])
    mask=a[:,5]>0
    selected=a[mask]
    accel=np.diff(a[:,2])/dt
    jerk=np.diff(accel)/dt
    ferrors=[pred-a[idx,1] for idx,pred in forecast_checks if idx<len(a)]
    energies=[np.mean((selected[:,i]-np.mean(selected[:,i]))**2) if len(selected) else 0 for i in range(3)]
    metrics=dict(scale=scale,plant_scale=plant_scale,prediction=prediction,lag=lag,
        delay=delay,noise=noise,route_case=route_case,initial_gap=initial_gap,plant_beta=plant_beta,
        min_gap=float(a[:,3].min()),collision=bool(np.any(a[:,3]<0)),
        emergency_samples=emergency_count,active_samples=int(mask.sum()),
        spacing_rmse=float(np.sqrt(np.mean(selected[:,4]**2))) if len(selected) else None,
        speed_rms_ratio_21=float(math.sqrt(energies[2]/energies[1])) if energies[1]>1e-9 else None,
        accel_rms=float(np.sqrt(np.mean(accel**2))),jerk_rms=float(np.sqrt(np.mean(jerk**2))),
        forecast_rmse=float(np.sqrt(np.mean(np.array(ferrors)**2))) if ferrors else None)
    return metrics,rows


def main():
    out=ROOT/'simulation_results'
    out.mkdir(exist_ok=True)
    results=[]
    traces={}
    for assumed in (1.,2.8):
        for actual in (1.,2.8):
            for prediction in ('current','trend','idm'):
                for case in ('nominal','delay_noise','slow_plant','close_start','hard_brake','command_scale_mismatch'):
                    options=dict(lag=1. if case=='slow_plant' else .5,
                        delay=.4 if case=='delay_noise' else 0.,
                        noise=2. if case=='delay_noise' else 0.,
                        initial_gap=160. if case=='close_start' else 420.,
                        hard_brake=case=='hard_brake',
                        plant_beta=(2.8 if actual==1. else 1.) if case=='command_scale_mismatch' else actual)
                    m,rows=run(assumed,actual,prediction,**options)
                    m['case']=case
                    results.append(m)
                    if assumed==actual and case=='nominal':
                        traces[f'{assumed}_{prediction}']=rows
        for prediction in ('current','trend','idm'):
            m,rows=run(assumed,assumed,prediction,route_case=True,duration=240.)
            m['case']='route_defaults'
            results.append(m)
            traces[f'route_{assumed}_{prediction}']=rows
    (out/'metrics.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    with (out/'metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
    for name,rows in traces.items():
        with (out/f'trajectory_{name}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
    for col,scale in enumerate((1.,2.8)):
        for pred in ('current','trend','idm'):
            rows=traces[f'{scale}_{pred}']
            axes[0,col].plot([r['t'] for r in rows],[r['gap'] for r in rows],label=pred)
            axes[1,col].plot([r['t'] for r in rows],[r['v2']/scale for r in rows],label=pred)
        axes[0,col].set_title(f'Matched plant and controller scale {scale}')
        axes[0,col].axhline(80,color='r',ls='--',label='TA emergency threshold')
        axes[0,col].set_ylabel('Bumper gap (coordinate units)')
        axes[1,col].set_ylabel('Student speed (reported units)')
        axes[1,col].set_xlabel('Time (s)')
        axes[0,col].legend()
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(cases=len(results),collisions=sum(r['collision'] for r in results),
        emergency_cases=sum(r['emergency_samples']>0 for r in results)),indent=2))


if __name__=='__main__':
    main()
