"""Compare the six unchanged submitted callbacks on identical route scenarios.

Uses actual VehicleState/history schema and TA controller activation rules.
No controller parameters, formulas, solver limits, or scaling constants are changed.
"""
from __future__ import annotations
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
from collections import Counter, deque
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
sys.path.insert(0,str(ROOT/'environment'))
from vehicle_io import VehicleState
from idm_controller import IDMController, IDMParams
from experiment_common import LeaderPerturbationManager
from controller import load_geometry
from simulate import xy

FILES={
    'PID-1':'controllers/unscaled/student_controller2.py',
    'PID-2.8':'controllers/scaled_2p8/student_controller2.py',
    'LQI-1':'controllers/lqi/unscaled/student_controller2.py',
    'LQI-2.8':'controllers/lqi/scaled_2p8/student_controller2.py',
    'MPC-1':'controllers/mpc/unscaled/student_controller2.py',
    'MPC-2.8':'controllers/mpc/scaled_2p8/student_controller2.py',
}


def load(name):
    spec=importlib.util.spec_from_file_location('candidate_'+name.replace('-','_').replace('.','_'),ROOT/FILES[name])
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def one(name,plant_scale,case,route,sections,duration=240.):
    start_import=time.perf_counter()
    m=load(name)
    import_ms=1000*(time.perf_counter()-start_import)
    if name.startswith('MPC') and m._ENGINE is None:
        raise RuntimeError(f'{name}: optimizer unavailable: {m._ENGINE_ERROR}')
    clock=[0.]
    if hasattr(m,'_CLOCK'): m._CLOCK=lambda:clock[0]
    rng=np.random.default_rng(5100)
    dt=.2
    # Same physical starting configuration for all six, within first selected straight.
    pos=np.array([4500.,3800.,3130. if case!='close_start' else 3390.])
    speed=np.full(3,40*plant_scale)
    idm=IDMController(IDMParams())
    leader=LeaderPerturbationManager();leader.reset(sections,40.)
    commands=np.full(3,40.)
    applied=commands.copy()
    pending=deque()
    history=[]
    rows=[]
    modes=Counter(); reasons=Counter()
    failures=0
    initial_time=1_800_000_000.
    active_cost=[]
    beta=plant_scale  # Hypothetical matched telemetry/command mapping, not identified lab physics.
    lag=1. if case=='slow_response' else .5
    delay=.4 if case=='delay_noise' else 0.
    noise=2. if case=='delay_noise' else 0.
    def section(s):
        return next((i for i,sec in enumerate(sections) if sec.contains(s,route.loop_length)),-1)
    def same(a,b):
        k=section(a)
        return k>=0 and k==section(b)
    with patch('time.monotonic',side_effect=lambda:clock[0]), \
         patch('time.time',side_effect=lambda:initial_time+clock[0]):
        for n in range(round(duration/dt)):
            t=n*dt;clock[0]=t
            route_s=pos%route.loop_length
            commands[0],_=leader.step(now_mono=t,leader_s=route_s[0],
                observed_velocity=speed[0]/plant_scale,base_speed=40.,loop_length=route.loop_length)
            if case=='hard_brake' and 15<=t<25: commands[0]=0.
            observations=[]
            for i,role in enumerate(('leader','middle','student')):
                px,py=xy(route,route_s[i])
                px+=rng.normal(0,noise);py+=rng.normal(0,noise)
                vr=max(0,speed[i]/plant_scale+rng.normal(0,noise*.03))
                state=VehicleState(role,'sim_'+role,initial_time+t,n,
                    dict(x=px,y=py,heading=0.,velocity=vr,center_distance=1.))
                observations.append(state)
                projected=route.project_xy(px,py).s
                history.append(dict(role=role,host=state.host,recv_time=state.recv_time,
                    t_from_start=t,seq=n,x=px,y=py,heading=0.,velocity=vr,s=projected))
            # TA gate and safety use observed projected positions; actual safety metrics use physical gap.
            ss=[history[-3+i]['s'] for i in range(3)]
            g01=route.forward_gap(ss[0],ss[1])-250.
            g12=route.forward_gap(ss[1],ss[2])-250.
            if same(ss[0],ss[1]):
                u=idm.next_velocity(observations[1].velocity,observations[0].velocity,g01,dt,v_max=70.)
            else:
                u=observations[0].velocity;idm.last_cmd_v=u
            commands[1]=round(0 if g01<80 else np.clip(u,0,100))
            active=same(ss[1],ss[2])
            mode='bypass';reason='noncontrol';call_ms=0.
            if active:
                tic=time.perf_counter()
                try:
                    u=float(m.student_control(*observations,history=history))
                    if not math.isfinite(u): raise ValueError('nonfinite command')
                    debug=getattr(m.student_control,'last_debug',{})
                    mode=debug.get('mode','feedback')
                    reason=debug.get('reason','')
                except Exception as exc:
                    failures+=1;u=observations[0].velocity
                    mode='wrapper_error_fallback';reason=type(exc).__name__
                call_ms=1000*(time.perf_counter()-tic)
                active_cost.append(call_ms);modes[mode]+=1;reasons[reason]+=1
            else:
                u=observations[0].velocity
            shield=g12<80
            commands[2]=round(0 if shield else np.clip(u,0,100))
            gap=pos[1]-pos[2]-250
            target0,h=(250.,1.2) if name.startswith('MPC') else (300.,1.8)
            closing=speed[2]-speed[1]
            rows.append(dict(t=t,v0=speed[0],v1=speed[1],v2=speed[2],gap=gap,
                own_error=gap-target0-h*speed[2],common_error=gap-300-1.8*speed[2],
                active=int(active),shield=int(shield),u2=commands[2],mode=mode,
                reason=reason,call_ms=call_ms,ttc=gap/closing if closing>1e-6 else None,
                progress=pos[2]-3130))
            pending.append((t+delay,commands.copy()))
            while pending and pending[0][0]<=t+1e-9: _,applied=pending.popleft()
            # Independent plant: heterogeneous lag, saturated acceleration, 0.05 s substeps.
            for _ in range(4):
                acc=np.clip((beta*applied-speed)/np.array([lag*.8,lag*1.2,lag]),-60*beta,35*beta)
                new=np.maximum(0,speed+.05*acc)
                pos+=.025*(speed+new);speed=new
    mask=np.array([r['active'] for r in rows],bool)
    data={k:np.array([r[k] for r in rows]) for k in ['v0','v1','v2','gap','own_error','common_error']}
    a=np.diff(data['v2'])/dt
    j=np.diff(a)/dt
    rms=lambda x:float(np.sqrt(np.mean(np.asarray(x)**2)))
    variation=lambda x:float(np.std(x[mask]))
    denominator=variation(data['v1'])
    ttc=[r['ttc'] for r in rows if r['active'] and r['ttc'] is not None]
    metrics=dict(controller=name,plant_scale=plant_scale,case=case,
        collision=bool(np.min(data['gap'])<0),min_gap=float(np.min(data['gap'])),
        shield_samples=sum(r['shield'] for r in rows),wrapper_errors=failures,
        active_samples=int(mask.sum()),own_gap_rmse=rms(data['own_error'][mask]),
        common_gap_rmse=rms(data['common_error'][mask]),
        variation_ratio=variation(data['v2'])/denominator if denominator>1e-8 else None,
        acceleration_rms=rms(a),jerk_rms=rms(j),mean_speed=float(np.mean(data['v2'])),
        min_ttc=min(ttc) if ttc else None,internal_modes=dict(modes),internal_reasons=dict(reasons),
        import_ms=import_ms,first_call_ms=active_cost[0] if active_cost else None,
        p95_call_ms=float(np.percentile(active_cost[1:],95)) if len(active_cost)>1 else None)
    return metrics,rows


def main():
    route,sections=load_geometry()
    before={name:hashlib.sha256((ROOT/file).read_bytes()).hexdigest() for name,file in FILES.items()}
    out=ROOT/'simulation_results/all_six';out.mkdir(parents=True,exist_ok=True)
    all_results=[]
    for plant in (1.,2.8):
        for case in ('nominal','delay_noise','slow_response','close_start','hard_brake'):
            for name in FILES:
                result,rows=one(name,plant,case,route,sections)
                all_results.append(result)
                if case=='nominal':
                    with (out/f'{name}_plant_{plant}.csv').open('w',newline='') as f:
                        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
                (out/'metrics.json').write_text(json.dumps(all_results,indent=2),encoding='utf-8')
                print(f'{len(all_results)}/60 {name} plant={plant} {case}: gapRMSE={result["own_gap_rmse"]:.1f}, '
                      f'ratio={result["variation_ratio"]:.3f}, modes={result["internal_modes"]}',flush=True)
    after={name:hashlib.sha256((ROOT/file).read_bytes()).hexdigest() for name,file in FILES.items()}
    assert before==after,'Controller source changed'
    (out/'source_hashes.json').write_text(json.dumps(before,indent=2))
    with (out/'metrics.csv').open('w',newline='') as f:
        flat=[{k:v for k,v in r.items() if not isinstance(v,dict)} for r in all_results]
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)


if __name__=='__main__': main()
