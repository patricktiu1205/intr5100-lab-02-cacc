"""Plant validation and causal ego closed-loop tests driven by recorded front vehicles.

All candidate controller files remain unchanged. Recorded ego trajectory is used
only for plant fitting/validation, initialization, and a physical reference.
"""
from pathlib import Path
from collections import deque,Counter
from unittest.mock import patch
import csv,json,os,sys,hashlib
import numpy as np
from scipy.signal import lfilter

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
sys.path.insert(0,str(ROOT/'environment'))
from vehicle_io import VehicleState
from simulate_six_controllers import load,FILES
from pid_controller import load_geometry
from simulate_pid import xy

RUNS={'PID':'group4_pid_2p8','MPC':'group4_mpc_2p8','LQI':'group4_lqi_2p8'}
OUT=ROOT/'results/three_controller_analysis'


def raw_data(folder):
    raw=ROOT/'data/raw'/folder
    read=lambda name:list(csv.DictReader((raw/name).open(encoding='utf-8-sig')))
    meta=json.loads((raw/'metadata.json').read_text())
    trajectory=read('trajectory.csv')
    cmd=[r for r in read('commands.csv') if r['role']=='student']
    data={}
    for role in ('leader','middle','student'):
        rr=sorted([r for r in trajectory if r['role']==role],key=lambda r:float(r['t_from_start']))
        t=np.array([float(r['t_from_start']) for r in rr])
        values={k:np.array([float(r[k]) for r in rr]) for k in ('s','velocity','x','y')}
        values['s']=np.unwrap(values['s']*2*np.pi/meta['route_loop_length'])*meta['route_loop_length']/(2*np.pi)
        data[role]=(t,values)
    return meta,cmd,data


def fit_plant(folder):
    """Robust coarse lag/delay search on displacement increments, with held-out blocks."""
    meta,cmd,data=raw_data(folder)
    t,ch=data['student']
    ct=np.array([float(r['t_from_start']) for r in cmd])
    u=np.round([float(r['v_cmd']) for r in cmd])
    start=max(ct[0],t[0]);end=min(ct[-1],t[-1])
    grid=np.arange(start,end,.05)
    # 1-second displacement is less sensitive to coordinate quantization than accelerations.
    bins=np.arange(np.ceil(start),np.floor(end),1.)
    observed=np.interp(bins+1,t,ch['s'])-np.interp(bins,t,ch['s'])
    # Drop intervals that contain telemetry holes >0.3 s or backward/implausible displacement.
    valid=(observed>=0)&(observed<250)
    for i,left in enumerate(bins):
        q=np.flatnonzero((t>=left-.15)&(t<=left+1.15))
        if len(q)<5 or np.max(np.diff(t[q]))>.3:valid[i]=False
    training=valid&((bins.astype(int)//5)%2==0)
    testing=valid&~training
    candidates=[]
    for lag in (.1,.2,.35,.5,.8,1.2,1.8):
        for delay in (0.,.1,.2,.3,.5):
            idx=np.clip(np.searchsorted(ct,grid-delay,side='right')-1,0,len(ct)-1)
            decay=np.exp(-.05/lag)
            filtered,_=lfilter([1-decay],[1,-decay],u[idx],zi=[decay*np.interp(start,t,ch['velocity'])])
            integral=np.cumsum(filtered)*.05
            unit=np.interp(bins+1,grid,integral)-np.interp(bins,grid,integral)
            useful=training&(unit>5)
            beta=float(np.clip(np.median(observed[useful]/unit[useful]),1,5))
            residual=beta*unit-observed
            # Huber loss, scale 15 coordinate units of 1-second displacement.
            z=abs(residual[training]);loss=float(np.mean(np.where(z<=15,.5*z*z,15*(z-7.5))))
            candidates.append(dict(beta=beta,lag=lag,delay=delay,training_loss=loss,
                validation_displacement_rmse=float(np.sqrt(np.mean(residual[testing]**2))),
                unit=unit,residual=residual))
    best=min(candidates,key=lambda r:r['training_loss'])
    prediction=observed+best['residual']
    validation=[]
    for i,left in enumerate(bins):
        validation.append(dict(t=float(left),observed_displacement=float(observed[i]),
            predicted_displacement=float(prediction[i]),valid=int(valid[i]),training=int(training[i])))
    result={k:v for k,v in best.items() if k not in ('unit','residual')}
    result.update(train_intervals=int(sum(training)),validation_intervals=int(sum(testing)),
        validation_mean_displacement=float(np.mean(observed[testing])),
        validation_relative_rmse=float(best['validation_displacement_rmse']/np.mean(observed[testing])),
        candidate_count=len(candidates))
    return result,validation


def simulate(method,source,plant,lag_multiplier=1.):
    meta,cmd,data=raw_data(RUNS[source]);route,sections=load_geometry()
    module=load(method+'-2.8')
    if method=='MPC' and module._ENGINE is None:raise RuntimeError(module._ENGINE_ERROR)
    times=np.array([float(r['t_from_start']) for r in cmd])
    lo=max(v[0][0] for v in data.values());hi=min(v[0][-1] for v in data.values())
    times=times[(times>=lo)&(times<=hi)]
    first=times[0]
    sample=lambda role,key,t:float(np.interp(t,data[role][0],data[role][1][key]))
    ego_s=sample('student','s',first)
    # Initial velocity is measured; all subsequent ego velocities come from the plant.
    ego_v=2.8*sample('student','velocity',first)
    gamma=2.8
    beta=plant['beta'];lag=plant['lag']*lag_multiplier;delay=plant['delay']
    history=[];pending=deque();applied=0.;clock=[first];wall0=1_800_000_000.
    if hasattr(module,'_CLOCK'):module._CLOCK=lambda:clock[0]
    records=[];modes=Counter();errors=0
    def section(s):return next((i for i,sec in enumerate(sections) if sec.contains(s,route.loop_length)),-1)
    with patch('time.monotonic',side_effect=lambda:clock[0]),patch('time.time',side_effect=lambda:wall0+clock[0]):
        for n,t in enumerate(times[:-1]):
            clock[0]=t
            observations=[]
            for role in ('leader','middle','student'):
                if role=='student':
                    s=ego_s;x,y=xy(route,s);v=ego_v/gamma
                else:
                    s=sample(role,'s',t);x=sample(role,'x',t);y=sample(role,'y',t);v=sample(role,'velocity',t)
                state=VehicleState(role,'replay_'+role,wall0+t,n,dict(x=x,y=y,heading=0.,velocity=v,center_distance=1.))
                observations.append(state)
                history.append(dict(role=role,host=state.host,seq=n,recv_time=wall0+t,t_from_start=t-first,
                    x=x,y=y,heading=0.,velocity=v,s=s%route.loop_length))
            mid_s=sample('middle','s',t)
            active=section(mid_s)>=0 and section(mid_s)==section(ego_s)
            if active:
                try:
                    desired=float(module.student_control(*observations,history=history))
                    if not np.isfinite(desired):raise ValueError('nonfinite')
                    mode=getattr(module.student_control,'last_debug',{}).get('mode','feedback')
                except Exception:
                    errors+=1;desired=observations[0].velocity;mode='wrapper_error'
                modes[mode]+=1
            else:desired=observations[0].velocity;mode='bypass'
            gap=mid_s-ego_s-250
            source_valid=True
            for role in ('leader','middle'):
                ts=data[role][0];right=min(len(ts)-1,int(np.searchsorted(ts,t)))
                left=max(0,right-1)
                if abs(ts[right]-t)>1e-8 and ts[right]-ts[left]>.3:source_valid=False
            shield=gap<float(meta['safety']['emergency_spacing'])
            command=round(0 if shield else np.clip(desired,0,100))
            standstill,headway=(250.,1.2) if method=='MPC' else (300.,1.8)
            records.append(dict(t=float(t),s=ego_s,v=ego_v,gap=gap,target=standstill+headway*ego_v,
                error=gap-standstill-headway*ego_v,active=int(active),command=command,mode=mode,
                shield=int(shield),source_data_valid=int(source_valid),middle_v=2.8*observations[1].velocity,
                recorded_ego_s=sample('student','s',t),recorded_ego_v=sample('student','velocity',t)*2.8))
            pending.append((t+delay,command))
            # Plant runs continuously through control/bypass switches; ego is never reset to its recording.
            substeps=max(1,int(np.ceil((times[n+1]-t)/.02)));step=(times[n+1]-t)/substeps
            for j in range(substeps):
                now=t+j*step
                while pending and pending[0][0]<=now+1e-9:_,applied=pending.popleft()
                nv=beta*applied+(ego_v-beta*applied)*np.exp(-step/lag)
                ego_s+=.5*(ego_v+nv)*step;ego_v=nv
    mask=np.array([r['active'] and r['source_data_valid'] for r in records],bool)
    err=np.array([r['error'] for r in records]);gaps=np.array([r['gap'] for r in records])
    result=dict(controller=method,source=source,lag_multiplier=lag_multiplier,
        own_rmse=float(np.sqrt(np.mean(err[mask]**2))),median_active_gap=float(np.median(gaps[mask])),
        min_gap=float(min(gaps)),collisions=int(sum(gaps<0)),shield_samples=sum(r['shield'] for r in records),
        active_samples=int(sum(mask)),wrapper_errors=errors,internal_modes=dict(modes),
        filled_gap_samples=sum(not r['source_data_valid'] for r in records),
        min_gap_after_5s=float(min(r['gap'] for r in records if r['t']>first+5)),
        position_rmse_vs_recording=float(np.sqrt(np.mean([(r['s']-r['recorded_ego_s'])**2 for r in records]))))
    return result,records


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    fits={};validation={};results=[]
    before={k:hashlib.sha256((ROOT/FILES[k+'-2.8']).read_bytes()).hexdigest() for k in RUNS}
    for source,folder in RUNS.items():
        fits[source],validation[source]=fit_plant(folder)
        with (OUT/f'plant_validation_{source.lower()}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(validation[source][0]));w.writeheader();w.writerows(validation[source])
        print('Plant',source,fits[source],flush=True)
        for method in RUNS:
            for multiplier in (1.,.5,2.):
                result,records=simulate(method,source,fits[source],multiplier)
                results.append(result)
                if multiplier==1:
                    with (OUT/f'replay_input_{source.lower()}_controller_{method.lower()}.csv').open('w',newline='') as f:
                        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
                print('Replay',source,method,multiplier,result['own_rmse'],result['min_gap'],flush=True)
    assert before=={k:hashlib.sha256((ROOT/FILES[k+'-2.8']).read_bytes()).hexdigest() for k in RUNS}
    (OUT/'closed_loop.json').write_text(json.dumps(dict(plant_fits=fits,runs=results,controller_hashes=before),indent=2),encoding='utf-8')


if __name__=='__main__':main()
