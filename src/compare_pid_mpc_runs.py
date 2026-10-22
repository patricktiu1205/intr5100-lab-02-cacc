"""Read-only straight-event comparison: Group 4 PID 2.8 and reported MPC 2.8 run."""
from pathlib import Path
from zipfile import ZipFile
import csv,io,json,os
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
OUT=ROOT/'results/pid_vs_mpc_2p8'


def load(folder,method):
    raw=ROOT/'data/raw'/folder
    read=lambda n:list(csv.DictReader(io.StringIO((raw/n).read_text(encoding='utf-8-sig'))))
    commands=[r for r in read('commands.csv') if r['role']=='student']
    trajectory=read('trajectory.csv');meta=json.loads((raw/'metadata.json').read_text(encoding='utf-8'))
    ct=np.array([float(r['t_from_start']) for r in commands])
    active=np.array([r['mode']=='student_straight' for r in commands])
    indices=np.flatnonzero(active)
    blocks=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1)
    sources={}
    L=meta['route_loop_length']
    for role in ('leader','middle','student'):
        rr=sorted([r for r in trajectory if r['role']==role],key=lambda r:float(r['t_from_start']))
        t=np.array([float(r['t_from_start']) for r in rr])
        v={k:np.array([float(r[k]) for r in rr]) for k in ('x','y','s','velocity')}
        v['s']=np.unwrap(v['s']*2*np.pi/L)*L/(2*np.pi)
        sources[role]=(t,v)
    def section(s):
        return next((i+1 for i,r in enumerate(meta['control_sections']) if r['start_s']<=s%L<=r['end_s']),0)
    target0,h=(250.,1.2) if method=='MPC' else (300.,1.8)
    events=[]
    for number,block in enumerate(blocks,1):
        if not len(block):continue
        grid=np.arange(np.ceil(ct[block[0]]*10)/10,np.floor(ct[block[-1]]*10)/10+.001,.1)
        valid=np.ones(len(grid),bool);values={}
        for role,(t,channels) in sources.items():
            b=np.clip(np.searchsorted(t,grid),0,len(t)-1);a=np.maximum(0,b-1)
            span=np.where(abs(t[b]-grid)<1e-8,0,t[b]-t[a])
            valid&=(grid>=t[0])&(grid<=t[-1])&(span<=.3)
            values[role]={k:np.interp(grid,t,v) for k,v in channels.items()}
        sm=np.array([section(s) for s in values['middle']['s']]);se=np.array([section(s) for s in values['student']['s']])
        valid&=(sm>0)&(sm==se)
        dropped=int(sum(~valid));grid=grid[valid]
        values={r:{k:v[valid] for k,v in channels.items()} for r,channels in values.items()}
        gap=(values['middle']['s']-values['student']['s'])%L-250.
        # Controller reference uses telemetry, not noisy position differentiation.
        target=target0+h*2.8*values['student']['velocity']
        common=300+1.8*2.8*values['student']['velocity']
        ci=np.clip(np.searchsorted(ct,grid,side='right')-1,0,len(ct)-1)
        sensor=np.array([commands[i]['reason']=='center_sensor_emergency' for i in ci])
        reported_std=np.std(values['middle']['velocity'])
        ratio=float(np.std(values['student']['velocity'])/reported_std) if reported_std>=1 else None
        rows=[dict(t=float(t),s_student=float(values['student']['s'][i]%L),gap=float(gap[i]),target=float(target[i]),
            error=float(gap[i]-target[i]),common_error=float(gap[i]-common[i]),
            leader_v=float(values['leader']['velocity'][i]),middle_v=float(values['middle']['velocity'][i]),
            student_v=float(values['student']['velocity'][i]),sensor=int(sensor[i])) for i,t in enumerate(grid)]
        summary=dict(event=number,section=int(sm[valid][0]),start=float(grid[0]),end=float(grid[-1]),samples=len(grid),dropped=dropped,
            gap_min=float(min(gap)),gap_median=float(np.median(gap)),gap_max=float(max(gap)),
            own_rmse=float(np.sqrt(np.mean((gap-target)**2))),bias=float(np.mean(gap-target)),
            common_rmse=float(np.sqrt(np.mean((gap-common)**2))),
            reported_speed_ratio=ratio,middle_reported_std=float(reported_std),
            entry_gap=float(gap[0]),exit_gap=float(gap[-1]),sensor_grid_samples=int(sum(sensor)))
        events.append(dict(summary=summary,rows=rows))
        with (OUT/f'{method.lower()}_event_{number}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    active_commands=[r for r in commands if r['mode']=='student_straight']
    summary=dict(method=method,duration=float(ct[-1]),active_calls=len(active_commands),total_calls=len(commands),
        gap_all_median=float(np.median([float(r['spacing']) for r in commands])),
        active_gap_min=float(min(float(r['spacing']) for r in active_commands)),
        active_gap_median=float(np.median([float(r['spacing']) for r in active_commands])),
        active_gap_max=float(max(float(r['spacing']) for r in active_commands)),
        active_spacing_flags=sum(r['reason']=='spacing_emergency' for r in active_commands),
        active_sensor_flags=sum(r['reason']=='center_sensor_emergency' for r in active_commands),
        bypass_sensor_flags=sum(r['reason']=='center_sensor_emergency' for r in commands if r['mode']!='student_straight'),
        initial_gap=float(commands[0]['spacing']),dt_median=float(np.median([float(r['dt']) for r in commands])),
        callback_errors=sum(r['mode']=='student_fallback' for r in commands),
        events=[e['summary'] for e in events])
    return dict(meta=meta,summary=summary,events=events)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    runs={method:load(archive,method) for method,archive in
          [('PID','group4_pid_2p8'),('MPC','group4_mpc_2p8')]}
    # Match by selected route section and occurrence, not by absolute run timestamps.
    common=[]
    for pe,me in zip(runs['PID']['events'],runs['MPC']['events']):
        assert pe['summary']['section']==me['summary']['section']
        lo=max(min(r['s_student'] for r in pe['rows']),min(r['s_student'] for r in me['rows']))
        hi=min(max(r['s_student'] for r in pe['rows']),max(r['s_student'] for r in me['rows']))
        entry=dict(event=pe['summary']['event'],section=pe['summary']['section'],route_start=lo,route_end=hi)
        # Use actual retained samples within shared route coverage: no fabricated values across telemetry holes.
        for method,event in [('PID',pe),('MPC',me)]:
            rows=[r for r in event['rows'] if lo<=r['s_student']<=hi]
            entry[method]=dict(samples=len(rows),gap_median=float(np.median([r['gap'] for r in rows])),
                own_rmse=float(np.sqrt(np.mean([r['error']**2 for r in rows]))),
                common_rmse=float(np.sqrt(np.mean([r['common_error']**2 for r in rows]))))
        common.append(entry)
    data=dict(runs={m:r['summary'] for m,r in runs.items()},common_route_windows=common,
        setup=dict(student_vehicle_changed=runs['PID']['meta']['ips']['student']!=runs['MPC']['meta']['ips']['student'],
            pid_emergency_spacing=runs['PID']['meta']['safety']['emergency_spacing'],
            mpc_emergency_spacing=runs['MPC']['meta']['safety']['emergency_spacing']))
    (OUT/'summary.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    p,m=(runs[k]['summary'] for k in ('PID','MPC'))
    lines=['# Physical straight-event comparison: PID 2.8 vs MPC 2.8','',
        'Sources: `group4.zip` (corrected Group 4 PID run) and `experiment2_20261020_145126.zip` '
        '(identified by the user as MPC 2.8). Analysis date: 2026-10-20. Controller and experiment code remain unchanged.', '',
        '## Main observations','',
        f'- PID active gap min/median/max: {p["active_gap_min"]:.1f} / {p["active_gap_median"]:.1f} / {p["active_gap_max"]:.1f}.',
        f'- MPC active gap min/median/max: {m["active_gap_min"]:.1f} / {m["active_gap_median"]:.1f} / {m["active_gap_max"]:.1f}.',
        f'- Spacing-emergency flags during active control: PID {p["active_spacing_flags"]}; MPC {m["active_spacing_flags"]}.',
        f'- Sensor flags during active control: PID {p["active_sensor_flags"]}; MPC {m["active_sensor_flags"]}. '
        f'MPC has {m["bypass_sensor_flags"]} additional sensor flag outside its active-control intervals.', '',
        '## Comparison limits that affect interpretation','',
        '- The corrected PID and MPC runs use the same vehicle identifiers for all three roles. '
        'The user also confirms the student vehicle did not physically change.',
        '- TA emergency-spacing threshold changed from 80 (PID) to 50 (MPC). Report common gap bands '
        'rather than treating absence of native emergency flags as equivalent safety evidence. '
        'Logged student gaps stayed above 80 in both runs, so this spacing shield did not activate '
        'and the threshold difference does not directly explain the observed student-gap behavior.',
        '- Launch gaps differ: student gap about 117 (PID) versus 103 (MPC); middle-to-leader gap also differs.',
        '- PID target is 300 + 1.8*(2.8*v_reported); MPC target is 250 + 1.2*(2.8*v_reported). '
        'At reported speed 40 these are 501.6 and 384.4 coordinate units. A smaller MPC gap is partly intentional.',
        '- Neither archive contains the deployed source/hash or MPC internal solver status. '
        'student_straight confirms execution of the callback, not optimizer success. '
        'The MPC file can return fallback commands without changing that wrapper mode. '
        'Results here describe the run labelled MPC, not a verified optimization-only run.',
        '- One run per controller supports descriptive findings, not causal superiority claims.', '',
        '## Event-based processing','',
        'As in Lab I, use per-straight events and a common 0.1 s grid with no extrapolation. '
        'Reject source spans above 0.3 s. Gate on logged student_straight and verified '
        'middle/student same selected section, rather than generic straight-road labels. '
        'The leader need not occupy that same straight. Match events by section and occurrence, '
        'not by whole-run elapsed time. Launch is event 1; event 4 is truncated by each run end.', '',
        'Gap is forward route distance minus 250. Target uses aligned telemetry because that is '
        'the controller input. Speed-variation ratios below use aligned reported velocity; '
        'they are NOT position-derived physical acceleration or a string-stability proof. '
        'Ratios are withheld when middle reported-speed standard deviation is below 1 unit. '
        'Position-derived traces in the PID run have jumps, so a physical jerk/TTC ranking '
        'would require additional data-quality work.', '',
        '## Full active events','',
        '| Event | Method | Section | Time window (s) | Gap entry → exit | Median gap | Own-target RMSE | Bias | Reported-speed SD ratio |',
        '|---|---|---:|---|---|---:|---:|---:|---:|']
    for i in range(len(runs['PID']['events'])):
        for method in ('PID','MPC'):
            e=runs[method]['events'][i]['summary']
            ratio='quiet input' if e['reported_speed_ratio'] is None else f'{e["reported_speed_ratio"]:.3f}'
            lines.append(f'| {i+1} | {method} | {e["section"]} | {e["start"]:.1f}-{e["end"]:.1f} | '
                f'{e["entry_gap"]:.1f} → {e["exit_gap"]:.1f} | {e["gap_median"]:.1f} | '
                f'{e["own_rmse"]:.1f} | {e["bias"]:.1f} | {ratio} |')
    lines+=['', 'Positive bias means gap above the method\'s own target; negative means below. '
        'The target and entry conditions differ, so own-target RMSE alone is not a fair overall ranking.', '',
        '## Shared route coverage within corresponding active events','',
        'Restrict each run to the intersection of student route coordinates for each matched event. '
        'Use only actual retained aligned samples; no resampling across source-data holes. '
        'This controls route coverage, but not inherited state, sampling density or disturbances.', '',
        '| Event | Shared route length | PID median gap | MPC median gap | PID own-target RMSE | MPC own-target RMSE |',
        '|---|---:|---:|---:|---:|---:|']
    for e in common:
        lines.append(f'| {e["event"]} | {e["route_end"]-e["route_start"]:.1f} | '
            f'{e["PID"]["gap_median"]:.1f} | {e["MPC"]["gap_median"]:.1f} | '
            f'{e["PID"]["own_rmse"]:.1f} | {e["MPC"]["own_rmse"]:.1f} |')
    lines+=['', '## Interpretation','',
        'In event 3, MPC median gap is about 381 versus PID 491. The approximately 110-unit '
        'difference is close to the 117.2-unit difference between the two nominal targets at '
        'reported speed 40. This supports a target-policy explanation for much of the smaller '
        'MPC spacing, rather than treating it as proof of better control.', '',
        'For the full event 3 window, MPC has lower own-target RMSE (37.4 versus 57.2), '
        'while PID has a lower reported-speed standard-deviation ratio (0.776 versus 0.834). '
        'In event 2, PID own-target RMSE is lower (157.3 versus 175.2), but its entry gap '
        'is also much closer to its target. These descriptive tradeoffs do not yield an '
        'unqualified winner.', '',
        'Compare event 2 entry gaps before attributing recovery to a method: both '
        'controllers inherit the preceding bypass trajectory. Event 3 provides a longer '
        'braking/recovery comparison. Event 4 has little middle-speed excitation, so '
        'speed-variation ratios would be unstable and uninformative.', '',
        'Do not interpret fewer native emergency flags as proof MPC is safer: the safety '
        'threshold and sensor conditions differ. Do not infer computation latency '
        'from the command dt column: that measures loop cadence, not callback duration.', '',
        '![Route-aligned event comparison](../results/pid_vs_mpc_2p8/comparison.png)', '',
        'Machine-readable event metrics and aligned event CSVs are in `results/pid_vs_mpc_2p8/`.', '']
    (ROOT/'docs/pid_mpc_comparison.md').write_text('\n'.join(lines),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,4,figsize=(17,8))
    colors={'PID':'#2878b5','MPC':'#e58b22'}
    for i in range(4):
        for method in ('PID','MPC'):
            e=runs[method]['events'][i];rows=e['rows']
            x=np.array([r['s_student'] for r in rows])
            # Draw separate segments at missing-time intervals; don't bridge removed telemetry.
            ts=np.array([r['t'] for r in rows]);breaks=np.flatnonzero(np.diff(ts)>.15)+1
            x=np.insert(x,breaks,np.nan)
            value=lambda key:np.insert(np.array([r[key] for r in rows]),breaks,np.nan)
            axes[0,i].plot(x,value('gap'),color=colors[method],label=method+' gap')
            axes[0,i].plot(x,value('target'),color=colors[method],ls='--',alpha=.7,label=method+' target')
            axes[1,i].plot(x,value('student_v'),color=colors[method],label=method+' student')
            axes[1,i].plot(x,value('middle_v'),color=colors[method],ls=':',alpha=.8,label=method+' middle')
        axes[0,i].set_title(f'Event {i+1}: selected section {common[i]["section"]}')
        axes[0,i].legend(fontsize=7);axes[1,i].legend(fontsize=7)
        axes[1,i].set_xlabel('Student route coordinate')
        for ax in axes[:,i]:ax.grid(alpha=.2)
    axes[0,0].set_ylabel('Bumper gap / target (cu)')
    axes[1,0].set_ylabel('Reported speed')
    fig.suptitle('Physical runs: active straight events aligned by route position\nSame vehicles; different safety thresholds and target gaps; MPC internal mode not logged')
    fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(data,indent=2))


if __name__=='__main__':main()
