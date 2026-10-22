"""Lab-I-style event alignment for the corrected Group 4 straight-control data."""
from zipfile import ZipFile
from pathlib import Path
import csv,io,json,os
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))


def main():
    raw=ROOT/'data/raw/group4_pid_2p8'
    read=lambda n:list(csv.DictReader(io.StringIO((raw/n).read_text(encoding='utf-8-sig'))))
    commands=[r for r in read('commands.csv') if r['role']=='student']
    trajectory=read('trajectory.csv');meta=json.loads((raw/'metadata.json').read_text(encoding='utf-8'))
    out=ROOT/'results/group4_pid_2p8/straight_events';out.mkdir(parents=True,exist_ok=True)
    sources={}
    for role in ('leader','middle','student'):
        rr=sorted([r for r in trajectory if r['role']==role],key=lambda r:float(r['t_from_start']))
        t=np.array([float(r['t_from_start']) for r in rr])
        sources[role]=(t,{k:np.array([float(r[k]) for r in rr]) for k in ('x','y','s','velocity')})
        sources[role][1]['s']=np.unwrap(sources[role][1]['s']*2*np.pi/meta['route_loop_length'])*meta['route_loop_length']/(2*np.pi)
    ct=np.array([float(r['t_from_start']) for r in commands])
    active=np.array([r['mode']=='student_straight' for r in commands])
    blocks=np.split(np.flatnonzero(active),np.flatnonzero(np.diff(np.flatnonzero(active))>1)+1)
    summaries=[];traces=[]
    def section(s):
        s%=meta['route_loop_length']
        return next((i+1 for i,r in enumerate(meta['control_sections']) if r['start_s']<=s<=r['end_s']),0)
    for number,block in enumerate(blocks,1):
        if not len(block):continue
        start,end=ct[block[0]],ct[block[-1]]
        grid=np.arange(np.ceil(start*10)/10,np.floor(end*10)/10+.001,.1)
        mask=np.ones(len(grid),bool);values={}
        for role,(t,channels) in sources.items():
            right=np.searchsorted(t,grid,side='left')
            left=np.clip(right-1,0,len(t)-1);right=np.clip(right,0,len(t)-1)
            exact=np.abs(t[right]-grid)<1e-8
            spans=np.where(exact,0,t[right]-t[left])
            mask&=(grid>=t[0])&(grid<=t[-1])&(spans<=.3)
            values[role]={k:np.interp(grid,t,v) for k,v in channels.items()}
        # Verify geometry rather than assuming a straight-zone tag implies active PID.
        middle_sections=np.array([section(s) for s in values['middle']['s']])
        student_sections=np.array([section(s) for s in values['student']['s']])
        mask&=(middle_sections>0)&(middle_sections==student_sections)
        removed=int(sum(~mask));grid=grid[mask]
        values={role:{k:v[mask] for k,v in channels.items()} for role,channels in values.items()}
        if len(grid)<10:raise ValueError('Insufficient aligned event')
        # Split data holes before differentiating, as well as splitting events.
        chunks=np.split(np.arange(len(grid)),np.flatnonzero(np.diff(grid)>.15)+1)
        for role in values:
            speed=np.full(len(grid),np.nan)
            for q in chunks:
                if len(q)<3:continue
                # Lab I convention: central chord, one-sided chord at ends.
                a=np.maximum(q[0],q-1);b=np.minimum(q[-1],q+1)
                speed[q]=np.hypot(values[role]['x'][b]-values[role]['x'][a],
                    values[role]['y'][b]-values[role]['y'][a])/(grid[b]-grid[a])
            values[role]['position_speed']=speed
        good=np.isfinite(values['student']['position_speed'])&np.isfinite(values['middle']['position_speed'])
        grid=grid[good];values={role:{k:v[good] for k,v in channels.items()} for role,channels in values.items()}
        gap=(values['middle']['s']-values['student']['s'])%meta['route_loop_length']-250
        target=300+1.8*2.8*values['student']['velocity']
        error=gap-target
        closing=values['student']['position_speed']-values['middle']['position_speed']
        ttc=gap[closing>1e-6]/closing[closing>1e-6]
        # Command reason applies from that decision to the next decision.
        ci=np.clip(np.searchsorted(ct,grid,side='right')-1,0,len(ct)-1)
        flags=np.array([commands[i]['emergency']=='1' for i in ci])
        std1=np.std(values['middle']['position_speed']);std2=np.std(values['student']['position_speed'])
        summary=dict(event=number,start=float(grid[0]),end=float(grid[-1]),samples=len(grid),
            dropped_grid_points=removed,section=int(middle_sections[mask][good][0]),
            gap_min=float(min(gap)),gap_median=float(np.median(gap)),gap_max=float(max(gap)),
            target_rmse=float(np.sqrt(np.mean(error**2))),target_bias=float(np.mean(error)),
            position_speed_std_ratio=float(std2/std1) if std1>1e-9 else None,
            min_ttc=float(min(ttc)) if len(ttc) else None,
            sensor_flag_grid_points=int(sum(flags)),launch_event=number==1)
        summaries.append(summary)
        rows=[dict(t=float(t),gap=float(gap[i]),target=float(target[i]),error=float(error[i]),
            leader_speed=float(values['leader']['position_speed'][i]),
            middle_speed=float(values['middle']['position_speed'][i]),
            student_speed=float(values['student']['position_speed'][i]),sensor_flag=int(flags[i])) for i,t in enumerate(grid)]
        with (out/f'event_{number}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        traces.append(rows)
    (out/'summary.json').write_text(json.dumps(summaries,indent=2),encoding='utf-8')
    lines=['# Group 4 PID 2.8: straight-event analysis','',
        'This is the primary controller-performance view. Whole-run plots remain useful '
        'for context, but curve/noncontrol samples are not scored as PID behavior.', '',
        '## Processing, following Lab I','',
        '- Reviewed Lab I preprocess_stage2.py, preprocess_stage3.py, preprocess_stage4.py, and analysis/metrics.py.',
        '- Identify events from consecutive student_straight command decisions. A generic zone_type=straight '
        'tag is insufficient: middle and student must share one of the three selected control straights.',
        '- Use a common 0.1 s grid, no extrapolation; discard points with source interpolation spans over 0.3 s. '
        'Align all three vehicles, but require only middle/student same-section membership for PID eligibility.',
        '- Derive measured speeds from aligned x/y using central chord differences, one-sided at ends, '
        'as in Lab I. Split data holes before differentiation. No extra 2.8 multiplication is applied '
        'to position-derived speeds.',
        '- Keep route bumper spacing (forward route distance minus 250), matching Lab II command logging. '
        'Lab I used heading projection; these definitions are not silently interchanged.',
        '- Reconstruct the controller target from reported student velocity using 300 + 1.8*2.8*v. '
        'This is a controller-internal reference, distinct from position-derived speed used for observed motion.',
        '- Retain genuine within-event stops and identify sensor intervention separately. Event 1 includes '
        'launch and sensor stopping, so report it separately from later events. No arbitrary transient cutoff '
        'is chosen to improve the scores.',
        '- TTC = gap / (student speed - middle speed) during closing only. Do not copy the reversed '
        'closing-speed sign stated in the Lab I metrics module docstring.', '',
        '| Event | Section | Common interval (s) | Points | Gap min/median/max | Target RMSE | Target bias |',
        '|---|---:|---|---:|---|---:|---:|']
    for s in summaries:
        lines.append(f'| {s["event"]} | {s["section"]} | {s["start"]:.1f}-{s["end"]:.1f} | {s["samples"]} | '
            f'{s["gap_min"]:.1f} / {s["gap_median"]:.1f} / {s["gap_max"]:.1f} | '
            f'{s["target_rmse"]:.1f} | {s["target_bias"]:.1f} |')
    later=[r for rows in traces[1:] for r in rows]
    lines+=['',f'Later events (2-4), pooled by retained sample: target RMSE '
        f'{np.sqrt(np.mean([r["error"]**2 for r in later])):.2f} coordinate units. '
        'This excludes the launch event explicitly, but still includes section-entry transients.', '',
        'Event 2 begins with an inherited short gap. Its larger error belongs in the report, '
        'while the preceding noncontrol segment is context rather than PID-controlled operation. '
        'Events 3 and 4 should be discussed individually; do not turn a whole-run gap median '
        'into a PID tracking metric.', '',
        'Position-derived speed variation and TTC are saved in JSON as descriptive diagnostics. '
        'Sharp position-derived student-speed spikes occur in events 1 and 4, while '
        'reported-speed traces are much smoother. These may reflect position/timing '
        'artifacts and require sample-level review before any acceleration, jerk, TTC '
        'or disturbance-amplification claim. No outliers were silently removed. '
        'Localization quantization affects derivatives; they do not prove string stability. '
        'No whole-run acceleration/jerk ranking is inferred. Event 4 is truncated by run end.', '',
        '![Straight-event view](../results/group4_pid_2p8/straight_events/events.png)', '']
    (ROOT/'docs/pid_straight_events.md').write_text('\n'.join(lines),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,4,figsize=(16,7))
    for i,rows in enumerate(traces):
        t=np.array([r['t'] for r in rows])
        breaks=np.flatnonzero(np.diff(t)>.15)+1
        plot_t=np.insert(t,breaks,np.nan)
        plot_values=lambda key:np.insert(np.array([r[key] for r in rows]),breaks,np.nan)
        axes[0,i].plot(plot_t,plot_values('gap'),label='Gap')
        axes[0,i].plot(plot_t,plot_values('target'),ls='--',label='PID target')
        axes[0,i].set_title(f'Event {i+1}'+(' (launch + sensor)' if i==0 else ''))
        for role in ('leader','middle','student'):
            axes[1,i].plot(plot_t,plot_values(role+'_speed'),label=role,lw=1)
        axes[0,i].legend(fontsize=8);axes[1,i].legend(fontsize=8)
        axes[1,i].set_xlabel('Run time (s)')
        for ax in axes[:,i]:ax.grid(alpha=.2)
    axes[0,0].set_ylabel('Coordinate gap');axes[1,0].set_ylabel('Position-derived speed (cu/s)')
    fig.suptitle('PID-controlled straight events only; common 0.1 s grid')
    fig.tight_layout();fig.savefig(out/'events.png',dpi=150);plt.close(fig)
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
