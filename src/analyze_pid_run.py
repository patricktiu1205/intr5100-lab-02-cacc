"""Read-only analysis of Group 4's corrected PID 2.8 physical run."""
from zipfile import ZipFile
from pathlib import Path
import csv,io,json,os
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))


def main():
    raw=ROOT/'data/raw/group4_pid_2p8'
    read=lambda n:list(csv.DictReader(io.StringIO((raw/n).read_text(encoding='utf-8-sig'))))
    commands=read('commands.csv');tr=read('trajectory.csv')
    meta=json.loads((raw/'metadata.json').read_text(encoding='utf-8'))
    out=ROOT/'results/group4_pid_2p8';out.mkdir(parents=True,exist_ok=True)
    student=[r for r in commands if r['role']=='student']
    ct=np.array([float(r['t_from_start']) for r in student])
    gap=np.array([float(r['spacing']) for r in student])
    active=np.array([r['mode']=='student_straight' for r in student])
    motion={}
    for role in ('leader','middle','student'):
        rr=sorted([r for r in tr if r['role']==role],key=lambda r:float(r['recv_time']))
        t=np.array([float(r['t_from_start']) for r in rr])
        v=np.array([float(r['velocity']) for r in rr])
        s=np.unwrap(np.array([float(r['s']) for r in rr])*2*np.pi/meta['route_loop_length'])*meta['route_loop_length']/(2*np.pi)
        motion[role]=(t,v,s)
    # Approximate latest state at command log time, never future interpolation.
    velocities={}
    ages={}
    for role,(t,v,s) in motion.items():
        idx=np.clip(np.searchsorted(t,ct,side='right')-1,0,len(t)-1)
        velocities[role]=v[idx];ages[role]=ct-t[idx]
    target=300+1.8*2.8*velocities['student']
    error=gap-target
    episodes=[]
    indices=np.flatnonzero(active)
    groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1)
    for group in groups:
        if not len(group):continue
        episodes.append(dict(start=float(ct[group[0]]),end=float(ct[group[-1]]),samples=len(group),
            gap_min=float(min(gap[group])),gap_median=float(np.median(gap[group])),gap_max=float(max(gap[group])),
            approximate_target_rmse=float(np.sqrt(np.mean(error[group]**2)))))
    emergencies=[{k:r[k] for k in ('t_from_start','reason','spacing','desired_velocity','v_cmd')}
                 for r in student if r['emergency']=='1']
    summary=dict(duration=float(ct[-1]),student_commands=len(student),active_commands=int(sum(active)),
        active_fraction=float(np.mean(active)),gap_percentiles=dict(zip(('min','q25','median','q75','max'),map(float,np.percentile(gap,[0,25,50,75,100])))),
        spacing_emergencies=sum(r['reason']=='spacing_emergency' for r in student),
        sensor_emergency_samples=len(emergencies),emergencies=emergencies,
        approximate_active_target_rmse=float(np.sqrt(np.mean(error[active]**2))),
        approximate_active_target_bias=float(np.mean(error[active])),
        active_nonzero_desired=int(sum(active[i] and float(r['desired_velocity'])>0 for i,r in enumerate(student))),
        max_logged_state_age=float(max(max(a) for a in ages.values())),episodes=episodes,
        dt_median=float(np.median([float(r['dt']) for r in student])))
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    lines=['# Group 4: corrected PID 2.8 physical run','',
        'Source: `group4.zip`, received 2026-10-20. User identifies this as the corrected '
        'Group 4 PID 2.8 run. The archive does not embed a controller source hash. '
        'No controller or experiment code was revised.', '',
        '## Main findings','',
        f'- Duration: {ct[-1]:.2f} s; {len(student)} student command decisions.',
        f'- Active PID: {sum(active)} decisions ({100*np.mean(active):.1f}%); remaining decisions use the TA leader-speed bypass.',
        f'- Logged bumper gap: minimum {min(gap):.2f}, median {np.median(gap):.2f}, maximum {max(gap):.2f} coordinate units.',
        f'- Spacing emergency triggers: {summary["spacing_emergencies"]}. Center-sensor emergency samples: {len(emergencies)}.',
        f'- Median actual control interval: {summary["dt_median"]:.3f} s, despite 0.15 s configured period.', '',
        'The constant-0.09 active-command failure from the earlier Group 1 deployment '
        'is absent. This run has changing desired-speed commands, repeated active '
        'control, and a bounded gap rather than progressive separation to thousands of units.', '',
        '## Gap target','',
        'The saved PID 2.8 design requests gap = 300 + 1.8 * (2.8 * reported speed). '
        'At reported speeds 20, 25 and 40, this is approximately 401, 426 and 502 '
        'coordinate units. A gap around 400-500 is therefore not automatically excessive '
        'for this particular design. The target itself is intentionally conservative.', '',
        f'Using latest logged telemetry to approximate the callback state, active target '
        f'RMSE is {summary["approximate_active_target_rmse"]:.2f} and mean signed error is '
        f'{summary["approximate_active_target_bias"]:.2f} coordinate units. These are approximate '
        'because exact callback input snapshots and callback-entry timestamps are unavailable. '
        'The first active episode includes launch; do not interpret its transient error '
        'as steady-state tracking performance.', '',
        '| Active episode (s) | Samples | Minimum gap | Median gap | Maximum gap | Approx. target RMSE |',
        '|---|---:|---:|---:|---:|---:|']
    for e in episodes:
        lines.append(f'| {e["start"]:.2f}-{e["end"]:.2f} | {e["samples"]} | {e["gap_min"]:.1f} | '
                     f'{e["gap_median"]:.1f} | {e["gap_max"]:.1f} | {e["approximate_target_rmse"]:.1f} |')
    lines += ['', '## Safety and interpretation','',
        'Two consecutive center-sensor emergency samples occur at approximately 7.82 and '
        '8.02 s with route bumper gaps 227.2 and 233.0. These are sensor-triggered events, '
        'not logged spacing-threshold violations. Route gap alone cannot establish whether '
        'the sensor observed a real obstacle or a transient. Desired command is already '
        'zero on those rows, so the records do not show a positive PID command being '
        'overridden on those particular samples.', '',
        'Minimum logged gap 92.3 exceeds the TA emergency threshold of 80 but is below '
        'the controller\'s 120-unit protective threshold. This occurs in the launch '
        'episode; the GUI issues initial start commands before subsequent closed-loop '
        'updates. The log alone cannot prove a physical collision did or did not occur.', '',
        'This is one physical run, with PID active less than half the time. It supports '
        'a successful execution observation and a bounded-gap result, not a method ranking '
        'or a proof of string stability. Disturbance propagation should be compared per '
        'matched event with the second controller once its data is available.', '',
        'The earlier archive `experiment2_20261020_135916.zip` is a deployment-error '
        'diagnostic run using Group 1\'s controller, per the user\'s confirmation. It '
        'is not a controlled baseline for evaluating Group 4 PID performance.', '',
        '![Run overview](../results/group4_pid_2p8/overview.png)', '']
    (ROOT/'docs/pid_physical_results.md').write_text('\n'.join(lines),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    axes[0].plot(ct,gap,label='Logged bumper gap')
    axes[0].plot(ct,target,ls='--',label='Approximate PID target')
    axes[0].axhline(80,color='red',ls=':',label='TA spacing emergency threshold')
    axes[0].set_ylabel('Coordinate gap');axes[0].legend(ncol=3,fontsize=8)
    for role,(t,v,s) in motion.items():axes[1].plot(t,v,label=role)
    axes[1].set_ylabel('Reported speed');axes[1].legend(ncol=3)
    axes[2].plot(ct,[float(r['desired_velocity']) for r in student],label='Desired speed')
    axes[2].plot(ct,[float(r['v_cmd']) for r in student],ls=':',label='Shielded command (before rounding)')
    axes[2].set_ylabel('Command units');axes[2].set_xlabel('Time (s)');axes[2].legend()
    for ax in axes:
        for e in episodes:ax.axvspan(e['start'],e['end'],color='green',alpha=.07)
        for event in emergencies:ax.axvline(float(event['t_from_start']),color='red',alpha=.4,lw=.8)
        ax.grid(alpha=.2)
    fig.suptitle('Group 4 PID 2.8: green shading = controller active; red lines = sensor flags')
    fig.tight_layout();fig.savefig(out/'overview.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
