"""Reproducible three-method physical report plus recorded-data closed-loop results."""
from pathlib import Path
import csv,json,os
import numpy as np
import compare_pid_mpc_runs as physical
from simulate_recorded_inputs import RUNS,raw_data

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/three_controller_analysis'
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))


def main():
    OUT.mkdir(parents=True,exist_ok=True);physical.OUT=OUT
    runs={m:physical.load(folder,m) for m,folder in RUNS.items()}
    details={}
    for method,folder in RUNS.items():
        meta,commands,data=raw_data(folder)
        increments=[];all_increments=[]
        for a,b in zip(commands,commands[1:]):
            if a['mode']==b['mode']=='student_straight':
                d=round(float(b['v_cmd']))-round(float(a['v_cmd']))
                all_increments.append(d)
                if a['emergency']==b['emergency']=='0':increments.append(d)
        details[method]=dict(command_increment_rms=float(np.sqrt(np.mean(np.array(increments)**2))),
            command_increment_p95=float(np.percentile(abs(np.array(increments)),95)),
            all_command_increment_rms=float(np.sqrt(np.mean(np.array(all_increments)**2))),
            command_increment_pairs=len(increments),
            active_below_80=sum(float(r['spacing'])<80 for r in commands if r['mode']=='student_straight'),
            active_below_120=sum(float(r['spacing'])<120 for r in commands if r['mode']=='student_straight'),
            input_gaps={role:dict(samples=len(t),gaps_over_point3=int(sum(np.diff(t)>.3)),max_gap=float(max(np.diff(t))))
                        for role,(t,channels) in data.items()})
        t,ch=data['student'];delta=np.diff(ch['s']);dt=np.diff(t)
        residual=delta-2.8*ch['velocity'][:-1]*dt
        # Diagnostic flags only, not deletions or claims of a known localization mechanism.
        q=np.flatnonzero((t[1:]>140)&(abs(residual)>15))
        details[method]['late_position_jumps']=[dict(t=float(t[i+1]),
            route_s=float(ch['s'][i+1]%meta['route_loop_length']),displacement=float(delta[i]),
            dt=float(dt[i]),reported_velocity=float(ch['velocity'][i+1])) for i in q]
    # Common route coverage for the same selected straight and occurrence in all three runs.
    matched=[]
    for i in range(4):
        events={m:r['events'][i] for m,r in runs.items()}
        assert len(set(e['summary']['section'] for e in events.values()))==1
        lo=max(min(row['s_student'] for row in e['rows']) for e in events.values())
        hi=min(max(row['s_student'] for row in e['rows']) for e in events.values())
        entry=dict(event=i+1,section=events['PID']['summary']['section'],route_start=lo,route_end=hi,methods={})
        for m,e in events.items():
            rows=[row for row in e['rows'] if lo<=row['s_student']<=hi]
            entry['methods'][m]=dict(samples=len(rows),median_gap=float(np.median([r['gap'] for r in rows])),
                own_target_rmse=float(np.sqrt(np.mean([r['error']**2 for r in rows]))))
        matched.append(entry)
    simulation=json.loads((OUT/'closed_loop.json').read_text())
    summary=dict(physical={m:r['summary'] for m,r in runs.items()},quality=details,matched_route=matched)
    (OUT/'physical_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    lines=['# PID, MPC and LQI 2.8: physical results and recorded-data closed-loop study','',
        'Analysis date: 2026-10-20. All three original controller files and the Shared Environment '
        'remain unchanged. Data are stored in `data/raw/group4_{pid,mpc,lqi}_2p8/` with archive manifests. '
        'The earlier Group 1 deployment-error run is excluded.', '',
        '## Findings','',
        '- All three labelled controller runs execute varying commands and maintain bounded gaps. '
        'There are no logged spacing-emergency flags in any active straight event.',
        '- MPC operates at smaller gaps, consistent with its shorter target policy. This is not '
        'by itself better tracking or better safety.',
        '- PID and LQI share the same target. LQI enters the braking event with a much larger gap '
        'and closes it substantially; its larger error in that event cannot be assigned solely '
        'to worse feedback control.',
        '- Reported-speed variation ratios favour PID in the two nonlaunch events with useful '
        'middle-speed excitation. These are descriptive telemetry ratios, not a proof of string stability.',
        '- Recorded-data ego closed-loop tests give MPC lower own-target RMSE in the assumed plant, '
        'but plant validation error is material and MPC has a different target. These do not override '
        'the physical comparison or establish a universal winner.', '',
        '## 1. Experimental comparability','',
        'Metadata identifies the same three vehicle roles/identifiers in all corrected runs; the user '
        'also confirms no physical student-vehicle change. Route, middle IDM v0/T/s0/a/b/delta, '
        'vehicle length, leader base speed and corner detector settings match. Student initial gaps '
        'and leader-middle initial gaps differ. The native TA spacing shield is 80 for PID '
        'and 50 for MPC/LQI. IDM min_spacing remains 80 for all.', '',
        'The three methods use these targets, with raw telemetry speed q:', '',
        '| Method | Target gap | Target at q=40 |',
        '|---|---|---:|',
        '| PID | 300 + 1.8 * 2.8q | 501.6 |',
        '| LQI | 300 + 1.8 * 2.8q | 501.6 |',
        '| MPC | 250 + 1.2 * 2.8q | 384.4 |', '',
        'Thus a 117.2-unit gap difference at q=40 is built into the design. Own-target errors '
        'measure execution of each policy, not a common optimization objective.', '',
        'No physical archive embeds source hashes or controller-internal diagnostics. Run labels '
        'come from the user. In particular, student_straight does not distinguish MPC optimizer '
        'output from its internal fallback. No firmware/communication-latency trace is available.', '',
        '## 2. Straight-event processing and data quality','',
        'Follow the Lab I event approach: identify consecutive student_straight decisions, verify '
        'middle/student share the same selected section, align all three trajectories on a 0.1 s '
        'grid without extrapolation, and reject interpolation spans above 0.3 s. Match by route '
        'section and occurrence, not whole-run time. The selected-section sequence is 3, 1, 2, 3. '
        'Event 1 includes launch, and event 4 ends when the recording stops.', '',
        'Spacing is forward route distance minus 250. Targets use aligned reported speed to match '
        'the controller input. No extra scaling is applied to position-derived speed. Sharp coordinate '
        'jumps were found in the PID data, and all datasets have some telemetry gaps around 0.5 s. '
        'These are not silently deleted as bad controller behavior. Physical acceleration, jerk '
        'and position-derived TTC need further outlier/timing validation before ranking methods.', '',
        '| Run | Leader gaps >0.3 s | Middle gaps >0.3 s | Student gaps >0.3 s |',
        '|---|---:|---:|---:|']
    for m,d in details.items():
        q=d['input_gaps'];lines.append(f'| {m} | {q["leader"]["gaps_over_point3"]} | {q["middle"]["gaps_over_point3"]} | {q["student"]["gaps_over_point3"]} |')
    lines+=['', 'A repeated coordinate-step feature occurs in the last straight near route '
        'coordinate 12,700. All three runs report speed 38, but one coordinate step is '
        'much larger than 2.8*38*dt would predict. These are flagged, not removed. '
        'They may reflect localization/timestamp behavior or position hold-and-catch-up; '
        'the logs do not identify the cause. Do not treat the resulting derivative spikes '
        'as proven physical acceleration or controller-induced discomfort.', '',
        '| Run | Time (s) | Route coordinate | Coordinate step | Sample interval (s) | Reported speed |',
        '|---|---:|---:|---:|---:|---:|']
    for m,d in details.items():
        for r in d['late_position_jumps']:
            lines.append(f'| {m} | {r["t"]:.3f} | {r["route_s"]:.1f} | {r["displacement"]:.1f} | {r["dt"]:.3f} | {r["reported_velocity"]:.0f} |')
    lines+=['', '## 3. Observed active-control overview','',
        'These statistics include launch but exclude noncontrol curves and bypass intervals.', '',
        '| Run | Duration (s) | Active decisions | Active gap min / median / max | Active sensor flags | Bypass sensor flags |',
        '|---|---:|---:|---|---:|---:|']
    for m,r in runs.items():
        s=r['summary'];lines.append(f'| {m} | {s["duration"]:.1f} | {s["active_calls"]} / {s["total_calls"]} | '
            f'{s["active_gap_min"]:.1f} / {s["active_gap_median"]:.1f} / {s["active_gap_max"]:.1f} | '
            f'{s["active_sensor_flags"]} | {s["bypass_sensor_flags"]} |')
    lines+=['', 'There are no active samples below the common 80-unit gap band. All three have '
        'launch samples below the controllers\' nominal 120-unit protective threshold. '
        'Sensor flags are obstacle-sensor events, not automatic evidence of a route-gap violation '
        'or a confirmed false positive. The different native spacing thresholds never triggered '
        'in these logs, so they do not directly explain the observed gap differences.', '',
        '## 4. Event-by-event tracking and disturbance behaviour','',
        '| Event | Method | Active interval (s) | Entry → exit gap | Own-target RMSE | Bias | Reported-speed SD ratio |',
        '|---|---|---|---|---:|---:|---:|']
    for i in range(4):
        for m,r in runs.items():
            e=r['events'][i]['summary'];ratio='quiet input' if e['reported_speed_ratio'] is None else f'{e["reported_speed_ratio"]:.3f}'
            lines.append(f'| {i+1} | {m} | {e["start"]:.1f}-{e["end"]:.1f} | {e["entry_gap"]:.1f} → {e["exit_gap"]:.1f} | '
                f'{e["own_rmse"]:.1f} | {e["bias"]:.1f} | {ratio} |')
    lines+=['', 'Positive bias means gap above target. Ratios are student/middle reported-speed '
        'standard deviation over the paired event; withheld when middle SD <1 raw unit. '
        'They are not physical acceleration/comfort metrics. Different event input histories '
        'and time/route coverage also affect the ratios.', '',
        '- **Event 1:** launch and different initial gaps dominate. PID also includes two consecutive '
        'center-sensor flags. Do not use this as the sole steady-state controller comparison.',
        '- **Event 2:** PID starts at about 241, MPC 147, LQI 292. LQI\'s entry is closer to '
        'the shared PID/LQI target; its lower error than PID here is not a controlled proof '
        'of faster recovery. All three expand the initially short gap.',
        '- **Event 3:** PID starts around 519 and ends 477; MPC 373 to 379; LQI 665 to 464. '
        'LQI inherits much more separation but removes about 201 units during active control. '
        'MPC holds near its smaller target policy. PID has lower telemetry speed variation '
        'than both others relative to each run\'s middle input.',
        '- **Event 4:** middle reported speed is constant, so ratio-based attenuation is '
        'not meaningful. Compare target bias and recovery only; the event is truncated.', '',
        '### Common route coverage','',
        'Intersection of ego route-coordinate coverage across all three corresponding events. '
        'Metrics use each run\'s retained samples inside that range, not interpolation over holes. '
        'This reduces route-coverage differences but does not equalize entry state or disturbances.', '',
        '| Event | Common distance (cu) | PID median gap / RMSE | MPC median gap / RMSE | LQI median gap / RMSE |',
        '|---|---:|---|---|---|']
    for e in matched:
        cells=[f'{e["methods"][m]["median_gap"]:.1f} / {e["methods"][m]["own_target_rmse"]:.1f}' for m in RUNS]
        lines.append(f'| {e["event"]} | {e["route_end"]-e["route_start"]:.1f} | '+' | '.join(cells)+' |')
    lines+=['', '### Command smoothness (not physical jerk)','',
        'Compute differences between integer-rounded sent-speed targets for consecutive active '
        'decisions only; exclude pairs containing a TA emergency flag. No derivatives across '
        'curve/bypass boundaries. These quantify command activity, not actual acceleration or '
        'comfort; early launch and each method\'s protections still contribute.', '',
        '| Method | Valid command pairs | RMS command increment | 95th percentile absolute increment |',
        '|---|---:|---:|---:|']
    for m,d in details.items():lines.append(f'| {m} | {d["command_increment_pairs"]} | {d["command_increment_rms"]:.2f} | {d["command_increment_p95"]:.1f} |')
    lines+=['', '## 5. Vehicle-response model: open-loop validation first','',
        'Fit a simple first-order command response with a separate command-to-coordinate-speed '
        'gain, lag and command delay. Use the recorded rounded shielded commands, including '
        'real stops. Fit on alternating 5 s blocks using robust 1 s route-displacement residuals; '
        'evaluate the other blocks. Reject intervals crossing large telemetry gaps or implausible '
        'negative/very large displacement. Candidate lags: 0.1-1.8 s; delays: 0-0.5 s.', '',
        '| Source run | Fitted gain | Lag (s) | Delay (s) | Held-out 1 s displacement RMSE (cu) | Relative RMSE |',
        '|---|---:|---:|---:|---:|---:|']
    for m,f in simulation['plant_fits'].items():lines.append(f'| {m} | {f["beta"]:.3f} | {f["lag"]:.2f} | {f["delay"]:.2f} | {f["validation_displacement_rmse"]:.2f} | {100*f["validation_relative_rmse"]:.1f}% |')
    lines+=['', 'This is an imperfect exploratory model, not a validated digital twin. PID/MPC '
        'fits hit the minimum tested lag, indicating weak identification or missing dynamics. '
        'The LQI fit selects a different lag despite the same vehicle, further warning against '
        'interpreting the fit as a physical constant. Interleaved held-out blocks are not '
        'independent-run validation. Gains near 2.9 do not establish a new scaling correction; '
        'they absorb model error and do not change the submitted controllers.', '',
        '## 6. Individual closed-loop simulations using actual test inputs','',
        'For each of the three recordings, replay measured leader and middle positions and '
        'reported speeds. Initialize simulated ego once from the measured initial state. '
        'Thereafter each controller receives only its own simulated ego state/history; its '
        'commands determine subsequent ego motion. The measured ego is never injected back '
        'into the feedback. Run the original three 2.8 files separately on each input recording.', '',
        'Use actual command decision timestamps, route gating, integer rounding and the source '
        'run\'s TA spacing threshold. Simulated telemetry maps coordinate speed by the 2.8 '
        'hypothesis; command response uses the independently fitted gain. The plant response '
        'is integrated at <=0.02 s steps. Sensor readings are nominal: recorded sensor events '
        'are not transferred to counterfactual trajectories. Source holes are filled linearly '
        'for continuity, but active samples requiring source interpolation spans >0.3 s are '
        'excluded from scoring. Dynamics across the holes remain uncertain.', '',
        'Callback computation does not advance simulated time. GUI scheduling, asynchronous '
        'packet arrival, and unmeasured end-to-end vehicle-command latency are not reconstructed. '
        'Fitted delay is an effective model term, not a direct network-latency measurement.', '',
        'Nine baseline runs plus half/double response-lag sensitivity yield 27 closed-loop '
        'runs. This isolates ego feedback and avoids adding middle-IDM model error. It does '
        'not simulate a fully endogenous three-vehicle platoon.', '',
        '| Recorded front-vehicle input | Simulated PID RMSE | Simulated MPC RMSE | Simulated LQI RMSE |',
        '|---|---:|---:|---:|']
    for source in RUNS:
        vals={r['controller']:r['own_rmse'] for r in simulation['runs'] if r['source']==source and r['lag_multiplier']==1}
        lines.append(f'| {source} run | {vals["PID"]:.1f} | {vals["MPC"]:.1f} | {vals["LQI"]:.1f} |')
    sim=simulation['runs']
    lines+=['',f'Across these 27 model runs: {sum(r["collisions"]>0 for r in sim)} runs with sampled negative gap; '
        f'{sum(r["shield_samples"]>0 for r in sim)} with TA spacing-shield activation; '
        f'{sum(r["wrapper_errors"] for r in sim)} callback exceptions.', '',
        'MPC initialization/optimizer was available in these simulations, and its internal '
        'modes are logged in `closed_loop.json`. Protective stop modes are not solver fallback. '
        'This simulated mode information does not recover missing optimizer status from the physical run.', '',
        'MPC has lower own-target RMSE in this model, while PID is lower than LQI. This is '
        'conditional on imperfect plant fits, synthetic telemetry behavior, nominal sensors '
        'and different target policies. Recorded-ego versus same-method simulated-ego '
        'position RMSE is about 79-121 cu in the baseline fits; the simulation does not '
        'reproduce physical trajectories exactly. Do not treat its ranking as experimental proof.', '',
        '## 7. Defensible report conclusion','',
        'The three physical runs show bounded following without logged spacing emergencies '
        'during active straight control. MPC uses a closer following policy and shows smaller '
        'own-target errors in several events. PID shows lower reported-speed variation in '
        'the nonlaunch disturbance events. LQI has a large inherited braking-event gap '
        'but closes it substantially. No single method is demonstrated best across all '
        'objectives from one nonidentically initialized run per method.', '',
        'The recorded-data closed-loop study is supplementary sensitivity evidence. Its '
        'useful contribution is applying each unchanged controller to the same recorded '
        'front-vehicle inputs and initial ego state within each scenario, while making '
        'plant-model shortcomings explicit. Further repetitions and better plant validation '
        'would be needed for a stronger performance ranking. No controller tuning is proposed here.', '',
        '## Artifacts and reproduction','',
        '- [Physical event comparison](../results/three_controller_analysis/physical_events.png)',
        '- [Model validation](../results/three_controller_analysis/plant_validation.png)',
        '- [Closed-loop versus recorded trajectories](../results/three_controller_analysis/closed_loop.png)',
        '- Physical metrics: `results/three_controller_analysis/physical_summary.json`.',
        '- Model fits and 27-run metrics: `results/three_controller_analysis/closed_loop.json`.',
        '- Aligned physical event CSVs and baseline closed-loop traces are in the same folder.', '',
        '```sh', 'python src/simulate_recorded_inputs.py',
        'python src/analyze_physical_runs.py', '```','']
    (ROOT/'docs/three_controller_analysis.md').write_text('\n'.join(lines),encoding='utf-8')
    plots(runs)
    print(json.dumps(dict(command_activity=details,matched_route=matched),indent=2))


def plots(runs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={'PID':'#2878b5','MPC':'#e58b22','LQI':'#31965b'}
    fig,axes=plt.subplots(2,4,figsize=(17,8))
    for i in range(4):
        for m,r in runs.items():
            rows=r['events'][i]['rows'];x=np.array([row['s_student'] for row in rows])
            breaks=np.flatnonzero(np.diff([row['t'] for row in rows])>.15)+1
            x=np.insert(x,breaks,np.nan)
            val=lambda key:np.insert(np.array([row[key] for row in rows]),breaks,np.nan)
            axes[0,i].plot(x,val('gap'),color=colors[m],label=m+' gap')
            axes[0,i].plot(x,val('target'),color=colors[m],ls='--',alpha=.55)
            axes[1,i].plot(x,val('student_v'),color=colors[m],label=m+' student')
            axes[1,i].plot(x,val('middle_v'),color=colors[m],ls=':',alpha=.6)
        axes[0,i].set_title(f'Event {i+1}'+(' (launch)' if i==0 else ''))
        axes[0,i].legend(fontsize=8);axes[1,i].legend(fontsize=7)
        axes[1,i].set_xlabel('Ego route coordinate (cu)')
        for ax in axes[:,i]:ax.grid(alpha=.2)
    axes[0,0].set_ylabel('Gap (solid) / target (dashed)')
    axes[1,0].set_ylabel('Reported speed; dotted = middle')
    fig.suptitle('Three physical runs: active straight events only, matched by route section')
    fig.tight_layout();fig.savefig(OUT/'physical_events.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,8))
    for ax,m in zip(axes,RUNS):
        rows=list(csv.DictReader((OUT/f'plant_validation_{m.lower()}.csv').open()))
        for key,label in [('observed_displacement','Measured'),('predicted_displacement','Model')]:
            ax.plot([float(r['t']) for r in rows],[float(r[key]) if r['valid']=='1' else np.nan for r in rows],label=label)
        ax.set_title(m+' command-response fit');ax.set_ylabel('1 s displacement (cu)');ax.legend();ax.grid(alpha=.2)
    axes[-1].set_xlabel('Run time (s)');fig.tight_layout();fig.savefig(OUT/'plant_validation.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(15,8))
    for col,source in enumerate(RUNS):
        for m in RUNS:
            rows=list(csv.DictReader((OUT/f'replay_input_{source.lower()}_controller_{m.lower()}.csv').open()))
            t=np.array([float(r['t']) for r in rows])
            axes[0,col].plot(t,[float(r['gap']) for r in rows],color=colors[m],label=m)
            axes[1,col].plot(t,[float(r['v']) for r in rows],color=colors[m],label=m)
        meta,cmd,data=raw_data(RUNS[source]);ct=np.array([float(r['t_from_start']) for r in cmd])
        axes[0,col].plot(ct,[float(r['spacing']) for r in cmd],color='black',ls='--',alpha=.5,label='Physical '+source)
        rt,rv=data['student'];axes[1,col].plot(rt,2.8*rv['velocity'],color='black',ls='--',alpha=.5)
        axes[0,col].set_title('Recorded '+source+' front-vehicle input')
        axes[0,col].legend(fontsize=8);axes[1,col].set_xlabel('Time (s)')
        for ax in axes[:,col]:ax.grid(alpha=.2)
    axes[0,0].set_ylabel('Simulated gap / recorded reference')
    axes[1,0].set_ylabel('Coordinate speed (recorded = 2.8q)')
    fig.suptitle('Ego closed-loop counterfactuals; imperfect fitted plant, not a physical replication')
    fig.tight_layout();fig.savefig(OUT/'closed_loop.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
