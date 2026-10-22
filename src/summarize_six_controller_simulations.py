"""Summarize six-controller simulation without collapsing tradeoffs into a score."""
import json
import os
from pathlib import Path
import csv
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.mplconfig'))
OUT=ROOT/'simulation_results/all_six'


def main():
    rows=json.loads((OUT/'metrics.json').read_text())
    assert len(rows)==60, 'Wait for the complete simulation matrix'
    lines=['# All-six controller simulation','',
        'Date: 2026-10-20. All six submitted controller files were left unchanged; '
        'source hashes were checked before and after execution.', '',
        '## Method','',
        '60 runs: 6 controllers x 2 hypothetical plant scales x 5 scenarios. '
        'Each run is 240 s at a 0.2 s control period; plant integration uses 0.05 s substeps. '
        'Scenarios are default route events, 0.4 s command delay plus noise, slower response, '
        '160-unit close start, and an additional abrupt leader stop. Seed: 5100.', '',
        'Every candidate is invoked through its original student_control callback with actual '
        'VehicleState objects and the TA history schema. The supplied IDM, leader event manager, '
        'map, section gates, command rounding and spacing shield are represented. The plant '
        'has heterogeneous first-order response lags and acceleration limits. All candidates '
        'see the same scenario, initial state, and noise sequence within a plant assumption.', '',
        'Plant scale means both coordinate-speed/reported-speed and coordinate-speed-target/command '
        'gain are set to that value. This imposes a hypothetical mapping; it does not reproduce '
        'or identify the unknown real-lab scaling mechanism. Both submitted versions are run '
        'under both assumptions without adjusting any scaling formulas.', '',
        '## Safety and execution','',
        f'- Collisions: {sum(r["collision"] for r in rows)} of 60 runs.',
        f'- Runs with student TA spacing-shield activation: {sum(r["shield_samples"]>0 for r in rows)}.',
        f'- Wrapper controller exceptions: {sum(r["wrapper_errors"] for r in rows)}.',
        f'- Minimum sampled physical bumper gap: {min(r["min_gap"] for r in rows):.2f} coordinate units.', '',
        '| Controller | MPC calls | Internal fallback calls | Internal stop calls | Worst steady p95 callback ms |',
        '|---|---:|---:|---:|---:|']
    names=list(dict.fromkeys(r['controller'] for r in rows))
    for name in names:
        group=[r for r in rows if r['controller']==name]
        counts={k:sum(r['internal_modes'].get(k,0) for r in group) for k in ('mpc','fallback','stop')}
        lines.append(f'| {name} | {counts["mpc"]} | {counts["fallback"]} | {counts["stop"]} | '
                     f'{max(r["p95_call_ms"] for r in group):.2f} |')
    lines += ['', 'The internal mode counts above are diagnostic modes exposed by the MPC files; '
        'PID/LQI do not expose the same stop/fallback classification, so their zero entries are not '
        'a count of all possible protective actions. Internal protective stops are distinct from exceptions or the TA shield. '
        'MPC fallback rates must accompany any MPC performance claims. Callback times exclude '
        'module import and exclude the first call from p95; import/first-call timings remain in JSON.', '',
        '## Default route scenario','',
        'Own-gap RMSE uses each method\'s nominal physical target: PID/LQI use 300 + 1.8v; MPC uses 250 + 1.2v. '
        'For mismatched scale versions this is not necessarily the numerical target inside the controller. '
        'Common-gap RMSE uses 300 + 1.8v for all as an explicitly chosen reference, not a neutral '
        'optimality criterion. Speeds v here are true simulated coordinate speeds. Ratio is '
        'student/middle standard deviation over controller-active samples. Below one indicates '
        'less variation in that finite window, not proven string stability.', '',
        '| Plant | Controller | Own-gap RMSE | Common-gap RMSE | Speed variation ratio | Min gap | Mean speed | Jerk RMS |',
        '|---:|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        if r['case']=='nominal':
            lines.append(f'| {r["plant_scale"]} | {r["controller"]} | {r["own_gap_rmse"]:.2f} | '
                f'{r["common_gap_rmse"]:.2f} | {r["variation_ratio"]:.3f} | {r["min_gap"]:.1f} | '
                f'{r["mean_speed"]:.2f} | {r["jerk_rms"]:.2f} |')
    lines += ['', '## Matched-version sensitivity summary','',
        'Each row aggregates five cases only where the submitted scale matches the hypothetical '
        'plant. This is a conditional comparison; the correct real-lab assumption is unknown.', '',
        '| Plant | Controller | Mean own-gap RMSE | Mean variation ratio | Worst min gap | Mean jerk RMS |',
        '|---:|---|---:|---:|---:|---:|']
    for plant in (1.,2.8):
        for method in ('PID','LQI','MPC'):
            name=f'{method}-{plant:g}'
            group=[r for r in rows if r['controller']==name and r['plant_scale']==plant]
            lines.append(f'| {plant} | {name} | {np.mean([r["own_gap_rmse"] for r in group]):.2f} | '
                f'{np.mean([r["variation_ratio"] for r in group]):.3f} | '
                f'{min(r["min_gap"] for r in group):.1f} | {np.mean([r["jerk_rms"] for r in group]):.2f} |')
    lines += ['', '## Preliminary conclusion','',
        'Among matching versions averaged across the five scenarios, PID has the lowest speed-variation '
        'ratio under both plant assumptions. MPC has the lowest error relative to its method-specific '
        'gap target, but uses a shorter headway and generally produces more jerk. LQI has the lowest '
        'mean jerk in the unscaled plant; PID has the lowest mean jerk in the scaled plant. '
        'There is no demonstrated all-objective winner.', '',
        '## Interpretation limits','',
        '- This compares the six submitted implementations, not universally optimal PID/LQI/MPC methods.',
        '- Different headway targets, horizons, protections and tunings prevent a single fair method score.',
        '- A low speed-variation ratio can reflect slow progress or excess gap; inspect mean speed and gaps too.',
        '- Active intervals vary with each controller trajectory and gate transitions; ratios are descriptive '
        'and not frequency-response estimates over identical windows.',
        '- Noise acts on positions/telemetry; command delay is simulated. Packet drops, asynchronous delivery, '
        'future sensor faults, lateral dynamics and controller-computation delay are not simulated.',
        '- Runtime measurements are real host timings, but simulation time does not advance during computation.',
        '- Acceleration/jerk use unsmoothed finite differences and include route transitions and command quantization.',
        '- No gains or scaling formulas were changed, and no automatic tuning was performed.',
        '- Simulated safety outcomes do not guarantee physical safety or select the correct scaling assumption.', '',
        '## Reproduce','', '```sh',
        'python -m pip install -r requirements_mpc.txt',
        'python src/simulate_six_controllers.py',
        'python src/summarize_six_controller_simulations.py', '```', '',
        'Files: [full metrics](../simulation_results/all_six/metrics.json), '
        '[summary CSV](../simulation_results/all_six/metrics.csv), '
        '[comparison chart](../simulation_results/all_six/comparison.png). '
        'Nominal trajectories for all six under both plant assumptions are saved alongside them.', '']
    (ROOT/'docs/six_controller_simulation_results.md').write_text('\n'.join(lines),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True)
    colors={'PID':'#2878b5','LQI':'#e58b22','MPC':'#31965b'}
    for col,plant in enumerate((1.,2.8)):
        for name in names:
            method, scale = name.lower().split('-')
            plant_label = f'{plant:g}'.replace(".", "p")
            filename = f'{method}_scale_{scale.replace(".", "p")}_plant_{plant_label}.csv'
            with (OUT/filename).open() as f: trace=list(csv.DictReader(f))
            t=[float(r['t']) for r in trace]
            style='-' if name.endswith(f'-{plant:g}') else '--'
            for row,key in enumerate(('gap','v2')):
                axes[row,col].plot(t,[float(r[key]) for r in trace],style,
                    color=colors[name.split('-')[0]],label=name,alpha=.85,lw=1.2)
        axes[0,col].set_title(f'Plant scale {plant}: default route events')
        axes[0,col].set_ylabel('Bumper gap (coordinate units)')
        axes[0,col].axhline(80,color='red',ls=':',lw=1)
        axes[1,col].set_ylabel('Coordinate speed / s')
        axes[1,col].set_xlabel('Time (s)')
        axes[0,col].legend(ncol=2,fontsize=8)
    fig.suptitle('Six unchanged submissions; solid = matching hypothesis, dashed = other version')
    fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
    print('All-six report and chart written')


if __name__=='__main__': main()
