"""Descriptive platoon metrics; no controller edits or calibrated fuel claims."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/three_controller_analysis'
records = []
for method in ('PID', 'MPC', 'LQI'):
    for event in range(1, 5):
        with (OUT / f'{method.lower()}_event_{event}.csv').open() as f:
            rows = list(csv.DictReader(f))
        a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        t, gap, v = a['t'], a['gap'], 2.8*a['student_v']
        moving = a['student_v'] >= 5
        std = {r: float(np.std(a[r+'_v'])) for r in ('leader', 'middle', 'student')}
        gains = {f'{out}/{inp}': (std[out]/std[inp] if std[inp] >= 1 else None)
                 for out, inp in [('middle','leader'),('student','middle'),('student','leader')]}
        # Accumulate only within adjacent retained samples, never across rejected holes.
        pair = (np.diff(t) <= .100001) & (np.diff(a['s_student']) >= 0)
        positive_kinetic = np.maximum(np.diff(.5*v*v), 0)
        distance = np.diff(a['s_student'])[pair].sum()
        rec = dict(method=method, event=event, gains=gains,
                   moving_samples=int(moving.sum()),
                   gap_time_median=float(np.median(gap[moving]/v[moving])),
                   space_time_median=float(np.median((gap[moving]+250)/v[moving])),
                   local_density_median=float(np.median(1000/(gap[moving]+250))),
                   local_flow_proxy_median=float(np.median(v[moving]/(gap[moving]+250))),
                   positive_kinetic_variation_per_distance=float(positive_kinetic[pair].sum()/distance),
                   retained_pair_distance=float(distance))
        records.append(rec)
(OUT/'macro_metrics.json').write_text(json.dumps(records,indent=2),encoding='utf8')
fig, axes = plt.subplots(1,2,figsize=(10,3.8))
colors = {'PID':'#2166ac','MPC':'#b35806','LQI':'#5e3c99'}
for method in colors:
    r = [x for x in records if x['method']==method]
    axes[0].plot(range(1,5),[x['gap_time_median'] for x in r],'o-',label=method,color=colors[method])
    axes[1].plot(range(1,4),[x['gains']['student/middle'] for x in r[:3]],'o-',label=method,color=colors[method])
axes[0].set(ylabel='Median bumper gap / estimated speed (s)',xlabel='Straight event',xticks=range(1,5))
axes[1].axhline(1,color='black',ls='--',lw=1)
axes[1].set(ylabel='Reported speed SD ratio ego / middle',xlabel='Straight event',xticks=range(1,4))
for ax in axes: ax.grid(alpha=.2); ax.legend()
fig.suptitle('Descriptive headways and disturbance propagation under the 2.8 hypothesis')
fig.tight_layout()
fig.savefig(OUT/'macro_metrics.png',dpi=190)
print(json.dumps(records,indent=2))
