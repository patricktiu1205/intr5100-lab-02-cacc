"""Generate a concise Markdown report from recorded simulation metrics."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
rows=json.loads((root/'simulation_results/metrics.json').read_text())
text=['# Offline simulation report','',
      'Run date: 2026-10-19. Python 3.12; deterministic seed 5100.', '',
      f'{len(rows)} cases: {sum(r["collision"] for r in rows)} collisions and '
      f'{sum(r["emergency_samples"]>0 for r in rows)} cases with student TA spacing-emergency triggers.',
      f'Minimum sampled bumper gap across all cases: {min(r["min_gap"] for r in rows):.2f} coordinate units.',
      '', '## Matched-scale comparisons','',
      'Spacing errors and forecast errors are in coordinate units and coordinate units/s, respectively. '
      'Speed RMS ratio is student/middle speed variation over active samples. Lower is not automatically better: '
      'excessive spacing error or lag can also suppress variation.', '',
      '| Scenario | Scale | Reference | Gap RMSE | Speed RMS ratio | Forecast RMSE |',
      '|---|---:|---|---:|---:|---:|']
for r in rows:
    if r['scale']==r['plant_scale'] and r['case'] in ('nominal','route_defaults'):
        text.append(f'| {r["case"]} | {r["scale"]} | {r["prediction"]} | '
                    f'{r["spacing_rmse"]:.2f} | {r["speed_rms_ratio_21"]:.3f} | {r["forecast_rmse"]:.2f} |')
text += ['', '## Interpretation', '',
    'IDM prediction reduces middle-speed forecast error and improves the nominal speed-variation ratio '
    'relative to the current-speed baseline. Nominal spacing RMSE increases for both scale hypotheses. '
    'On the actual route, the scaled predictive controller also has slightly higher spacing RMSE than '
    'the scaled current-speed baseline. The predictive design is therefore not uniformly superior.', '',
    'No automatic gain search was performed. The next tuning decision is how much spacing precision to '
    'trade for anticipation and attenuation, particularly the blend weight and prediction horizon.', '',
    '## Coverage and limits', '',
    '- 72 long-straight cases: 2 controller scales x 2 telemetry scales x 3 references x 6 scenarios.',
    '- 6 actual-route cases: 2 matching scale hypotheses x 3 references; 240 s each.',
    '- Long-straight scenarios include nominal ramps, 0.4 s delay plus position/velocity noise, '
    'slower response, 160-unit close starts, abrupt leader stopping, and independent command-scale mismatch.',
    '- Integration/control steps are 0.2 s. Middle plant lag is 1.2 times the configured scenario lag; '
    'the predictor assumes 0.5 s. The simulator uses the supplied IDM implementation, not our replica.',
    '- Actual-route runs reproduce default event scheduling and control-section switching, with ideal '
    'lateral tracking. The straight tests deliberately use a different prescribed leader waveform.',
    '- Acceleration and jerk metrics use unsmoothed finite differences of simulated motion; command '
    'quantization affects jerk. They are not directly comparable with filtered Lab I metrics.',
    '- No collision in these discrete scenarios establishes neither continuous-time safety nor string stability.',
    '- The results cannot choose the correct 2.8 hypothesis; platform measurements are required.', '',
    '## Outputs', '',
    '- [All metrics](../simulation_results/metrics.csv)',
    '- [Machine-readable metrics](../simulation_results/metrics.json)',
    '- [Nominal comparison chart](../simulation_results/comparison.png)',
    '- Selected nominal and actual-route traces are in `simulation_results/trajectory_*.csv`.', '']
(root/'docs/pid_simulation_results.md').write_text('\n'.join(text),encoding='utf-8')
print('Report written')
