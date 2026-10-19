# Offline simulation report

Run date: 2026-10-19. Python 3.12; deterministic seed 5100.

78 cases: 0 collisions and 0 cases with student TA spacing-emergency triggers.
Minimum sampled bumper gap across all cases: 128.35 coordinate units.

## Matched-scale comparisons

Spacing errors and forecast errors are in coordinate units and coordinate units/s, respectively. Speed RMS ratio is student/middle speed variation over active samples. Lower is not automatically better: excessive spacing error or lag can also suppress variation.

| Scenario | Scale | Reference | Gap RMSE | Speed RMS ratio | Forecast RMSE |
|---|---:|---|---:|---:|---:|
| nominal | 1.0 | current | 11.10 | 0.963 | 1.15 |
| nominal | 1.0 | trend | 12.37 | 0.950 | 0.40 |
| nominal | 1.0 | idm | 12.82 | 0.943 | 0.28 |
| route_defaults | 1.0 | current | 37.42 | 0.926 | 1.25 |
| route_defaults | 1.0 | trend | 37.34 | 0.929 | 1.40 |
| route_defaults | 1.0 | idm | 37.25 | 0.923 | 0.74 |
| nominal | 2.8 | current | 32.16 | 0.893 | 4.84 |
| nominal | 2.8 | trend | 38.73 | 0.884 | 2.61 |
| nominal | 2.8 | idm | 39.12 | 0.864 | 1.02 |
| route_defaults | 2.8 | current | 48.49 | 0.879 | 4.75 |
| route_defaults | 2.8 | trend | 50.17 | 0.885 | 5.39 |
| route_defaults | 2.8 | idm | 49.23 | 0.875 | 2.74 |

## Interpretation

IDM prediction reduces middle-speed forecast error and improves the nominal speed-variation ratio relative to the current-speed baseline. Nominal spacing RMSE increases for both scale hypotheses. On the actual route, the scaled predictive controller also has slightly higher spacing RMSE than the scaled current-speed baseline. The predictive design is therefore not uniformly superior.

No automatic gain search was performed. The next tuning decision is how much spacing precision to trade for anticipation and attenuation, particularly the blend weight and prediction horizon.

## Coverage and limits

- 72 long-straight cases: 2 controller scales x 2 telemetry scales x 3 references x 6 scenarios.
- 6 actual-route cases: 2 matching scale hypotheses x 3 references; 240 s each.
- Long-straight scenarios include nominal ramps, 0.4 s delay plus position/velocity noise, slower response, 160-unit close starts, abrupt leader stopping, and independent command-scale mismatch.
- Integration/control steps are 0.2 s. Middle plant lag is 1.2 times the configured scenario lag; the predictor assumes 0.5 s. The simulator uses the supplied IDM implementation, not our replica.
- Actual-route runs reproduce default event scheduling and control-section switching, with ideal lateral tracking. The straight tests deliberately use a different prescribed leader waveform.
- Acceleration and jerk metrics use unsmoothed finite differences of simulated motion; command quantization affects jerk. They are not directly comparable with filtered Lab I metrics.
- No collision in these discrete scenarios establishes neither continuous-time safety nor string stability.
- The results cannot choose the correct 2.8 hypothesis; platform measurements are required.

## Outputs

- [All metrics](simulation_results/metrics.csv)
- [Machine-readable metrics](simulation_results/metrics.json)
- [Nominal comparison chart](simulation_results/comparison.png)
- Selected nominal and actual-route traces are in `simulation_results/trajectory_*.csv`.
