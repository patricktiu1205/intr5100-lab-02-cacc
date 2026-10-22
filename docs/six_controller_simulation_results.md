# All-six controller simulation

Date: 2026-10-20. All six submitted controller files were left unchanged; source hashes were checked before and after execution.

## Method

60 runs: 6 controllers x 2 hypothetical plant scales x 5 scenarios. Each run is 240 s at a 0.2 s control period; plant integration uses 0.05 s substeps. Scenarios are default route events, 0.4 s command delay plus noise, slower response, 160-unit close start, and an additional abrupt leader stop. Seed: 5100.

Every candidate is invoked through its original student_control callback with actual VehicleState objects and the TA history schema. The supplied IDM, leader event manager, map, section gates, command rounding and spacing shield are represented. The plant has heterogeneous first-order response lags and acceleration limits. All candidates see the same scenario, initial state, and noise sequence within a plant assumption.

Plant scale means both coordinate-speed/reported-speed and coordinate-speed-target/command gain are set to that value. This imposes a hypothetical mapping; it does not reproduce or identify the unknown real-lab scaling mechanism. Both submitted versions are run under both assumptions without adjusting any scaling formulas.

## Safety and execution

- Collisions: 0 of 60 runs.
- Runs with student TA spacing-shield activation: 0.
- Wrapper controller exceptions: 0.
- Minimum sampled physical bumper gap: 160.00 coordinate units.

| Controller | MPC calls | Internal fallback calls | Internal stop calls | Worst steady p95 callback ms |
|---|---:|---:|---:|---:|
| PID-1 | 0 | 0 | 0 | 1.39 |
| PID-2.8 | 0 | 0 | 0 | 1.12 |
| LQI-1 | 0 | 0 | 0 | 1.12 |
| LQI-2.8 | 0 | 0 | 0 | 1.14 |
| MPC-1 | 5101 | 1 | 0 | 1.84 |
| MPC-2.8 | 4925 | 0 | 0 | 1.82 |

The internal mode counts above are diagnostic modes exposed by the MPC files; PID/LQI do not expose the same stop/fallback classification, so their zero entries are not a count of all possible protective actions. Internal protective stops are distinct from exceptions or the TA shield. MPC fallback rates must accompany any MPC performance claims. Callback times exclude module import and exclude the first call from p95; import/first-call timings remain in JSON.

## Default route scenario

Own-gap RMSE uses each method's nominal physical target: PID/LQI use 300 + 1.8v; MPC uses 250 + 1.2v. For mismatched scale versions this is not necessarily the numerical target inside the controller. Common-gap RMSE uses 300 + 1.8v for all as an explicitly chosen reference, not a neutral optimality criterion. Speeds v here are true simulated coordinate speeds. Ratio is student/middle standard deviation over controller-active samples. Below one indicates less variation in that finite window, not proven string stability.

| Plant | Controller | Own-gap RMSE | Common-gap RMSE | Speed variation ratio | Min gap | Mean speed | Jerk RMS |
|---:|---|---:|---:|---:|---:|---:|---:|
| 1.0 | PID-1 | 38.83 | 38.83 | 0.908 | 332.3 | 34.60 | 4.01 |
| 1.0 | PID-2.8 | 126.26 | 126.26 | 0.673 | 420.0 | 34.23 | 3.26 |
| 1.0 | LQI-1 | 37.99 | 37.99 | 1.024 | 302.1 | 34.50 | 4.29 |
| 1.0 | LQI-2.8 | 112.14 | 112.14 | 0.591 | 369.7 | 34.29 | 3.88 |
| 1.0 | MPC-1 | 30.70 | 64.81 | 1.276 | 281.1 | 34.83 | 4.97 |
| 1.0 | MPC-2.8 | 96.98 | 40.75 | 0.862 | 336.3 | 34.52 | 4.97 |
| 2.8 | PID-1 | 100.27 | 100.27 | 0.967 | 334.1 | 95.89 | 12.94 |
| 2.8 | PID-2.8 | 49.61 | 49.61 | 0.889 | 397.4 | 95.37 | 11.23 |
| 2.8 | LQI-1 | 104.59 | 104.59 | 0.964 | 306.2 | 95.88 | 14.78 |
| 2.8 | LQI-2.8 | 52.73 | 52.73 | 0.911 | 376.1 | 95.34 | 13.48 |
| 2.8 | MPC-1 | 66.99 | 166.34 | 1.107 | 268.3 | 96.19 | 17.18 |
| 2.8 | MPC-2.8 | 30.14 | 99.79 | 0.943 | 311.5 | 95.83 | 15.32 |

## Matched-version sensitivity summary

Each row aggregates five cases only where the submitted scale matches the hypothetical plant. This is a conditional comparison; the correct real-lab assumption is unknown.

| Plant | Controller | Mean own-gap RMSE | Mean variation ratio | Worst min gap | Mean jerk RMS |
|---:|---|---:|---:|---:|---:|
| 1.0 | PID-1 | 46.09 | 0.810 | 160.0 | 4.79 |
| 1.0 | LQI-1 | 47.31 | 0.936 | 160.0 | 4.40 |
| 1.0 | MPC-1 | 33.07 | 1.193 | 160.0 | 5.53 |
| 2.8 | PID-2.8 | 57.89 | 0.861 | 160.0 | 12.28 |
| 2.8 | LQI-2.8 | 61.47 | 0.877 | 160.0 | 13.62 |
| 2.8 | MPC-2.8 | 34.00 | 0.924 | 160.0 | 18.44 |

## Preliminary conclusion

Among matching versions averaged across the five scenarios, PID has the lowest speed-variation ratio under both plant assumptions. MPC has the lowest error relative to its method-specific gap target, but uses a shorter headway and generally produces more jerk. LQI has the lowest mean jerk in the unscaled plant; PID has the lowest mean jerk in the scaled plant. There is no demonstrated all-objective winner.

## Interpretation limits

- This compares the six submitted implementations, not universally optimal PID/LQI/MPC methods.
- Different headway targets, horizons, protections and tunings prevent a single fair method score.
- A low speed-variation ratio can reflect slow progress or excess gap; inspect mean speed and gaps too.
- Active intervals vary with each controller trajectory and gate transitions; ratios are descriptive and not frequency-response estimates over identical windows.
- Noise acts on positions/telemetry; command delay is simulated. Packet drops, asynchronous delivery, future sensor faults, lateral dynamics and controller-computation delay are not simulated.
- Runtime measurements are real host timings, but simulation time does not advance during computation.
- Acceleration/jerk use unsmoothed finite differences and include route transitions and command quantization.
- No gains or scaling formulas were changed, and no automatic tuning was performed.
- Simulated safety outcomes do not guarantee physical safety or select the correct scaling assumption.

## Reproduce

```sh
python -m pip install -r requirements-mpc.txt
python src/simulate_six_controllers.py
python src/summarize_six_controller_simulations.py
```

Files: [full metrics](../simulation_results/all_six/metrics.json), [summary CSV](../simulation_results/all_six/metrics.csv), [comparison chart](../simulation_results/all_six/comparison.png). Nominal trajectories for all six under both plant assumptions are saved alongside them.
