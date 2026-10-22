# PID, MPC and LQI 2.8: physical results and recorded-data closed-loop study

Analysis date: 2026-10-20. All three original controller files and the Shared Environment remain unchanged. Data are stored in `data/raw/group4_{pid,mpc,lqi}_2p8/` with archive manifests. The earlier Group 1 deployment-error run is excluded.

## Findings

- All three labelled controller runs execute varying commands and maintain bounded gaps. There are no logged spacing-emergency flags in any active straight event.
- MPC operates at smaller gaps, consistent with its shorter target policy. This is not by itself better tracking or better safety.
- PID and LQI share the same target. LQI enters the braking event with a much larger gap and closes it substantially; its larger error in that event cannot be assigned solely to worse feedback control.
- Reported-speed variation ratios favour PID in the two nonlaunch events with useful middle-speed excitation. These are descriptive telemetry ratios, not a proof of string stability.
- Recorded-data ego closed-loop tests give MPC lower own-target RMSE in the assumed plant, but plant validation error is material and MPC has a different target. These do not override the physical comparison or establish a universal winner.

## 1. Experimental comparability

Metadata identifies the same three vehicle roles/identifiers in all corrected runs; the user also confirms no physical student-vehicle change. Route, middle IDM v0/T/s0/a/b/delta, vehicle length, leader base speed and corner detector settings match. Student initial gaps and leader-middle initial gaps differ. The native TA spacing shield is 80 for PID and 50 for MPC/LQI. IDM min_spacing remains 80 for all.

The three methods use these targets, with raw telemetry speed q:

| Method | Target gap | Target at q=40 |
|---|---|---:|
| PID | 300 + 1.8 * 2.8q | 501.6 |
| LQI | 300 + 1.8 * 2.8q | 501.6 |
| MPC | 250 + 1.2 * 2.8q | 384.4 |

Thus a 117.2-unit gap difference at q=40 is built into the design. Own-target errors measure execution of each policy, not a common optimization objective.

No physical archive embeds source hashes or controller-internal diagnostics. Run labels come from the user. In particular, student_straight does not distinguish MPC optimizer output from its internal fallback. No firmware/communication-latency trace is available.

## 2. Straight-event processing and data quality

Follow the Lab I event approach: identify consecutive student_straight decisions, verify middle/student share the same selected section, align all three trajectories on a 0.1 s grid without extrapolation, and reject interpolation spans above 0.3 s. Match by route section and occurrence, not whole-run time. The selected-section sequence is 3, 1, 2, 3. Event 1 includes launch, and event 4 ends when the recording stops.

Spacing is forward route distance minus 250. Targets use aligned reported speed to match the controller input. No extra scaling is applied to position-derived speed. Sharp coordinate jumps were found in the PID data, and all datasets have some telemetry gaps around 0.5 s. These are not silently deleted as bad controller behavior. Physical acceleration, jerk and position-derived TTC need further outlier/timing validation before ranking methods.

| Run | Leader gaps >0.3 s | Middle gaps >0.3 s | Student gaps >0.3 s |
|---|---:|---:|---:|
| PID | 5 | 5 | 7 |
| MPC | 9 | 13 | 9 |
| LQI | 10 | 7 | 7 |

A repeated coordinate-step feature occurs in the last straight near route coordinate 12,700. All three runs report speed 38, but one coordinate step is much larger than 2.8*38*dt would predict. These are flagged, not removed. They may reflect localization/timestamp behavior or position hold-and-catch-up; the logs do not identify the cause. Do not treat the resulting derivative spikes as proven physical acceleration or controller-induced discomfort.

| Run | Time (s) | Route coordinate | Coordinate step | Sample interval (s) | Reported speed |
|---|---:|---:|---:|---:|---:|
| PID | 149.449 | 12706.1 | 52.3 | 0.097 | 38 |
| MPC | 151.548 | 12688.1 | 67.3 | 0.097 | 38 |
| LQI | 154.967 | 12754.6 | 60.6 | 0.089 | 38 |

## 3. Observed active-control overview

These statistics include launch but exclude noncontrol curves and bypass intervals.

| Run | Duration (s) | Active decisions | Active gap min / median / max | Active sensor flags | Bypass sensor flags |
|---|---:|---:|---|---:|---:|
| PID | 153.4 | 337 / 768 | 92.3 / 385.4 / 537.8 | 2 | 0 |
| MPC | 156.9 | 368 / 785 | 103.2 / 293.9 / 400.5 | 0 | 1 |
| LQI | 160.1 | 362 / 801 | 105.6 / 425.7 / 675.9 | 0 | 1 |

There are no active samples below the common 80-unit gap band. All three have launch samples below the controllers' nominal 120-unit protective threshold. Sensor flags are obstacle-sensor events, not automatic evidence of a route-gap violation or a confirmed false positive. The different native spacing thresholds never triggered in these logs, so they do not directly explain the observed gap differences.

## 4. Event-by-event tracking and disturbance behaviour

| Event | Method | Active interval (s) | Entry → exit gap | Own-target RMSE | Bias | Reported-speed SD ratio |
|---|---|---|---|---:|---:|---:|
| 1 | PID | 0.3-28.3 | 116.4 → 411.4 | 114.5 | -84.2 | 0.861 |
| 1 | MPC | 0.1-29.8 | 103.2 → 315.6 | 94.9 | -84.1 | 0.898 |
| 1 | LQI | 0.3-33.4 | 113.9 → 458.1 | 107.5 | -45.6 | 0.860 |
| 2 | PID | 59.1-69.2 | 241.4 → 387.2 | 157.3 | -147.6 | 0.866 |
| 2 | MPC | 61.1-72.4 | 146.9 → 262.2 | 175.2 | -170.2 | 1.010 |
| 2 | LQI | 64.5-74.4 | 291.6 → 438.5 | 137.0 | -119.8 | 1.002 |
| 3 | PID | 110.1-130.2 | 518.6 → 477.1 | 57.2 | 46.1 | 0.776 |
| 3 | MPC | 112.3-133.3 | 372.6 → 379.4 | 37.4 | 30.2 | 0.834 |
| 3 | LQI | 116.7-135.4 | 664.6 → 463.6 | 131.6 | 102.3 | 0.795 |
| 4 | PID | 145.7-153.3 | 414.0 → 455.5 | 55.6 | -54.0 | quiet input |
| 4 | MPC | 147.8-156.8 | 305.6 → 342.1 | 52.0 | -50.3 | quiet input |
| 4 | LQI | 150.7-159.9 | 401.1 → 468.3 | 61.3 | -57.5 | quiet input |

Positive bias means gap above target. Ratios are student/middle reported-speed standard deviation over the paired event; withheld when middle SD <1 raw unit. They are not physical acceleration/comfort metrics. Different event input histories and time/route coverage also affect the ratios.

- **Event 1:** launch and different initial gaps dominate. PID also includes two consecutive center-sensor flags. Do not use this as the sole steady-state controller comparison.
- **Event 2:** PID starts at about 241, MPC 147, LQI 292. LQI's entry is closer to the shared PID/LQI target; its lower error than PID here is not a controlled proof of faster recovery. All three expand the initially short gap.
- **Event 3:** PID starts around 519 and ends 477; MPC 373 to 379; LQI 665 to 464. LQI inherits much more separation but removes about 201 units during active control. MPC holds near its smaller target policy. PID has lower telemetry speed variation than both others relative to each run's middle input.
- **Event 4:** middle reported speed is constant, so ratio-based attenuation is not meaningful. Compare target bias and recovery only; the event is truncated.

### Common route coverage

Intersection of ego route-coordinate coverage across all three corresponding events. Metrics use each run's retained samples inside that range, not interpolation over holes. This reduces route-coverage differences but does not equalize entry state or disturbances.

| Event | Common distance (cu) | PID median gap / RMSE | MPC median gap / RMSE | LQI median gap / RMSE |
|---|---:|---|---|---|
| 1 | 2114.9 | 363.6 / 115.4 | 279.5 / 83.2 | 423.4 / 80.4 |
| 2 | 1051.5 | 347.2 / 160.0 | 207.9 / 181.3 | 376.7 / 132.7 |
| 3 | 1903.7 | 491.4 / 57.0 | 381.6 / 38.3 | 577.0 / 131.9 |
| 4 | 829.2 | 439.1 / 55.6 | 326.4 / 52.8 | 432.3 / 64.8 |

### Command smoothness (not physical jerk)

Compute differences between integer-rounded sent-speed targets for consecutive active decisions only; exclude pairs containing a TA emergency flag. No derivatives across curve/bypass boundaries. These quantify command activity, not actual acceleration or comfort; early launch and each method's protections still contribute.

| Method | Valid command pairs | RMS command increment | 95th percentile absolute increment |
|---|---:|---:|---:|
| PID | 330 | 1.48 | 4.0 |
| MPC | 364 | 1.50 | 3.0 |
| LQI | 358 | 1.46 | 3.0 |

## 5. Vehicle-response model: open-loop validation first

Fit a simple first-order command response with a separate command-to-coordinate-speed gain, lag and command delay. Use the recorded rounded shielded commands, including real stops. Fit on alternating 5 s blocks using robust 1 s route-displacement residuals; evaluate the other blocks. Reject intervals crossing large telemetry gaps or implausible negative/very large displacement. Candidate lags: 0.1-1.8 s; delays: 0-0.5 s.

| Source run | Fitted gain | Lag (s) | Delay (s) | Held-out 1 s displacement RMSE (cu) | Relative RMSE |
|---|---:|---:|---:|---:|---:|
| PID | 2.914 | 0.10 | 0.00 | 17.87 | 16.7% |
| MPC | 2.937 | 0.10 | 0.10 | 17.64 | 16.5% |
| LQI | 2.973 | 0.80 | 0.00 | 16.46 | 15.8% |

This is an imperfect exploratory model, not a validated digital twin. PID/MPC fits hit the minimum tested lag, indicating weak identification or missing dynamics. The LQI fit selects a different lag despite the same vehicle, further warning against interpreting the fit as a physical constant. Interleaved held-out blocks are not independent-run validation. Gains near 2.9 do not establish a new scaling correction; they absorb model error and do not change the submitted controllers.

## 6. Individual closed-loop simulations using actual test inputs

For each of the three recordings, replay measured leader and middle positions and reported speeds. Initialize simulated ego once from the measured initial state. Thereafter each controller receives only its own simulated ego state/history; its commands determine subsequent ego motion. The measured ego is never injected back into the feedback. Run the original three 2.8 files separately on each input recording.

Use actual command decision timestamps, route gating, integer rounding and the source run's TA spacing threshold. Simulated telemetry maps coordinate speed by the 2.8 hypothesis; command response uses the independently fitted gain. The plant response is integrated at <=0.02 s steps. Sensor readings are nominal: recorded sensor events are not transferred to counterfactual trajectories. Source holes are filled linearly for continuity, but active samples requiring source interpolation spans >0.3 s are excluded from scoring. Dynamics across the holes remain uncertain.

Callback computation does not advance simulated time. GUI scheduling, asynchronous packet arrival, and unmeasured end-to-end vehicle-command latency are not reconstructed. Fitted delay is an effective model term, not a direct network-latency measurement.

Nine baseline runs plus half/double response-lag sensitivity yield 27 closed-loop runs. This isolates ego feedback and avoids adding middle-IDM model error. It does not simulate a fully endogenous three-vehicle platoon.

| Recorded front-vehicle input | Simulated PID RMSE | Simulated MPC RMSE | Simulated LQI RMSE |
|---|---:|---:|---:|
| PID run | 137.8 | 79.5 | 146.0 |
| MPC run | 113.0 | 69.4 | 119.9 |
| LQI run | 120.5 | 71.0 | 133.8 |

Across these 27 model runs: 0 runs with sampled negative gap; 0 with TA spacing-shield activation; 0 callback exceptions.

MPC initialization/optimizer was available in these simulations, and its internal modes are logged in `closed_loop.json`. Protective stop modes are not solver fallback. This simulated mode information does not recover missing optimizer status from the physical run.

MPC has lower own-target RMSE in this model, while PID is lower than LQI. This is conditional on imperfect plant fits, synthetic telemetry behavior, nominal sensors and different target policies. Recorded-ego versus same-method simulated-ego position RMSE is about 79-121 cu in the baseline fits; the simulation does not reproduce physical trajectories exactly. Do not treat its ranking as experimental proof.

## 7. Defensible report conclusion

The three physical runs show bounded following without logged spacing emergencies during active straight control. MPC uses a closer following policy and shows smaller own-target errors in several events. PID shows lower reported-speed variation in the nonlaunch disturbance events. LQI has a large inherited braking-event gap but closes it substantially. No single method is demonstrated best across all objectives from one nonidentically initialized run per method.

The recorded-data closed-loop study is supplementary sensitivity evidence. Its useful contribution is applying each unchanged controller to the same recorded front-vehicle inputs and initial ego state within each scenario, while making plant-model shortcomings explicit. Further repetitions and better plant validation would be needed for a stronger performance ranking. No controller tuning is proposed here.

## Artifacts and reproduction

- [Physical event comparison](../results/three_controller_analysis/physical_events.png)
- [Model validation](../results/three_controller_analysis/plant_validation.png)
- [Closed-loop versus recorded trajectories](../results/three_controller_analysis/closed_loop.png)
- Physical metrics: `results/three_controller_analysis/physical_summary.json`.
- Model fits and 27-run metrics: `results/three_controller_analysis/closed_loop.json`.
- Aligned physical event CSVs and baseline closed-loop traces are in the same folder.

```sh
python src/simulate_recorded_inputs.py
python src/analyze_physical_runs.py
```
