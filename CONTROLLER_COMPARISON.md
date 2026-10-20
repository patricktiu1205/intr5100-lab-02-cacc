# Six-controller review and test plan

Reviewed 2026-10-20. The four groupmate files were relocated without changing
their contents. Original filenames and SHA-256 hashes are recorded in
`controllers/GROUPMATE_IMPORTS.json`.

## Inventory

Each folder contains a submission-ready filename, `student_controller2.py`.
Copy only the selected file into the TA Shared Environment for a run.

| Run ID | Method | Scale hypothesis | Folder under controllers/ |
|---|---|---:|---|
| PID-1 | IDM-predictive PID | 1 | `unscaled/` |
| PID-2.8 | IDM-predictive PID | 2.8 | `scaled_2p8/` |
| LQI-1 | IDM-predictive LQI | 1 | `lqi/unscaled/` |
| LQI-2.8 | IDM-predictive LQI | 2.8 | `lqi/scaled_2p8/` |
| MPC-1 | Speed-command MPC | 1 | `mpc/unscaled/` |
| MPC-2.8 | Speed-command MPC | 2.8 | `mpc/scaled_2p8/` |

Existing PID paths were retained so their build scripts, tests, and links continue
to work. Do not run `build_variants.py` on the groupmate files: it generates only
the PID pair from our maintained source.

## What the methods actually implement

### PID pair

Feedback uses gap error, accumulated error, and a filtered error derivative.
The desired-speed reference blends current and IDM-predicted middle speed.
The middle predictor uses leader history and reproduces the default middle IDM
in its original reported-speed inputs. The physical rollout applies gamma and
beta separately; the two distributed versions pair their values at 1 or 2.8.
Target spacing is 300 + 1.8 * coordinate speed. Prediction horizon is 0.8 s.

### LQI pair (integral LQR, not plain LQR)

The three normalized feedback states are spacing error, student-minus-middle
speed, and integrated spacing error. The discrete Riccati equation is solved
using standard-library matrix loops and cached for rounded timestep/headway.
This is linear quadratic regulation with an integral state (LQI).

It uses the same style of 0.8 s middle-IDM forecast and a blended speed reference,
then combines reference tracking with optimal-state feedback. Desired acceleration
is converted to a desired speed using the assumed response lag. Target spacing is
300 + 1.8 * coordinate speed, like the PID pair.

The scaled version converts reported speeds before motion prediction and converts
the final output back into command units. Its IDM replica still receives raw
telemetry-scale speeds. No extra factor is applied to positions or gaps.

Review limits: the LQI model and response lag are assumptions; a Riccati solution
does not establish stability of the full nonlinear, switching, saturated platform.
New timestep keys can require another gain calculation. Both LQI files defer map
loading and initial gain calculation until their first controller call.

### MPC pair

The optimizer chooses 12 raw speed commands with a 0.2 s prediction step, giving
a 2.4 s horizon. The model includes first-order speed response, gap dynamics,
command bounds, command-rate bounds, and a predicted minimum gap of 120.
Its cost balances gap error, tracking middle and leader speeds, and command changes.
Target spacing is 250 + 1.2 * coordinate speed.

Unlike PID/LQI, it does NOT roll out the middle IDM. It separately extrapolates
leader and middle speed using recent, decaying acceleration trends. History
samples require receive timestamps and matching host identifiers; actual TA
recorded history provides these fields.

The pair differs only in SPEED_SCALE. The optimizer's command variable is already
in raw command units, so its output is not divided again. SPEED_SCALE multiplies
telemetry for model speed and maps raw commands to coordinate-speed targets.
The feedback fallback does perform a final conversion because it first constructs
a coordinate-speed target.

The optimizer initializes at import, accepts only an optimal solver result and
checks returned constraints. Infeasibility produces a stop. Other missing/failed
solver cases use a gap-aware feedback fallback, reported through
`student_control.last_debug`. A TA wrapper mode of `student_straight` alone does
NOT prove MPC ran successfully.

## Dependencies and requirement compatibility

| Method | Direct third-party dependencies in submitted file | Supplied support dependency |
|---|---|---|
| PID | None | route_map_utils (which uses PyYAML) and map |
| LQI | None | route_map_utils (which uses PyYAML) and map |
| MPC in optimizer mode | NumPy, CVXPY; OSQP solver | No map import in its own code |
| MPC fallback | Standard library | No map import in its own code |

If the rule is strictly standard-library-only for student algorithms, the current
MPC optimizer is not compliant. Running its fallback does not turn the run into
an MPC experiment. Resolve permitted packages with the TA or groupmate before
treating it as an eligible MPC submission. Do not quietly relabel fallback runs.

Optional optimizer dependencies are listed in `requirements-mpc.txt`; they have
not been installed or validated in the current compatibility run.

## Shared Environment compatibility results

All six files were independently copied into the original Shared Environment
setup and loaded via `ta_experiment2_gui.Experiment2GUI` offscreen. Synthetic
VehicleState messages and in-memory command sinks exercised the real control loop.
The original Shared Environment remained byte-for-byte unchanged.

Each passed 34 command cycles including active control, middle noncontrol mode,
student bypass, re-entry, sensor stopping, and new-run reset, without the TA's
controller-error fallback. MPC passed these interface checks in its INTERNAL
feedback fallback only: CVXPY/OSQP were unavailable. Its optimizer is untested.

Observed first control-loop duration was approximately 217 ms for PID and
223-230 ms for LQI on this computer, including lazy map initialization; this
exceeds the configured 150 ms period. This is a startup timing concern, not a
steady-state benchmark. Consider pre-initialization before timing a physical run
or discuss startup handling with the TA; the imported originals were not changed.
MPC fallback took roughly 1.3 ms or less here, which says nothing about solver cost.

Repeat with:

```powershell
.\.venv\Scripts\python.exe src/test_dropin.py --all
```

The machine-readable result is `simulation_results/dropin_checks_all.json`.
These are integration checks, not a closed-loop ranking. Earlier 78-case simulation
results apply to PID configurations only, not to the new MPC/LQI pairs.

## Fair six-controller experiment

1. Keep leader waveform, middle IDM parameters, cars/order, initial position/gap,
   route settings, and enabled safety rules the same across runs. Save metadata.
2. If time allows, repeat each controller and rotate run order to reduce battery
   and session-order effects. Record each actual run ID against the inventory ID.
3. Evaluate all six using the same active-section masks and metric definitions.
   Separate section transitions and sensor interventions from ordinary control.
4. Report minimum spacing, TTC in consistent units, interventions, middle-to-student
   speed-disturbance attenuation, acceleration/jerk, and travel progress/speed.
5. Report gap error relative to each controller's own target AND common quantities
   such as actual gap/headway distributions. MPC's shorter target makes raw
   target-error comparisons alone an unfair method ranking.
6. For MPC, capture internal mode/solver status and count fallback/stop samples.
   Exclude or explicitly label mixed-mode runs; the standard command CSV does
   not automatically capture `last_debug`.
7. Select the best observed tradeoff based on predefined priorities (first safety
   and completion, then spacing/attenuation/comfort), not one favorable metric.
8. Describe results as a comparison of these six implementations/tunings. Their
   targets, horizons, safeguards, and tuning differ, so it is not a controlled
   demonstration that one abstract control method is always superior.

Keep measured-versus-command scale unresolved until platform data establishes it.
Testing six candidates cannot by itself explain the Lab I 2.8 discrepancy.
