# Lab II: IDM-informed predictive PID

INTR 5100 lab work. Implements desired-speed PID control for vehicle 2
behind an IDM-controlled middle vehicle and a prescribed-speed leader.

The folder now also contains the groupmates' LQI and MPC pairs, preserved unchanged.
See [six-controller analysis and test plan](CONTROLLER_COMPARISON.md) and the
[submission index](controllers/README.md). Previous simulation results cover PID
only; initial all-six integration results explicitly identify MPC's fallback mode.

For the subsequent full closed-loop comparison, including active MPC optimizers,
see [all-six simulation results](ALL_SIX_SIMULATION_REPORT.md). Reproduce with
`src/simulate_all.py` followed by `src/report_all.py`. The six controller files
remain unchanged.

## Status

Implemented and tested offline on Python 3.12. Six automated tests pass; 78
simulation cases record no collisions or TA spacing-emergency triggers. This is
an initial implementation with hand-selected gains, not a physical validation or
a proof of safety/string stability. See [simulation results](SIMULATION_REPORT.md).

## Controller versions

| File | Telemetry factor gamma | Command factor beta |
|---|---:|---:|
| `controllers/unscaled/student_controller2.py` | 1 | 1 |
| `controllers/scaled_2p8/student_controller2.py` | 2.8 | 2.8 |

Both are standalone files with the required `student_control(..., history=None)`
interface. For TA deployment, copy **one** version next to the supplied experiment
files using the name `student_controller2.py`. Supporting files must be unchanged.
The original shared template in the course folder has not been replaced.

The empirical 2.8 ratio is specific to Lab I. Its cause and the desired-speed
command scale are unresolved. Gamma converts reported speed to coordinate speed;
beta maps commands to coordinate-speed targets. They are separate parameters,
paired provisionally in these two versions. Position-derived speeds must never
receive another 2.8 multiplication.

## How it works

1. Project current positions to the supplied closed route; calculate bumper gaps.
2. Fit recent velocity history to estimate leader acceleration.
3. Replicate the default middle IDM's command calculation in its original units.
4. Roll out predicted middle motion using a short horizon and assumed response lag.
5. Blend current and predicted middle speed; add measured-gap PID correction.
6. Divide coordinate-speed output by beta; apply output limits and closing protection.

The PID uses a 300-coordinate-unit standstill gap, 1.8 s headway, and initial gains
Kp=0.12/s, Ki=0.002/s², Kd=0.08. The prediction horizon is 0.8 s, blend weight 0.45,
and assumed response lag 0.5 s. These are provisional design choices, not optimized
gains or measurements of the platform. Configuration is in `src/controller.py`.

Integral anti-windup, filtered derivative, elapsed-time handling, inactive-interval
restart, stale-telemetry stopping, and present center-sensor checks are included.
The local safety envelope is heuristic; the TA's safety shield remains external.

See [the design plan](PID_Controller_Plan.md) for equations and assumptions.

## Isolated setup (Windows PowerShell)

Use a dedicated environment, not the Lab I environment:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`requirements.txt` gives compatible dependency ranges; `requirements-lock.txt`
records exact versions used for this run. Install the lock instead for replication.
The offline tests do not require Qt, a running GUI, vehicle IPs, or network access.

## Rebuild, test, and simulate

Run from this repository's root:

```powershell
.\.venv\Scripts\python.exe src/build_variants.py
.\.venv\Scripts\python.exe -m unittest discover -s src -p "test_*.py" -v
.\.venv\Scripts\python.exe src/simulate.py
.\.venv\Scripts\python.exe src/make_report.py
```

Simulation uses the original `IDMController` as its middle-controller oracle,
independent of our replica, and a plant with heterogeneous response lags. It tests
the two controller assumptions against matching and mismatching plant scales,
delay/noise, slow response, close starts, abrupt stopping, and independently
mismatched command scale. Six additional runs use the actual route and default
one-shot leader event manager with section switching.

## Drop-in compatibility check

Both generated files were separately copied into isolated copies of the actual
Shared Environment and loaded through `ta_experiment2_gui.Experiment2GUI`, with
Qt offscreen. The actual `_control_step` processed synthetic `VehicleState`
telemetry and in-memory command sinks. Both passed 34 command cycles covering
active control, middle fallback, student bypass, re-entry, center-sensor safety,
and new-run reset, with no controller-error fallback. Original Shared Environment
files were verified unchanged by SHA-256. No physical/network clients were used.

To repeat in the course folder:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-gui.txt
.\.venv\Scripts\python.exe src/test_dropin.py
```

This check requires the full original `../Shared Environment` folder, including
the TA GUI files, alongside the repository. The minimal `environment/` copy used
by offline simulation does not include those GUI entry files. Detailed results:
`simulation_results/dropin_checks.json`. GUI dependencies are optional for the
simulation; `requirements-lock.txt` now records the full tested environment.

## Project contents

- `src/controller.py`: maintained standalone controller source.
- `controllers/`: generated submission variants; rebuild after source changes.
- `src/simulate.py`: deterministic longitudinal simulation (seed 5100).
- `src/test_controller.py`: replica equivalence, interface, lifecycle, and loop checks.
- `simulation_results/`: metrics, selected traces, and comparison chart.
- `environment/`: unmodified copies of five supplied support files needed offline.
- `References/READING_GUIDE.txt`: bibliography, source links, and reading notes.

Downloaded research PDFs and the lab guideline are excluded from Git. The supplied
environment code/map are course materials included here for private reproducibility;
no public redistribution license is asserted. No license is assigned to those files.

## Known limits and lab checklist

- Confirm actual TA IDM and route settings match defaults.
- Identify command response, telemetry scale, and delay before trusting predictions.
- Shadow IDM state and phase are approximate because the real private command memory
  is not supplied. Future sensor overrides cannot be predicted.
- Histories are bounded and current pose ages are handled, but this is not a full
  delayed-state observer. Prediction depends on a simple leader extrapolation.
- Physical lateral dynamics, localization faults, packet-loss patterns, and future
  range-sensor readings are not simulated. The route model imposes perfect tracking.
- Metrics are finite-scenario observations; spacing and smoothness trade off.
- Each variant is a candidate for supervised lab testing, not an approved deployment.
