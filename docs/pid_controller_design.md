# Lab II: IDM-informed predictive PID controller plan

Status: discussion draft; no controller implementation or final gains yet.

## 1. Objective and scope

Design the controller for vehicle 2 in the formation:

**Leader (0) -> Middle, predefined IDM (1) -> Student, our controller (2)**

Use the current states and recorded histories of all three vehicles to maintain a
safe, consistent gap and reduce propagation of leader speed disturbances. Keep PID
as the feedback method; use an IDM model of the middle vehicle to provide an
anticipatory speed reference.

Prepare two otherwise equivalent versions: one assuming no scale conversion and
one assuming a factor of 2.8. The cause of the Lab I discrepancy and the physical
meaning of the desired-speed command remain unresolved lab-setting questions.

Only `student_controller2.py` will be modified for submission. Preserve:

```python
def student_control(leader_state, middle_state, student_state, history=None) -> float:
    ...
```

Return a desired-speed command. Keep supporting experiment files unchanged.

## 2. Default environment assumed

| Setting | Default |
|---|---:|
| Leader base command | 40 |
| First selected straight event | Ramp toward 55 over 8 s |
| Second selected straight event | Ramp toward 25 over 8 s |
| Third selected straight event | Ramp toward 20 over 10 s |
| Middle IDM v0 | 70 |
| Middle IDM T | 1.2 |
| Middle IDM s0 | 250 |
| Middle IDM a | 20 |
| Middle IDM b | 30 |
| Middle IDM exponent | 4 |
| Vehicle length | 250 coordinate units |
| IDM spacing floor | 80 |
| Emergency bumper spacing | Below 80 |
| Maximum command | 100 |
| IDM command increase/decrease limits | 50 / 80 command units per second |
| Configured control period | 0.15 s |

Leader events are one-shot per run. An event ends when the leader leaves its
section; outside an active event, the command returns to base speed. The third
event is named `stop_go` in code but does not command a full stop by default.

The GUI checks control eligibility on a 0.1 s timer, so actual control intervals
can exceed 0.15 s. The middle uses measured elapsed control time. Our function
receives no explicit timestep.

The middle IDM is active only when vehicles 0 and 1 share a selected control
straight. Our controller is called only when vehicles 1 and 2 share a selected
control straight. These activation conditions are independent. Outside its IDM
condition, the middle receives the leader's reported velocity as desired speed.
The safety shield applies after either mode.

## 3. Architecture

```text
Current states + recent history
              |
              v
Filtered state estimates and leader forecast
              |
              v
Replica of middle controller + assumed vehicle response
              |
              v
Predicted middle speed at a short horizon
              |
              v
Anticipatory speed reference + measured-gap PID correction
              |
              v
Output conversion, command limits, return desired speed
```

The predictor supplies advance information. PID corrects the actual measured gap
and compensates for forecast errors. Retain a usable feedback-only mode when
history or prediction is unreliable.

## 4. Unit ledger and the two versions

Use superscript `r` for reported velocity, `x` for coordinate-based motion, and `u`
for desired-speed commands. Define separate assumptions:

\[
v_i^x=\gamma v_i^r,
\qquad
v_{\mathrm{target},i}^x=\beta u_i.
\]

| Quantity | Internal convention |
|---|---|
| Position, route coordinate, spacing, vehicle length | Coordinate units |
| Time and prediction horizon | Seconds |
| Internal motion speed | Coordinate units/s |
| Internal motion acceleration | Coordinate units/s² |
| Reported velocity | Telemetry units; convert with gamma |
| Desired-speed command | Command units; convert with beta |

| Version | gamma | beta |
|---|---:|---:|
| Unscaled | 1 | 1 |
| Scaled | 2.8 | 2.8 |

Pairing beta with gamma is a provisional hypothesis. Keep them distinct in the
design so later evidence can change either independently.

Rules:

- Multiply reported velocity by gamma when constructing coordinate-speed states.
- Acceleration derived from reported-velocity history also requires gamma.
- Speed or acceleration derived directly from position history already uses
  coordinate units: do not multiply by 2.8 again.
- Convert a predicted speed command to a coordinate-speed target with beta.
- Convert our final coordinate-speed target to a command by dividing by beta.
- Do not scale coordinate spacing or the platform vehicle-length constant.
- Do not silently apply Lab I's converted calibration parameters to the default
  middle IDM. We are predicting the supplied controller, not correcting it.

## 5. State estimation and spacing

1. Read current states; use recent per-role history with distinct timestamps.
2. Estimate smooth velocity and acceleration trends over a bounded recent window.
   Start with a simple local fit or low-pass filter before considering Kalman or
   learned models.
3. Respect asynchronous telemetry: align estimates to a common current time with
   bounded extrapolation and check state age.
4. Estimate actual elapsed time between controller calls. Store the last call
   timestamp; use a monotonic clock for runtime intervals. Use history timestamps
   for telemetry fits. Do not mistake telemetry interval for control-call interval.
5. Reset persistent controller state on a new run; handle long inactive intervals
   and section re-entry without integrating across them.

Use forward route distance minus vehicle length for bumper spacing. Route
coordinates wrap at the loop boundary and must be unwrapped for motion fitting.
The current state objects expose x/y but not route coordinate directly; history
contains route projections. Resolve how to project current states consistently
using the supplied map utilities within the permitted controller file.

A same-straight heading projection may be a fallback for the immediate gap, but
must not be used blindly for the leader-middle gap when they occupy different
route elements.

## 6. Middle-vehicle predictor

### 6.1 Predict the software command first

Mirror the default middle controller's actual computations:

1. Determine whether leader and middle share a selected control straight.
2. Compute coordinate-based route spacing, subtracting vehicle length.
3. In IDM mode, evaluate the supplied IDM with **reported velocities and its
   unchanged default parameters**, including the spacing floor.
4. Convert acceleration to command using elapsed time; reproduce nonnegative
   bounds, v0 cap, last-command initialization, and command-rate limits.
5. In noncontrol mode, use the leader's reported velocity and reproduce the
   middle controller's last-command update.
6. Apply predictable spacing-based safety rules and command rounding.

This replica is identical in both versions at the same measured input state.
No extra 2.8 factor belongs inside the replicated IDM calculation. Its future
commands may differ between versions because the projected future states differ.

The middle controller's private last-command state and exact update phase are
not passed to us. Maintain an approximate shadow state, synchronize where possible,
and reduce prediction confidence after startup, mode changes, or interventions.
Future center-sensor readings cannot be predicted from route spacing alone.

### 6.2 Predict physical motion separately

Use the predicted command as input to a simple vehicle-response model:

\[
\dot v_1^x=\frac{\beta\hat u_1-v_1^x}{\tau_v},
\qquad
\dot x_1=v_1^x.
\]

Here tau_v is an assumed response lag, not a known platform constant. Use bounded
acceleration and sensitivity checks rather than presenting the model as exact.
To query the software replica at a future state, map its predicted coordinate
speed back to reported velocity using `v^r = v^x / gamma`.

Start with a short, filtered constant-acceleration forecast for the leader. Limit
extrapolated acceleration and speed. The known default leader event schedule can
be a later comparison, but should not be assumed perfectly synchronized with
physical motion or used as future measured information.

Roll forward leader and middle together, updating spacing and control-section
membership. Prediction horizon, integration step, and response lag remain tuning
choices; do not assume the horizon equals the controller timestep.

## 7. PID with an anticipatory speed reference

Blend present and predicted middle speed:

\[
v_{\mathrm{ref}}^x=(1-\lambda)\hat v_1^x(t)
  +\lambda\hat v_1^x(t+\tau_p),
\qquad 0\leq\lambda\leq1.
\]

Use actual measured spacing for feedback:

\[
s_{\mathrm{des}}=s_{\mathrm{standstill}}+h\hat v_2^x,
\qquad e=s_{12}-s_{\mathrm{des}},
\]

\[
v_{\mathrm{des}}^x=v_{\mathrm{ref}}^x+K_Pe+K_I I+K_D D,
\qquad
I\approx\int e\,dt,\quad D\approx\dot e,
\]

\[
u_2=v_{\mathrm{des}}^x/\beta.
\]

This PID outputs a speed correction, not an acceleration. Consequently, Kp has
units 1/s, Ki has units 1/s², and Kd is dimensionless. Do not reuse gains from an
acceleration-output formulation.

For a speed-dependent desired gap, the exact derivative is:

\[
\dot e=v_1^x-v_2^x-h a_2^x.
\]

Estimate it from filtered error history initially; relative speed alone omits the
changing-target term. Make filtering consistent with the speed used in the target.

Begin with modest lambda and weak or zero integral action. Add integral action
only if it reduces persistent bias. Use conditional integration or back-calculation
for saturation, bounded integral contribution, and re-entry reset/blending.

Do not add a second equivalent acceleration-feedforward term initially: this
could count the same prediction twice. Close-gap/rapid-closing protection must
dominate a prediction requesting acceleration. Define its thresholds and braking
policy before physical testing; do not rely solely on the TA's emergency shield.

## 8. Development and comparison sequence

1. Confirm route projection, controller timing, reset behavior, and the unit ledger.
2. Build an offline replica of the middle command logic and check representative
   IDM, noncontrol, saturation, and emergency cases against the supplied code.
3. Establish PID without prediction (`lambda = 0`). Tune proportional and damping
   behavior before adding integral action.
4. Add IDM-based prediction and compare it against a simple historical trend
   predictor. This tests whether the known IDM adds useful forecasting accuracy.
5. Compare the following configurations under each scale hypothesis:

   | Configuration | Purpose |
   |---|---|
   | PID, current middle-speed reference | Feedback baseline |
   | PID, trend-predicted middle reference | Value of simple history forecasting |
   | PID, IDM-predicted middle reference | Value of known middle dynamics and leader information |

6. Initially keep design parameters equal across the scaled and unscaled versions
   to isolate the hypothesis difference. Record any later independent retuning.
   Equal numeric settings do not imply equal physical behavior under both hypotheses.
7. Package standalone submission variants without altering supporting files.
   Only one selected variant should occupy the TA-imported filename at a time.

## 9. Validation before and during the lab

Offline simulation is a sensitivity study, not physical validation. A simulator
using the same assumed response model as the predictor can overstate its benefit.
Vary plant lag, delay, scale, initial spacing, and middle-model mismatch independently.

Check:

- Leader acceleration, braking, return-to-base transitions, and section switching.
- Short and insufficient history, repeated samples, stale data, and noisy position.
- Loop-coordinate wrapping, controller inactivity, re-entry, and new-run reset.
- Speed saturation, integral windup, command quantization, and closing-gap behavior.
- Prediction error versus horizon; compare IDM and trend forecasts.
- Execution cost on growing history; avoid rescanning the full run each call.

For lab evaluation, retain actual metadata/settings and compare matched control
sections/events. Separate controller-active intervals, transitions, and safety
overrides rather than pooling all behavior indiscriminately.

| Objective | Candidate indicators |
|---|---|
| Spacing | Error RMSE/MAE, minimum spacing, headway distribution |
| Disturbance attenuation | Speed-deviation RMS/energy ratios for vehicle 2 relative to 1 and 0 |
| Safety | Closing-phase TTC with consistent speed units; spacing and sensor interventions separately |
| Smoothness | Acceleration RMS, filtered jerk RMS, command variation |
| Prediction | Middle-speed forecast MAE/RMSE at each horizon |
| Robustness | Performance under delay, noise, model and scale mismatch |

Finite-run attenuation indicators are empirical results, not a general proof of
string stability. Suppressed speed variation alone is insufficient if it comes
from excessive lag or unacceptable spacing error.

## 10. Decisions still open

- Standstill spacing, time headway, PID gains, derivative filter, integral limits.
- Prediction horizon, blend weight, and behavior when confidence falls.
- Vehicle response lag, acceleration bounds, and synchronization of the shadow IDM.
- Exact close-gap protection and safe output-rate policy.
- Whether actual TA settings match the defaults and how the desired-speed scale
  relates to reported velocity and measured motion.

## 11. References and their role

Downloaded PDFs and source/version notes: [references/README.md](../references/README.md).

- [Naus et al., 2010](https://doi.org/10.1109/ACC.2010.5531596): time-headway
  spacing, filtered velocity, PD feedback and acceleration feedforward.
- [Cantas et al., author manuscript](https://arxiv.org/abs/2306.08182): practical
  CACC implementation and separation of control design from vehicle actuation.
- [Wang et al., 2020](https://doi.org/10.1016/j.trc.2019.04.010): two-predecessor
  information and adaptive PD/feedforward control.
- [Wu et al., 2019](https://doi.org/10.1109/ACCESS.2019.2928004): estimated
  acceleration as feedforward when communicated information is unavailable.
- [Mosharafian et al., 2021](https://arxiv.org/abs/2111.07162): trajectory
  prediction and uncertainty, as background rather than a PID implementation.
- [Tamilarasan and Guvenc, manuscript](https://arxiv.org/abs/2306.01971): gain
  selection and driveability evaluation.

The proposed IDM-predictive speed-reference PID is our adaptation to this lab.
The cited papers do not establish stability or safety guarantees for this exact
controller, the lab platform, or either scaling assumption.
