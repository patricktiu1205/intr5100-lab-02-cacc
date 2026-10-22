# -*- coding: utf-8 -*-
"""Lab II: standalone look-ahead speed-command MPC.

Copy this file as student_controller2.py beside the supplied TA entry point.
The two supplied variants differ ONLY in SPEED_SCALE (1.0 or 2.8).

Model route speed = SPEED_SCALE * raw telemetry velocity.
The same gain maps raw speed COMMANDS into predicted route speed.
The returned command is already in raw command units: do not multiply it again.
Positions, vehicle length and gaps remain in native map-coordinate units.

MPC dependencies: numpy, cvxpy, osqp. A clearly diagnosed feedback fallback is
available if those packages/solver are absent. No networking or GUI is started.
Model gain/lag and tuning are assumptions, not physical-car validation results.
"""
from __future__ import annotations

import math
import time
import warnings

SPEED_SCALE = 2.8

# Match these settings to the actual TA GUI; only defaults are available here.
VEHICLE_LENGTH = 250.0
EMERGENCY_GAP = 80.0
SENSOR_STOP_THRESHOLD = 0.15
MAX_COMMAND = 100.0

# MPC design choices, not parameters measured from the physical cars.
PREDICTION_DT = 0.20
HORIZON = 12
RESPONSE_TAU = 0.50
MIN_PREDICTED_GAP = 120.0
STANDSTILL_GAP = 250.0
TARGET_HEADWAY = 1.20
COMMAND_RATE_UP = 30.0
COMMAND_RATE_DOWN = 80.0
GAP_SCALE = 250.0
SPEED_NORMALIZER = 100.0
COMMAND_NORMALIZER = 50.0
GAP_WEIGHT = 3.0
FRONT_SPEED_WEIGHT = 3.0
LEADER_SPEED_WEIGHT = 0.40
COMMAND_CHANGE_WEIGHT = 1.0
SOLVER_TIME_LIMIT = 0.025

MAX_STATE_AGE = 0.60
MAX_STATE_SKEW = 0.30
MAX_POSE_AGE = None  # Set a seconds threshold only after TA confirms this field.
RESET_AFTER_INACTIVE = 0.80
CLOSING_GUARD_TIME = 1.0
FORECAST_ACCEL_LIMIT = 20.0  # raw velocity units per second
FORECAST_ACCEL_DECAY = 0.70
HISTORY_TAIL_LIMIT = 240

_CLOCK = time.monotonic
_CACHE = {}
_EMPTY_HISTORY = []
_WARNED = set()
_ENGINE = None
_ENGINE_ERROR = ""
try:
    import numpy as np
    import cvxpy as cp
except Exception as exc:
    np = cp = None
    _ENGINE_ERROR = "MPC dependencies unavailable: " + type(exc).__name__


def _clip(value, lower, upper):
    return max(lower, min(float(value), upper))


def _warn_once(key, message):
    if key not in _WARNED:
        warnings.warn(message, RuntimeWarning, stacklevel=2)
        _WARNED.add(key)


def _read_state(state):
    result = {}
    raw_data = getattr(state, "data", None)
    for name in ("velocity", "x", "y"):
        # Native VehicleState.get_float silently maps malformed values to zero.
        # Validate its underlying packet rather than accepting that replacement.
        value = raw_data[name] if isinstance(raw_data, dict) else getattr(state, name)
        result[name] = float(value)
        if not math.isfinite(result[name]):
            raise ValueError("nonfinite_" + name)
    if result["velocity"] < 0 or result["velocity"] > MAX_COMMAND * 2:
        raise ValueError("invalid_velocity_range")
    result["velocity"] = max(0.0, result["velocity"])
    stamp = getattr(state, "recv_time", None)
    if stamp is not None:
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError("nonfinite_receive_time")
    result["recv_time"] = stamp
    result["seq"] = getattr(state, "seq", None)
    result["host"] = getattr(state, "host", None)
    result["data"] = raw_data if isinstance(raw_data, dict) else {}
    if "pose_ready" in result["data"] and result["data"]["pose_ready"] not in (True, 1, "true", "True", "1"):
        raise ValueError("pose_not_ready")
    if "pose_age" in result["data"]:
        pose_age = float(result["data"]["pose_age"])
        if not math.isfinite(pose_age) or pose_age < 0:
            raise ValueError("invalid_pose_age")
        if MAX_POSE_AGE is not None and pose_age > MAX_POSE_AGE:
            raise ValueError("stale_pose")
    return result


def _set_result(command, mode, reason, **fields):
    command = _clip(command, 0.0, MAX_COMMAND)
    if not math.isfinite(command):
        command = 0.0
        mode, reason = "stop", "nonfinite_output"
    _CACHE["previous_command"] = command
    student_control.last_debug = dict(
        mode=mode, reason=reason, speed_scale=SPEED_SCALE,
        raw_command=command, **fields,
    )
    return float(command)


def _current_route_s(history, role, state):
    """Use only an exact current-state match, never a stale projected position."""
    if state["seq"] is None:
        return None
    for row in reversed(history[-HISTORY_TAIL_LIMIT:]):
        if row.get("role") != role or row.get("seq") != state["seq"]:
            continue
        if state["host"] is not None and row.get("host") != state["host"]:
            continue
        try:
            value = float(row["s"])
            if math.isfinite(value):
                return value
        except (KeyError, TypeError, ValueError):
            pass
    return None


def _measure_gap(middle, student, history):
    # The wrapper only calls us when both cars share a selected straight.
    # Straight distance therefore provides a direction-independent fallback.
    center_distance = math.hypot(middle["x"] - student["x"],
                                 middle["y"] - student["y"])
    sf = _current_route_s(history, "middle", middle)
    se = _current_route_s(history, "student", student)
    if sf is not None and se is not None:
        route_distance = sf - se
        if -1.20 * center_distance - 10.0 <= route_distance < 0:
            raise ValueError("middle_behind_student")
        # A negative difference can be a loop-coordinate wrap. Do not use it as
        # a negative physical distance; use the shared-straight geometry instead.
        if 0.0 < route_distance <= 1.20 * center_distance + 10.0:
            return route_distance - VEHICLE_LENGTH, "current_history_route"
    return center_distance - VEHICLE_LENGTH, "shared_straight_distance"


def _raw_acceleration(history, role, state):
    pairs = []
    for row in reversed(history[-HISTORY_TAIL_LIMIT:]):
        if row.get("role") != role:
            continue
        if state["host"] is not None and row.get("host") != state["host"]:
            continue
        try:
            t = float(row["recv_time"])
            q = float(row["velocity"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(t) and math.isfinite(q)):
            continue
        latest = state["recv_time"]
        if latest is None or t > latest + 1e-6 or latest - t > 0.9:
            continue
        pairs.append((t, q))
        if len(pairs) >= 10:
            break
    if len(pairs) < 3:
        return 0.0
    # Fit a causal slope. The forecast decays it rather than extrapolating it
    # indefinitely. No future observations or offline smoothing are used.
    t0 = pairs[0][0]
    ts = [p[0] - t0 for p in pairs]
    if max(ts) - min(ts) < 0.15:
        return 0.0
    tm = sum(ts) / len(ts)
    qm = sum(p[1] for p in pairs) / len(pairs)
    denominator = sum((t - tm) ** 2 for t in ts)
    slope = sum((t - tm) * (p[1] - qm) for t, p in zip(ts, pairs)) / max(denominator, 1e-9)
    return _clip(slope, -FORECAST_ACCEL_LIMIT, FORECAST_ACCEL_LIMIT)


def _forecast(raw_speed, raw_acceleration):
    values = []
    for j in range(HORIZON + 1):
        t = j * PREDICTION_DT
        raw = raw_speed + raw_acceleration * FORECAST_ACCEL_DECAY * (
            1.0 - math.exp(-t / FORECAST_ACCEL_DECAY))
        values.append(SPEED_SCALE * _clip(raw, 0.0, MAX_COMMAND))
    return values


def _feedback_command(gap, raw_leader, raw_middle, raw_student):
    w0, w1, w2 = [SPEED_SCALE * q for q in (raw_leader, raw_middle, raw_student)]
    error = gap - STANDSTILL_GAP - TARGET_HEADWAY * w2
    desired_route_speed = w1 + 0.12 * error + 0.40 * (w1 - w2) + 0.15 * (w0 - w1)
    # Only this feedback fallback optimizes a route-speed target, so it must
    # convert that target back to command units here. QP commands are already raw.
    return _clip(desired_route_speed / SPEED_SCALE, 0.0, MAX_COMMAND)


class _MPC:
    def __init__(self):
        n, h = HORIZON, PREDICTION_DT
        self.g0 = cp.Parameter()
        self.w0 = cp.Parameter(nonneg=True)
        self.previous = cp.Parameter(nonneg=True)
        self.first_dt = cp.Parameter(nonneg=True)
        self.front = cp.Parameter(n + 1, nonneg=True)
        self.leader = cp.Parameter(n + 1, nonneg=True)
        self.g = cp.Variable(n + 1)
        self.w = cp.Variable(n + 1)
        self.c = cp.Variable(n)
        dc = cp.hstack([self.c[0] - self.previous, self.c[1:] - self.c[:-1]])
        a = math.exp(-h / RESPONSE_TAU)
        D = RESPONSE_TAU * (1.0 - a)
        constraints = [
            self.g[0] == self.g0, self.w[0] == self.w0,
            self.g >= MIN_PREDICTED_GAP, self.w >= 0,
            self.c >= 0, self.c <= MAX_COMMAND,
            dc[0] <= COMMAND_RATE_UP * self.first_dt,
            dc[0] >= -COMMAND_RATE_DOWN * self.first_dt,
            dc[1:] <= COMMAND_RATE_UP * h,
            dc[1:] >= -COMMAND_RATE_DOWN * h,
        ]
        for j in range(n):
            constraints.extend([
                self.w[j + 1] == a * self.w[j] + SPEED_SCALE * (1.0 - a) * self.c[j],
                self.g[j + 1] == self.g[j] + h * self.front[j]
                - D * self.w[j] - SPEED_SCALE * (h - D) * self.c[j],
            ])
        error = self.g[1:] - STANDSTILL_GAP - TARGET_HEADWAY * self.w[1:]
        cost = (
            GAP_WEIGHT * cp.sum_squares(error / GAP_SCALE)
            + FRONT_SPEED_WEIGHT * cp.sum_squares((self.w[1:] - self.front[1:]) / SPEED_NORMALIZER)
            + LEADER_SPEED_WEIGHT * cp.sum_squares((self.w[1:] - self.leader[1:]) / SPEED_NORMALIZER)
            + COMMAND_CHANGE_WEIGHT * cp.sum_squares(dc / COMMAND_NORMALIZER)
        )
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def solve(self, gap, speed, previous, front, leader, first_dt, reset=False):
        self.g0.value = gap
        self.w0.value = speed
        self.previous.value = previous
        self.first_dt.value = first_dt
        self.front.value = np.asarray(front)
        self.leader.value = np.asarray(leader)
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                self.problem.solve(solver=cp.OSQP, warm_start=not reset,
                                   time_limit=SOLVER_TIME_LIMIT, max_iter=3000,
                                   eps_abs=1e-4, eps_rel=1e-4, verbose=False)
        except Exception as exc:
            return None, "solver_error:" + type(exc).__name__, None
        if self.problem.status != cp.OPTIMAL or self.c.value is None:
            return None, str(self.problem.status), None
        command = np.asarray(self.c.value, dtype=float)
        gaps = np.asarray(self.g.value, dtype=float)
        speeds = np.asarray(self.w.value, dtype=float)
        changes = np.diff(np.r_[previous, command])
        tolerance = 0.03
        valid = (np.isfinite(np.r_[command, gaps, speeds]).all()
                 and gaps.min() >= MIN_PREDICTED_GAP - tolerance
                 and speeds.min() >= -tolerance
                 and command.min() >= -tolerance and command.max() <= MAX_COMMAND + tolerance
                 and changes[0] <= COMMAND_RATE_UP * first_dt + tolerance
                 and changes[0] >= -COMMAND_RATE_DOWN * first_dt - tolerance
                 and changes[1:].max() <= COMMAND_RATE_UP * PREDICTION_DT + tolerance
                 and changes[1:].min() >= -COMMAND_RATE_DOWN * PREDICTION_DT - tolerance)
        if not valid:
            return None, "constraint_check_failed", None
        return float(command[0]), "optimal", float(gaps.min())


def student_control(leader_state, middle_state, student_state, history=None) -> float:
    """Return one finite raw desired-speed command for the student vehicle."""
    started = time.perf_counter()
    now = _CLOCK()
    history = history if isinstance(history, list) else _EMPTY_HISTORY
    try:
        leader, middle, student = [_read_state(s) for s in
                                   (leader_state, middle_state, student_state)]
        stamps = [s["recv_time"] for s in (leader, middle, student)
                  if s["recv_time"] is not None]
        wall = time.time()
        if any(wall - t > MAX_STATE_AGE or t - wall > 1.0 for t in stamps):
            return _set_result(0.0, "stop", "stale_state")
        if stamps and max(stamps) - min(stamps) > MAX_STATE_SKEW:
            return _set_result(0.0, "stop", "asynchronous_state_skew")
        gap, geometry = _measure_gap(middle, student, history)
        if not math.isfinite(gap):
            return _set_result(0.0, "stop", "invalid_gap")
        measured_gap = gap
        # Align to current receive time using the selected speed hypothesis.
        # Keep only reductions: do not grant extra clearance from extrapolation.
        age_adjustment = 0.0
        if middle["recv_time"] is not None and student["recv_time"] is not None:
            correction = SPEED_SCALE * (
                middle["velocity"] * max(0.0, wall - middle["recv_time"])
                - student["velocity"] * max(0.0, wall - student["recv_time"]))
            age_adjustment = min(0.0, correction)
            gap += age_adjustment
        hosts = tuple(s["host"] for s in (leader, middle, student))
        seqs = tuple(s["seq"] for s in (leader, middle, student))
        elapsed = now - _CACHE.get("last_call", now - PREDICTION_DT)
        old_seqs = _CACHE.get("seqs", seqs)
        backwards = any(isinstance(a, (int, float)) and isinstance(b, (int, float)) and a < b
                        for a, b in zip(seqs, old_seqs))
        reset = (not _CACHE or elapsed > RESET_AFTER_INACTIVE or elapsed < 0
                 or _CACHE.get("history_id") != id(history)
                 or len(history) < _CACHE.get("history_len", 0)
                 or hosts != _CACHE.get("hosts", hosts) or backwards)
        previous = (_clip(student["velocity"], 0, MAX_COMMAND) if reset
                    else _CACHE.get("previous_command", student["velocity"]))
        dt = _clip(elapsed, 0.05, 0.40) if not reset else PREDICTION_DT
        _CACHE.update(last_call=now, history_id=id(history), history_len=len(history),
                      hosts=hosts, seqs=seqs)

        sensor = float(student["data"].get("center_distance", 0.0))
        if math.isfinite(sensor) and 0.0 < sensor < SENSOR_STOP_THRESHOLD:
            return _set_result(0.0, "stop", "center_sensor", gap=gap, geometry=geometry)
        closing = SPEED_SCALE * max(0.0, student["velocity"] - middle["velocity"])
        if gap <= MIN_PREDICTED_GAP or (closing > 0 and (gap - EMERGENCY_GAP) / closing < 0.6):
            return _set_result(0.0, "stop", "gap_or_closing_emergency", gap=gap, geometry=geometry)

        front = _forecast(middle["velocity"], _raw_acceleration(history, "middle", middle))
        leading = _forecast(leader["velocity"], _raw_acceleration(history, "leader", leader))
        predicted_min_gap = None
        if _ENGINE is not None:
            command, status, predicted_min_gap = _ENGINE.solve(
                gap, SPEED_SCALE * student["velocity"], previous,
                front, leading, dt, reset)
            if command is None and "infeasible" in status:
                return _set_result(0.0, "stop", status, gap=gap, geometry=geometry)
            mode = "mpc" if command is not None else "fallback"
        else:
            command, mode, status = None, "fallback", "mpc_unavailable"
        if command is None:
            _warn_once("fallback", "MPC unavailable/failed; using gap-aware feedback fallback. "
                       "Check student_control.last_debug and install numpy cvxpy osqp. " + _ENGINE_ERROR)
            command = _feedback_command(gap, leader["velocity"], middle["velocity"], student["velocity"])

        command = _clip(command, previous - COMMAND_RATE_DOWN * dt,
                        previous + COMMAND_RATE_UP * dt)
        # A final spacing/closing cap may override a down-slew limit when needed.
        # This is an additional nominal guard, not a proven physical safety set.
        closing_cap = (SPEED_SCALE * middle["velocity"]
                       + max(0.0, gap - MIN_PREDICTED_GAP) / CLOSING_GUARD_TIME) / SPEED_SCALE
        if command > closing_cap:
            command = closing_cap
            status += ";closing_cap"
        return _set_result(command, mode, status, gap=gap, geometry=geometry,
                           measured_gap=measured_gap, gap_age_adjustment=age_adjustment,
                           raw_speeds=[s["velocity"] for s in (leader, middle, student)],
                           model_speeds=[SPEED_SCALE * s["velocity"] for s in (leader, middle, student)],
                           predicted_min_gap=predicted_min_gap,
                           reset=reset, elapsed_ms=1000.0 * (time.perf_counter() - started))
    except Exception as exc:
        # Do not escape to the TA wrapper's leader-speed exception fallback.
        return _set_result(0.0, "stop", "invalid_input:" + type(exc).__name__)


student_control.last_debug = {"mode": "not_called", "speed_scale": SPEED_SCALE}

# Build/canonicalize the small QP at module import, before the GUI timer starts.
if cp is not None:
    try:
        _ENGINE = _MPC()
        equilibrium_speed = SPEED_SCALE * 40.0
        result = _ENGINE.solve(
            STANDSTILL_GAP + TARGET_HEADWAY * equilibrium_speed,
            equilibrium_speed, 40.0,
            [equilibrium_speed] * (HORIZON + 1),
            [equilibrium_speed] * (HORIZON + 1), PREDICTION_DT, True)
        if result[0] is None:
            raise RuntimeError("MPC warmup failed: " + result[1])
    except Exception as exc:
        _ENGINE = None
        _ENGINE_ERROR = "MPC initialization failed: " + str(exc)
