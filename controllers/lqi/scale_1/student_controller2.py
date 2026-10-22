"""Standalone predictive LQI controller with unscaled velocity units."""
from __future__ import annotations

import math
import sys
import time
from collections import deque
from functools import lru_cache
from pathlib import Path

STATE_SCALES = (300.0, 30.0, 540.0)
ACCELERATION_SCALE = 20.0
LQI_Q = (4.0, 2.0, 1.0)


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def middle_command(v, vf, spacing, dt, last, active=True):
    """Replica: returns sent command and private pre-shield IDM memory."""
    if active:
        s = max(spacing, 80.0)
        star = 250.0 + max(0.0, 1.2*v + v*(v-vf)/(2*math.sqrt(600)))
        a = 20.0*(1.0-(max(v, 0)/70.0)**4-(star/s)**2)
        desired = clip(v+a*max(dt, .001), 0, 70)
        if last <= 0:
            last = min(desired, 70)
        desired = clip(desired, last-80*dt, last+50*dt)
        last = desired
    else:
        desired = vf
        last = desired
    return round(0 if spacing < 80 else clip(desired, 0, 100)), last


def _multiply(left, right):
    return [[sum(left[i][k] * right[k][j] for k in range(3))
             for j in range(3)] for i in range(3)]


def _transpose(matrix):
    return [[matrix[j][i] for j in range(3)] for i in range(3)]


@lru_cache(maxsize=64)
def _lqi_gain(dt, headway):
    """Solve the normalized, discrete 3-state LQI Riccati equation."""
    e_scale, speed_scale, integral_scale = STATE_SCALES
    physical_a = [
        [1.0, -dt, 0.0],
        [0.0, 1.0, 0.0],
        [dt, -0.5 * dt * dt, 1.0],
    ]
    physical_b = [
        -headway * dt - 0.5 * dt * dt,
        dt,
        -0.5 * headway * dt * dt - dt**3 / 6.0,
    ]
    scales = (e_scale, speed_scale, integral_scale)
    a = [[physical_a[i][j] * scales[j] / scales[i]
          for j in range(3)] for i in range(3)]
    b = [physical_b[i] * ACCELERATION_SCALE / scales[i]
         for i in range(3)]
    at = _transpose(a)
    p = [[LQI_Q[i] if i == j else 0.0 for j in range(3)]
         for i in range(3)]

    for _ in range(10000):
        pb = [sum(p[i][j] * b[j] for j in range(3))
              for i in range(3)]
        denominator = 1.0 + sum(b[i] * pb[i] for i in range(3))
        atpb = [sum(at[i][j] * pb[j] for j in range(3))
                for i in range(3)]
        atpa = _multiply(at, _multiply(p, a))
        next_p = [[atpa[i][j] - atpb[i] * atpb[j] / denominator
                   + (LQI_Q[i] if i == j else 0.0)
                   for j in range(3)] for i in range(3)]
        next_p = [[0.5 * (next_p[i][j] + next_p[j][i])
                   for j in range(3)] for i in range(3)]
        difference = max(abs(next_p[i][j] - p[i][j])
                         for i in range(3) for j in range(3))
        magnitude = max(1.0, max(abs(next_p[i][j])
                                 for i in range(3) for j in range(3)))
        p = next_p
        if difference <= 1e-10 * magnitude:
            break
    else:
        raise ArithmeticError('LQI Riccati iteration did not converge')

    pb = [sum(p[i][j] * b[j] for j in range(3))
          for i in range(3)]
    denominator = 1.0 + sum(b[i] * pb[i] for i in range(3))
    pa = _multiply(p, a)
    return tuple(sum(b[i] * pa[i][j] for i in range(3)) / denominator
                 for j in range(3))


def load_geometry():
    here = Path(__file__).resolve().parent
    candidates = [here] + [p/name for p in here.parents
                           for name in ('environment','Shared Environment')]
    for folder in candidates:
        if (folder/'6e68_map.yaml').exists():
            if str(folder) not in sys.path:
                sys.path.insert(0, str(folder))
            from route_map_utils import RouteAnalyzer, longest_straight_sections
            route = RouteAnalyzer.from_map(folder/'6e68_map.yaml')
            return route, longest_straight_sections(route, 3)
    raise FileNotFoundError('Place this file beside the supplied map and route_map_utils.py')


class PredictiveLQI:
    def __init__(self, route=None, sections=None, prediction='idm', weight=.45):
        self.route, self.sections = (load_geometry() if route is None else (route, sections))
        self.prediction, self.weight = prediction, weight
        self.standstill, self.headway = 300., 1.8
        self.horizon, self.lag = .8, .5
        self.samples = {r: deque(maxlen=12) for r in ('leader','middle','student')}
        self.history_id = None
        self.history_len = 0
        self.reset()

    def reset(self):
        self.last_t = None
        self.integral = 0.
        self.last_output = None
        self.shadow = 0.
        self.diagnostics = {}
        for q in self.samples.values():
            q.clear()

    def section(self, s):
        return next((i for i, sec in enumerate(self.sections)
                     if sec.contains(s, self.route.loop_length)), -1)

    def same(self, a, b):
        i = self.section(a)
        return i >= 0 and i == self.section(b)

    def ingest(self, history):
        if history is None:
            return
        if id(history) != self.history_id or len(history) < self.history_len:
            self.reset()
        self.history_id, self.history_len = id(history), len(history)
        for row in history[-90:]:
            role = row.get('role')
            if role not in self.samples:
                continue
            q = self.samples[role]
            t = float(row['t_from_start'])
            if not q or t > q[-1][0]:
                v = float(row['velocity'])
                if math.isfinite(v):
                    q.append((t, v))

    def acceleration(self, role):
        q = self.samples[role]
        if len(q) < 3 or q[-1][0]-q[0][0] < .15:
            return 0.
        mt = sum(t for t,v in q)/len(q)
        mv = sum(v for t,v in q)/len(q)
        den = sum((t-mt)**2 for t,v in q)
        return clip(sum((t-mt)*(v-mv) for t,v in q)/max(den,1e-9), -30, 30)

    def forecast(self, s0, s1, v0, v1, dt):
        """Roll out in the same unscaled velocity and route-coordinate units."""
        a0 = self.acceleration('leader')
        if self.prediction == 'trend':
            return max(0, v1+self.acceleration('middle')*self.horizon)
        if self.prediction == 'current':
            return v1
        last = self.shadow
        elapsed = 0.
        while elapsed < self.horizon-1e-9:
            step = min(dt, self.horizon-elapsed)
            gap = self.route.forward_gap(s0,s1)-250
            cmd,last = middle_command(v1,v0,gap,step,last,self.same(s0,s1))
            a1 = clip((cmd-v1)/self.lag,-80,50)
            nv1, nv0 = max(0,v1+a1*step), max(0,v0+a0*step)
            s0 += .5*(v0+nv0)*step
            s1 += .5*(v1+nv1)*step
            v0,v1 = nv0,nv1
            elapsed += step
        return v1

    def step(self, leader, middle, student, history=None, now=None):
        self.ingest(history)
        now = time.monotonic() if now is None else now
        raw_dt = .2 if self.last_t is None else now-self.last_t
        reentry = self.last_t is None or raw_dt > .65 or raw_dt <= 0
        dt = clip(raw_dt,.05,.35)
        self.last_t = now
        states = (leader,middle,student)
        wall_now = time.time()
        ages = [max(0.,wall_now-float(getattr(z,'recv_time',wall_now))) for z in states]
        if max(ages) > .6:
            self.integral = 0.
            self.last_output = 0.
            self.diagnostics = dict(reason='stale_telemetry')
            return 0.
        vals = [(float(z.x),float(z.y),float(z.velocity)) for z in states]
        if not all(math.isfinite(v) for row in vals for v in row):
            self.integral = 0.
            return 0.
        s = [self.route.project_xy(x,y).s for x,y,v in vals]
        v = [max(0,row[2]) for row in vals]
        s = [p+speed*min(age,.3) for p,speed,age in zip(s,v,ages)]
        gap = self.route.forward_gap(s[1],s[2])-250
        if reentry:
            self.integral = 0.
            self.last_output = v[2]
            self.shadow = vals[1][2]
        _, self.shadow = middle_command(vals[1][2],vals[0][2],
            self.route.forward_gap(s[0],s[1])-250,dt,self.shadow,self.same(s[0],s[1]))
        pred = self.forecast(s[0],s[1],v[0],v[1],dt)
        reference = (1-self.weight)*v[1]+self.weight*pred
        error = gap-self.standstill-self.headway*v[2]
        relative_speed = v[2]-v[1]
        candidate_i = clip(self.integral+error*dt,-3000,3000)
        gain = _lqi_gain(round(dt, 2), self.headway)

        def command_acceleration(integral):
            state = (error/STATE_SCALES[0],
                     relative_speed/STATE_SCALES[1],
                     integral/STATE_SCALES[2])
            feedback = -ACCELERATION_SCALE*sum(k*x for k,x in zip(gain,state))
            feedforward = (reference-v[2])/self.lag
            return clip(feedforward+feedback,-80,50)

        acceleration = command_acceleration(candidate_i)
        raw = v[2]+self.lag*acceleration
        previous_raw = v[2]+self.lag*command_acceleration(self.integral)
        rate = clip(raw,self.last_output-45*dt,self.last_output+25*dt)
        output = clip(rate,0,100)
        closing = max(0,v[2]-v[1])
        margin = max(0,gap-120)
        safe_closing = math.sqrt(2*35*margin)
        safe_cmd = v[1]+safe_closing
        output = min(output,safe_cmd)
        data = getattr(student,'data',{})
        center = float(data.get('center_distance',0.) or 0.)
        protective = (gap < 120 or (0 < center < .15)
                      or (closing > 0 and margin/max(closing,1e-6) < 1.))
        if protective:
            output = 0.
        if (abs(output-raw)<1e-9
                or (raw>output and raw<previous_raw)
                or (raw<output and raw>previous_raw)):
            self.integral = candidate_i
        if protective:
            self.integral = 0.
        self.last_output = output
        self.diagnostics = dict(gap=gap,error=error,relative_speed=relative_speed,
            predicted_middle=pred,reference=reference,
            lqi_acceleration=acceleration,protective=protective,
            integral=self.integral,dt=dt)
        return float(output)


_controller = None


def student_control(leader_state, middle_state, student_state, history=None) -> float:
    global _controller
    if _controller is None:
        _controller = PredictiveLQI()
    return _controller.step(leader_state,middle_state,student_state,history)