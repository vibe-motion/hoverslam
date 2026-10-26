"""Challenge tasks (spec/README.md section 8): mirror of src/flight/tasks.js.

load_task resolves `extends`; sample_initial draws the seeded initial condition; TaskTracker keeps one episode's
rewards / termination / success / metrics for vehicle `e` of a Twin. Plain floats, same expression order as JS.
"""
from __future__ import annotations

import copy
import math

from . import fmath
from .model import LIMITS
from .rng import Mulberry32
from .spec import TASKS, LAUNCH, WORLD
from .world import BOXES, GROUND, KIND

DEG = 180 / math.pi
_STREET = [b for b in BOXES if b['kind'] == KIND['street']]


def _merge(a: dict, b: dict) -> dict:
    out = dict(a)
    for k, v in b.items():
        out[k] = _merge(a[k], v) if isinstance(v, dict) and isinstance(a.get(k), dict) else v
    return out


def load_task(id_or_task) -> dict:
    raw = TASKS[id_or_task] if isinstance(id_or_task, str) else id_or_task
    if raw is None:
        raise KeyError(id_or_task)
    if not raw.get('extends'):
        return copy.deepcopy(raw)
    parent = load_task(raw['extends'])
    child = {k: v for k, v in raw.items() if k != 'extends'}
    t = _merge(parent, copy.deepcopy(child))
    t['metrics'] = list(dict.fromkeys([*parent.get('metrics', []), *child.get('metrics', [])]))
    t['extends'] = raw['extends']
    return t


def sample_initial(task: dict, seed: int, preset: str | None = None) -> dict:
    r = Mulberry32(seed)
    I = task['initial']
    rnd = I.get('randomize', {})
    draws = {}
    if I['type'] == 'launch':
        k = math.floor(r() * len(I['presets']))
        name = preset if preset is not None else I['presets'][k]
        P = LAUNCH['presets'][name]
        speed, angle, scale = P['speed'], P['angle'], 1
        for key, (a, b) in rnd.items():
            x = r.uniform(a, b)
            draws[key] = x
            if key == 'speed_kmh':
                speed += x
            elif key == 'angle_deg':
                angle += x
            elif key == 'spin_scale':
                scale *= x
            else:
                raise ValueError(f'unknown randomize key {key}')
        return {'type': 'launch', 'speed': speed, 'angle': angle, 'spin': [s * scale for s in P['spin']],
                'lead': P.get('lead', LAUNCH['defaults']['lead']), 'preset': name, 'draws': draws}
    if I['type'] == 'rest':
        heading = I.get('heading_deg', 0)
        for key, (a, b) in rnd.items():
            x = r.uniform(a, b)
            draws[key] = x
            if key == 'yaw_deg':
                heading += x
            else:
                raise ValueError(f'unknown randomize key {key}')
        return {'type': 'rest', 'x': I['position'][0], 'z': I['position'][2], 'heading_deg': heading, 'preset': None,
                'draws': draws}
    raise ValueError(f"unknown initial type {I['type']}")


def street_level(x: float, z: float) -> float:
    y = GROUND['y']
    for b in _STREET:
        dx, dz = x - b['c'][0], z - b['c'][2]
        lx, lz = b['cy'] * dx - b['sy'] * dz, b['sy'] * dx + b['cy'] * dz
        if abs(lx) < b['e'][0] and abs(lz) < b['e'][2] and b['top'] > y:
            y = b['top']
    return y


def tilt_of(q) -> float:
    x, z = float(q[0]), float(q[2])
    c = 1 - 2 * (x * x + z * z)
    return fmath.acos(1.0 if c > 1 else (-1.0 if c < -1 else c))


def height_of(p, q, comB) -> float:
    x, y, z, w = (float(a) for a in q)
    r3, r4, r5 = 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)
    return float(p[1]) - (r3 * float(comB[0]) + r4 * float(comB[1]) + r5 * float(comB[2])) - street_level(float(p[0]), float(p[2]))


def _norm3(v) -> float:
    return math.sqrt(float(v[0]) * float(v[0]) + float(v[1]) * float(v[1]) + float(v[2]) * float(v[2]))


class TaskTracker:
    """Episode bookkeeping of vehicle e of a coldgas.twin.Twin; same rules as tasks.js TaskTracker."""

    def __init__(self, task: dict):
        self.task = task
        self.S = task.get('success', {})
        self.Rw = task.get('reward', {})

    def reset(self, tw, e: int = 0):
        self.e = e
        self.cdt = tw.dt * tw.nsub
        self.m0 = self.m_prev = float(tw.prop.m[e])
        self.ret, self.done, self.reason, self.success = 0.0, False, None, False
        self.td, self.hold_max_tilt = None, 0.0
        self.hold_t, self.hold_best, self.hold_done, self.h_err2, self.h_err_n = 0.0, 0.0, False, 0.0, 0
        self.recovered, self.recover_t, self.recover_h = False, None, None
        self.stop_t = self.stop_d = None
        self.origin = (float(tw.S[0, e]), float(tw.S[2, e]))

    def update(self, tw):
        e, T, S, Rw = self.e, self.task, self.S, self.Rw
        t = float(tw.k[e] * tw.dt)
        p, q, w, comB = tw.S[0:3, e], tw.S[3:7, e], tw.S[10:13, e], tw.C[:, e]
        r = 0.0
        m = float(tw.prop.m[e])
        dm = self.m_prev - m
        self.m_prev = m
        if Rw.get('gas_per_kg'):
            r += Rw['gas_per_kg'] * dm
        tilt = tilt_of(q)
        left = bool(tw.left_deck[e])
        if Rw.get('tilt_per_rad_after_edge') and left:
            r += Rw['tilt_per_rad_after_edge'] * tilt
        h = height_of(p, q, comB)
        if T.get('target'):
            tg = T['target']
            err = h - tg['height']
            if not self.hold_done:
                if Rw.get('height_error_per_m'):
                    r += Rw['height_error_per_m'] * abs(err)
                self.h_err2 += err * err
                self.h_err_n += 1
                self.hold_t = self.hold_t + self.cdt if abs(err) <= tg['tolerance'] else 0.0
                if self.hold_t > self.hold_best:
                    self.hold_best = self.hold_t
                if self.hold_t >= tg['hold'] - 1e-9:
                    self.hold_done = True
        if T.get('recover_by_height') is not None and not self.recovered and left:
            hc = float(p[1]) - street_level(float(p[0]), float(p[2]))
            if hc >= T['recover_by_height'] and tilt <= S['recover_tilt_deg'] / DEG and _norm3(w) <= S['recover_angular_rate']:
                self.recovered, self.recover_t, self.recover_h = True, t, hc
        if tw.td[e] and self.td is None:
            d = tw.td_S[:, e]
            self.td = {'t': float(tw.td_t[e]), 'vy': abs(float(d[8])), 'vh': math.sqrt(float(d[7]) * float(d[7]) + float(d[9]) * float(d[9])),
                       'tilt': tilt_of(d[3:7]), 'rate': _norm3(d[10:13]), 'x': float(d[0]), 'z': float(d[2]),
                       'wheels': int(tw.td_wheels[e])}
        if self.td is not None and t > self.td['t'] and tilt > self.hold_max_tilt:
            self.hold_max_tilt = tilt
        terminated = truncated = False
        reason = None
        hold = S.get('upright_hold', 0)
        dx, dz = float(p[0]) - self.origin[0], float(p[2]) - self.origin[1]
        if tw.hs[e] and S.get('hull_strike') is False:
            terminated, reason = True, 'hull_strike'
        elif tw.wo[e] and S.get('wheel_overload', S.get('hull_strike')) is False:  # see tasks.js (spec/README.md 5.11)
            terminated, reason = True, 'wheel_overload'
        elif S.get('kerb') is False and tw.kerb_hit[e]:  # a loaded wheel on a kerb (5.13) where the task forbids it
            terminated, reason = True, 'kerb'
        elif float(p[1]) < LIMITS['floor_y'] or math.sqrt(dx * dx + dz * dz) > LIMITS['max_range']:
            terminated, reason = True, 'out_of_bounds'
        elif self.td is not None and T.get('target') and not self.hold_done:
            terminated, reason = True, 'landed_early'
        elif self.td is not None and t >= self.td['t'] + hold - 1e-9 and (
                'stop_speed' not in S or math.sqrt(float(tw.S[7, e]) * float(tw.S[7, e]) + float(tw.S[9, e]) * float(tw.S[9, e])) <= S['stop_speed']):
            terminated, reason = True, ('stopped' if 'stop_speed' in S else 'landed')  # stop tasks: horizontal COM speed
        elif t >= T['horizon'] - 1e-9:
            truncated, reason = True, 'timeout'
        if terminated or truncated:
            if reason == 'stopped':
                sx, sz = float(p[0]) - self.td['x'], float(p[2]) - self.td['z']
                self.stop_t, self.stop_d = t - self.td['t'], math.sqrt(sx * sx + sz * sz)
            self.success = reason in ('landed', 'stopped') and self.passes() and not (
                reason == 'stopped' and 'stop_within' in S and not self.stop_t <= S['stop_within'])
            r += self.terminal_reward(reason)
            self.done, self.reason = True, reason
        self.ret += r
        return r, terminated, truncated, reason

    def passes(self) -> bool:
        S, d = self.S, self.td
        if d is None:
            return False
        if 'max_vertical_speed' in S and not d['vy'] <= S['max_vertical_speed']:
            return False
        if 'max_tilt_deg' in S and not (d['tilt'] <= S['max_tilt_deg'] / DEG and self.hold_max_tilt <= S['max_tilt_deg'] / DEG):
            return False
        if 'max_angular_rate' in S and not d['rate'] <= S['max_angular_rate']:
            return False
        if 'max_horizontal_speed' in S and not d['vh'] <= S['max_horizontal_speed']:
            return False
        if 'max_pad_distance' in S and not self.pad_distance() <= S['max_pad_distance']:
            return False
        if 'recover_tilt_deg' in S and not self.recovered:
            return False
        if self.task.get('target') and not self.hold_done:
            return False
        return True

    def pad_distance(self):
        pad = self.task.get('pad')
        if not pad or self.td is None:
            return None
        dx, dz = self.td['x'] - pad['center'][0], self.td['z'] - pad['center'][2]
        return math.sqrt(dx * dx + dz * dz)

    def terminal_reward(self, reason) -> float:
        """Terminal reward of the final step (self.success already decided)."""
        Rw, d = self.Rw, self.td
        r = 0.0
        if self.success:
            r += Rw.get('terminal_success', 0)
        if reason == 'hull_strike':
            r += Rw.get('terminal_hull_strike', 0)
        if reason == 'wheel_overload':
            r += Rw.get('terminal_wheel_overload', Rw.get('terminal_hull_strike', 0))
        if reason == 'kerb':
            r += Rw.get('terminal_kerb', Rw.get('terminal_hull_strike', 0))
        if reason in ('timeout', 'out_of_bounds', 'landed_early'):
            r += Rw.get('terminal_timeout', 0)
        if d is not None:
            if Rw.get('terminal_vertical_speed_per_mps'):
                r += Rw['terminal_vertical_speed_per_mps'] * d['vy']
            if Rw.get('terminal_tilt_per_rad'):
                r += Rw['terminal_tilt_per_rad'] * d['tilt']
            if Rw.get('terminal_pad_distance_per_m'):
                r += Rw['terminal_pad_distance_per_m'] * self.pad_distance()
        return r

    def metrics(self, tw) -> dict:
        e, d, T = self.e, self.td, self.task
        allm = {
            'success': self.success,
            'touchdown_vertical_speed': d['vy'] if d else None,
            'touchdown_horizontal_speed': d['vh'] if d else None,
            'touchdown_tilt_deg': d['tilt'] * DEG if d else None,
            'touchdown_angular_rate': d['rate'] if d else None,
            'touchdown_wheels': bin(d['wheels']).count('1') if d else None,
            'peak_accel_g': float(tw.peak_sf[e]) / WORLD['gravity'],
            'hull_strike': bool(tw.hs[e]),
            'wheel_overload': bool(tw.wo[e]),
            'wheel_peak_normal': max(tw.peak_N[e].tolist()), 'wheel_peak_lat': max(tw.peak_lat[e].tolist()),
            'wheel_lat_impulse': max(tw.lat_impulse[e].tolist()),
            'gas_used_kg': self.m0 - float(tw.prop.m[e]),
            'landing_point': [d['x'], d['z']] if d else None,
            'hold_time': self.hold_best if T.get('target') else None,
            'rms_height_error': math.sqrt(self.h_err2 / self.h_err_n) if T.get('target') and self.h_err_n else None,
            'pad_distance': self.pad_distance(),
            'recovered': self.recovered if T.get('recover_by_height') is not None else None,
            'recover_time': self.recover_t, 'recover_height': self.recover_h,
            'stopped': self.reason == 'stopped', 'stop_time': self.stop_t, 'stop_distance': self.stop_d,
            'kerb': bool(tw.kerb_hit[e]),
        }
        out = {k: allm.get(k) for k in T.get('metrics', [])}
        out.update({'return': self.ret, 'reason': self.reason, 't_end': float(tw.k[e] * tw.dt), 'touchdown_t': d['t'] if d else None})
        return out
