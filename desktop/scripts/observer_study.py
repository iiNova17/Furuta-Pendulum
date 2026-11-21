"""Estimator x LQR study on the 2026-11-20 model, with the measured pendulum lag and the shaft-coupling play.

    python desktop/observer_study.py

Port of the 2026-11-18 study (est_study.m): the plant is the re-identified model (estimation.rhs with stick-slip),
the firmware actuator (1 ms tick, slew 100/s, clip 0.5) and quantized sensors. The pendulum reading lags the
encoder by `lag` (9 / 12 / 15 ms measured on 2026-11-20). Play stress test: the encoder (motor side) leads the arm
by up to b/2 in the direction of the motor current (b = 3 deg, the play fitted from the small/slow runs).
Estimators: FD40 = filtered differences at 40 Hz (firmware 1.11), MODEL = the study's model observer (state at
the pendulum sample time, alpha-beta correction at 10 Hz, rolled forward through the stored duties).
Test: balance from 5 deg with a push (+1.5 rad/s on the pendulum at 1 s), 4 s, friction compensation off.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab import estimation as E  # noqa: E402

P0 = E.full(json.loads((ROOT / 'data/estimation/suite_20261120/export/parameters.json').read_text())['params'])
Vs = 12.0; TS = 0.004; DT = 0.001; NSUB = 4
ENC = 2 * math.pi / 1496; PEN = 2 * math.pi / 4096
K_NOW = np.array([-0.17947961822813555, -2.1550334773773545, -0.12564334394062221, -0.15857564945069502])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rk4(x, d, p, h):
    f = lambda y: np.array(E.rhs(y, d, p, Vs)[0])  # noqa: E731
    k1 = f(x); k2 = f(x + h / 2 * k1); k3 = f(x + h / 2 * k2); k4 = f(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def lqr(max_x, max_u, p=P0, Ts=TS):
    lin = E.linear_models(p, Vs)
    A = np.real(lin['upright']['A']); B = np.real(lin['upright']['Bd'])
    M = expm(np.block([[A, B], [np.zeros((1, 5))]]) * Ts); Ad, Bd = M[:4, :4], M[:4, 4:]
    mx = np.array([math.radians(max_x[0]), math.radians(max_x[1]), max_x[2], max_x[3]])
    Q = np.diag(1 / mx ** 2); R = np.array([[1 / max_u ** 2]])
    Pm = solve_discrete_are(Ad, Bd, Q, R)
    return np.linalg.solve(R + Bd.T @ Pm @ Bd, Bd.T @ Pm @ Ad).ravel()


class FD40:
    def __init__(self, nd=0):
        self.a = math.exp(-2 * math.pi * 40 * TS); self.first = True

    def step(self, y, u):
        th = np.array([y[0], wrap(y[1])])
        if self.first:
            self.prev = th.copy(); self.w = np.zeros(2); self.first = False
        dd = th - self.prev; dd[1] = wrap(dd[1])
        self.w = self.a * self.w + (1 - self.a) * dd / TS; self.prev = th
        return np.array([th[0], th[1], self.w[0], self.w[1]])


class Model:
    """The 2026-11-18 study observer (est_study.m, kind 'MODEL'), nd ticks of prediction."""
    def __init__(self, nd=3, p=P0, f_ab=10.0):
        self.nd = nd; self.p = p; self.first = True
        lam = math.exp(-2 * math.pi * f_ab * TS); self.al = 1 - lam ** 2; self.be = (1 - lam) ** 2

    def step(self, y, u):
        th = np.array([y[0], wrap(y[1])])
        if self.first:
            self.ub = np.zeros(self.nd + 1); self.y0b = np.full(self.nd + 1, th[0])
            self.xd = np.array([th[0], th[1], 0.0, 0.0]); self.first = False
        self.ub = np.r_[self.ub[1:], u]; self.y0b = np.r_[self.y0b[1:], th[0]]
        xd = self.xd
        for _ in range(2):
            xd = rk4(xd, float(np.clip(self.ub[0], -.5, .5)), self.p, TS / 2)
        r = np.array([self.y0b[0] - xd[0], wrap(th[1] - xd[1])])
        xd = xd.copy(); xd[:2] += self.al * r; xd[2:] += self.be / TS * r; xd[1] = wrap(xd[1])
        self.xd = xd; x = xd.copy()
        for j in range(1, self.nd + 1):
            for _ in range(2):
                x = rk4(x, float(np.clip(self.ub[j], -.5, .5)), self.p, TS / 2)
        x[1] = wrap(x[1])
        return x


def simulate(K, est, plant, lag, play_deg, T=4.0, push_t=1.0, push=1.5, seed=1):
    """Returns pass, pendulum rms (deg, after 2 s), duty at limit (%), current rms (A)."""
    p = plant
    x = np.array([0.0, math.radians(5), 0.0, 0.0])
    nlag = int(round(lag / DT)); hist = [x[1]] * (nlag + 1)
    d = 0.0; u = 0.0; b = math.radians(play_deg)
    log_th1, log_d, log_i = [], [], []
    for k in range(int(T / TS)):
        # sensors at the controller tick: encoder (motor side, play lead), pendulum (lagged), quantized
        _, i_now = E.rhs(x, d, p, Vs)
        lead = 0.5 * b * math.tanh(i_now / 0.05)
        y0 = round((x[0] + lead) / ENC) * ENC
        y1 = round(hist[0] / PEN) * PEN
        xh = est.step((y0, y1), u)
        u = float(-K @ xh)
        for s in range(NSUB):
            d += np.clip(np.clip(u, -.5, .5) - d, -0.1, 0.1)              # slew 100/s at 1 ms, clip 0.5
            x = rk4(x, d, p, DT / 2); x = rk4(x, d, p, DT / 2)
            t = k * TS + (s + 1) * DT
            if abs(t - push_t) < DT / 2:
                x[3] += push
            hist = hist[1:] + [x[1]]
            log_th1.append(wrap(x[1])); log_d.append(d); log_i.append(E.rhs(x, d, p, Vs)[1])
            if abs(wrap(x[1])) > math.radians(35) or abs(x[0]) > math.radians(100) or not np.all(np.isfinite(x)):
                return False, float('nan'), float('nan'), float('nan')
    th = np.degrees(np.array(log_th1)); n0 = int(2.0 / DT)
    return True, float(np.sqrt(np.mean(th[n0:] ** 2))), 100 * float(np.mean(np.abs(log_d) >= 0.49)), \
        float(np.sqrt(np.mean(np.array(log_i) ** 2)))


def plants():
    out = [('nominal', P0)]
    for g in (0.9, 1.1):
        q = dict(P0); [q.__setitem__(k, q[k] * g) for k in ('G', 'Jp', 'h', 'b1', 'tp')]; out.append((f'pendulum x{g}', q))
    for f in (0.7, 1.3):
        q = dict(P0); q['Beq'] *= f; out.append((f'arm friction x{f}', q))
    return out


if __name__ == '__main__':
    t0 = time.time()
    designs = [('K now (Table 6 design)', K_NOW)]
    for th1, uu in [(8, 0.10), (5, 0.10), (5, 0.20), (3, 0.20), (3, 0.30)]:
        designs.append((f'Bryson [30 {th1} 20 30], duty {uu:.2f}', lqr([30, th1, 20, 30], uu)))
    ests = [('FD40', lambda: FD40()), ('MODEL nd3', lambda: Model(3))]
    PL = plants()
    rows = []
    print(f'{"design":34s} {"estimator":10s} {"lag":>5s} {"play":>5s}  pass  pend rms  duty@lim  I rms')
    for dn, K in designs:
        for en, mk in ests:
            for lag in (0.009, 0.012, 0.015):
                for play in (0.0, 3.0):
                    res = [simulate(K, mk(), pq, lag, play) for _, pq in PL]
                    ok = [r for r in res if r[0]]
                    rows.append(dict(design=dn, K=K.tolist(), est=en, lag_ms=lag * 1e3, play_deg=play, passed=len(ok),
                                     of=len(res), pend_rms=float(np.mean([r[1] for r in ok])) if ok else None,
                                     sat=float(np.mean([r[2] for r in ok])) if ok else None,
                                     irms=float(np.mean([r[3] for r in ok])) if ok else None))
                    r = rows[-1]
                    print(f'{dn:34s} {en:10s} {lag * 1e3:4.0f}ms {play:4.0f}deg  {len(ok)}/{len(res)}  '
                          + (f'{r["pend_rms"]:6.2f} deg  {r["sat"]:5.1f} %  {r["irms"]:.2f} A' if ok else '   -'), flush=True)
    out = ROOT / 'data/estimation/suite_20261120/observer_study.json'
    out.write_text(json.dumps({'created': time.strftime('%Y-%m-%d %H:%M'), 'rows': rows}, indent=2))
    print(f'saved {out.relative_to(ROOT)} in {time.time() - t0:.0f} s')
