"""LQI: the LQR with an arm-integral state, designed jointly (dlqr on the augmented system), 2026-11-21.

    python desktop/scripts/lqi_design.py

Firmware 1.13's PID law is the LQI law:  u = Kp1 th1 + Kd1 w1 + Kp0 (th0 - r) + Ki0 sum(th0 - r) Ts + Kd0 w0,
the sum forward Euler (a tick uses the sum before adding its own error), from the start of the run. So an LQI design
u = -[K | KI] [x; z],  z(k+1) = z(k) + Ts (th0 - r),  maps 1:1 onto the PID knobs:
    Kp0 = -K0, Kp1 = -K1, Kd0 = -K2, Kd1 = -K3, Ki0 = -KI           (stab 1, no firmware change)
Weights: the LQR's Bryson limits [30 deg, 8 deg, 20, 30], duty 0.10, plus a limit z_max [rad s] on the integral.
Closed loop as observer_study.py (the study that reproduced the rig): 20 Nov model with stick-slip, firmware tick
(slew 100/s, clip 0.5), quantized sensors, pendulum reading lagging by `lag`, encoder leading by up to play/2,
model observer (nd 3, f_ab 10 Hz, 1 RK4 substep), plus the rig's +0.9 deg upright offset in the pendulum reading
(what puts the LQR's arm at about -15 deg). Balance from 5 deg, push +1.5 rad/s at 4 s, 8 s.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
from furuta_lab import estimation as E  # noqa: E402
import observer_study as S  # noqa: E402

P0 = E.full(json.loads((ROOT / 'data/estimation/suite_20261120/export/parameters.json').read_text())['params'])
S.P0 = P0
TS = 0.004
BRYSON = [30, 8, 20, 30]; DUTY = 0.10
OFFSET_DEG = 0.9


def model_d():
    lin = E.linear_models(P0, 12.0)
    A = np.real(lin['upright']['A']); B = np.real(lin['upright']['Bd'])
    M = expm(np.block([[A, B], [np.zeros((1, 5))]]) * TS)
    return M[:4, :4], M[:4, 4:]


def lqi(z_max):
    """K (1x5) for u = -K [x; z]; z_max None = plain LQR (integral gain 0)."""
    Ad, Bd = model_d()
    mx = np.array([math.radians(BRYSON[0]), math.radians(BRYSON[1]), BRYSON[2], BRYSON[3]])
    R = np.array([[1 / DUTY ** 2]])
    if z_max is None:
        Q = np.diag(1 / mx ** 2); P = solve_discrete_are(Ad, Bd, Q, R)
        K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad).ravel()
        return np.r_[K, 0.0]
    Aa = np.zeros((5, 5)); Aa[:4, :4] = Ad; Aa[4, 0] = TS; Aa[4, 4] = 1.0
    Ba = np.vstack([Bd, [[0.0]]])
    Q = np.diag(np.r_[1 / mx ** 2, 1 / z_max ** 2])
    P = solve_discrete_are(Aa, Ba, Q, R)
    return np.linalg.solve(R + Ba.T @ P @ Ba, Ba.T @ P @ Aa).ravel()


def poles(K):
    Ad, Bd = model_d()
    Aa = np.zeros((5, 5)); Aa[:4, :4] = Ad; Aa[4, 0] = TS; Aa[4, 4] = 1.0
    Ba = np.vstack([Bd, [[0.0]]])
    return np.log(np.linalg.eigvals(Aa - Ba @ K[None])) / TS


def simulate(K, plant, lag, play_deg, T=8.0, push_t=4.0, push=1.5):
    p = plant
    x = np.array([0.0, math.radians(5), 0.0, 0.0])
    nlag = int(round(lag / S.DT)); hist = [x[1]] * (nlag + 1)
    d = 0.0; u = 0.0; b = math.radians(play_deg); z = 0.0
    est = S.Model(3, p=P0)
    log = {'t': [], 'th0': [], 'th1': [], 'd': [], 'i': []}
    for k in range(int(T / TS)):
        _, i_now = E.rhs(x, d, p, 12.0)
        y0 = round((x[0] + 0.5 * b * math.tanh(i_now / 0.05)) / S.ENC) * S.ENC
        y1 = round((hist[0] + math.radians(OFFSET_DEG)) / S.PEN) * S.PEN
        xh = est.step((y0, y1), u)
        u = float(-K[:4] @ xh - K[4] * z)
        z += TS * xh[0]                                       # firmware: sum after use, forward Euler
        for s in range(S.NSUB):
            d += np.clip(np.clip(u, -.5, .5) - d, -0.1, 0.1)
            x = S.rk4(x, d, p, S.DT / 2); x = S.rk4(x, d, p, S.DT / 2)
            t = k * TS + (s + 1) * S.DT
            if abs(t - push_t) < S.DT / 2:
                x[3] += push
            hist = hist[1:] + [x[1]]
            log['t'].append(t); log['th0'].append(x[0]); log['th1'].append(S.wrap(x[1])); log['d'].append(d)
            log['i'].append(E.rhs(x, d, p, 12.0)[1])
            if abs(S.wrap(x[1])) > math.radians(35) or abs(x[0]) > math.radians(100) or not np.all(np.isfinite(x)):
                return None
    L = {k: np.array(v) for k, v in log.items()}
    t = L['t']; th1 = np.degrees(L['th1']); th0 = np.degrees(L['th0'])
    pre = (t > 2.0) & (t < 4.0); post = t >= 4.0; last = t > 7.0
    ret = np.where(post & (np.abs(th1) > 2.0))[0]
    return dict(rms=float(th1[pre].std()), arm_mean=float(th0[last].mean()), arm_std=float(th0[pre].std()),
                push_peak=float(np.abs(th1[post]).max()), push_arm=float(np.abs(th0[post]).max()),
                recover=float(t[ret[-1]] - 4.0) if len(ret) else 0.0,
                sat=float(100 * np.mean(np.abs(L['d']) >= 0.49)), irms=float(np.sqrt(np.mean(L['i'] ** 2))))


def plants():
    out = [('nominal', P0)]
    for g in (0.9, 1.1):
        q = dict(P0)
        for k in ('G', 'Jp', 'h', 'b1', 'tp'):
            q[k] *= g
        out.append((f'pendulum x{g}', q))
    return out


if __name__ == '__main__':
    t0 = time.time()
    K_lqr = lqi(None)
    K_pid = K_lqr.copy(); K_pid[4] = -0.1                      # Nedal's PID: LQR gains + Ki0 0.1
    designs = [('LQR (no integral)', K_lqr), ('PID (LQR + Ki0 0.1)', K_pid)]
    for zm in (2.0, 1.0, 0.5, 0.25, 0.1):
        designs.append((f'LQI z_max {zm:g} rad s', lqi(zm)))
    print('design                        gains -> knobs Kp0 Kp1 Kd0 Kd1 Ki0              slowest pole [1/s]')
    for n, K in designs:
        pl = poles(K); slow = pl[np.argmin(np.abs(pl.real))]
        print(f'{n:28s}  {(-K).round(4)}   {slow.real:+.2f}{slow.imag:+.2f}j')
    print('\ndesign                       lag  play  kept | pend rms | arm mean (last 1 s) | arm std | push peak, arm, recover | sat | I rms')
    rows = []
    for n, K in designs:
        for lag in (0.009, 0.012, 0.015):
            for play in (0.0, 3.0):
                res = [simulate(K, q, lag, play) for _, q in plants()]
                ok = [r for r in res if r]
                if not ok:
                    print(f'{n:28s} {lag * 1e3:3.0f}ms {play:3.0f}deg  0/{len(res)}'); continue
                m = {k: float(np.mean([r[k] for r in ok])) for k in ok[0]}
                rows.append(dict(design=n, lag=lag, play=play, kept=len(ok), of=len(res), **m))
                print(f'{n:28s} {lag * 1e3:3.0f}ms {play:3.0f}deg  {len(ok)}/{len(res)} | {m["rms"]:5.2f} deg | '
                      f'{m["arm_mean"]:+6.1f} deg | {m["arm_std"]:4.1f} | {m["push_peak"]:4.1f} deg, {m["push_arm"]:4.0f} deg, '
                      f'{m["recover"]:4.2f} s | {m["sat"]:3.1f} % | {m["irms"]:.2f} A', flush=True)
    out = ROOT / 'data/estimation/suite_20261120/lqi_design.json'
    out.write_text(json.dumps({'created': time.strftime('%Y-%m-%d %H:%M'), 'designs': {n: (-K).tolist() for n, K in designs},
                               'rows': rows}, indent=2))
    print(f'saved {out.relative_to(ROOT)} in {time.time() - t0:.0f} s')
