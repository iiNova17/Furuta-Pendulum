"""Fit the shaft-coupling model (coupling_model.py) on the 2026-11-20 fit runs, judge it on the validation runs.

    python desktop/coupling_fit.py

Stage 1: coupling only (rJm, k, c, b), the rest is the rigid model of variant E (reidentify_variants.py).
Stage 2: coupling + arm friction and coupling inertia (Beq, tcp, tcn, h), as the rigid refinement did.
Score: fit % of the encoder (arm) angle, pendulum angle and current on b1_validation, 2 and 3 (never fitted).
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy import optimize

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import coupling_model as C  # noqa: E402
import reidentify as R  # noqa: E402
import reidentify_variants as V  # noqa: E402
from furuta_lab import estimation as E  # noqa: E402

R.OPTS['pend_delay'] = 0.010           # timing correction of variant E (best for today's runs)
OUT = R.ROOT / 'data/estimation/suite_20261120'
BASE = json.loads((OUT / 'variants.json').read_text())['best_model']
SUB = 8


def fit_stage(base, free, start, preps):
    U, X0, Y, fs = C.windows(preps, 4.0)
    sd = np.array([np.std(Y[..., 0] - Y[:, :1, 0]), np.std(Y[..., 1] - np.pi), np.std(Y[..., 2])])
    names = list(free)
    z0 = np.log([start[k] for k in names])
    calls = [0]

    def P_of(z):
        cp = dict(start); cp.update({k: math.exp(v) for k, v in zip(names, z)})
        return C.params(base, cp), cp

    def fun(z):
        P, _ = P_of(z)
        if P['rJm'] >= 0.98:
            return np.full(Y.size, 1e3)
        out = C.simulate_windows(P, U, X0, 1 / fs, preps[0]['Vs'], SUB)
        r = ((out - Y) / sd).ravel()
        calls[0] += 1
        return np.nan_to_num(r, nan=1e3, posinf=1e3, neginf=-1e3)
    c0 = .5 * float(np.sum(fun(z0) ** 2))
    sol = optimize.least_squares(fun, z0, diff_step=0.02, max_nfev=60, x_scale=1.0)
    P, cp = P_of(sol.x)
    return P, cp, c0, float(sol.cost), calls[0]


def score(P_or_rigid, rigid=False):
    res = []
    for p in V.VAL:
        pp = R.prep(p)
        if rigid:
            f = E.simulate(P_or_rigid, pp)['fit']
            res.append({k: f[k] for k in ('arm angle', 'pendulum angle', 'current')})
        else:
            res.append(C.simulate_run(P_or_rigid, pp, SUB))
    m = {k: float(np.mean([r[k] for r in res])) for k in res[0]}
    return m, res


if __name__ == '__main__':
    t0 = time.time()
    preps = [R.prep(V.FIT_ALL[k]) for k in 'ABC']
    fs0 = float(np.median([q['fs'] for q in preps]))
    for q in preps:
        q['fs'] = fs0
    rig, _ = score(BASE, rigid=True)
    print('rigid variant E      ' + '  '.join(f'{k} {v:.1f}' for k, v in rig.items()))
    start = {'rJm': 0.5, 'k': 50.0, 'c': 0.05, 'b': math.radians(3.0)}
    P1, cp1, c0, c1, n1 = fit_stage(BASE, ('rJm', 'k', 'c', 'b'), start, preps)
    s1, _ = score(P1)
    print(f'stage 1 coupling     ' + '  '.join(f'{k} {v:.1f}' for k, v in s1.items())
          + f'   | cost {c0:.4g} -> {c1:.4g} ({n1} evals): rJm {cp1["rJm"]:.3f}  k {cp1["k"]:.4g} A/rad  c {cp1["c"]:.3g}'
          f'  play {math.degrees(cp1["b"]):.2f} deg  (motor-side resonance {math.sqrt(cp1["k"] / (cp1["rJm"] * BASE["Ja"])) / (2 * math.pi):.1f} Hz)')
    base2 = dict(BASE)
    start2 = dict(cp1); start2.update({k: BASE[k] for k in ('Beq', 'tcp', 'tcn', 'h')})
    P2, cp2, c0b, c2, n2 = fit_stage({k: v for k, v in BASE.items()}, ('rJm', 'k', 'c', 'b', 'Beq', 'tcp', 'tcn', 'h'), start2, preps)
    s2, _ = score(P2)
    print(f'stage 2 + arm terms  ' + '  '.join(f'{k} {v:.1f}' for k, v in s2.items())
          + f'   | cost {c0b:.4g} -> {c2:.4g} ({n2} evals): rJm {cp2["rJm"]:.3f}  k {cp2["k"]:.4g}  c {cp2["c"]:.3g}'
          f'  play {math.degrees(cp2["b"]):.2f} deg  Beq x{cp2["Beq"] / BASE["Beq"]:.2f}  tc+ x{cp2["tcp"] / BASE["tcp"]:.2f}'
          f'  tc- x{cp2["tcn"] / BASE["tcn"]:.2f}  h x{cp2["h"] / BASE["h"]:.2f}')
    (OUT / 'coupling.json').write_text(json.dumps(R.clean({
        'created': time.strftime('%Y-%m-%d %H:%M'), 'pend_delay_ms': 10.0, 'base': 'variants.json best_model (variant E)',
        'rigid_score': rig, 'stage1': {'coupling': cp1, 'score': s1, 'cost': [c0, c1]},
        'stage2': {'coupling': cp2, 'score': s2, 'cost': [c0b, c2], 'params': P2}}), indent=2))
    print(f'saved {(OUT / "coupling.json").relative_to(R.ROOT)} in {time.time() - t0:.0f} s')
