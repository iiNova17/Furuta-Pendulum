"""Reference prototype, part 2, for docs/HANDOFF_PARAMETER_ESTIMATION.md (not app code).

Identification from the assembled rig only (one free swing + square-wave runs), a fast
batched simulator, and the simulation-error refinement of [Ja, Beq, tc+, tc-].
Parameters are "per K_tau": every torque-like quantity is divided by K_tau, so the
torque input is simply the current. Order: NAMES.

Run:  .venv/Scripts/python.exe docs/prototypes/twin_prototype.py     (about 2 minutes)
"""
import time
import numpy as np
from scipy import optimize
from estimation_prototype import (TRUE, G, command, sim_furuta, sim_swing, record, lumped, prep, lp, ols, grow,
                                  resting, edges, freeswing_ratio, hold)

NAMES = ['Rm', 'Kb', 'Ja', 'Jp', 'h', 'G', 'Beq', 'b1', 'tcp', 'tcn']
REFINED = [[2], [6], [8], [9]]          # Ja, Beq, tc+, tc-; the rest stays at the least-squares value


def rhs(x, d, th, Vs, eps=.1):
    """Eq. 93 divided by K_tau, vectorised over leading axes. Open circuit when duty is exactly 0."""
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1, tcp, tcn = [th[..., k] for k in range(10)]
    t1, w0, w1 = x[..., 1], x[..., 2], x[..., 3]; s = np.sin(t1); c = np.cos(t1)
    cur = np.where(d != 0, (d * Vs - Kb * w0) / Rm, 0.)
    fr = np.where(w0 > 0, tcp, tcn) * np.tanh(w0 / eps)
    f0 = cur - Beq * w0 - fr - 2 * Jp * s * c * w0 * w1 + h * s * w1 ** 2
    f1 = Jp * s * c * w0 ** 2 - b1 * w1 + Gk * s
    m00 = Ja + Jp * s * s; det = m00 * Jp - (h * c) ** 2
    return np.stack([w0, w1, (Jp * f0 - h * c * f1) / det, (-h * c * f0 + m00 * f1) / det], -1), cur


def sim_windows(th, U, X0, dt, Vs):
    """th (P,10) parameter sets; U (W,n) duty at step dt; X0 (W,4) initial states.
    Simulates all P x W cases at once (RK4). Returns X (P,W,n,4) and current I (P,W,n)."""
    P_ = th.shape[0]; W, n = U.shape; x = np.broadcast_to(X0, (P_, W, 4)).copy()
    X = np.empty((P_, W, n, 4)); I = np.empty((P_, W, n)); th = th[:, None, :]
    for k in range(n):
        d = np.broadcast_to(U[:, k], (P_, W))
        k1, cur = rhs(x, d, th, Vs); k2, _ = rhs(x + .5 * dt * k1, d, th, Vs); k3, _ = rhs(x + .5 * dt * k2, d, th, Vs); k4, _ = rhs(x + dt * k3, d, th, Vs)
        X[:, :, k] = x; I[:, :, k] = cur; x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return X, I


def windows(recs, fs, fd, Vs, seconds, sub=2):
    """Cut runs into windows; each starts from the filtered measured state."""
    n = int(seconds * fs); U = []; X0 = []; Y = []
    for rec in recs:
        s = prep(rec, fs, fd, Vs); e = int(fs / 2)
        for a in range(e, len(rec['t']) - n - e, n):
            U.append(np.repeat(rec['app'][a:a + n], sub)); X0.append([s['th0'][a], s['th1'][a], s['w0'][a], s['w1'][a]])
            Y.append(np.column_stack([s['th0'][a:a + n], s['th1'][a:a + n], s['i'][a:a + n]]))
    return np.array(U), np.array(X0), np.array(Y)


def residual(th, U, X0, Y, fs, Vs, sub=2):
    """Simulated minus measured [arm angle, pendulum angle, current], each divided by its SD."""
    X, I = sim_windows(th, U, X0, 1 / (fs * sub), Vs)
    sim = np.stack([X[:, :, ::sub, 0], X[:, :, ::sub, 1], I[:, :, ::sub]], -1)
    sd = np.array([np.std(Y[..., 0] - Y[:, :1, 0]), np.std(Y[..., 1] - np.pi), np.std(Y[..., 2])])
    return ((sim - Y) / sd).reshape(th.shape[0], -1), sim


def fitpct(sim, Y):
    """100 (1 - |y - yhat| / |y - mean y|) for arm angle (relative to window start), pendulum angle, current."""
    out = []
    for k in range(3):
        ref = Y[:, :1, k] if k == 0 else 0
        y = Y[..., k] - ref; z = sim[..., k] - ref
        out.append(100 * (1 - np.linalg.norm(z - y) / np.linalg.norm(y - y.mean())))
    return out


def refine(th0, U, X0, Y, fs, Vs, groups=REFINED):
    """Least squares on the simulation error. One batched simulation gives residual and Jacobian."""
    def expand(Z):
        th = np.repeat(th0[None], len(Z), 0)
        for j, idx in enumerate(groups): th[:, idx] *= np.exp(Z[:, j:j + 1])
        return th
    fun = lambda z: residual(expand(z[None]), U, X0, Y, fs, Vs)[0][0]
    def jac(z):
        h = 1e-3; Z = z[None] + np.vstack([np.zeros(len(z)), h * np.eye(len(z))]); r = residual(expand(Z), U, X0, Y, fs, Vs)[0]
        return ((r[1:] - r[0]) / h).T
    sol = optimize.least_squares(fun, np.zeros(len(groups)), jac=jac, bounds=(-2, 2), max_nfev=40)
    return expand(sol.x[None])[0], sol


def assembled_ls(swing, runs, fs, fd, fp, Vs, wmin=.3):
    """Least-squares model from the assembled rig: electrical, pendulum and arm equations."""
    free = freeswing_ratio(swing, fs, fd, fp); S = [prep(r, fs, fd, Vs) for r in runs]
    Xe, ye, Xp, yp, Xa, ya = [], [], [], [], [], []
    step = max(1, int(fs / (4 * fp)))
    for r, s in zip(runs, S):
        keep = edges(len(r['t']), fs, fp); w0, a0, w1, a1 = s['w0'], s['a0'], s['w1'], s['a1']; sn = np.sin(s['th1']); c = np.cos(s['th1'])
        drv = np.flatnonzero(keep & ~grow(r['app'] == 0, int(fs / fp)))[::step]
        Xe.append(lp(np.column_stack([s['V'], w0]), fs, fp)[drv]); ye.append(lp(s['i'], fs, fp)[drv])
        ip = np.flatnonzero(keep)[::step]
        Xp.append(lp(np.column_stack([c * a0, a1 - sn * c * w0 ** 2, w1]), fs, fp)[ip]); yp.append(lp(sn, fs, fp)[ip])
    b, _, _ = ols(np.vstack(Xe), np.concatenate(ye)); Rm = 1 / b[0]; Kb = -b[1] / b[0]
    (rh, rJ, rb), _, _ = ols(np.vstack(Xp), np.concatenate(yp))
    for r, s in zip(runs, S):
        keep = edges(len(r['t']), fs, fp); w0, a0, w1, a1 = s['w0'], s['a0'], s['w1'], s['a1']; sn = np.sin(s['th1']); c = np.cos(s['th1'])
        Tr = rJ * (sn * sn * a0 + 2 * sn * c * w0 * w1) + rh * (c * a1 - sn * w1 ** 2)
        pos = (w0 > wmin).astype(float); neg = -(w0 < -wmin).astype(float)       # Coulomb friction per direction
        mv = np.flatnonzero(keep & ~grow(resting(w0, fs, wmin), int(fs / fp / 2)) & (np.abs(w0) > wmin))[::step]
        Xa.append(lp(np.column_stack([a0, Tr, w0, pos, neg]), fs, fp)[mv]); ya.append(lp(s['i'], fs, fp)[mv])
    a, _, _ = ols(np.vstack(Xa), np.concatenate(ya))
    th = np.array([Rm, Kb, a[0], a[1] * rJ, a[1] * rh, a[1], max(a[2], 1e-5), a[1] * rb, a[3], a[4]])
    return th, dict(Larm=G * rh, rJ_free=free['rJ'], rJ_driven=rJ)


def linear_models(th, Vs):
    """Eq. 99 (upright) and Eq. 98 (hanging) from per-K_tau parameters; input volts, Bd for duty."""
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1 = th[:8]; beta = 1 / Rm; Bp = Beq + Kb / Rm; D = Ja * Jp - h * h
    A4 = np.array([[0, 0, D, 0], [0, 0, 0, D], [0, -h * Gk, -Jp * Bp, h * b1], [0, Ja * Gk, h * Bp, -Ja * b1]]) / D
    B4 = beta / D * np.array([0, 0, Jp, -h])
    Ah = np.array([[0, 0, D, 0], [0, 0, 0, D], [0, -h * Gk, -Jp * Bp, -h * b1], [0, -Ja * Gk, -h * Bp, -Ja * b1]]) / D
    Bh = beta / D * np.array([0, 0, Jp, h])
    return dict(A4=A4, B4=B4, Bd=Vs * B4, Ah=Ah, Bh=Bh)


if __name__ == '__main__':
    P = dict(TRUE); L = lumped(P); Kt = P['Kt']; fs = 250; Vs = P['Vs']
    truth = np.array([P['Rm'], P['Kb'], L['Ja'] / Kt, L['Jp'] / Kt, L['h'] / Kt, L['mgl'] / Kt, L['Beq'] / Kt, L['b1'] / Kt, P['tc'] / Kt, P['tc'] / Kt])
    sq = lambda A, f: (lambda t: A * np.where((f * t + .25) % 1 < .5, 1., -1.))

    def run(amp, freqs, seed, sig=.12):
        seg = [(.5, hold(0))] + [(4., sq(amp, f)) for f in freqs] + [(1., hold(0))]
        t, app = command(seg); X, I = sim_furuta(t, app, P, [0, np.pi, 0, 0])
        return record(t, X[:, 0], I, app, fs, np.random.default_rng(seed), sig, th1=X[:, 1])

    tt = np.arange(0, 25, 1e-3); z = np.zeros(len(tt)); scores = {'least squares': [], 'refined': [], 'true parameters': []}; errs = {'least squares': [], 'refined': []}
    for seed in range(3):
        swing = record(tt, z, z, z, fs, np.random.default_rng(10 * seed + 1), .12, th1=sim_swing(tt, P, .6))
        fit_runs = [run(.3, (1.5, 2., 3., 4., 2.5), 10 * seed + 2), run(.4, (2., 3., 4., 5., 2.5), 10 * seed + 3)]
        val = run(.35, (1.75, 2.5, 3.5, 2.), 10 * seed + 4)
        th0, info = assembled_ls(swing, fit_runs, fs, 40, 10, Vs)
        U, X0, Y = windows(fit_runs, fs, 40, Vs, 4.); Uv, X0v, Yv = windows([val], fs, 40, Vs, 16.)
        t0 = time.time(); th1, sol = refine(th0, U, X0, Y, fs, Vs)
        if seed == 0: print('refinement took %.0f s; Larm error %+.1f %%; Jp/G free swing vs driven %+.2f %%' % (time.time() - t0, 100 * (info['Larm'] / P['Larm'] - 1), 100 * (info['rJ_driven'] / info['rJ_free'] - 1)))
        for name, th in [('least squares', th0), ('refined', th1), ('true parameters', truth)]:
            scores[name].append(fitpct(residual(th[None], Uv, X0v, Yv, fs, Vs)[1][0], Yv))
            if name in errs: errs[name].append(100 * (th / truth - 1))
    print('Validation run, one 16 s open-loop simulation, fit % (arm angle, pendulum angle, current):')
    for k, v in scores.items(): print('  %-16s %s' % (k, np.round(np.mean(v, 0))))
    print('Parameter error %, mean over seeds (largest):')
    for k, v in errs.items():
        e = np.array(v); print('  %-16s ' % k + '  '.join('%s %+.0f (%.0f)' % (n, m, x) for n, m, x in zip(NAMES, e.mean(0), np.abs(e).max(0))))
    M = linear_models(th1, Vs); Mt = linear_models(truth, Vs)
    print('Upright poles, refined model:', np.round(np.sort(np.linalg.eigvals(M['A4']).real), 2), ' true:', np.round(np.sort(np.linalg.eigvals(Mt['A4']).real), 2))
