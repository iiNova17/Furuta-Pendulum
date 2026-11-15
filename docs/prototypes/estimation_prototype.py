"""Reference prototype for docs/HANDOFF_PARAMETER_ESTIMATION.md (not app code).

Simulates the bench (Eq. numbers refer to Report/furuta_model.pdf), records it the
way the firmware streams it (encoder quantisation, noisy current, bridge
high-impedance at zero duty) and fits it with the pipeline the handoff specifies.
Also runs the app's current estimators on the same data for comparison.

Run:  .venv/Scripts/python.exe docs/prototypes/estimation_prototype.py
"""
import sys
from pathlib import Path
import numpy as np
from scipy import signal

G = 9.80665
CPR = 1496.          # arm encoder counts per output revolution (34 x 11 x 4, unverified)
TRUE = dict(Rm=5.5, Kb=.39, eta=.7, Jmo=6.5e-4, bmo=4e-4, tc=.06, JL=2e-5,
            m=.05, Lp=.10, Larm=.12, J1=1.67e-4, J0=3e-4, b0=1e-4, b1=5e-5, Vs=12.)
TRUE['Kt'] = TRUE['eta'] * TRUE['Kb']      # K_tau = eta_g * K_b  (Eq. 62/69)


# ---------------------------------------------------------------- simulation
def command(segments, dt=1e-3, slew=10.):
    """Applied duty as the firmware produces it: slew limit, through zero before reversing."""
    total = sum(s[0] for s in segments); n = int(total / dt); t = np.arange(n) * dt
    req = np.zeros(n); at = 0
    for dur, fn in segments:
        ix = (t >= at) & (t < at + dur); req[ix] = fn(t[ix] - at); at += dur
    app = np.zeros(n); a = 0.
    for k in range(n):
        tgt = req[k]
        if a * tgt < 0: tgt = 0
        a += np.clip(tgt - a, -slew * dt, slew * dt)
        if abs(a) < 1e-9: a = 0.
        app[k] = a
    return t, app


def sgn(w, eps=.02):
    return np.tanh(w / eps)


def rk4(f, x, dt):
    k1 = f(x); k2 = f(x + .5 * dt * k1); k3 = f(x + .5 * dt * k2); k4 = f(x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def bridge_current(d, w0, P):
    """Eq. 88 while driven. Zero duty = TB6612 standby = open circuit: i = 0, no back-EMF braking."""
    return (d * P['Vs'] - P['Kb'] * w0) / P['Rm'] if d != 0 else 0.


def sim_motor(t, app, P, J):
    """Stage 1 plant, Eq. 89."""
    dt = t[1] - t[0]; th = np.zeros(len(t)); cur = np.zeros(len(t)); x = np.zeros(2)
    for k in range(len(t)):
        d = app[k]; th[k] = x[0]; cur[k] = bridge_current(d, x[1], P)
        x = rk4(lambda x: np.array([x[1], (P['Kt'] * bridge_current(d, x[1], P) - P['bmo'] * x[1] - P['tc'] * sgn(x[1])) / J]), x, dt)
    return th, cur


def lumped(P):
    """Eq. 77."""
    return dict(Ja=P['J0'] + P['Jmo'] + P['m'] * P['Larm'] ** 2, Jp=P['J1'] + P['m'] * P['Lp'] ** 2,
                h=P['m'] * P['Larm'] * P['Lp'], Beq=P['b0'] + P['bmo'], b1=P['b1'], mgl=P['m'] * G * P['Lp'])


def furuta_rhs(x, cur, P, L):
    """Mechanical part of Eq. 84 with the measured/derived current as torque input."""
    th0, th1, w0, w1 = x; s = np.sin(th1); c = np.cos(th1)
    f0 = P['Kt'] * cur - L['Beq'] * w0 - P['tc'] * sgn(w0) - 2 * L['Jp'] * s * c * w0 * w1 + L['h'] * s * w1 ** 2
    f1 = L['Jp'] * s * c * w0 ** 2 - L['b1'] * w1 + L['mgl'] * s
    a = np.linalg.solve([[L['Ja'] + L['Jp'] * s * s, L['h'] * c], [L['h'] * c, L['Jp']]], [f0, f1])
    return np.array([w0, w1, a[0], a[1]])


def sim_furuta(t, app, P, x0):
    """Stage 2 plant, Eq. 93 (inductance neglected)."""
    dt = t[1] - t[0]; L = lumped(P); X = np.zeros((len(t), 4)); cur = np.zeros(len(t)); x = np.array(x0, float)
    for k in range(len(t)):
        d = app[k]; X[k] = x; cur[k] = bridge_current(d, x[2], P)
        x = rk4(lambda x: furuta_rhs(x, bridge_current(d, x[2], P), P, L), x, dt)
    return X, cur


def sim_swing(t, P, phi0):
    """Arm clamped, Eq. 97. Returns theta1 = phi + pi."""
    dt = t[1] - t[0]; L = lumped(P); x = np.array([phi0, 0.]); out = np.zeros(len(t))
    for k in range(len(t)):
        out[k] = x[0]; x = rk4(lambda x: np.array([x[1], (-L['b1'] * x[1] - L['mgl'] * np.sin(x[0])) / L['Jp']]), x, dt)
    return out + np.pi


def record(t, th0, cur, app, fs, rng, sigma_i, th1=None):
    """Decimate to the stream rate: encoder quantisation, current noise, AS5600 12 bit."""
    step = int(round(1 / (fs * (t[1] - t[0])))); ix = np.arange(0, len(t), step)
    out = dict(t=t[ix], th0=np.round(th0[ix] * CPR / (2 * np.pi)) * 2 * np.pi / CPR,
               i=cur[ix] + rng.normal(0, sigma_i, len(ix)), app=app[ix])
    if th1 is not None:
        out['th1'] = np.round(th1[ix] * 4096 / (2 * np.pi)) * 2 * np.pi / 4096 + rng.normal(0, .0005, len(ix))
    return out


# ------------------------------------------------------------------ pipeline
def lp(x, fs, fc):
    """Zero-phase low-pass (2nd-order Butterworth, forward-backward)."""
    return signal.sosfiltfilt(signal.butter(2, fc, fs=fs, output='sos'), x, axis=0)


def prep(rec, fs, fd, Vs):
    """Report rule: the same zero-phase filter (cut-off fd) on every measured signal, then central differences."""
    s = {'i': lp(rec['i'], fs, fd), 'V': lp(rec['app'] * Vs, fs, fd), 'th0': lp(rec['th0'], fs, fd)}
    s['w0'] = np.gradient(s['th0'], 1 / fs); s['a0'] = np.gradient(s['w0'], 1 / fs)
    if 'th1' in rec:
        s['th1'] = lp(np.unwrap(rec['th1']), fs, fd)
        s['w1'] = np.gradient(s['th1'], 1 / fs); s['a1'] = np.gradient(s['w1'], 1 / fs)
    return s


def ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None); r = y - X @ beta
    cov = np.linalg.inv(X.T @ X) * (r @ r) / max(1, len(y) - X.shape[1])
    return beta, np.sqrt(np.diag(cov)), 1 - (r @ r) / np.sum((y - y.mean()) ** 2)


def fit(X, y, fs, fp, mask):
    """Filter the left side and every regressor column with the same low-pass fp (so nonlinear
    regressors such as sgn(w) are treated like the measured signals), decimate, keep masked rows."""
    Xf = lp(X, fs, fp); yf = lp(y, fs, fp)
    return ols(Xf[np.flatnonzero(mask)[::max(1, int(fs / (4 * fp)))]], yf[np.flatnonzero(mask)[::max(1, int(fs / (4 * fp)))]])


def grow(mask, n):
    return np.convolve(mask.astype(float), np.ones(2 * n + 1), 'same') > 0


def resting(w, fs, wmin, seconds=.15):
    """True where |w| <= wmin for at least `seconds` (arm stuck, friction is not tc*sgn(w))."""
    slow = np.abs(w) <= wmin; out = np.zeros(len(w), bool); k = 0
    while k < len(w):
        if slow[k]:
            j = k
            while j < len(w) and slow[j]: j += 1
            if (j - k) / fs >= seconds: out[k:j] = True
            k = j
        else: k += 1
    return out


def edges(n, fs, fp):
    keep = np.zeros(n, bool); e = int(3 * fs / fp); keep[e:-e] = True; return keep


def stage1(rec, fs, fd, fp, Vs, eta, wmin=.5):
    """Eq. 92. Electrical equation solved for the noisy current: i = V/Rm - (Kb/Rm) w."""
    s = prep(rec, fs, fd, Vs); keep = edges(len(rec['t']), fs, fp)
    driven = keep & ~grow(rec['app'] == 0, int(fs / fp))
    b, _, r2e = fit(np.column_stack([s['V'], s['w0']]), s['i'], fs, fp, driven)
    Rm = 1 / b[0]; Kb = -b[1] / b[0]
    sg = np.sign(s['w0']) * (np.abs(s['w0']) > wmin)
    moving = keep & ~grow(resting(s['w0'], fs, wmin), int(fs / fp / 2)) & (np.abs(s['w0']) > wmin)
    q, _, r2m = fit(np.column_stack([s['a0'], s['w0'], sg]), s['i'], fs, fp, moving)
    Kt = eta * Kb
    return dict(Rm=Rm, Kb=Kb, Kt=Kt, J=q[0] * Kt, b=q[1] * Kt, tc=q[2] * Kt, r2e=r2e, r2m=r2m)


def stage2(rec, fs, fd, fp, Vs, Kt, tc, mgl, wmin=.3):
    """Eq. 95, p = [Ja, Jp, h, Beq, b1]; the two joint equations are weighted by their residual SD."""
    s = prep(rec, fs, fd, Vs); n = len(rec['t']); keep = edges(n, fs, fp)
    w0, a0, w1, a1 = s['w0'], s['a0'], s['w1'], s['a1']; sn = np.sin(s['th1']); c = np.cos(s['th1']); z = np.zeros(n)
    sg = np.sign(w0) * (np.abs(w0) > wmin)
    moving = keep & ~grow(resting(w0, fs, wmin), int(fs / fp / 2)) & (np.abs(w0) > wmin)
    Xa = np.column_stack([a0, sn * sn * a0 + 2 * sn * c * w0 * w1, c * a1 - sn * w1 ** 2, w0, z]); ya = Kt * s['i'] - tc * sg
    Xp = np.column_stack([z, a1 - sn * c * w0 ** 2, c * a0, z, w1]); yp = mgl * sn
    Xaf = lp(Xa, fs, fp); yaf = lp(ya, fs, fp); Xpf = lp(Xp, fs, fp); ypf = lp(yp, fs, fp)
    step = max(1, int(fs / (4 * fp))); ia = np.flatnonzero(moving)[::step]; ip = np.flatnonzero(keep)[::step]
    wa = wp = 1.
    for _ in range(4):
        p, se, _ = ols(np.vstack([Xaf[ia] / wa, Xpf[ip] / wp]), np.r_[yaf[ia] / wa, ypf[ip] / wp])
        wa = np.std(yaf[ia] - Xaf[ia] @ p); wp = np.std(ypf[ip] - Xpf[ip] @ p)
    return dict(Ja=p[0], Jp=p[1], h=p[2], Beq=p[3], b1=p[4], se=se)


def freeswing(rec, fs, fd, fp, mgl):
    """Eq. 97 as a regression: -mgLp sin(phi) = Jp phi'' + b1 phi'."""
    s = prep(rec, fs, fd, 0.); phi = s['th1'] - np.pi
    q, _, r2 = fit(np.column_stack([s['a1'], s['w1']]), -mgl * np.sin(phi), fs, fp, edges(len(phi), fs, fp))
    return dict(Jp=q[0], b1=q[1], wn=np.sqrt(mgl / q[0]), zeta=q[1] / (2 * np.sqrt(mgl * q[0])), r2=r2)


# ------------------------------------------- geometry-free Stage 2 (handoff section "Scale")
def freeswing_ratio(rec, fs, fd, fp):
    """Eq. 97 without any known constant: sin(phi) = -(Jp/G) phi'' - (b1/G) phi',  G = m g Lp."""
    s = prep(rec, fs, fd, 0.); phi = s['th1'] - np.pi
    q, se, r2 = fit(np.column_stack([-s['a1'], -s['w1']]), np.sin(phi), fs, fp, edges(len(phi), fs, fp))
    return dict(rJ=q[0], rb=q[1], se=se, r2=r2)


def added_mass(rJ_plain, rJ_loaded, dm, d):
    """Absolute G = m g Lp and Jp from two free swings, the second with a known mass dm at distance d
    from the pivot:  G/Jp = 1/rJ_plain,  (G + dm g d)/(Jp + dm d^2) = 1/rJ_loaded."""
    A1 = 1 / rJ_plain; A2 = 1 / rJ_loaded
    Jp = dm * (G * d - A2 * d * d) / (A2 - A1)
    return dict(G=A1 * Jp, Jp=Jp)


def stage2_ratios(rec, fs, fd, fp, Vs, tc_over_Kt=None, wmin=.3):
    """Stage 2 with no geometry and no torque constant.
    Pendulum equation (Eq. 93b divided by G):  sin th1 = (h/G) c th0'' + (Jp/G)(th1'' - s c th0'^2) + (b1/G) th1'
    Arm equation (Eq. 93a divided by Ktau):    i = (Ja/Kt) th0'' + (G/Kt) Tr + (Beq/Kt) th0' + (tc/Kt) sgn
      with Tr = (Jp/G)(s^2 th0'' + 2 s c th0' th1') + (h/G)(c th1'' - s th1'^2)  from the pendulum fit."""
    s = prep(rec, fs, fd, Vs); n = len(rec['t']); keep = edges(n, fs, fp)
    w0, a0, w1, a1 = s['w0'], s['a0'], s['w1'], s['a1']; sn = np.sin(s['th1']); c = np.cos(s['th1'])
    q, sep, r2p = fit(np.column_stack([c * a0, a1 - sn * c * w0 ** 2, w1]), sn, fs, fp, keep)
    rh, rJ, rb = q
    Tr = rJ * (sn * sn * a0 + 2 * sn * c * w0 * w1) + rh * (c * a1 - sn * w1 ** 2)
    sg = np.sign(w0) * (np.abs(w0) > wmin)
    moving = keep & ~grow(resting(w0, fs, wmin), int(fs / fp / 2)) & (np.abs(w0) > wmin)
    if tc_over_Kt is None:
        a, sea, r2a = fit(np.column_stack([a0, Tr, w0, sg]), s['i'], fs, fp, moving)
    else:
        a, sea, r2a = fit(np.column_stack([a0, Tr, w0]), s['i'] - tc_over_Kt * sg, fs, fp, moving); a = np.r_[a, tc_over_Kt]
    return dict(rh=rh, rJ=rJ, rb=rb, Larm=G * rh, Ja_K=a[0], G_K=a[1], Beq_K=a[2], tc_K=a[3], r2p=r2p, r2a=r2a)


def absolute(r, Gabs):
    """Apply the one absolute reference (G = m g Lp) to the ratio results."""
    Kt = Gabs / r['G_K']
    return dict(Kt=Kt, Ja=r['Ja_K'] * Kt, Jp=r['rJ'] * Gabs, h=r['rh'] * Gabs, Beq=r['Beq_K'] * Kt, b1=r['rb'] * Gabs, tc=r['tc_K'] * Kt, Larm=r['Larm'])


# ---------------------------------------------------------------------- demo
def show(name, est, true):
    print('  %-34s' % name + '  '.join('%s=%.4g (%+.1f%%)' % (k, est[k], 100 * (est[k] / true[k] - 1)) for k in true if k in est))


def spread(res, true):
    return '  '.join('%s %+.1f%% +-%.1f%%' % (k, 100 * (np.mean([r[k] for r in res]) / true[k] - 1), 100 * np.std([r[k] for r in res]) / true[k]) for k in true)


def hold(v): return lambda t: np.full(len(t), v)
def chirp(A, f0, f1, T): return lambda t: A * np.sin(2 * np.pi * (f0 * t + (f1 - f0) * t * t / (2 * T)))
def square(A, f): return lambda t: A * np.where((f * t) % 1 < .5, 1., -1.)


def stage1_sequence():
    seg = [(.5, hold(0))]
    for s in (1, -1):
        seg += [(.8, hold(s * d)) for d in (.3, .5, .7)] + [(1., hold(0))]
    return seg + [(10., chirp(.6, .3, 6, 10)), (.5, hold(0))]


def core_rows(rec):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'desktop'))
    from furuta_lab import wire
    a = np.zeros(len(rec['t']), wire.CORE); a['device_us'] = np.round(rec['t'] * 1e6)
    a['count'] = np.round(rec['th0'] * CPR / (2 * np.pi)); a['current_mv'] = 2500 + 185 * rec['i']; a['adc_raw'] = 3000
    a['validity'] = 7; a['applied'] = rec['app']
    if 'th1' in rec: a['pend_raw'] = np.mod(np.round((rec['th1'] - np.pi) * 4096 / (2 * np.pi) + 2048), 4096)
    return a


if __name__ == '__main__':
    P = TRUE; L = lumped(P); J = P['Jmo'] + P['JL']; rng = np.random.default_rng(3)
    true1 = dict(Rm=P['Rm'], Kb=P['Kb'], Kt=P['Kt'], J=J, b=P['bmo'], tc=P['tc'])
    true2 = {k: L[k] for k in ('Ja', 'Jp', 'h', 'Beq', 'b1')}
    t, app = command(stage1_sequence()); th, cur = sim_motor(t, app, P, J)
    print('STAGE 1 (true tau_m = %.1f ms). sigma = current noise per stream sample.' % (1e3 * P['Rm'] * J / (P['Rm'] * P['bmo'] + P['Kt'] * P['Kb'])))
    for fs, sig, fd in [(100, .12, 20), (250, .12, 40), (250, .03, 40)]:
        res = [stage1(record(t, th, cur, app, fs, np.random.default_rng(k), sig), fs, fd, 10, P['Vs'], P['eta']) for k in range(20)]
        print('  new pipeline  fs=%d sigma=%.2f A fd=%d fp=10, 20 noise seeds:  ' % (fs, sig, fd) + spread(res, true1))
    print('FREE SWING (0.6 rad release, 25 s)')
    tt = np.arange(0, 25, 1e-3); zero = np.zeros(len(tt))
    r = freeswing(record(tt, zero, zero, zero, 250, rng, .12, th1=sim_swing(tt, P, .6)), 250, 40, 10, L['mgl'])
    show('new pipeline fs=250', r, dict(Jp=L['Jp'], b1=L['b1']))
    print('STAGE 2 (pendulum hanging)')
    cases = [('chirp 35 % 1.2-4 Hz 20 s', [(.5, hold(0)), (20., chirp(.35, 1.2, 4, 20)), (1., hold(0))]),
             ('square 30 % at 1.5/2/3/4/2.5 Hz', [(.5, hold(0))] + [(4., square(.3, f)) for f in (1.5, 2., 3., 4., 2.5)] + [(1., hold(0))])]
    runs = {}
    for label, seg in cases:
        t2, app2 = command(seg); X, I = sim_furuta(t2, app2, P, [0, np.pi, 0, 0]); runs[label] = (t2, app2, X, I)
        print('  %s: arm %.0f..%.0f deg, pendulum +-%.0f deg, |i| max %.2f A' % (label, np.degrees(X[:, 0].min()), np.degrees(X[:, 0].max()), np.degrees(np.abs(X[:, 1] - np.pi).max()), np.abs(I).max()))
        for fs, sig, fd in [(100, .12, 20), (250, .12, 40), (250, .03, 40)]:
            res = [stage2(record(t2, X[:, 0], I, app2, fs, np.random.default_rng(k), sig, th1=X[:, 1]), fs, fd, 10, P['Vs'], P['Kt'], P['tc'], L['mgl']) for k in range(20)]
            print('    new pipeline fs=%d sigma=%.2f: ' % (fs, sig) + spread(res, true2))
    print('GEOMETRY-FREE CHAIN: no m, Lp, Larm, eta_g entered; one known added mass sets the scale')
    sq = lambda A, f: (lambda t: A * np.where((f * t + .25) % 1 < .5, 1., -1.))      # phase pi/2: arm swings about its centre
    seg = [(.5, hold(0))] + [(4., sq(.3, f)) for f in (1.5, 2., 3., 4., 2.5)] + [(1., hold(0))]
    t3, app3 = command(seg); X3, I3 = sim_furuta(t3, app3, P, [0, np.pi, 0, 0])
    print('  centred square sweep 30 %%: arm %.0f..%.0f deg, pendulum +-%.0f deg' % (np.degrees(X3[:, 0].min()), np.degrees(X3[:, 0].max()), np.degrees(np.abs(X3[:, 1] - np.pi).max())))
    dm, d = .0085, .19                                                 # e.g. a coin taped near the tip
    Pm = dict(P); Lm_ = lumped(P); Gm = Lm_['mgl'] + dm * G * d; Jpm = Lm_['Jp'] + dm * d * d
    swing_loaded = lambda tt, phi0: np.pi + _swing(tt, Gm, Jpm, P['b1'], phi0)
    def _swing(tt, Gx, Jx, bx, phi0):
        x = np.array([phi0, 0.]); out = np.zeros(len(tt)); dt = tt[1] - tt[0]
        for k in range(len(tt)):
            out[k] = x[0]; x = rk4(lambda x: np.array([x[1], (-bx * x[1] - Gx * np.sin(x[0])) / Jx]), x, dt)
        return out
    plain = sim_swing(tt, P, .6); loaded = swing_loaded(tt, .6)
    truth = dict(Larm=P['Larm'], G=L['mgl'], Jp=L['Jp'], h=L['h'], b1=L['b1'], Kt=P['Kt'], eta=P['eta'], Ja=L['Ja'], Beq=L['Beq'], tc=P['tc'], mLp=P['m'] * P['Lp'])
    for sig, fixtc in [(.12, True), (.03, True), (.12, False)]:
        res = []
        for k in range(20):
            g = np.random.default_rng(k)
            f1 = freeswing_ratio(record(tt, zero, zero, zero, 250, g, sig, th1=plain), 250, 40, 10)
            f2 = freeswing_ratio(record(tt, zero, zero, zero, 250, g, sig, th1=loaded), 250, 40, 10)
            ref = added_mass(f1['rJ'], f2['rJ'], dm * (1 + .01 * g.standard_normal()), d + .001 * g.standard_normal())   # 1 % mass, 1 mm position error
            s1 = stage1(record(t, th, cur, app, 250, g, sig), 250, 40, 10, P['Vs'], 1.)
            r = stage2_ratios(record(t3, X3[:, 0], I3, app3, 250, g, sig, th1=X3[:, 1]), 250, 40, 10, P['Vs'], s1['tc'] / s1['Kt'] if fixtc else None)
            a = absolute(r, ref['G']); a.update(G=ref['G'], eta=a['Kt'] / s1['Kb'], mLp=ref['G'] / G); res.append(a)
        print('  sigma=%.2f A, tc %s:  ' % (sig, 'held at Stage 1' if fixtc else 'refitted') + spread(res, truth))
    res = []
    for k in range(20):
        g = np.random.default_rng(k)
        r = stage2_ratios(record(t3, X3[:, 0], I3, app3, 250, g, .12, th1=X3[:, 1]), 250, 40, 10, P['Vs'], P['tc'] / P['Kt'])
        a = absolute(r, L['mgl']); a.update(eta=a['Kt'] / P['Kb']); res.append(a)
    print('  exact G (m, Lp measured), sigma=0.12:  ' + spread(res, {k: truth[k] for k in ('Larm', 'Kt', 'eta', 'Ja', 'Jp', 'h', 'b1', 'Beq')}))
    print('APP ESTIMATORS AS THEY ARE TODAY (desktop/furuta_lab/analysis.py), same data, 250 Hz, sigma 0.12 A')
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'desktop'))
        from furuta_lab import analysis as A
        cfg = {'currentZeroMv': 2500, 'sensitivityMvA': 185, 'assumedBusV': 12, 'countsPerRev': CPR, 'pendDownRaw': 2048, 'armSign': 1, 'pendSign': 1}
        r = A.motor_parameters(core_rows(record(t, th, cur, app, 250, rng, .12)), cfg, CPR, P['Kt'])
        e = r['electrical_parameters']; m = r['mechanical_parameters']
        show('motor_parameters', dict(Rm=e['R_ohm'], Kb=e['Ke_V_s_per_rad'], J=m['J_kg_m2'], b=m['B_Nm_s_per_rad'], tc=m['Tc_Nm']), true1)
        t2, app2, X, I = runs[cases[0][0]]
        geo = {'mass': P['m'], 'arm_length': P['Larm'], 'com': P['Lp'], 'gear': 1., 'efficiency': 1., 'Kt': P['Kt']}
        p = A.assembly_parameters(core_rows(record(t2, X[:, 0], I, app2, 250, rng, .12, th1=X[:, 1])), cfg, geo)['parameters']
        show('assembly_parameters', dict(Ja=p['arm_inertia_kg_m2'] + P['m'] * P['Larm'] ** 2, Jp=p['pendulum_com_inertia_kg_m2'] + P['m'] * P['Lp'] ** 2,
                                         Beq=p['arm_viscous_Nm_s'], b1=p['pendulum_viscous_Nm_s']), true2)
        print('    (assembly_parameters also refits Coulomb friction: %.3g Nm, true %.3g)' % (p['arm_coulomb_Nm'], P['tc']))
    except Exception as ex:
        print('  comparison skipped:', ex)
