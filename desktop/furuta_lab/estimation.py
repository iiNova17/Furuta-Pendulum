"""Parameter estimation for the Furuta rig, done on the assembled rig only.

Model (report Eq. 93 with the arm equation divided by K_tau, G = m g Lp):

    i  = (V - Kb*w0)/Rm  while the applied duty is nonzero;  i = 0 at zero duty (bridge open)
    (Ja + Jp s^2) a0 + h c a1 + 2 Jp s c w0 w1 - h s w1^2 + Beq w0 + tc(+/-) sgn(w0) = i
    h c a0 + Jp a1 - Jp s c w0^2 + b1 w1 - G s = 0          s = sin(th1), c = cos(th1), th1 = 0 upright

Ten numbers describe the plant: Rm, Kb and eight "per K_tau" values (torque in amperes of
motor current). The linear models, poles and the simulation depend only on these; SI
values need one optional absolute reference (scale()).

Estimation chain: free swing -> electrical -> pendulum equation -> arm equation ->
simulation-error refinement of Ja, Beq, tc+, tc- -> validation on a separate recording.
Reference prototypes: docs/prototypes/estimation_prototype.py, twin_prototype.py.

Pure functions, no Qt.
"""
import json
import math
import warnings
from pathlib import Path
import numpy as np
from scipy import optimize, signal

from .analysis import spans

GRAVITY = 9.80665
PEND_COUNTS = 4096
PARAMS = ['Rm', 'Kb', 'Ja', 'Jp', 'h', 'G', 'Beq', 'b1', 'tcp', 'tcn', 'tp']
# key: (symbol, unit identified, SI unit after scaling, meaning)
INFO = {
    'Rm': ('Rm', 'Ω', 'Ω', 'armature resistance (incl. driver)'),
    'Kb': ('Kb', 'V·s/rad', 'V·s/rad', 'back-EMF constant, output shaft'),
    'Ja': ('Ja/Kτ', 'A·s²/rad', 'kg·m²', 'arm-side inertia (arm, reflected rotor, pendulum mass at the tip)'),
    'Jp': ('Jp/Kτ', 'A·s²/rad', 'kg·m²', 'pendulum inertia about its pivot'),
    'h': ('h/Kτ', 'A·s²/rad', 'kg·m²', 'inertial coupling m·Larm·Lp'),
    'G': ('G/Kτ', 'A', 'N·m', 'gravity torque coefficient m·g·Lp'),
    'Beq': ('Beq/Kτ', 'A·s/rad', 'N·m·s/rad', 'arm viscous friction'),
    'b1': ('b1/Kτ', 'A·s/rad', 'N·m·s/rad', 'pendulum viscous friction'),
    'tcp': ('τc+/Kτ', 'A', 'N·m', 'arm Coulomb friction, positive direction'),
    'tcn': ('τc−/Kτ', 'A', 'N·m', 'arm Coulomb friction, negative direction'),
    'tp': ('τp/Kτ', 'A', 'N·m', 'pendulum joint dry friction'),
}
SIM_EPS = .1          # rad/s, arm friction smoothing tanh(w/eps) in the simulator
SIM_EPS1 = .05        # rad/s, pendulum friction smoothing
DRY_DEADBAND = .05    # rad/s, sgn(w1) is 0 below this in the regressions
# Measured on the rig (2026-11-15, three held-out runs): refining the coupling h (least squares underestimates it,
# the arm acceleration is noisy) and the pendulum damping b1, tp (the driven pendulum is damped differently from
# the free swing) improved every validation run; refining the arm friction made the open-loop arm drift worse.
REFINE_FREE = ('h', 'b1', 'tp')


# ------------------------------------------------------------------ helpers
def lp(x, fs, fc):
    """Zero-phase 2nd-order Butterworth (forward-backward)."""
    sos = signal.butter(2, fc, fs=fs, output='sos')
    x = np.asarray(x, float)
    pad = min(3 * 6, x.shape[0] - 1)
    return signal.sosfiltfilt(sos, x, axis=0, padlen=pad if pad > 0 else 0)


def grow(mask, n):
    if n <= 0:
        return mask.copy()
    return np.convolve(mask.astype(float), np.ones(2 * n + 1), 'same') > 0


def resting(w, fs, wmin, seconds=.15):
    """True where |w| <= wmin for at least `seconds` (stuck arm: friction is not tc*sgn(w))."""
    slow = np.abs(w) <= wmin
    out = np.zeros(len(w), bool)
    d = np.diff(np.r_[0, slow.astype(int), 0])
    for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)):
        if (b - a) / fs >= seconds:
            out[a:b] = True
    return out


def finite(x):
    return x is not None and isinstance(x, (int, float, np.floating, np.integer)) and math.isfinite(x)


def filter_delay_s(config):
    """Low-frequency group delay of the MCU current filter, in seconds."""
    f = (config or {}).get('filter') or {}
    kind = f.get('type', 'bypass'); hz = float((config or {}).get('adcHz') or 2000)
    if kind == 'biquad':
        sos = list(f.get('sos') or [1, 0, 0, 1, 0, 0])
        _, gd = signal.group_delay((sos[:3], sos[3:]), w=[min(1., hz / 100)], fs=hz)
        return float(gd[0]) / hz
    if kind == 'ema':
        a = float(f.get('alpha', .1)); return (1 - a) / a / hz
    if kind == 'moving':
        return (int(f.get('window', 8)) - 1) / 2 / hz
    return 0.


def _increasing(t, x):
    keep = np.r_[True, np.diff(t) > 0]
    return t[keep], x[keep]


# ------------------------------------------------------------------ prepare
def prepare(rows, config, Vs=12., fd=None, polarity=1, need_current=True, name='', manifest=None, cpr_default=None, pend_sign=None,
            pend_delay=0.):
    """Convert recorded core rows to physical signals on a uniform grid, filtered and differentiated.

    Same zero-phase low-pass (cut-off fd) on every signal, then central differences.
    Each channel uses its own timestamp (encoder device_us, AS5600 pend_us, ADC adc_us).
    The MCU-filtered current is used when the run's MCU filter is active, with its delay removed.
    pend_delay (s): the pendulum angle lags the arm signals by this much; its timestamps are moved earlier.
    """
    config = config or {}
    if rows is None or len(rows) < 100:
        raise ValueError(f'{name or "Recording"}: too few samples')
    cpr = config.get('countsPerRev'); notes = []
    if not finite(cpr) or cpr <= 0:
        if not finite(cpr_default) or cpr_default <= 0:
            raise ValueError(f'{name}: the recording has no encoder scale (countsPerRev). Apply the encoder scale before recording.')
        cpr = float(cpr_default)
        notes.append(f'{name}: no encoder scale in the recording; using {cpr:g} counts/rev from Setup (gear × cycles × quadrature).')
    down = config.get('pendDownRaw')
    if not finite(down) or down < 0:
        raise ValueError(f'{name}: the recording has no pendulum-down reference. Set it before recording.')
    zero = config.get('currentZeroMv'); sens = config.get('sensitivityMvA')
    if need_current and (not finite(zero) or not finite(sens) or sens <= 0):
        raise ValueError(f'{name}: the recording has no current zero. Zero the current before recording.')
    bits = 7 if need_current else 3
    t_all = (rows['device_us'].astype(np.int64) - int(rows['device_us'][0])) * 1e-6
    good = (rows['validity'] & bits) == bits
    parts, _ = spans(t_all, np.where(good, 0., np.nan))
    ix = max(parts, key=len)
    if len(ix) < 100:
        raise ValueError(f'{name}: need at least 100 contiguous valid samples')
    a = rows[ix]; t0 = int(a['device_us'][0])
    tc = (a['device_us'].astype(np.int64) - t0) * 1e-6
    dt = float(np.median(np.diff(tc))); fs = 1 / dt
    n = int(math.floor(tc[-1] / dt)) + 1; grid = np.arange(n) * dt
    arm_sign = config.get('armSign', 1) or 1
    pend_sign = pend_sign if pend_sign in (1, -1) else (config.get('pendSign', 1) or 1)
    th0_s = arm_sign * (a['count'].astype(np.int64) - int(a['count'][0])).astype(float) * 2 * np.pi / cpr
    th0 = np.interp(grid, *_increasing(tc, th0_s))
    tp = (a['pend_us'].astype(np.int64) - t0) * 1e-6
    if not np.any(tp > 0):
        tp = tc
    tp = tp - float(pend_delay or 0.)
    th1_s = np.unwrap(pend_sign * (a['pend_raw'].astype(float) - down) * 2 * np.pi / PEND_COUNTS + np.pi)
    th1 = np.interp(grid, *_increasing(tp, th1_s))
    kind = ((config.get('filter') or {}).get('type') or 'bypass')
    use_filtered = need_current and kind != 'bypass'
    delay = filter_delay_s(config) if use_filtered else 0.
    if need_current:
        ta = (a['adc_us'].astype(np.int64) - t0) * 1e-6 - delay
        if not np.any(ta > -1):
            ta = tc - delay
        mv = (a['filtered_mv'] if use_filtered else a['current_mv']).astype(float)
        cur = np.interp(grid, *_increasing(ta, polarity * (mv - zero) / sens))
    else:
        cur = np.zeros(n)
    k = np.clip(np.searchsorted(tc, grid + 1e-9, 'right') - 1, 0, len(a) - 1)
    app = a['applied'].astype(float)[k]
    if fd is None:
        fd = 40. if fs >= 200 else 20.
    fd = float(min(fd, .4 * fs))
    s = {'name': name, 't': grid, 'fs': fs, 'fd': fd, 'Vs': float(Vs), 'polarity': polarity,
         'raw': {'th0': th0, 'th1': th1, 'i': cur, 'app': app}, 'app': app,
         'counts_per_rad': cpr / (2 * np.pi), 'current_source': ('MCU filter (' + kind + f', delay {delay * 1e3:.1f} ms removed)') if use_filtered else 'raw ADC',
         'delay_ms': delay * 1e3, 'samples': len(rows), 'span': (int(ix[0]), int(ix[-1])),
         'config': config, 'notes': notes, 'counts_per_rev': cpr, 'pend_sign': pend_sign, 'pend_delay': float(pend_delay or 0.)}
    s['th0'] = lp(th0, fs, fd); s['th1'] = lp(th1, fs, fd); s['i'] = lp(cur, fs, fd) if need_current else cur
    s['V'] = lp(app * Vs, fs, fd)
    s['w0'] = np.gradient(s['th0'], dt); s['a0'] = np.gradient(s['w0'], dt)
    s['w1'] = np.gradient(s['th1'], dt); s['a1'] = np.gradient(s['w1'], dt)
    seq = a['seq'].astype(np.int64); d = np.diff(seq)
    stride = float(np.median(d)) if len(d) else 1.
    s['missing'] = int(np.sum(np.maximum(d / max(stride, 1) - 1, 0))) if stride > 0 else 0
    s['saturated'] = bool(np.any(a['limits'] & 1))
    fin = (manifest or {}).get('final_status') or {}
    s['fault'] = fin.get('fault') or ''
    s['final_state'] = fin.get('state', '')
    return s


def masks(p, fp=10., wmin=.3, standstill_s=.15):
    n = len(p['t']); fs = p['fs']; e = int(3 * fs / fp)
    edge = np.zeros(n, bool)
    if n > 2 * e:
        edge[e:n - e] = True
    driven = edge & ~grow(p['app'] == 0, int(fs / fp))
    rest = resting(p['w0'], fs, wmin, standstill_s)
    moving = edge & ~grow(rest, int(fs / fp / 2)) & (np.abs(p['w0']) > wmin)
    # Free swing: drop samples where the pendulum is displaced but still (held by hand before release
    # or caught afterwards), then keep the longest remaining stretch.
    held = resting(p['w1'], fs, .3, .15) & (np.abs(p['th1'] - np.pi) > np.radians(5))
    free = edge & ~grow(held, int(.25 * fs))
    d = np.diff(np.r_[0, free.astype(int), 0]); starts = np.flatnonzero(d == 1); ends = np.flatnonzero(d == -1)
    swing = np.zeros(n, bool)
    if len(starts):
        k = int(np.argmax(ends - starts)); swing[starts[k]:ends[k]] = True
    return {'edge': edge, 'driven': driven, 'moving': moving, 'resting': rest, 'swing': swing}


# ------------------------------------------------------------------ least squares
def _ols(X, y):
    X = np.asarray(X, float); y = np.asarray(y, float)
    n, k = X.shape
    if n <= k + 2:
        raise ValueError(f'Only {n} usable samples for {k} unknowns: record more motion')
    scale = np.linalg.norm(X, axis=0)
    if np.any(scale < 1e-12):
        raise ValueError('A regressor is zero in every usable sample: the recording does not excite this term')
    Xn = X / scale
    bn, *_ = np.linalg.lstsq(Xn, y, rcond=None)
    beta = bn / scale; r = y - X @ beta
    s2 = float(r @ r) / max(1, n - k)
    covn = np.linalg.pinv(Xn.T @ Xn) * s2
    cov = covn / np.outer(scale, scale)
    ss = float(np.sum((y - y.mean()) ** 2))
    return beta, cov, (1 - float(r @ r) / ss if ss > 0 else float('nan')), float(np.linalg.cond(Xn))


def _fit(preps, build, mask_name, fp, wmin, names):
    """Filter the left side and every regressor with the same low-pass fp, decimate to about 4 fp,
    keep the masked rows of every run, solve. build(p) -> (X, y, contribution labels)."""
    Xs, ys, cache = [], [], []
    for p in preps:
        fs = p['fs']; f = min(fp, .4 * fs)
        X, y = build(p)
        Xf = lp(X, fs, f); yf = lp(y, fs, f)
        m = masks(p, f, wmin)[mask_name]
        step = max(1, int(fs / (4 * f)))
        idx = np.flatnonzero(m)[::step]
        Xs.append(Xf[idx]); ys.append(yf[idx]); cache.append((p, Xf, yf, m))
    X = np.vstack(Xs); y = np.concatenate(ys)
    if not len(y):
        raise ValueError('No usable samples in these recordings for this fit')
    beta, cov, r2, cond = _ols(X, y)
    runs = []
    for p, Xf, yf, m in cache:
        runs.append({'name': p['name'], 't': p['t'], 'measured': yf, 'predicted': Xf @ beta,
                     'parts': {nm: Xf[:, k] * beta[k] for k, nm in enumerate(names)}, 'mask': m})
    se = np.sqrt(np.maximum(np.diag(cov), 0))
    return {'coef': beta, 'cov': cov, 'se': se, 'r2': r2, 'cond': cond, 'rows': int(len(y)), 'runs': runs, 'names': names}


def dry(w):
    return np.sign(w) * (np.abs(w) > DRY_DEADBAND)


def fit_freeswing(preps, fp=10., coulomb=True):
    """Arm held, motor off: sin(phi) = -(Jp/G) phi'' - (b1/G) phi' - (tp/G) sgn(phi'),  phi = th1 - pi.
    The joint has viscous and dry friction; the dry part shows as a constant amplitude loss per swing."""
    def build(p):
        cols = [-p['a1'], -p['w1']] + ([-dry(p['w1'])] if coulomb else [])
        return np.column_stack(cols), np.sin(p['th1'] - np.pi)
    r = _fit(preps, build, 'swing', fp, .3, ['inertia', 'damping'] + (['dry friction'] if coulomb else []))
    rJ, rb = r['coef'][:2]; sJ, sb = r['se'][:2]
    rc, sc = (float(r['coef'][2]), float(r['se'][2])) if coulomb else (0., 0.)
    if rJ <= 0:
        raise ValueError('Free swing fit gave a negative inertia ratio: is the pendulum-down reference right?')
    wn = 1 / math.sqrt(rJ); zeta = rb / (2 * math.sqrt(rJ))
    r.update(rJ=float(rJ), rb=float(rb), rJ_se=float(sJ), rb_se=float(sb), wn=wn, wn_se=.5 * wn * sJ / rJ,
             zeta=zeta, zeta_se=abs(zeta) * math.hypot(sb / max(abs(rb), 1e-30), .5 * sJ / rJ),
             length=GRAVITY * rJ, freq_hz=wn / (2 * math.pi), rc=rc, rc_se=sc)
    # Cross-check: damped period from zero crossings; amplitude; arm stillness.
    periods, amps, arm = [], [], []
    for p in preps:
        m = masks(p, fp)['swing']; t = p['t'][m]; phi = (p['th1'] - np.pi)[m]
        z = np.flatnonzero(np.sign(phi[:-1]) * np.sign(phi[1:]) < 0)
        tz = t[z] - phi[z] * (t[z + 1] - t[z]) / (phi[z + 1] - phi[z])
        if len(tz) >= 3:
            periods.append(2 * float(np.mean(np.diff(tz))))
        amps.append(float(np.max(np.abs(phi))) if len(phi) else 0.)
        arm.append(float(np.ptp(p['raw']['th0'][m])) * p['counts_per_rad'] if m.any() else 0.)
        run = next(x for x in r['runs'] if x['name'] == p['name'])
        run['phi'] = p['th1'] - np.pi
        run['phi_sim'] = simulate_swing(rJ, rb, p, fp, rc)
    r.update(period_s=float(np.mean(periods)) if periods else None, amplitude_deg=float(np.degrees(max(amps))) if amps else 0.,
             arm_counts=float(max(arm)) if arm else 0.)
    return r


def simulate_swing(rJ, rb, p, fp=10., rc=0.):
    m = masks(p, fp)['swing']; idx = np.flatnonzero(m)
    out = np.full(len(p['t']), np.nan)
    if not len(idx):
        return out
    k0 = idx[0]; dt = 1 / p['fs'] / 2; x = p['th1'][k0] - np.pi; w = p['w1'][k0]

    def f(x, w):
        return w, -(rb * w + rc * math.tanh(w / SIM_EPS1) + math.sin(x)) / rJ
    for k in range(k0, len(out)):
        out[k] = x
        for _ in range(2):
            a = f(x, w); b = f(x + .5 * dt * a[0], w + .5 * dt * a[1])
            c = f(x + .5 * dt * b[0], w + .5 * dt * b[1]); e = f(x + dt * c[0], w + dt * c[1])
            x += dt / 6 * (a[0] + 2 * b[0] + 2 * c[0] + e[0]); w += dt / 6 * (a[1] + 2 * b[1] + 2 * c[1] + e[1])
    return out


def fit_electrical(preps, fp=10.):
    """Driven samples only: i = V/Rm - (Kb/Rm) w0 (current is the noisy signal, so it is on the left)."""
    def build(p):
        return np.column_stack([p['V'], p['w0']]), p['i']
    r = _fit(preps, build, 'driven', fp, .3, ['V/Rm', '−Kb·ω0/Rm'])
    a, b = r['coef']; C = r['cov']
    if a == 0:
        raise ValueError('Current does not follow the voltage at all: check the current sensor')
    Rm = 1 / a; Kb = -b / a
    gR = np.array([-1 / a ** 2, 0]); gK = np.array([b / a ** 2, -1 / a])
    r.update(Rm=float(Rm), Kb=float(Kb), Rm_se=float(math.sqrt(max(gR @ C @ gR, 0))), Kb_se=float(math.sqrt(max(gK @ C @ gK, 0))))
    # Report's Eq. 92 form, V on the left, for comparison only.
    Xs, ys = [], []
    for p in preps:
        fs = p['fs']; f = min(fp, .4 * fs); m = masks(p, f)['driven']; idx = np.flatnonzero(m)[::max(1, int(fs / (4 * f)))]
        Xs.append(lp(np.column_stack([p['i'], p['w0']]), fs, f)[idx]); ys.append(lp(p['V'], fs, f)[idx])
    try:
        c2, _, _, _ = _ols(np.vstack(Xs), np.concatenate(ys)); r['eq92'] = {'Rm': float(c2[0]), 'Kb': float(c2[1])}
    except ValueError:
        r['eq92'] = None
    return r


def fit_pendulum(preps, fp=10., fixed=None):
    """sin th1 = (h/G) c a0 + (Jp/G)(a1 - s c w0^2) + (b1/G) w1 + (tp/G) sgn(w1).
    fixed: free-swing values {rJ, rb, rc (+ _se)}; then only the coupling h/G is fitted from the driven runs
    (recommended: damping and dry friction are not separable from a small timing lag in driven data).
    Without fixed, h/G, Jp/G and b1/G are fitted and the dry friction is taken as 0."""
    def terms(p):
        s = np.sin(p['th1']); c = np.cos(p['th1'])
        return s, c * p['a0'], p['a1'] - s * c * p['w0'] ** 2, p['w1'], dry(p['w1'])

    if fixed:
        rJ, rb, rc = fixed['rJ'], fixed['rb'], fixed.get('rc', 0.)

        def build(p):
            s, cpl, inert, damp, dr = terms(p)
            return cpl[:, None], s - rJ * inert - rb * damp - rc * dr
        r = _fit(preps, build, 'edge', fp, .3, ['coupling'])
        rh = float(r['coef'][0]); rh_se = float(r['se'][0])
        num = den = 0.
        for p, run in zip(preps, r['runs']):
            fs = p['fs']; f = min(fp, .4 * fs); s, cpl, inert, damp, dr = terms(p)
            parts = {'inertia': lp(rJ * inert, fs, f), 'damping': lp(rb * damp, fs, f), 'dry friction': lp(rc * dr, fs, f)}
            fixed_sum = parts['inertia'] + parts['damping'] + parts['dry friction']
            run['measured'] = run['measured'] + fixed_sum; run['predicted'] = run['predicted'] + fixed_sum
            run['parts'].update(parts)
            m = run['mask']; e = run['measured'][m] - run['predicted'][m]
            num += float(e @ e); den += float(np.sum((run['measured'][m] - run['measured'][m].mean()) ** 2))
        r['r2'] = 1 - num / den if den > 0 else float('nan')
        r.update(rh=rh, rJ=float(rJ), rb=float(rb), rc=float(rc), rh_se=rh_se, rJ_se=float(fixed.get('rJ_se', 0.)),
                 rb_se=float(fixed.get('rb_se', 0.)), rc_se=float(fixed.get('rc_se', 0.)), source='free swing')
    else:
        def build(p):
            s, cpl, inert, damp, dr = terms(p)
            return np.column_stack([cpl, inert, damp]), s
        r = _fit(preps, build, 'edge', fp, .3, ['coupling', 'inertia', 'damping'])
        rh, rJ, rb = map(float, r['coef'])
        r.update(rh=rh, rJ=rJ, rb=rb, rc=0., rh_se=float(r['se'][0]), rJ_se=float(r['se'][1]), rb_se=float(r['se'][2]),
                 rc_se=0., source='driven runs')
    r.update(Larm=GRAVITY * r['rh'], Larm_se=GRAVITY * r['rh_se'])
    return r


def estimate_pend_delay(make_preps, fixed=None, delays_ms=range(0, 16), fp=10.):
    """Pendulum timing lag that best fits the pendulum equation. make_preps(delay_s) -> prepared runs.
    Returns (best_ms, [(ms, R2), ...]); the best value is refined by a parabola through the top three."""
    rows = []
    for ms in delays_ms:
        r = fit_pendulum(make_preps(ms / 1000), fp, fixed)
        rows.append((float(ms), float(r['r2'])))
    k = int(np.argmax([r2 for _, r2 in rows])); best = rows[k][0]
    if 0 < k < len(rows) - 1:
        (x0, y0), (x1, y1), (x2, y2) = rows[k - 1:k + 2]
        den = (y0 - 2 * y1 + y2)
        if den < 0:
            best = x1 + .5 * (x1 - x0) * (y0 - y2) / den
    return float(best), rows


def fit_arm(preps, pend, fp=10., wmin=.3, symmetric=False):
    """i = Ja a0 + G Tr + Beq w0 + tc+ [w0>0] - tc- [w0<0],
    Tr = (Jp/G)(s^2 a0 + 2 s c w0 w1) + (h/G)(c a1 - s w1^2) with Jp/G, h/G from the pendulum fit."""
    rh, rJ = pend['rh'], pend['rJ']

    def build(p):
        s = np.sin(p['th1']); c = np.cos(p['th1']); w0 = p['w0']
        Tr = rJ * (s * s * p['a0'] + 2 * s * c * w0 * p['w1']) + rh * (c * p['a1'] - s * p['w1'] ** 2)
        pos = (w0 > wmin).astype(float); neg = -(w0 < -wmin).astype(float)
        cols = [p['a0'], Tr, w0] + ([pos + neg] if symmetric else [pos, neg])
        return np.column_stack(cols), p['i']
    names = ['inertia', 'pendulum reaction', 'viscous'] + (['Coulomb'] if symmetric else ['Coulomb +', 'Coulomb −'])
    r = _fit(preps, build, 'moving', fp, wmin, names)
    c = list(r['coef']); s = list(r['se'])
    if symmetric:
        c.append(c[3]); s.append(s[3])
    r.update({k: float(v) for k, v in zip(['Ja', 'G', 'Beq', 'tcp', 'tcn'], c)})
    r.update({k + '_se': float(v) for k, v in zip(['Ja', 'G', 'Beq', 'tcp', 'tcn'], s)})
    r['symmetric'] = symmetric
    return r


def assemble(elec, pend, arm):
    """The eleven parameters and approximate standard errors."""
    p = {'Rm': elec['Rm'], 'Kb': elec['Kb'], 'Ja': arm['Ja'], 'G': arm['G'], 'Beq': arm['Beq'],
         'tcp': arm['tcp'], 'tcn': arm['tcn']}
    p['Jp'] = arm['G'] * pend['rJ']; p['h'] = arm['G'] * pend['rh']; p['b1'] = arm['G'] * pend['rb']
    p['tp'] = arm['G'] * pend.get('rc', 0.)
    rel = lambda v, s: abs(s / v) if v else float('inf')
    rg = rel(arm['G'], arm['G_se'])
    se = {'Rm': elec['Rm_se'], 'Kb': elec['Kb_se'], 'Ja': arm['Ja_se'], 'G': arm['G_se'], 'Beq': arm['Beq_se'],
          'tcp': arm['tcp_se'], 'tcn': arm['tcn_se'],
          'Jp': abs(p['Jp']) * math.hypot(rg, rel(pend['rJ'], pend['rJ_se'])),
          'h': abs(p['h']) * math.hypot(rg, rel(pend['rh'], pend['rh_se'])),
          'b1': abs(p['b1']) * math.hypot(rg, rel(pend['rb'], pend['rb_se'])),
          'tp': abs(p['tp']) * math.hypot(rg, rel(pend.get('rc', 0.), pend.get('rc_se', 0.))) if p['tp'] else 0.}
    return {k: float(p[k]) for k in PARAMS}, {k: float(se[k]) for k in PARAMS}


# ------------------------------------------------------------------ simulation
def full(params):
    """Parameter dict with every key of PARAMS (older sessions have no pendulum dry friction: 0)."""
    return {k: float(params.get(k, 0.)) for k in PARAMS}


def _vec(params):
    return np.array([full(params)[k] for k in PARAMS])


def _rhs_batch(x, d, th, Vs):
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1, tcp, tcn, tp = [th[..., k] for k in range(11)]
    t1, w0, w1 = x[..., 1], x[..., 2], x[..., 3]; s = np.sin(t1); c = np.cos(t1)
    cur = np.where(d != 0, (d * Vs - Kb * w0) / Rm, 0.)
    fr = np.where(w0 > 0, tcp, tcn) * np.tanh(w0 / SIM_EPS)
    f0 = cur - Beq * w0 - fr - 2 * Jp * s * c * w0 * w1 + h * s * w1 ** 2
    f1 = Jp * s * c * w0 ** 2 - b1 * w1 - tp * np.tanh(w1 / SIM_EPS1) + Gk * s
    m00 = Ja + Jp * s * s; det = m00 * Jp - (h * c) ** 2
    return np.stack([w0, w1, (Jp * f0 - h * c * f1) / det, (-h * c * f0 + m00 * f1) / det], -1), cur


def sim_batch(th, U, X0, dt, Vs):
    """th (P,11) parameter sets; U (W,n) duty at step dt; X0 (W,4). All P x W cases at once (RK4).
    Returns states (P,W,n,4) and current (P,W,n)."""
    P_ = th.shape[0]; W, n = U.shape; x = np.broadcast_to(X0, (P_, W, 4)).copy()
    X = np.empty((P_, W, n, 4)); I = np.empty((P_, W, n)); th = th[:, None, :]
    for k in range(n):
        d = np.broadcast_to(U[:, k], (P_, W))
        k1, cur = _rhs_batch(x, d, th, Vs); k2, _ = _rhs_batch(x + .5 * dt * k1, d, th, Vs)
        k3, _ = _rhs_batch(x + .5 * dt * k2, d, th, Vs); k4, _ = _rhs_batch(x + dt * k3, d, th, Vs)
        X[:, :, k] = x; I[:, :, k] = cur; x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return X, I


def rhs(x, d, params, Vs):
    """Nonlinear model x' = f(x, duty) for one state; returns (x', current). Same physics as sim_batch."""
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1, tcp, tcn, tp = [full(params)[k] for k in PARAMS]
    t1, w0, w1 = x[1], x[2], x[3]; s = math.sin(t1); c = math.cos(t1)
    cur = (d * Vs - Kb * w0) / Rm if d != 0 else 0.
    fr = (tcp if w0 > 0 else tcn) * math.tanh(w0 / SIM_EPS)
    f0 = cur - Beq * w0 - fr - 2 * Jp * s * c * w0 * w1 + h * s * w1 * w1
    f1 = Jp * s * c * w0 * w0 - b1 * w1 - tp * math.tanh(w1 / SIM_EPS1) + Gk * s
    m00 = Ja + Jp * s * s; m01 = h * c; det = m00 * Jp - m01 * m01
    return (w0, w1, (Jp * f0 - m01 * f1) / det, (-m01 * f0 + m00 * f1) / det), cur


def _sim_one(params, app, x0, dt, Vs, sub=2):
    n = len(app); X = np.empty((n, 4)); I = np.empty(n); x = list(x0); h = dt / sub
    for k in range(n):
        d = float(app[k]); X[k] = x
        for j in range(sub):
            a, cur = rhs(x, d, params, Vs)
            if j == 0:
                I[k] = cur
            b, _ = rhs([x[q] + .5 * h * a[q] for q in range(4)], d, params, Vs)
            c, _ = rhs([x[q] + .5 * h * b[q] for q in range(4)], d, params, Vs)
            e, _ = rhs([x[q] + h * c[q] for q in range(4)], d, params, Vs)
            x = [x[q] + h / 6 * (a[q] + 2 * b[q] + 2 * c[q] + e[q]) for q in range(4)]
        if not all(math.isfinite(v) for v in x):
            X[k + 1:] = np.nan; I[k + 1:] = np.nan; break
    return X, I


def fit_percent(y, yhat):
    y = np.asarray(y, float); yhat = np.asarray(yhat, float); m = np.isfinite(y) & np.isfinite(yhat)
    if m.sum() < 3:
        return float('nan')
    d = np.linalg.norm(y[m] - y[m].mean())
    return float(100 * (1 - np.linalg.norm(yhat[m] - y[m]) / d)) if d > 0 else float('nan')


def simulate(params, p, resync_s=0., cancel=None):
    """Model driven by the recorded duty. resync_s = 0 simulates the whole run from its first state;
    otherwise the model restarts from the measured (filtered) state every resync_s seconds.
    Measured signals are the prepared (zero-phase filtered at fd) ones."""
    fs = p['fs']; n = len(p['t']); s0 = min(int(.5 * fs), n // 10)
    L = int(resync_s * fs) if resync_s and resync_s > 0 else n - s0
    out = {k: np.full(n, np.nan) for k in ('th0', 'th1', 'w0', 'w1', 'i')}
    for a in range(s0, n, max(L, 2)):
        if cancel is not None and cancel.is_set():
            raise RuntimeError('Simulation canceled')
        b = min(n, a + L)
        X, I = _sim_one(params, p['app'][a:b], [p['th0'][a], p['th1'][a], p['w0'][a], p['w1'][a]], 1 / fs, p['Vs'])
        for k, name in enumerate(('th0', 'th1', 'w0', 'w1')):
            out[name][a:b] = X[:, k]
        out['i'][a:b] = I
    meas = {k: p[k] for k in ('th0', 'th1', 'w0', 'w1', 'i')}
    fit = {'arm angle': fit_percent(meas['th0'], out['th0']), 'pendulum angle': fit_percent(meas['th1'], out['th1']),
           'arm speed': fit_percent(meas['w0'], out['w0']), 'current': fit_percent(meas['i'], out['i'])}
    m = np.isfinite(out['th0'])
    rms = {'arm angle (deg)': float(np.degrees(np.sqrt(np.mean((out['th0'][m] - meas['th0'][m]) ** 2)))),
           'pendulum angle (deg)': float(np.degrees(np.sqrt(np.mean((out['th1'][m] - meas['th1'][m]) ** 2)))),
           'current (A)': float(np.sqrt(np.mean((out['i'][m] - meas['i'][m]) ** 2)))}
    return {'name': p['name'], 't': p['t'], 'sim': out, 'meas': meas, 'app': p['app'], 'fit': fit, 'rms': rms,
            'start': s0, 'resync_s': resync_s}


# ------------------------------------------------------------------ refinement
def _windows(preps, seconds, sub=2):
    fs = preps[0]['fs']
    if any(abs(p['fs'] - fs) > 1e-6 * fs for p in preps):
        raise ValueError('Refinement needs recordings at the same stream rate')
    n = int(seconds * fs); U, X0, Y = [], [], []
    for p in preps:
        e = int(fs / 2)
        for a in range(e, len(p['t']) - n - e + 1, n):
            U.append(np.repeat(p['app'][a:a + n], sub)); X0.append([p['th0'][a], p['th1'][a], p['w0'][a], p['w1'][a]])
            Y.append(np.column_stack([p['th0'][a:a + n], p['th1'][a:a + n], p['i'][a:a + n]]))
    if not U:
        raise ValueError(f'Recordings are shorter than one {seconds:g} s window')
    return np.array(U), np.array(X0), np.array(Y), fs


def refine(params, preps, window_s=4., free=REFINE_FREE, progress=None, cancel=None, max_nfev=40, weights=(1., 1., 1.)):
    """Adjust the `free` parameters to minimise simulated-minus-measured arm angle, pendulum angle and current
    over windows that start from the measured state. Each signal is divided by its SD."""
    U, X0, Y, fs = _windows(preps, window_s)
    Vs = preps[0]['Vs']; sub = 2; dt = 1 / (fs * sub)
    th0 = _vec(params)
    floors = {'Beq': 1e-5, 'tcp': 1e-4, 'tcn': 1e-4, 'b1': 1e-7, 'tp': 0.}
    for k, nm in enumerate(PARAMS):
        if th0[k] <= 0:
            if nm in floors:
                th0[k] = floors[nm]
            else:
                raise ValueError(f'{INFO[nm][0]} is not positive ({th0[k]:.3g}); fix the earlier fit first')
    groups = [[PARAMS.index(f)] for f in free]
    sd = np.array([np.std(Y[..., 0] - Y[:, :1, 0]), np.std(Y[..., 1] - np.pi), np.std(Y[..., 2])])
    sd = np.where(sd > 0, sd, 1.) / np.asarray(weights, float)
    calls = [0]

    def expand(Z):
        th = np.repeat(th0[None], len(Z), 0)
        for j, idx in enumerate(groups):
            th[:, idx] *= np.exp(Z[:, j:j + 1])
        return th

    def residual(th):
        if cancel is not None and cancel.is_set():
            raise RuntimeError('Refinement canceled')
        X, I = sim_batch(th, U, X0, dt, Vs)
        sim = np.stack([X[:, :, ::sub, 0], X[:, :, ::sub, 1], I[:, :, ::sub]], -1)
        r = ((sim - Y) / sd).reshape(th.shape[0], -1)
        return np.nan_to_num(r, nan=1e3, posinf=1e3, neginf=-1e3)

    def fun(z):
        r = residual(expand(z[None]))[0]; calls[0] += 1
        if progress:
            progress(f'Refining by simulation: evaluation {calls[0]}, cost {.5 * float(r @ r):.4g}')
        return r

    def jac(z):
        hh = 1e-3; Z = z[None] + np.vstack([np.zeros(len(z)), hh * np.eye(len(z))])
        r = residual(expand(Z))
        return ((r[1:] - r[0]) / hh).T
    cost0 = .5 * float(np.sum(residual(th0[None])[0] ** 2))
    sol = optimize.least_squares(fun, np.zeros(len(groups)), jac=jac, bounds=(-2, 2), max_nfev=max_nfev)
    th1 = expand(sol.x[None])[0]
    new = dict(zip(PARAMS, map(float, th1))); params = full(params)
    change = {nm: 100 * (new[nm] / float(params[nm]) - 1) if float(params[nm]) else float('nan') for nm in free}
    return {'params': new, 'cost_before': cost0, 'cost_after': float(sol.cost), 'change_pct': change, 'nfev': int(sol.nfev),
            'windows': int(len(U)), 'window_s': window_s, 'free': list(free), 'at_bound': bool(np.any(np.abs(sol.x) > 1.99))}


# ------------------------------------------------------------------ linear models
def _upright(p, Vs):
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1 = (p[k] for k in PARAMS[:8])
    Bp = Beq + Kb / Rm; D = Ja * Jp - h * h
    A = np.array([[0, 0, D, 0], [0, 0, 0, D], [0, -h * Gk, -Jp * Bp, h * b1], [0, Ja * Gk, h * Bp, -Ja * b1]]) / D
    B = np.array([[0], [0], [Jp], [-h]]) / (Rm * D)
    return A, B


def _hanging(p, Vs):
    Rm, Kb, Ja, Jp, h, Gk, Beq, b1 = (p[k] for k in PARAMS[:8])
    Bp = Beq + Kb / Rm; D = Ja * Jp - h * h
    A = np.array([[0, 0, D, 0], [0, 0, 0, D], [0, -h * Gk, -Jp * Bp, -h * b1], [0, -Ja * Gk, -h * Bp, -Ja * b1]]) / D
    B = np.array([[0], [0], [Jp], [h]]) / (Rm * D)
    return A, B


def _trim(c):
    c = np.real(np.asarray(c, float)); big = np.max(np.abs(c)) if len(c) else 0.
    k = 0
    while k < len(c) - 1 and abs(c[k]) <= 1e-9 * big:
        k += 1
    return c[k:]


def _minimal_tf(num, den):
    """Zeros, poles, gain from the polynomials, with pole-zero pairs at the same place cancelled."""
    num = _trim(num); den = _trim(den)
    z = list(np.roots(num)) if len(num) > 1 else []; pl = list(np.roots(den))
    k = float(num[0] / den[0])
    for zz in list(z):
        for j, pp in enumerate(pl):
            if abs(zz - pp) <= 1e-6 * max(1., abs(pp)):
                z.remove(zz); pl.pop(j); break
    z = np.array(z); pl = np.array(pl)
    num = np.real(k * np.poly(z)) if len(z) else np.array([k])
    den = np.real(np.poly(pl))
    return {'num': num, 'den': den, 'zeros': z, 'poles': pl, 'gain': k}


def _system(A, B, C, D, Vs, Ts):
    poles = np.linalg.eigvals(A)
    ctrb = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(A.shape[0])])
    out = {'A': A, 'B': B, 'Bd': B * Vs, 'C': C, 'D': D, 'Dd': D * Vs, 'poles': poles,
           'ctrb_rank': int(np.linalg.matrix_rank(ctrb)), 'ctrb_cond': float(np.linalg.cond(ctrb)), 'tf': {}}
    for row, name in ((0, 'theta0/V'), (1, 'theta1/V')):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', signal.BadCoefficients)
            num, den = signal.ss2tf(A, B, C[row:row + 1], D[row:row + 1])
        out['tf'][name] = {'full_num': _trim(num[0]), 'full_den': _trim(den), **_minimal_tf(num[0], den)}
    Ad, Bdv, _, _, _ = signal.cont2discrete((A, B, C, D), Ts, 'zoh')
    out.update(Ts=Ts, Ad=Ad, Bd_v=Bdv, Bd_duty=Bdv * Vs, poles_d=np.linalg.eigvals(Ad))
    return out


def linear_models(params, Vs=12., Ts=1e-3, Lm=None):
    """Upright (report Eq. 99) and hanging (Eq. 98) models; input armature voltage V, or duty (B*Vs).
    Outputs: arm angle, pendulum angle, current (i = (V - Kb w0)/Rm). Hanging state uses phi = th1 - pi.
    Lm (H): also the five-state model of Eq. 86 with the current as a state."""
    p = full(params)
    C = np.array([[1., 0, 0, 0], [0, 1, 0, 0], [0, 0, -p['Kb'] / p['Rm'], 0]])
    D = np.array([[0.], [0], [1 / p['Rm']]])
    up = _system(*_upright(p, Vs), C, D, Vs, Ts)
    hang = _system(*_hanging(p, Vs), C, D, Vs, Ts)
    res = {'upright': up, 'hanging': hang, 'Vs': Vs, 'Ts': Ts,
           'friction_V': {'+': p['Rm'] * p['tcp'], '−': p['Rm'] * p['tcn']},
           'friction_duty': {'+': p['Rm'] * p['tcp'] / Vs, '−': p['Rm'] * p['tcn'] / Vs},
           'Delta': p['Ja'] * p['Jp'] - p['h'] ** 2, 'Beq_prime': p['Beq'] + p['Kb'] / p['Rm']}
    if Lm and Lm > 0:
        Rm, Kb, Ja, Jp, h, Gk, Beq, b1 = (p[k] for k in PARAMS[:8]); Dl = Ja * Jp - h * h
        A5 = np.array([[0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, -h * Gk / Dl, -Jp * Beq / Dl, h * b1 / Dl, Jp / Dl],
                       [0, Ja * Gk / Dl, h * Beq / Dl, -Ja * b1 / Dl, -h / Dl], [0, 0, -Kb / Lm, 0, -Rm / Lm]])
        B5 = np.array([[0], [0], [0], [0], [1 / Lm]])
        res['five_state'] = {'A': A5, 'B': B5, 'Bd': B5 * Vs, 'poles': np.linalg.eigvals(A5), 'Lm': Lm}
    return res


# ------------------------------------------------------------------ scale (optional)
def added_mass(rJ_plain, rJ_loaded, dm, d):
    """Absolute G = m g Lp and Jp from two free swings, the second with a point mass dm (kg) at d (m):
    G/Jp = 1/rJ_plain,  (G + dm g d)/(Jp + dm d^2) = 1/rJ_loaded."""
    A1 = 1 / rJ_plain; A2 = 1 / rJ_loaded
    if abs(A2 - A1) < 1e-12:
        raise ValueError('The two swings have the same frequency: move the mass away from the centre of oscillation')
    Jp = dm * (GRAVITY * d - A2 * d * d) / (A2 - A1)
    return {'G': A1 * Jp, 'Jp': Jp, 'sensitivity': abs(A2 - A1) / A1}


def scale(params, se=None, mode='nominal', **kw):
    """One absolute reference turns the per-K_tau values into SI units.
    nominal: K_tau = Kb (eta_g = 1).   mass: kw rJ_plain, rJ_plain_se, rJ_loaded, rJ_loaded_se, dm, dm_tol, d, d_tol (kg, m).
    weighed: kw m (kg), Lp (m).        eta: kw eta."""
    p = full(params); se = se or {}
    out = {'mode': mode, 'notes': []}
    if mode == 'nominal':
        Kt, Kt_se = p['Kb'], se.get('Kb', 0.)
        out['notes'].append('Nominal scale Kτ = Kb (gearbox efficiency 1). Relative values and all models are unaffected.')
    elif mode == 'mass':
        rng = np.random.default_rng(0); N = 4000
        r1 = kw['rJ_plain'] + rng.normal(0, kw.get('rJ_plain_se', 0.), N)
        r2 = kw['rJ_loaded'] + rng.normal(0, kw.get('rJ_loaded_se', 0.), N)
        dm = kw['dm'] + rng.normal(0, kw.get('dm_tol', 0.), N); d = kw['d'] + rng.normal(0, kw.get('d_tol', 0.), N)
        g = p['G'] + rng.normal(0, se.get('G', 0.), N)
        nominal = added_mass(kw['rJ_plain'], kw['rJ_loaded'], kw['dm'], kw['d'])
        A1 = 1 / r1; A2 = 1 / r2; Jp = dm * (GRAVITY * d - A2 * d * d) / (A2 - A1); Gabs = A1 * Jp
        Kts = Gabs / g
        Kt = nominal['G'] / p['G']; Kt_se = float(np.std(Kts))
        out.update(G_abs=nominal['G'], G_abs_se=float(np.std(Gabs)), Jp_abs=nominal['Jp'], sensitivity=nominal['sensitivity'])
        if nominal['sensitivity'] < .03:
            out['notes'].append(f'The added mass changed the swing frequency by only {100 * nominal["sensitivity"]:.1f} %: '
                                'move it closer to the tip (away from the centre of oscillation) or use a heavier mass.')
    elif mode == 'weighed':
        Gabs = kw['m'] * GRAVITY * kw['Lp']; Kt = Gabs / p['G']
        Kt_se = Kt * (se.get('G', 0.) / p['G'] if p['G'] else 0.)
        out['notes'].append('m·g·Lp from the weighed mass and the balance point.')
    elif mode == 'eta':
        Kt = kw['eta'] * p['Kb']; Kt_se = kw['eta'] * se.get('Kb', 0.)
        out['notes'].append(f'Assumed gearbox efficiency ηg = {kw["eta"]:g}.')
    else:
        raise ValueError('Unknown scale mode')
    out.update(Kt=float(Kt), Kt_se=float(Kt_se), eta=float(Kt / p['Kb']), eta_se=float(Kt_se / p['Kb']),
               mLp=float(p['G'] * Kt / GRAVITY))
    out['si'] = {k: (p[k] * Kt if k not in ('Rm', 'Kb') else p[k]) for k in PARAMS}
    if out['eta'] > 1.05:
        out['notes'].append(f'ηg = Kτ/Kb = {out["eta"]:.2f} > 1 is not physical: check Vs, the current-sensor gain or the reference.')
    return out


# ------------------------------------------------------------------ checks
def _rel_flags(names, values, ses, label='weakly identified'):
    out = []
    for nm in names:
        v, s = values.get(nm), ses.get(nm)
        if v is None or s is None:
            continue
        if v == 0 or abs(s / v) > .3:
            out.append(('warn', f'{INFO[nm][0] if nm in INFO else nm}: ±{abs(s / v) * 100 if v else float("inf"):.0f} % — {label}.'))
    return out


def check_prepared(p):
    out = [('info', n) for n in p.get('notes', [])]
    if p['fs'] < 240:
        out.append(('warn', f'{p["name"]}: stream rate {p["fs"]:.0f} Hz. 250 Hz is recommended (Step 0 → Apply estimation recording settings).'))
    if p['missing']:
        out.append(('warn', f'{p["name"]}: {p["missing"]} stream samples missing; the longest gap-free span is used.'))
    if p['saturated']:
        out.append(('warn', f'{p["name"]}: duty was clipped at the maximum duty during the run.'))
    big = float(np.degrees(np.max(np.abs(p['th1'] - np.pi)))) if len(p['th1']) else 0.
    if big > 60:
        out.append(('warn', f'{p["name"]}: the pendulum swung ±{big:.0f}° from hanging. Above about ±45° the motion is strongly '
                            'nonlinear and an open-loop simulation cannot follow it for long; use a lower amplitude or avoid '
                            'frequencies near the pendulum natural frequency (Step 1).'))
    if p.get('fault'):
        out.append(('warn', f'{p["name"]}: the run ended in a fault ({p["fault"]}). Data before the fault is still usable.'))
    return out


def check_polarity(preps):
    s = 0.
    for p in preps:
        m = masks(p)['driven']; s += float(np.sum(p['V'][m] * p['i'][m]))
    if s < 0:
        return [('error', 'Current polarity looks inverted (current opposes the applied voltage). '
                          'Set “Current polarity” to −1 in Step 0 and fit again.')]
    return []


def check_swing(r):
    out = []
    if r['arm_counts'] > 2:
        out.append(('warn', f'The arm moved {r["arm_counts"]:.0f} counts during the swing; hold it still and record again.'))
    if r['amplitude_deg'] < 5:
        out.append(('warn', f'Swing amplitude only {r["amplitude_deg"]:.1f}°; release it 30–40° from hanging for a clearer fit.'))
    if r['period_s'] and r['wn'] > 0:
        expected = 2 * math.pi / (r['wn'] * math.sqrt(max(1 - r['zeta'] ** 2, 1e-9)))
        dev = abs(r['period_s'] / expected - 1)
        if dev > .03:
            out.append(('warn', f'Zero-crossing period {r["period_s"]:.3f} s differs from the fit by {dev * 100:.1f} % '
                                '(large swings lengthen the period; the fit includes that).'))
    if r['zeta'] <= 0:
        out.append(('warn', 'Fitted pendulum damping is not positive.'))
    if r.get('rc', 0) < -2 * r.get('rc_se', 0) and r.get('rc', 0) < -1e-3:
        out.append(('warn', 'Fitted pendulum dry friction is negative; record a cleaner swing (arm held still, 30–40° release).'))
    return out + _rel_flags(['rJ', 'rb'], {'rJ': r['rJ'], 'rb': r['rb']}, {'rJ': r['rJ_se'], 'rb': r['rb_se']})


def check_electrical(r):
    out = []
    if r['Rm'] <= 0:
        out.append(('error', f'Rm = {r["Rm"]:.3g} Ω is not positive: check the current zero and polarity.'))
    if r['Kb'] <= 0:
        out.append(('error', f'Kb = {r["Kb"]:.3g} is not positive: the arm turns opposite to positive duty. '
                             'Flip armSign (Setup) or swap the motor leads.'))
    return out + _rel_flags(['Rm', 'Kb'], r, {'Rm': r['Rm_se'], 'Kb': r['Kb_se']})


def check_pendulum(r, swing=None):
    out = []
    if r['rb'] <= 0:
        out.append(('error', f'Pendulum damping b1/G = {r["rb"]:.3g} is not positive: the simulated pendulum would be unstable. '
                             'Use the free-swing values (Step 4 option) and/or set the pendulum timing correction (Step 0).'))
    if r['rh'] <= 0:
        out.append(('error', 'Coupling h/G is not positive: the pendulum angle runs opposite to the model convention '
                             '(positive = leaning the way the arm tip moves). In Step 0 set “Pendulum sign” to the opposite value '
                             'and fit Steps 4 and 5 again; no new recording is needed.'))
    if r['rh'] > 0 and not .05 <= r['Larm'] <= .4:
        out.append(('warn', f'Larm = {r["Larm"] * 100:.1f} cm is outside 5–40 cm: check the encoder scale.'))
    if swing and r.get('source') != 'free swing':
        dev = r['rJ'] / swing['rJ'] - 1
        if abs(dev) > .05:
            out.append(('warn', f'Jp/G from the driven runs differs from the free swing by {dev * 100:+.1f} %.'))
    return out


def check_arm(r, pend):
    out = []
    for k in ('Ja', 'G'):
        if r[k] <= 0:
            out.append(('error', f'{INFO[k][0]} is not positive: the arm is not excited enough or a sign is wrong.'))
    for k in ('Beq', 'tcp', 'tcn'):
        if r[k] < 0:
            out.append(('warn', f'{INFO[k][0]} is negative ({r[k]:.3g}); it is set to a small positive value for simulation.'))
    if r['Ja'] > 0 and r['G'] > 0 and r['Ja'] * r['G'] * pend['rJ'] <= (r['G'] * pend['rh']) ** 2:
        out.append(('error', 'Ja·Jp ≤ h²: the mass matrix is not positive definite. Refit with more arm motion.'))
    if r['G'] > 0 and pend['rJ'] > 0:
        pass
    return out + _rel_flags(['Ja', 'G', 'Beq', 'tcp', 'tcn'], r, {k: r[k + '_se'] for k in ('Ja', 'G', 'Beq', 'tcp', 'tcn')})


def check_refine(r):
    out = [('warn', f'Refinement changed {INFO[k][0]} by {v:+.0f} %; compare the overlays before accepting.')
           for k, v in r['change_pct'].items() if abs(v) > 30]
    if r['at_bound']:
        out.append(('warn', 'A refined parameter hit its search limit (×7.4): the least-squares start may be wrong.'))
    if r['cost_after'] > r['cost_before'] * .999:
        out.append(('warn', 'Refinement did not improve the simulation error.'))
    return out


def check_validation(sim):
    return [('warn', f'{k}: fit {v:.0f} % (< 80 %).') for k, v in sim['fit'].items() if k != 'current' and np.isfinite(v) and v < 80]


def check_params(params):
    out = []
    params = full(params)
    for k in PARAMS:
        if k == 'tp':
            if params[k] < 0:
                out.append(('warn', f'{INFO[k][0]} = {params[k]:.3g} is negative.'))
            continue
        if params[k] <= 0:
            out.append(('error' if k not in ('Beq', 'tcp', 'tcn') else 'warn', f'{INFO[k][0]} = {params[k]:.3g} is not positive'
                        + (': the simulated pendulum would be unstable.' if k == 'b1' else '.')))
    if params['Ja'] * params['Jp'] <= params['h'] ** 2:
        out.append(('error', 'Ja·Jp ≤ h²: the model is not physical.'))
    return out


def checks_markdown(items):
    if not items:
        return '✓ All checks passed.'
    icon = {'error': '✗', 'warn': '⚠', 'ok': '✓', 'info': 'ℹ'}
    return '\n\n'.join(f'{icon.get(level, "•")} {text}' for level, text in items)


# ------------------------------------------------------------------ excitation
def square_sweep(amplitude_pct, freqs, seconds_each=4.):
    """Centred square waves: firmware segments. phase = pi/2 starts a quarter period in, so the arm
    swings about its starting point; each frequency lasts a whole number of periods."""
    seg = [{'kind': 'hold', 'duration': .5, 'amplitude': 0., 'offset': 0., 'f0': 1., 'f1': 3.}]
    for f in freqs:
        f = float(f)
        if f <= 0:
            raise ValueError('Frequencies must be positive')
        periods = max(1, int(round(seconds_each * f)))
        seg.append({'kind': 'square', 'duration': periods / f, 'amplitude': float(amplitude_pct) / 100, 'offset': 0.,
                    'f0': f, 'f1': f, 'phase': math.pi / 2, 'duty': .5})
    seg.append({'kind': 'hold', 'duration': 1., 'amplitude': 0., 'offset': 0., 'f0': 1., 'f1': 3.})
    return seg


def predict_arm_swing(amplitude_pct, freq, Vs, Kb, deadzone=0.):
    """Upper bound of the arm swing (deg, about the centre) for a centred square wave:
    speed w = (A Vs - Vdead)/Kb, swing +-w/(4f)."""
    w = max(0., amplitude_pct / 100 * Vs - deadzone * Vs) / Kb
    return math.degrees(w / (4 * freq))


def reversal_note(amplitude_pct, fmax, slew_per_s):
    """Time the firmware slew limit needs for one reversal, against the shortest half period."""
    if not slew_per_s or slew_per_s <= 0:
        return None, None
    rev = 2 * amplitude_pct / 100 / slew_per_s; half = 1 / (2 * fmax)
    return rev, rev / half


# ------------------------------------------------------------------ text and export
def fmt_poly(c, var='s', digits=4):
    c = np.real(np.asarray(c, float)); c = np.where(np.abs(c) < 1e-12 * max(1., np.abs(c).max()), 0., c)
    n = len(c) - 1; terms = []
    for k, v in enumerate(c):
        if v == 0:
            continue
        p = n - k; s = f'{abs(v):.{digits}g}'
        if p >= 1 and s == '1':
            s = ''
        term = s + ('' if p == 0 else (var if p == 1 else f'{var}^{p}'))
        terms.append(('− ' if v < 0 else '+ ') + term)
    if not terms:
        return '0'
    out = ' '.join(terms)
    return out[2:] if out.startswith('+ ') else '−' + out[2:]


def fmt_matrix(M, digits=5):
    M = np.atleast_2d(np.real(M))
    cells = [[f'{v:.{digits}g}' for v in row] for row in M]
    w = max(len(c) for row in cells for c in row)
    return '\n'.join('[ ' + '  '.join(c.rjust(w) for c in row) + ' ]' for row in cells)


def fmt_roots(r, digits=4):
    out = []
    for z in sorted(np.atleast_1d(r), key=lambda z: (np.real(z), np.imag(z))):
        if abs(np.imag(z)) < 1e-9 * max(1, abs(z)):
            out.append(f'{np.real(z):.{digits}g}')
        elif np.imag(z) > 0:
            out.append(f'{np.real(z):.{digits}g} ± {np.imag(z):.{digits}g}j')
    return ', '.join(out) if out else '—'


def params_table(params, se=None, si=None):
    se = se or {}
    head = '| Parameter | Value | ± (approx.) | Unit | Meaning |' + (' SI value |' if si else '') + '\n'
    head += '|---|---|---|---|---|' + ('---|' if si else '') + '\n'
    rows = []
    for k in PARAMS:
        sym, unit, si_unit, desc = INFO[k]; v = full(params)[k]; s = se.get(k)
        rel = f'{abs(s / v) * 100:.1f} %' if s is not None and v else '—'
        row = f'| {sym} | {v:.4g} | {rel} | {unit} | {desc} |'
        if si:
            row += f' {si[k]:.4g} {si_unit} |'
        rows.append(row)
    return head + '\n'.join(rows)


def model_markdown(params, se, lin, scl=None, larm=None):
    p = full(params); Vs = lin['Vs']
    up, hg = lin['upright'], lin['hanging']
    parts = ['### Identified parameters', params_table(p, se, scl['si'] if scl else None)]
    if larm is not None:
        parts.append(f'Arm length from the data: **Larm = {larm * 100:.2f} cm**.')
    if scl:
        parts.append(f'Scale: Kτ = {scl["Kt"]:.4g} N·m/A (±{scl["Kt_se"]:.2g}), ηg = Kτ/Kb = {scl["eta"]:.3f}, '
                     f'm·Lp = {scl["mLp"] * 1000:.2f} g·m. ' + ' '.join(scl['notes']))
    parts.append(f'Friction as equivalent voltage: +{lin["friction_V"]["+"]:.3g} V ({100 * lin["friction_duty"]["+"]:.1f} % duty), '
                 f'−{lin["friction_V"]["−"]:.3g} V ({100 * lin["friction_duty"]["−"]:.1f} % duty). '
                 f'Back-EMF damping Kb/Rm = {p["Kb"] / p["Rm"]:.3g} against Beq/Kτ = {p["Beq"]:.3g} (A·s/rad).')
    parts.append('### Nonlinear model (θ1 = 0 upright, current in A, torques in amperes of motor current)\n\n```\n'
                 f'i  = (V − {p["Kb"]:.4g}·θ̇0)/{p["Rm"]:.4g}   while duty ≠ 0;   i = 0 at zero duty (driver standby);   V = duty·{Vs:g}\n'
                 f'({p["Ja"]:.4g} + {p["Jp"]:.4g}·sin²θ1)·θ̈0 + {p["h"]:.4g}·cosθ1·θ̈1 + {2 * p["Jp"]:.4g}·sinθ1·cosθ1·θ̇0·θ̇1'
                 f' − {p["h"]:.4g}·sinθ1·θ̇1² + {p["Beq"]:.4g}·θ̇0 + τc·sgn θ̇0 = i\n'
                 f'   τc = {p["tcp"]:.4g} (θ̇0 > 0), {p["tcn"]:.4g} (θ̇0 < 0)\n'
                 f'{p["h"]:.4g}·cosθ1·θ̈0 + {p["Jp"]:.4g}·θ̈1 − {p["Jp"]:.4g}·sinθ1·cosθ1·θ̇0² + {p["b1"]:.4g}·θ̇1 + {p["tp"]:.4g}·sgn θ̇1 − {p["G"]:.4g}·sinθ1 = 0\n```')
    for title, sysm, state in (('Upright (θ1 = 0), report Eq. 99', up, 'x = [θ0, θ1, θ̇0, θ̇1]'),
                               ('Hanging (θ1 = π), report Eq. 98', hg, 'x = [θ0, φ, θ̇0, φ̇], φ = θ1 − π')):
        unstable = [z for z in sysm['poles'] if np.real(z) > 1e-6]
        tc = ''.join(f' Unstable pole {np.real(z):.4g} rad/s → time constant {1000 / np.real(z):.1f} ms.' for z in unstable)
        parts.append(f'### {title}\n\n{state}; input V (volts) or duty. Outputs y = [θ0, θ1, i].\n\n'
                     f'```\nA =\n{fmt_matrix(sysm["A"])}\n\nB (per volt) =\n{fmt_matrix(sysm["B"])}\n\nBd (per unit duty, = {Vs:g}·B) =\n{fmt_matrix(sysm["Bd"])}\n\n'
                     f'C =\n{fmt_matrix(sysm["C"])}\n\nD (per volt) =\n{fmt_matrix(sysm["D"])}\n```\n\n'
                     f'Poles: {fmt_roots(sysm["poles"])} rad/s.{tc} Controllability rank {sysm["ctrb_rank"]} of 4 '
                     f'(condition {sysm["ctrb_cond"]:.3g}).')
        for name, tfd in sysm['tf'].items():
            label = 'θ0(s)/V(s)' if name == 'theta0/V' else 'θ1(s)/V(s)'
            parts.append(f'**{label}** = ({fmt_poly(tfd["num"])}) / ({fmt_poly(tfd["den"])})  \n'
                         f'zeros {fmt_roots(tfd["zeros"])} · poles {fmt_roots(tfd["poles"])} · gain {tfd["gain"]:.4g}')
        parts.append(f'Discrete (zero-order hold, Ts = {sysm["Ts"] * 1000:g} ms):\n\n```\nAd =\n{fmt_matrix(sysm["Ad"], 7)}\n\n'
                     f'Bd per volt =\n{fmt_matrix(sysm["Bd_v"], 7)}\n```\n\nDiscrete poles: {fmt_roots(sysm["poles_d"], 6)}.')
    if 'five_state' in lin:
        f5 = lin['five_state']
        parts.append(f'### Five-state model with Lm = {f5["Lm"] * 1000:g} mH (report Eq. 86)\n\n```\nA5 =\n{fmt_matrix(f5["A"])}\n\nB5 =\n{fmt_matrix(f5["B"])}\n```\n\n'
                     f'Poles: {fmt_roots(f5["poles"])} rad/s.')
    return '\n\n'.join(parts)


def _m_matrix(M):
    M = np.atleast_2d(np.real(M))
    return '[' + '; '.join(' '.join(f'{v:.10g}' for v in row) for row in M) + ']'


def export(folder, params, se, lin, scl=None, larm=None, info=None):
    """Write parameters.json, furuta_model.m, furuta_rhs.m, parameters.tex into folder. Returns paths."""
    params = full(params)
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    Vs = lin['Vs']; Ts = lin['Ts']
    data = {'units': 'torque-like parameters are divided by K_tau (torque in amperes of motor current)',
            'params': params, 'se': se, 'Vs': Vs, 'Larm_m': larm, 'scale': scl and {k: v for k, v in scl.items()},
            'upright': {k: np.real(lin['upright'][k]).tolist() for k in ('A', 'B', 'Bd', 'C', 'D')},
            'hanging': {k: np.real(lin['hanging'][k]).tolist() for k in ('A', 'B', 'Bd', 'C', 'D')},
            'poles_upright': [[float(np.real(z)), float(np.imag(z))] for z in lin['upright']['poles']],
            'info': info or {}}
    paths = []
    p = folder / 'parameters.json'; p.write_text(json.dumps(data, indent=2, default=float), encoding='utf-8'); paths.append(p)
    lines = ['% Furuta pendulum model identified with Furuta Lab.',
             '% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.',
             '% State x = [theta0; theta1; dtheta0; dtheta1]; theta1 = 0 upright. Input V (volts) or duty (B*Vs).',
             'p = struct();'] + [f'p.{k} = {params[k]:.10g};' for k in PARAMS] + \
            [f'p.Vs = {Vs:.10g};', f'p.eps = {SIM_EPS:g};  % arm friction smoothing (rad/s) used by furuta_rhs',
             f'p.eps1 = {SIM_EPS1:g};  % pendulum friction smoothing (rad/s)']
    if larm is not None:
        lines.append(f'p.Larm = {larm:.10g};  % m, from the data')
    if scl:
        lines.append(f'p.Ktau = {scl["Kt"]:.10g};  % N*m/A ({scl["mode"]} scale); SI value = per-Ktau value * Ktau')
        lines.append(f'p.eta_g = {scl["eta"]:.10g};')
    lines.append(f'Ts = {Ts:.10g};')
    for tag, key in (('up', 'upright'), ('down', 'hanging')):
        s = lin[key]
        lines += [f'A_{tag} = {_m_matrix(s["A"])};', f'B_{tag} = {_m_matrix(s["B"])};  % per volt',
                  f'Bd_{tag} = {_m_matrix(s["Bd"])};  % per unit duty',
                  f'C_{tag} = {_m_matrix(s["C"])};', f'D_{tag} = {_m_matrix(s["D"])};',
                  f"sys_{tag} = ss(A_{tag}, B_{tag}, C_{tag}, D_{tag}, 'InputName', 'V', 'OutputName', {{'theta0','theta1','i'}});",
                  f"sys_{tag}_duty = ss(A_{tag}, Bd_{tag}, C_{tag}, D_{tag}*p.Vs, 'InputName', 'duty', 'OutputName', {{'theta0','theta1','i'}});",
                  f'G_{tag} = minreal(tf(sys_{tag}));', f"sysd_{tag} = c2d(sys_{tag}, Ts, 'zoh');"]
    lines.append('% Nonlinear model: see furuta_rhs.m, e.g. ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], [0; pi; 0; 0])')
    p = folder / 'furuta_model.m'; p.write_text('\n'.join(lines) + '\n', encoding='utf-8'); paths.append(p)
    rhs_m = """function [dx, i] = furuta_rhs(x, d, p)
%FURUTA_RHS Nonlinear Furuta model identified with Furuta Lab.
%   x = [theta0; theta1; dtheta0; dtheta1] (theta1 = 0 upright), d = duty in [-1, 1].
%   Torque-like parameters are divided by K_tau (torque in amperes of motor current).
%   At zero duty the driver is in standby: the motor is open-circuit and i = 0.
s = sin(x(2)); c = cos(x(2)); w0 = x(3); w1 = x(4);
if d ~= 0
    i = (d*p.Vs - p.Kb*w0)/p.Rm;
else
    i = 0;
end
if w0 > 0, tc = p.tcp; else, tc = p.tcn; end
f0 = i - p.Beq*w0 - tc*tanh(w0/p.eps) - 2*p.Jp*s*c*w0*w1 + p.h*s*w1^2;
f1 = p.Jp*s*c*w0^2 - p.b1*w1 - p.tp*tanh(w1/p.eps1) + p.G*s;
M = [p.Ja + p.Jp*s^2, p.h*c; p.h*c, p.Jp];
a = M \\ [f0; f1];
dx = [w0; w1; a(1); a(2)];
end
"""
    p = folder / 'furuta_rhs.m'; p.write_text(rhs_m, encoding='utf-8'); paths.append(p)
    tex = ['% Rows for a tabular with columns: symbol & value & relative uncertainty & unit \\\\',
           '% Torque-like parameters are divided by $K_\\tau$.']
    texsym = {'Rm': 'R_m', 'Kb': 'K_b', 'Ja': 'J_a/K_\\tau', 'Jp': 'J_p/K_\\tau', 'h': 'h/K_\\tau', 'G': 'mgL_p/K_\\tau',
              'Beq': 'B_{eq}/K_\\tau', 'b1': 'b_1/K_\\tau', 'tcp': '\\tau_{c+}/K_\\tau', 'tcn': '\\tau_{c-}/K_\\tau'}
    texsym['tp'] = '\\tau_p/K_\\tau'
    texunit = {'Rm': '\\Omega', 'Kb': 'V\\,s/rad', 'Ja': 'A\\,s^2/rad', 'Jp': 'A\\,s^2/rad', 'h': 'A\\,s^2/rad', 'G': 'A',
               'Beq': 'A\\,s/rad', 'b1': 'A\\,s/rad', 'tcp': 'A', 'tcn': 'A', 'tp': 'A'}
    for k in PARAMS:
        rel = abs(se.get(k, 0) / params[k]) * 100 if params[k] else 0
        tex.append(f'${texsym[k]}$ & {params[k]:.4g} & {rel:.1f}\\,\\% & $\\mathrm{{{texunit[k]}}}$ \\\\')
    if larm is not None:
        tex.append(f'$L_{{arm}}$ & {larm:.4g} & — & $\\mathrm{{m}}$ \\\\')
    p = folder / 'parameters.tex'; p.write_text('\n'.join(tex) + '\n', encoding='utf-8'); paths.append(p)
    return paths
