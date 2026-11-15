"""Simulated bench for tests and synthetic recordings (no hardware).

Plant: the Furuta model with inductance neglected (report Eq. 93), driven by the
duty the firmware applies (slew limit, through zero before reversing). Zero duty
puts the TB6612 in standby, so the motor is open-circuit: i = 0, no back-EMF
braking. Recordings are made the way the firmware streams them: encoder
quantisation, AS5600 12-bit angle, noisy current.

The numbers in TRUE are plausible, not measured on the rig.
"""
import json
import math
import struct
import time
from pathlib import Path
import numpy as np
from scipy import signal

from . import wire

G = 9.80665
CPR = 1496.
TRUE = dict(Rm=5.5, Kb=.39, eta=.7, Jmo=6.5e-4, bmo=4e-4, tc=.06,
            m=.05, Lp=.10, Larm=.12, J1=1.67e-4, J0=3e-4, b0=1e-4, b1=5e-5, tp=0., Vs=12.)
TRUE['Kt'] = TRUE['eta'] * TRUE['Kb']


def lumped(P=TRUE):
    """Report Eq. 77, absolute units."""
    return dict(Ja=P['J0'] + P['Jmo'] + P['m'] * P['Larm'] ** 2, Jp=P['J1'] + P['m'] * P['Lp'] ** 2,
                h=P['m'] * P['Larm'] * P['Lp'], Beq=P['b0'] + P['bmo'], b1=P['b1'], G=P['m'] * G * P['Lp'])


def true_params(P=TRUE):
    """The ten identified numbers (torque-like ones divided by K_tau), as estimation.PARAMS."""
    L = lumped(P); k = P['Kt']
    return dict(Rm=P['Rm'], Kb=P['Kb'], Ja=L['Ja'] / k, Jp=L['Jp'] / k, h=L['h'] / k, G=L['G'] / k,
                Beq=L['Beq'] / k, b1=L['b1'] / k, tcp=P['tc'] / k, tcn=P['tc'] / k, tp=P.get('tp', 0.) / k)


def command(segments, dt=1e-3, slew=10.):
    """Applied duty: segments are (seconds, f(t)) pairs; slew limit; through zero before reversing."""
    total = sum(s[0] for s in segments); n = int(round(total / dt)); t = np.arange(n) * dt
    req = np.zeros(n); at = 0.
    for dur, fn in segments:
        ix = (t >= at - 1e-12) & (t < at + dur - 1e-12); req[ix] = fn(t[ix] - at); at += dur
    app = np.zeros(n); a = 0.
    for k in range(n):
        tgt = req[k]
        if a * tgt < 0:
            tgt = 0.
        a += min(max(tgt - a, -slew * dt), slew * dt)
        if abs(a) < 1e-9:
            a = 0.
        app[k] = a
    return t, app


def firmware_segments(segments, dt=1e-3, slew=10.):
    """Same as command() for firmware segment dicts (hold, square)."""
    def fn(s):
        if s['kind'] == 'hold':
            return lambda t: np.full(len(t), s['amplitude'])
        if s['kind'] == 'square':
            f = s['f0']; ph = s.get('phase', 0.) / (2 * math.pi); d = s.get('duty', .5)
            return lambda t: s['amplitude'] * np.where((f * t + ph) % 1 < d, 1., -1.)
        raise ValueError(s['kind'])
    return command([(s['duration'], fn(s)) for s in segments], dt, slew)


def _furuta(P, L, eps=.02):
    Rm, Kb, Kt, tc, Vs, tp = P['Rm'], P['Kb'], P['Kt'], P['tc'], P['Vs'], P.get('tp', 0.)
    Ja, Jp, h, Beq, b1, Gm = L['Ja'], L['Jp'], L['h'], L['Beq'], L['b1'], L['G']

    def rhs(x0, x1, x2, x3, d):
        s = math.sin(x1); c = math.cos(x1)
        cur = (d * Vs - Kb * x2) / Rm if d != 0 else 0.
        f0 = Kt * cur - Beq * x2 - tc * math.tanh(x2 / eps) - 2 * Jp * s * c * x2 * x3 + h * s * x3 * x3
        f1 = Jp * s * c * x2 * x2 - b1 * x3 - tp * math.tanh(x3 / eps) + Gm * s
        m00 = Ja + Jp * s * s; m01 = h * c; det = m00 * Jp - m01 * m01
        return x2, x3, (Jp * f0 - m01 * f1) / det, (-m01 * f0 + m00 * f1) / det, cur
    return rhs


def sim_furuta(t, app, P=TRUE, x0=(0., math.pi, 0., 0.)):
    """RK4 at the step of t. Returns states (n, 4) and current (n,)."""
    rhs = _furuta(P, lumped(P)); dt = float(t[1] - t[0]); n = len(t)
    X = np.empty((n, 4)); cur = np.empty(n); x = list(x0)
    for k in range(n):
        d = float(app[k]); X[k] = x
        a = rhs(*x, d); cur[k] = a[4]
        b = rhs(*[x[j] + .5 * dt * a[j] for j in range(4)], d)
        c = rhs(*[x[j] + .5 * dt * b[j] for j in range(4)], d)
        e = rhs(*[x[j] + dt * c[j] for j in range(4)], d)
        x = [x[j] + dt / 6 * (a[j] + 2 * b[j] + 2 * c[j] + e[j]) for j in range(4)]
    return X, cur


def sim_swing(t, P=TRUE, phi0=.6, extra=None):
    """Arm held, motor off (report Eq. 97). extra=(dm, d): a point mass dm at distance d from the pivot.
    Returns theta1 = phi + pi."""
    L = lumped(P); Gm, Jp = L['G'], L['Jp']
    if extra:
        Gm += extra[0] * G * extra[1]; Jp += extra[0] * extra[1] ** 2
    b1 = L['b1']; tp = P.get('tp', 0.); dt = float(t[1] - t[0]); x0, x1 = phi0, 0.; out = np.empty(len(t))

    def f(p, w):
        return w, (-b1 * w - tp * math.tanh(w / .02) - Gm * math.sin(p)) / Jp
    for k in range(len(t)):
        out[k] = x0
        a = f(x0, x1); b = f(x0 + .5 * dt * a[0], x1 + .5 * dt * a[1])
        c = f(x0 + .5 * dt * b[0], x1 + .5 * dt * b[1]); e = f(x0 + dt * c[0], x1 + dt * c[1])
        x0 += dt / 6 * (a[0] + 2 * b[0] + 2 * c[0] + e[0]); x1 += dt / 6 * (a[1] + 2 * b[1] + 2 * c[1] + e[1])
    return out + math.pi


def to_rows(t, th0, th1, cur, app, fs=250., sigma_i=.12, seed=0, config=None, mcu_filter=None, adc_hz=1000., pend_lag=0.):
    """Core records as streamed: every 1/fs s, encoder quantised, AS5600 12 bit, noisy current.
    mcu_filter: biquad sos row [b0 b1 b2 1 a1 a2] applied causally at adc_hz to the noisy current
    (t must then be sampled at 1/adc_hz)."""
    cfg = default_config() if config is None else config
    rng = np.random.default_rng(seed); dt = float(t[1] - t[0])
    step = int(round(1 / (fs * dt))); ix = np.arange(0, len(t), step)
    noisy = cur + rng.normal(0, sigma_i, len(cur))
    mv = cfg['currentZeroMv'] + cfg['sensitivityMvA'] * noisy
    fmv = mv
    if mcu_filter is not None:
        if abs(dt * adc_hz - 1) > 1e-6:
            raise ValueError('Simulate at the ADC rate to apply an MCU filter')
        sos = np.asarray(mcu_filter, float)[None]
        fmv = signal.sosfilt(sos, mv, zi=signal.sosfilt_zi(sos) * mv[0])[0]
    a = np.zeros(len(ix), wire.CORE)
    us = np.round(t[ix] * 1e6).astype(np.uint64) + 1_000_000
    a['device_us'] = us; a['pend_us'] = us; a['adc_us'] = us; a['apply_us'] = us
    a['seq'] = np.arange(len(ix)) * int(round(500 / fs))
    a['count'] = np.round(cfg['armSign'] * th0[ix] * cfg['countsPerRev'] / (2 * np.pi)).astype(np.int64)
    th1s = np.interp(t[ix] - pend_lag, t, th1) if pend_lag else th1[ix]     # sensor reports an older angle
    raw = (cfg['pendSign'] * (th1s - np.pi) * 4096 / (2 * np.pi) + cfg['pendDownRaw'])
    a['pend_raw'] = np.mod(np.round(raw), 4096).astype(np.uint16)
    a['current_mv'] = mv[ix]; a['filtered_mv'] = fmv[ix]; a['adc_raw'] = 3000
    a['requested'] = app[ix]; a['applied'] = app[ix]
    a['validity'] = 7; a['mag_status'] = 0x20; a['state'] = np.where(app[ix] != 0, 3, 1)
    return a


def default_config():
    return {'countsPerRev': CPR, 'armSign': 1, 'pendSign': 1, 'pendDownRaw': 2048, 'currentZeroMv': 2500.,
            'sensitivityMvA': 185., 'assumedBusV': 12., 'adcHz': 2000, 'streamHz': 250,
            'filter': {'type': 'bypass', 'alpha': .1, 'window': 8, 'sos': [1, 0, 0, 1, 0, 0]},
            'maxDuty': 1., 'travelCounts': 1e6, 'slewPerSecond': 10.}


def write_run(path, rows, config, metadata, status=None):
    """Write a run folder exactly as Device.start_record/finish_record does (manifest.json + samples.bin)."""
    path = Path(path); path.mkdir(parents=True, exist_ok=False)
    per = (2026 - 22) // wire.CORE.itemsize
    with (path / 'samples.bin').open('wb') as f:
        for k in range(0, len(rows), per):
            part = rows[k:k + per]
            payload = struct.pack('<QIIIH', 1, 1, 1, 0, len(part)) + part.tobytes()
            f.write(len(payload).to_bytes(4, 'little') + payload)
    st = status or {'state': 'DISARMED', 'centered': True, 'centerCount': '0'}
    manifest = {'schema': 2, 'source': 'simulation', 'created': time.time(), 'config': config, 'status': st,
                'metadata': metadata, 'complete': True, 'samples': len(rows), 'final_status': st, 'link_errors': 0}
    (path / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return path


def synthetic_session(folder, seed=0, sigma_i=.12, fs=250.):
    """Free swing + two fit runs + one validation run, written as run folders. Returns their paths."""
    from . import estimation as E
    folder = Path(folder); cfg = default_config(); out = {}
    t = np.arange(0, 20, 1e-3); z = np.zeros(len(t))
    th1 = sim_swing(t, TRUE, .6)
    out['swing'] = write_run(folder / 'swing', to_rows(t, z, th1, z, z, fs, sigma_i, seed, cfg), cfg,
                             {'purpose': 'free_swing', 'estimation_step': 'swing'})
    presets = [('fit_a', 30, (1.5, 2., 3., 4., 2.5)), ('fit_b', 40, (2., 3., 4., 5., 2.5)), ('validation', 35, (1.75, 2.5, 3.5, 2.))]
    for k, (name, amp, freqs) in enumerate(presets):
        seg = E.square_sweep(amp, freqs, 4.)
        tt, app = firmware_segments(seg)
        X, cur = sim_furuta(tt, app)
        rows = to_rows(tt, X[:, 0], X[:, 1], cur, app, fs, sigma_i, seed + k + 1, cfg)
        out[name] = write_run(folder / name, rows, cfg, {'purpose': 'estimation', 'segments': seg,
                                                          'estimation_step': 'validation' if name == 'validation' else 'fit'})
    return out
