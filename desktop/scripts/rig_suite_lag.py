"""Pendulum-reading lag relative to the arm encoder, from any recording where the arm moves.

    python desktop/rig_suite_lag.py RUN_DIR [t_start t_end] [--win 0.2]

Method (no linearization, works with large swings): the model's pendulum equation (report eq. 89, row 2,
Table 6), driven by the MEASURED arm motion (encoder, zero-phase smoothed, differentiated), is integrated over
short windows that each start from the measured pendulum state. The measured pendulum is shifted by tau and
the tau with the smallest error is the lag of the pendulum reading behind the encoder (positive = the pendulum
reading is late). Same model and smoothing for every recording, so motor-driven and hand-driven runs compare.

Fast: all windows are integrated at once (numpy arrays) with fixed-step RK4 on the recording's uniform grid,
and tau is searched coarse (2 ms) then fine (0.25 ms). Windows are short (0.2 s by default) because near
upright the pendulum is unstable and errors in the start state grow quickly.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from furuta_lab.backend import load_run  # noqa: E402

Jp = 0.0001853092537; h = 0.0002012270497; G = 0.02898730616; b1 = 0.0001208246778
tp = 0.0006378819055; eps1 = 0.05


def load(path):
    a, m = load_run(path)
    c = m['config']
    t = (a['device_us'] - a['device_us'][0]) / 1e6
    th0 = c['armSign'] * a['count'] / c['countsPerRev'] * 2 * np.pi
    d = ((a['pend_raw'].astype(int) - c['pendDownRaw'] + 2048) % 4096 - 2048) * 2 * np.pi / 4096 * c['pendSign']
    th1 = np.unwrap(d + np.pi)                       # model convention: 0 up, pi hanging
    return t, th0, th1


def prepare(path, fc=25.0):
    t, th0, th1 = load(path)
    fs = round(1 / np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], 1 / fs)
    th0u = np.interp(tu, t, th0)
    th1u = np.interp(tu, t, th1)
    sos = butter(4, fc, fs=fs, output='sos')
    th0s = sosfiltfilt(sos, th0u)                    # zero phase: adds no delay
    w0 = np.gradient(th0s, tu)
    a0 = np.gradient(w0, tu)
    w1 = np.gradient(sosfiltfilt(sos, th1u), tu)
    return dict(fs=fs, tu=tu, th0s=th0s, w0=w0, a0=a0, th1=th1u, w1=w1)


def accel(q, w, a0, w0):
    s, c = np.sin(q), np.cos(q)
    f1 = Jp * s * c * w0 ** 2 - b1 * w - tp * np.tanh(w / eps1) + G * s
    return (f1 - h * c * a0) / Jp


def window_error(D, k0, L, tau):
    """RMS error [deg] of all windows (start indices k0, L steps) with the measurement shifted by tau [s]."""
    tu, fs = D['tu'], D['fs']
    dt = 1 / fs
    sh = lambda x, k: np.interp(tu[k] + tau, tu, x)   # noqa: E731  measured value tau later
    q = sh(D['th1'], k0)
    w = sh(D['w1'], k0)
    err = [np.zeros_like(q)]
    for j in range(L - 1):
        k = k0 + j
        a_0, a_1 = D['a0'][k], D['a0'][k + 1]
        v_0, v_1 = D['w0'][k], D['w0'][k + 1]
        am, vm = 0.5 * (a_0 + a_1), 0.5 * (v_0 + v_1)
        k1q, k1w = w, accel(q, w, a_0, v_0)
        k2q, k2w = w + dt / 2 * k1w, accel(q + dt / 2 * k1q, w + dt / 2 * k1w, am, vm)
        k3q, k3w = w + dt / 2 * k2w, accel(q + dt / 2 * k2q, w + dt / 2 * k2w, am, vm)
        k4q, k4w = w + dt * k3w, accel(q + dt * k3q, w + dt * k3w, a_1, v_1)
        q = q + dt / 6 * (k1q + 2 * k2q + 2 * k3q + k4q)
        w = w + dt / 6 * (k1w + 2 * k2w + 2 * k3w + k4w)
        err.append(q - sh(D['th1'], k + 1))
    return np.degrees(np.sqrt(np.mean(np.concatenate(err) ** 2)))


def lag(path, t0=None, t1=None, win=0.2, step=0.1):
    D = prepare(path)
    tu, fs = D['tu'], D['fs']
    L = int(round(win * fs))
    lo = t0 if t0 is not None else tu[0] + 0.5
    hi = t1 if t1 is not None else tu[-1] - 0.5
    k0 = []
    for ts in np.arange(lo, hi - win, step):
        k = int(np.searchsorted(tu, ts))
        if k + L + 10 < len(tu) and np.ptp(D['th0s'][k:k + L]) > np.radians(0.5):   # the arm moves
            k0.append(k)
    k0 = np.array(k0)
    if len(k0) < 5:
        raise RuntimeError('too little arm motion in this recording')
    coarse = np.arange(-0.010, 0.0301, 0.002)
    ec = np.array([window_error(D, k0, L, tau) for tau in coarse])
    c = coarse[np.argmin(ec)]
    fine = np.arange(c - 0.003, c + 0.00301, 0.00025)
    ef = np.array([window_error(D, k0, L, tau) for tau in fine])
    k = int(np.argmin(ef))
    best = fine[k]
    if 0 < k < len(fine) - 1:                         # parabola through the minimum
        y0, y1, y2 = ef[k - 1:k + 2]
        best += 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) * (fine[1] - fine[0])
    return best, coarse, ec, ef.min(), len(k0)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    win = 0.2
    if '--win' in sys.argv:
        win = float(sys.argv[sys.argv.index('--win') + 1])
        args = [a for a in args if a != sys.argv[sys.argv.index('--win') + 1]]
    p = args[0]
    t0 = float(args[1]) if len(args) > 1 else None
    t1 = float(args[2]) if len(args) > 2 else None
    best, coarse, ec, emin, n = lag(p, t0, t1, win)
    print(f'{Path(p).name}: pendulum reading lag {best * 1e3:+.1f} ms ({n} windows of {win:.2f} s); '
          f'rms error at 0 ms {ec[np.argmin(abs(coarse))]:.3f} deg, at best {emin:.3f} deg; curve '
          + ' '.join(f'{tt * 1e3:.0f}:{e:.2f}' for tt, e in zip(coarse[::2], ec[::2])))
