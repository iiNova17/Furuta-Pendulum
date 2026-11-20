"""Gear play between the motor encoder and the arm, from recordings where the motor drives the arm.

    python desktop/rig_suite_backlash.py RUN_DIR [RUN_DIR ...]

The arm is modelled as following the motor-side angle (encoder) through a dead band (play operator) of total
width b: the arm only moves once the motor has crossed the play. The model pendulum (rig_suite_lag) is driven
by that arm angle; b and a residual delay tau are found by grid search on the window error. If one b with a
near-zero tau fits runs with small and large arm motion, the lag measured earlier (14.7 ms small, 9.1 ms large)
is gear play, not delay.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rig_suite_lag as L  # noqa: E402


def play(x, b):
    """Play (backlash) operator: output follows x but only after x moved b/2 past it."""
    y = np.empty_like(x); y[0] = x[0]; h = b / 2
    for k in range(1, len(x)):
        y[k] = min(max(y[k - 1], x[k] - h), x[k] + h)
    return y


def with_play(D0, b, fc=25.0):
    D = dict(D0)
    arm = play(D0['th0_raw'], b)
    sos = butter(4, fc, fs=D['fs'], output='sos')
    D['th0s'] = sosfiltfilt(sos, arm)
    D['w0'] = np.gradient(D['th0s'], D['tu'])
    D['a0'] = np.gradient(D['w0'], D['tu'])
    return D


def fit(path, widths_deg=np.arange(0, 4.01, 0.25), taus=np.arange(-0.004, 0.0161, 0.001), win=0.2, step=0.1):
    D0 = L.prepare(path)
    t, th0, _ = L.load(path)
    D0['th0_raw'] = np.interp(D0['tu'], t, th0)
    tu, fs = D0['tu'], D0['fs']
    Ln = int(round(win * fs))
    k0 = np.array([int(np.searchsorted(tu, ts)) for ts in np.arange(tu[0] + 0.5, tu[-1] - 0.5 - win, step)
                   if np.ptp(D0['th0s'][int(np.searchsorted(tu, ts)):int(np.searchsorted(tu, ts)) + Ln]) > np.radians(0.5)])
    E = np.full((len(widths_deg), len(taus)), np.nan)
    for i, b in enumerate(widths_deg):
        D = with_play(D0, np.radians(b))
        for j, tau in enumerate(taus):
            E[i, j] = L.window_error(D, k0, Ln, tau)
    i, j = np.unravel_index(np.nanargmin(E), E.shape)
    # best tau with no play, for comparison
    j0 = int(np.nanargmin(E[0]))
    return dict(b=widths_deg[i], tau=taus[j], err=E[i, j], tau_noplay=taus[j0], err_noplay=E[0, j0], E=E,
                widths=widths_deg, taus=taus, n=len(k0))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        r = fit(p)
        print(f'{Path(p).name[:34]:34s} play {r["b"]:4.2f} deg + delay {r["tau"] * 1e3:+5.1f} ms -> {r["err"]:.3f} deg'
              f'   |  delay only: {r["tau_noplay"] * 1e3:+5.1f} ms -> {r["err_noplay"]:.3f} deg   ({r["n"]} windows)')
