"""Two-mass model of the rig: motor side and arm side joined by the shaft coupling (play + spring + damper).

Why: on 2026-11-20 the pendulum lagged the motor encoder by 14.7 ms for small arm motion and 9.1 ms for large
(rig_suite_lag), and a dead band of about 3 deg plus a few ms explained the small/slow runs (rig_suite_backlash).
The person at the rig: "the shaft coupling, mainly", already as tight as it gets. So model it.

States x = [thm, tha, th1, wm, wa, w1]: motor-side angle (what the encoder measures, at the arm scale), arm angle,
pendulum angle (0 up), and their speeds. Units as the identified model (torques in amperes of motor current).

    motor side : Jm am = i - Beq wm - tc(wm) - tau_c                  i = (d Vs - Kb wm)/Rm, 0 at zero duty
    arm side   : (Jarm + Jp s^2) aa + h c a1 = tau_c - 2 Jp s c wa w1 + h s w1^2
    pendulum   : h c aa + Jp a1 = Jp s c wa^2 - b1 w1 - tp sgn(w1) + G s
    coupling   : z = thm - tha;  e = dz(z, b) (zero inside the play of total width b);
                 tau_c = k e + c (wm - wa) while engaged (|z| > b/2), 0 inside the play

Ja (identified, arm-side inertia of the rigid model) = Jm + Jarm; rJm = Jm / Ja is fitted.
With b = 0 and k -> infinity the model is the rigid estimation model (estimation.rhs, same friction).
"""
import math

import numpy as np

from furuta_lab import estimation as E

COUPLING = ['rJm', 'k', 'c', 'b']


def rhs_batch(x, d, P, Vs):
    """x (..., 6); d (...,); P dict of arrays broadcastable to x[..., 0]. Returns dx, current."""
    thm, tha, t1, wm, wa, w1 = (x[..., j] for j in range(6))
    s, cth = np.sin(t1), np.cos(t1)
    Jm = P['rJm'] * P['Ja']; Jarm = P['Ja'] - Jm
    cur = np.where(d != 0, (d * Vs - P['Kb'] * wm) / P['Rm'], 0.)
    fr = np.where(wm > 0, P['tcp'], P['tcn']) * np.tanh(wm / E.SIM_EPS)
    z = thm - tha; hb = P['b'] / 2
    e = np.where(z > hb, z - hb, np.where(z < -hb, z + hb, 0.))
    engaged = np.abs(z) > hb
    tau = P['k'] * e + np.where(engaged, P['c'] * (wm - wa), 0.)
    am = (cur - P['Beq'] * wm - fr - tau) / Jm
    f0 = tau - 2 * P['Jp'] * s * cth * wa * w1 + P['h'] * s * w1 ** 2
    f1 = P['Jp'] * s * cth * wa ** 2 - P['b1'] * w1 - P['tp'] * np.tanh(w1 / E.SIM_EPS1) + P['G'] * s
    m00 = Jarm + P['Jp'] * s * s; m01 = P['h'] * cth; det = m00 * P['Jp'] - m01 * m01
    aa = (P['Jp'] * f0 - m01 * f1) / det
    a1 = (-m01 * f0 + m00 * f1) / det
    return np.stack([wm, wa, w1, am, aa, a1], -1), cur


def simulate_windows(P, U, X0, dt, Vs, sub):
    """U (W, n) duty per sample; X0 (W, 4) measured [th0, th1, w0, w1] at the window start (arm assumed engaged
    with the motor and at the same speed). Returns encoder angle, pendulum angle, current (W, n)."""
    W, n = U.shape
    x = np.zeros((W, 6))
    x[:, 0] = X0[:, 0]; x[:, 1] = X0[:, 0]; x[:, 2] = X0[:, 1]; x[:, 3] = X0[:, 2]; x[:, 4] = X0[:, 2]; x[:, 5] = X0[:, 3]
    out = np.empty((W, n, 3)); h = dt / sub
    for k in range(n):
        d = U[:, k]
        _, cur = rhs_batch(x, d, P, Vs)
        out[:, k] = np.stack([x[:, 0], x[:, 2], cur], -1)
        for _ in range(sub):
            k1, _ = rhs_batch(x, d, P, Vs); k2, _ = rhs_batch(x + .5 * h * k1, d, P, Vs)
            k3, _ = rhs_batch(x + .5 * h * k2, d, P, Vs); k4, _ = rhs_batch(x + h * k3, d, P, Vs)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return out


def windows(preps, seconds):
    fs = float(np.median([p['fs'] for p in preps])); n = int(seconds * fs); U, X0, Y = [], [], []
    for p in preps:
        e = int(fs / 2)
        for a in range(e, len(p['t']) - n - e + 1, n):
            U.append(p['app'][a:a + n]); X0.append([p['th0'][a], p['th1'][a], p['w0'][a], p['w1'][a]])
            Y.append(np.column_stack([p['th0'][a:a + n], p['th1'][a:a + n], p['i'][a:a + n]]))
    return np.array(U), np.array(X0), np.array(Y), fs


def params(base, coupling):
    P = {k: float(v) for k, v in E.full(base).items()}
    P.update({k: float(v) for k, v in coupling.items()})
    return P


def simulate_run(P, p, sub=8):
    """Whole-run simulation from the measured state 0.5 s in (as estimation.simulate); fit % per signal."""
    fs = p['fs']; s0 = min(int(.5 * fs), len(p['t']) // 10)
    U = p['app'][None, s0:]; X0 = np.array([[p['th0'][s0], p['th1'][s0], p['w0'][s0], p['w1'][s0]]])
    out = simulate_windows(P, U, X0, 1 / fs, p['Vs'], sub)[0]
    meas = {'arm angle': p['th0'][s0:], 'pendulum angle': p['th1'][s0:], 'current': p['i'][s0:]}
    sim = {'arm angle': out[:, 0], 'pendulum angle': out[:, 1], 'current': out[:, 2]}
    return {k: E.fit_percent(meas[k], sim[k]) for k in meas}
