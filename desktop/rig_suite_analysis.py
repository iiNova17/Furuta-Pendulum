"""Quick analyses of rig_suite recordings (2026-11-20 suite).

    python desktop/rig_suite_analysis.py freeswing RUN_DIR      period, damping, model check
"""
import sys
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, str(Path(__file__).resolve().parent))
from furuta_lab.backend import load_run  # noqa: E402

# report Table 6 (matlab/model/furuta_params.m)
P = dict(Jp=0.0001853092537, G=0.02898730616, b1=0.0001208246778, tp=0.0006378819055)


def pend_from_down(rows, cfg):
    """Pendulum angle from hanging [deg], unwrapped, sign as the model (pendSign)."""
    d = ((rows['pend_raw'].astype(int) - cfg['pendDownRaw'] + 2048) % 4096 - 2048) * 360 / 4096 * cfg['pendSign']
    return np.degrees(np.unwrap(np.radians(d)))


def freeswing(path):
    rows, man = load_run(path)
    c = man['config']
    t = (rows['device_us'] - rows['device_us'][0]) / 1e6
    phi = pend_from_down(rows, c)
    assert np.ptp(rows['count']) == 0, 'arm moved'
    # release: last time the angle is near its extreme before the swing starts
    big = np.abs(phi) > 0.6 * np.abs(phi).max()
    k_rel = np.argmax(big)
    while k_rel + 1 < len(phi) and abs(phi[k_rel + 1]) >= abs(phi[k_rel]) - 0.05:
        k_rel += 1
    s = slice(k_rel, None)
    pk, _ = find_peaks(np.abs(phi[s]), prominence=0.3)
    pk = pk + k_rel
    amp = np.abs(phi[pk])
    tp = t[pk]
    # zero crossings for the period (half periods), robust to quantization
    sgn = np.sign(phi[s])
    zc = np.where(np.diff(sgn) != 0)[0] + k_rel
    tz = t[zc] - phi[zc] * (t[zc + 1] - t[zc]) / (phi[zc + 1] - phi[zc])
    half = np.diff(tz)
    print(f'{Path(path).name}: released at {t[k_rel]:.2f} s from {phi[k_rel]:+.1f} deg; {len(pk)} peaks; '
          f'amplitude {amp[0]:.1f} -> {amp[-1]:.1f} deg in {tp[-1] - tp[0]:.1f} s')
    # period vs amplitude (large swings are slower), small-amplitude period vs the model
    for lo, hi in [(40, 70), (20, 40), (10, 20), (3, 10), (0, 3)]:
        sel = [(k, h) for k, h in enumerate(half) if lo <= np.interp(tz[k], tp, amp) < hi]
        if len(sel) >= 4:
            T = 2 * np.mean([h for _, h in sel])
            print(f'  amplitude {lo:2d}-{hi:2d} deg: period {T:.4f} s ({len(sel)} half periods)')
    w0 = np.sqrt(P['G'] / P['Jp'])
    print(f'  model small-amplitude period (no damping) {2 * np.pi / w0:.4f} s;'
          f' large-amplitude correction at 60 deg x{1 + (np.radians(60) ** 2) / 16:.4f}')
    # decay per period: viscous gives constant ratio, dry friction a constant decrement
    if len(amp) > 6:
        dec = -np.diff(amp)            # amplitude loss per half period (peaks alternate sides)
        mid = 0.5 * (amp[1:] + amp[:-1])
        A = np.vstack([mid, np.ones_like(mid)]).T
        (kv, kc), *_ = np.linalg.lstsq(A, dec, rcond=None)
        # model: per half period, viscous loss = pi*b1/(2*Jp*w)*amp ... loss/amp = pi*zeta; Coulomb loss = 2*tp/G (rad)
        zeta_model = P['b1'] / (2 * np.sqrt(P['G'] * P['Jp']))
        print(f'  loss per half swing = {kv:.4f} x amplitude + {kc:.3f} deg'
              f'  (model: {np.pi * zeta_model:.4f} x amplitude + {np.degrees(2 * P["tp"] / P["G"]):.3f} deg)')


if __name__ == '__main__':
    {'freeswing': freeswing}[sys.argv[1]](sys.argv[2])
