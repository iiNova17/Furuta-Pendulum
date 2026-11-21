"""Performance metrics of onboard runs on the rig (catch transient, steady state, push recovery).

    python desktop/scripts/rig_metrics.py RUN_DIR [RUN_DIR ...]

Definitions follow matlab/simulink_r2026a/lib/harness_metrics.m (brief sec. 4.7) where the rig allows them;
the pendulum band is 5 deg held 1 s (the rig's balance wobble peaks at 5-6.5 deg) instead of the harness's 1 deg.
- catch transient, from the first entry into STABILIZE: pendulum angle at the catch, peak |theta1| in the first
  1.5 s, settling time = first time after which |theta1| stays inside the band for 2 s, arm peak |theta0|.
- steady state, last 5 s of STABILIZE: theta1 rms / mean / peak, arm mean (steady error from the 0 reference) and
  std, duty rms / peak, current rms, % of time at the duty limit.
- pushes (pendulum excursions beyond 9 deg while STABILIZE, at least 1.5 s apart): peak, time back inside the band
  (2 s hold), and whether the pendulum was kept.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
from furuta_lab.backend import load_run  # noqa: E402
import rig_suite as rs  # noqa: E402

BAND = 5.0; HOLD = 1.0; PUSH = 9.0


def settle_time(t, x, t0, band=BAND, hold=HOLD):
    """First time after t0 at which |x| <= band holds for `hold` seconds (relative to t0); NaN if never."""
    inside = np.abs(x) <= band
    k = np.searchsorted(t, t0)
    for i in range(k, len(t)):
        if inside[i]:
            j = np.searchsorted(t, t[i] + hold)
            if j >= len(t):
                return float('nan')
            if inside[i:j].all():
                return float(t[i] - t0)
    return float('nan')


def metrics(path):
    a, m = load_run(path)
    c = m['config']
    t = (a['device_us'] - a['device_us'][0]) / 1e6
    th0, th1 = rs.angles(a, c)
    th1 = np.degrees(np.angle(np.exp(1j * np.radians(th1))))
    i = rs.current_a(a, c); u = a['requested']; ctrl = a['ctrl']
    st = np.where(ctrl == 3)[0]
    out = {'run': Path(path).name, 'controller': m['metadata'].get('controller'),
           'stab': (m['metadata'].get('start') or {}).get('stab', m['metadata'].get('onboard_params', {}).get('stab'))}
    sw = np.where(ctrl == 2)[0]
    out['t_capture'] = float(t[st[0]] - t[sw[0]]) if len(st) and len(sw) else float('nan')
    if not len(st):
        return out
    tc = t[st[0]]
    e = (t >= tc) & (t < tc + 1.5)
    # overshoot: after the hand-over the pendulum crosses upright; overshoot = largest swing past it within 1.5 s,
    # in % of the hand-over angle
    x = th1[e]; z = np.where(np.sign(x) != np.sign(x[0]))[0]
    out['th1_overshoot_pct'] = float(100 * np.abs(x[z]).max() / abs(x[0])) if len(z) else 0.0
    out['th1_t_cross'] = float(t[e][z[0]] - tc) if len(z) else float('nan')
    out.update(th1_at_catch=float(th1[st[0]]), th1_peak_catch=float(np.abs(th1[e]).max()),
               th1_settle_catch=settle_time(t, th1, tc), th0_peak_catch=float(np.abs(th0[e]).max()))
    te = t[st[-1]]
    s = (ctrl == 3) & (t > te - 5)
    out.update(balanced_s=float(te - tc), th1_rms=float(th1[s].std()), th1_mean=float(th1[s].mean()),
               th1_peak=float(np.abs(th1[s]).max()), th0_mean=float(th0[s].mean()), th0_std=float(th0[s].std()),
               u_rms=float(np.sqrt(np.mean(u[s] ** 2))), u_peak=float(np.abs(u[s]).max()),
               i_rms=float(np.sqrt(np.mean(i[s] ** 2))), sat_pct=float(100 * np.mean(np.abs(a['applied'][s]) >= 0.49)))
    # pushes
    pushes = []; last = -9.0
    stab = ctrl == 3
    for k in np.where(stab & (np.abs(th1) > PUSH) & (t > tc + 1.0))[0]:
        if t[k] - last < 1.5:
            continue
        last = t[k]
        w = (t >= t[k]) & (t < t[k] + 1.0)
        kept = bool(stab[(t >= t[k]) & (t < t[k] + 2.5)].all())
        pushes.append(dict(t=float(t[k] - tc), peak=float(np.abs(th1[w]).max()),
                           recover=settle_time(t, th1, t[k]) if kept else float('nan'), kept=kept))
    out['pushes'] = pushes
    return out


if __name__ == '__main__':
    for p in sys.argv[1:]:
        r = metrics(p)
        print(f"== {r['run']} ({r['controller']}, stab {r['stab']}): capture after {r['t_capture']:.2f} s of swing-up")
        if 'th1_at_catch' in r:
            print(f"   catch: pendulum {r['th1_at_catch']:+.1f} deg at hand-over, crosses upright after {r['th1_t_cross']:.2f} s, overshoot {r['th1_overshoot_pct']:.0f} %, "
                  f"settled within {BAND:g} deg after {r['th1_settle_catch']:.2f} s; arm peak {r['th0_peak_catch']:.0f} deg")
            print(f"   steady (last 5 s, balanced {r['balanced_s']:.1f} s): pendulum rms {r['th1_rms']:.2f} deg, mean {r['th1_mean']:+.2f}, "
                  f"peak {r['th1_peak']:.1f} | arm mean {r['th0_mean']:+.1f} deg, std {r['th0_std']:.1f} | duty rms {r['u_rms']:.3f} "
                  f"peak {r['u_peak']:.2f}, at limit {r['sat_pct']:.0f} % | current rms {r['i_rms']:.2f} A")
            for q in r['pushes']:
                print(f"   push at catch+{q['t']:.1f} s: peak {q['peak']:.1f} deg, "
                      + (f"back within {BAND:g} deg after {q['recover']:.2f} s" if q['kept'] else 'LOST (pendulum fell)'))
