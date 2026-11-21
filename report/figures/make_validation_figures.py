"""Open-loop validation figures of the identified 2026-11-20 model, for the report furuta_model.tex.

Every open-loop run of 2026-11-20 is simulated from its measured initial state with the recorded applied duty and the
final parameters (data/estimation/suite_20261120/export/parameters.json, pendulum timing 10.0 ms), using the same
simulator, preparation and fit definition (NRMSE fit %) as the estimation (desktop/furuta_lab/estimation.py).
Writes here:  fig_val_freeswing.pdf, fig_val_fit.pdf, fig_val_validation.pdf, fig_val_sine.pdf, fig_val_chirp.pdf,
fig_val_breakaway.pdf, val_fits.json, val_fits.csv.

    python report/figures/make_validation_figures.py
(needs matplotlib: the project .venv has none, the system Python 3.11 has it together with numpy and scipy)
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'desktop'))
sys.path.insert(0, str(ROOT / 'desktop' / 'scripts'))
from furuta_lab import estimation as E  # noqa: E402
import reidentify as R  # noqa: E402

NAVY, STEEL, LIGHT, TERRA, GREY, GREEN = '#143C6E', '#43638B', '#4C87AD', '#CC7451', '#888888', '#2d6a3e'
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['CMU Serif', 'Latin Modern Roman', 'Computer Modern Roman', 'DejaVu Serif'],
                     'mathtext.fontset': 'cm', 'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
                     'ytick.labelsize': 8, 'axes.titlesize': 8, 'axes.grid': True, 'grid.color': '#dddddd', 'grid.linewidth': .6,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'savefig.bbox': 'tight', 'savefig.pad_inches': .02, 'lines.linewidth': 1.0})
TW = 6.3
deg = np.degrees

PARAMS = json.loads((ROOT / 'data/estimation/suite_20261120/export/parameters.json').read_text())['params']
VAR = json.loads((ROOT / 'data/estimation/suite_20261120/variants.json').read_text())
RES = json.loads((ROOT / 'data/estimation/suite_20261120/result.json').read_text())
DELAY = VAR['pend_delay_ms'] / 1000          # 10.015 ms, as in the estimation
SWING_WINDOWS = [(10.0, 15.6), (29.8, 35.4)]  # the two releases of b0_freeswing1 used in the estimation

# key, purpose, input, role   (role: fit / validation / breakaway)
RUNS = [
    ('fitA', 'fit run', 'square 25 %, 1, 1.4, 3.5, 4.5 Hz', 'fit'),
    ('fitB', 'fit run', 'square 20 %, 1.8, 2.2, 2.7 Hz', 'fit'),
    ('fitC', 'fit run', 'square 30 %, 1, 3, 4.2 Hz', 'fit'),
    ('fitD', 'fit run, left out', 'square 15 %, 0.8, 1.2, 4, 6 Hz', 'validation'),
    ('validation', 'validation 1', 'square 22 %, 1.1, 3, 4 Hz', 'validation'),
    ('validation2', 'validation 2', 'square 28 %, 1.3, 2.6, 3.8, 5 Hz', 'validation'),
    ('validation3', 'validation 3', 'square 24 %, 0.9, 1.5, 3.3, 4.2 Hz', 'validation'),
    ('sine6_small', 'lag test, small arm motion', 'sine 18 %, 6 Hz', 'validation'),
    ('sine6_large', 'lag test, large arm motion', 'sine 35 %, 6 Hz', 'validation'),
    ('slowsine', 'slow reversals, play', 'sine 18 %, 1 Hz', 'validation'),
    ('chirp', 'frequency response', 'log chirp 25 %, 1.5 to 15 Hz', 'validation'),
    ('breakaway', 'breakaway map, high range', 'alternating pulses 10 to 16 %', 'breakaway'),
    ('breakaway_low', 'breakaway map, low range (position 1)', 'alternating pulses 3 to 10 %', 'breakaway'),
    ('breakaway_low', 'breakaway map, low range (position 2)', 'alternating pulses 3 to 10 %', 'breakaway'),
    ('breakaway_low', 'breakaway map, low range (position 3)', 'alternating pulses 3 to 10 %', 'breakaway'),
]
GROUPS = {
    'fit': ['fitA', 'fitB', 'fitC'],
    'validation': ['validation', 'validation2', 'validation3'],
    'sine': ['sine6_small', 'sine6_large', 'slowsine'],
    'chirp': ['chirp', 'fitD'],
    'breakaway': ['breakaway', 'breakaway_low#1', 'breakaway_low#2', 'breakaway_low#3'],
}


def run_dirs():
    """(label, path) for every run; the three breakaway_low runs in time order as breakaway_low#1..3."""
    out = {}
    for key, *_ in RUNS:
        found = R.suite_runs('b1_' + key)
        if key == 'breakaway_low':
            for i, p in enumerate(found):
                out[f'{key}#{i + 1}'] = p
        else:
            out[key] = found[0]
    return out


def free_swing():
    """Free swing of b0_freeswing1: both releases, simulated with (a) the free-swing identification and (b) the
    parameters of the final model (b1, tp refined on the driven runs)."""
    path = R.suite_runs('b0_freeswing1')[0]
    sw = RES['swing']
    sets = {'free-swing fit': (sw['rJ'], sw['rb'], sw['rc']),
            'final model': (PARAMS['Jp'] / PARAMS['G'], PARAMS['b1'] / PARAMS['G'], PARAMS['tp'] / PARAMS['G'])}
    out = {'windows': [], 'identified': {'wn': sw['wn'], 'freq_hz': sw['freq_hz'], 'zeta': sw['zeta'], 'rJ': sw['rJ'], 'rb': sw['rb'],
                                         'rc': sw['rc'], 'rc_deg': float(np.degrees(sw['rc']))},
           'final': {'rJ': sets['final model'][0], 'rb': sets['final model'][1], 'rc': sets['final model'][2],
                     'wn': 1 / np.sqrt(sets['final model'][0]), 'zeta': sets['final model'][1] / (2 * np.sqrt(sets['final model'][0]))}}
    data = []
    for w in SWING_WINDOWS:
        p = R.prep(path, False, w)
        m = E.masks(p, R.OPTS['fp'])['swing']; idx = np.flatnonzero(m)
        t = p['t']; phi = p['th1'] - np.pi
        sims = {k: E.simulate_swing(*v[:2], p, R.OPTS['fp'], v[2]) for k, v in sets.items()}
        k0 = idx[0]; k1 = idx[-1] + 1          # the estimation's swing mask: release to the end of the free swing
        # th1 is unwrapped and carries a multiple of 2 pi (the pendulum was turned past the top before the window);
        # the model is periodic, so remove it from the measurement and the simulations for plotting and for the peaks
        off = 2 * np.pi * np.round(np.median(phi[k0:]) / (2 * np.pi))
        phi = phi - off; sims = {k: s - off for k, s in sims.items()}
        rec = {'window': list(w), 'fit': {k: E.fit_percent(phi[k0:k1], s[k0:k1]) for k, s in sims.items()}}
        # measured oscillation: zero crossings, peaks
        ph = phi[k0:k1]; tt = t[k0:k1]
        z = np.flatnonzero(np.sign(ph[:-1]) * np.sign(ph[1:]) < 0)
        tz = tt[z] - ph[z] * (tt[z + 1] - tt[z]) / (ph[z + 1] - ph[z])
        half = np.diff(tz)
        pk = np.sort(np.r_[find_peaks(ph, prominence=.01)[0], find_peaks(-ph, prominence=.01)[0]])
        amp = np.abs(ph[pk]); tpk = tt[pk]
        # amplitude-dependent half-period: attribute each half-period to the mean of its two end amplitudes
        per = 2 * half
        mid = .5 * (tz[1:] + tz[:-1])
        a_mid = np.interp(mid, tpk, amp)
        rec.update(amp0_deg=float(deg(amp[0])), peaks_t=(tpk - tt[0]).tolist(), peaks_deg=deg(amp).tolist(),
                   period_mean=float(np.mean(per)), period_small=float(np.mean(per[a_mid < np.radians(20)])) if np.any(a_mid < np.radians(20)) else None,
                   period_first=float(per[0]), period_last=float(per[-1]), n_half=int(len(half)),
                   loss_deg_per_half=float(deg(-np.polyfit(np.arange(len(amp)), amp, 1)[0])),
                   zeta_eff=float(-np.polyfit(np.arange(len(amp)), np.log(amp), 1)[0] / np.pi))
        sim_pk = {}
        for k, s in sims.items():
            q = s[k0:k1]; ok = np.isfinite(q)
            pq = np.sort(np.r_[find_peaks(q[ok], prominence=.01)[0], find_peaks(-q[ok], prominence=.01)[0]])
            sim_pk[k] = (tt[ok][pq] - tt[0], deg(np.abs(q[ok][pq])))
        data.append((t, phi, sims, k0, k1, sim_pk, rec))
        out['windows'].append(rec)
    out['fit_pct_mean'] = {k: float(np.mean([r['fit'][k] for r in out['windows']])) for k in sets}

    # ---- figure
    fig = plt.figure(figsize=(TW, 4.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, .95], hspace=.42, wspace=.28)
    for j, (t, phi, sims, k0, k1, sim_pk, rec) in enumerate(data):
        ax = fig.add_subplot(gs[0, j])
        tt = t - t[k0]
        ax.plot(tt[:k1], deg(phi[:k1]), color=NAVY, label='measured')
        ax.plot(tt[k0:k1], deg(sims['free-swing fit'][k0:k1]), color=TERRA, lw=.9, label='simulation, free-swing fit')
        ax.plot(tt[k0:k1], deg(sims['final model'][k0:k1]), color=GREEN, lw=.9, ls='--', label='simulation, final model')
        ax.set_xlim(-.2, tt[k1]); ax.set_xlabel('Time after release [s]')
        if j == 0:
            ax.set_ylabel(r'$\theta_1-\pi$ [deg]')
        ax.set_title(f'release {j + 1}: fit {rec["fit"]["free-swing fit"]:.1f} % / {rec["fit"]["final model"]:.1f} %', fontsize=8)
    ax = fig.add_subplot(gs[1, :])
    for j, (t, phi, sims, k0, k1, sim_pk, rec) in enumerate(data):
        mk = ['o', 's'][j]
        ax.semilogy(rec['peaks_t'], rec['peaks_deg'], mk, color=NAVY, ms=3.5, mfc='w', ls='none',
                    label=f'measured, release {j + 1}')
        ax.semilogy(*sim_pk['free-swing fit'], '-', color=TERRA, lw=.8, marker=mk, ms=2, label='simulation, free-swing fit' if j == 0 else None)
        ax.semilogy(*sim_pk['final model'], '--', color=GREEN, lw=.8, marker=mk, ms=2, label='simulation, final model' if j == 0 else None)
    ax.set_xlabel('Time after release [s]'); ax.set_ylabel('Peak amplitude [deg]')
    ax.legend(frameon=False, fontsize=7, ncol=2, loc='lower left')
    idn, fin = out['identified'], out['final']
    mp = np.mean([r['period_small'] for r in out['windows'] if r['period_small']])
    txt = (f"measured: period {mp:.4f} s below $20^\\circ$, loss {np.mean([r['loss_deg_per_half'] for r in out['windows']]):.2f}$^\\circ$ per half period\n"
           f"free-swing fit: $f$ = {idn['freq_hz']:.3f} Hz, $\\zeta$ = {idn['zeta']:.3f}, $\\tau_p/G$ = {idn['rc_deg']:.2f}$^\\circ$\n"
           f"final model: $f$ = {fin['wn'] / 2 / np.pi:.3f} Hz, $\\zeta$ = {fin['zeta']:.3f}, $\\tau_p/G$ = {np.degrees(fin['rc']):.2f}$^\\circ$")
    ax.text(.99, .97, txt, transform=ax.transAxes, ha='right', va='top', fontsize=7)
    fig.savefig(HERE / 'fig_val_freeswing.pdf'); plt.close(fig)
    return out


def simulate_runs():
    dirs = run_dirs(); out = {}
    for key, purpose, inp, role in RUNS:
        pass
    seen = {}
    rows = []
    for key, purpose, inp, role in RUNS:
        label = key
        if key == 'breakaway_low':
            seen[key] = seen.get(key, 0) + 1; label = f'{key}#{seen[key]}'
        p = R.prep(dirs[label], True, None, DELAY)
        r = E.simulate(PARAMS, p)
        r2 = E.simulate(PARAMS, p, 2.0)
        out[label] = (p, r)
        rows.append({'run': label, 'folder': dirs[label].name, 'purpose': purpose, 'input': inp, 'role': role,
                     'duration_s': float(p['t'][-1]), 'fs_hz': float(p['fs']),
                     'fit_arm_angle': r['fit']['arm angle'], 'fit_pendulum_angle': r['fit']['pendulum angle'],
                     'fit_arm_speed': r['fit']['arm speed'], 'fit_current': r['fit']['current'],
                     'rms_arm_deg': r['rms']['arm angle (deg)'], 'rms_pend_deg': r['rms']['pendulum angle (deg)'],
                     'rms_current_A': r['rms']['current (A)'],
                     'restart2s_arm_angle': r2['fit']['arm angle'], 'restart2s_pendulum_angle': r2['fit']['pendulum angle'],
                     'restart2s_arm_speed': r2['fit']['arm speed'], 'restart2s_current': r2['fit']['current'],
                     'max_pend_deg': float(deg(np.max(np.abs(p['th1'] - np.pi)))), 'arm_min_deg': float(deg(p['th0'].min())),
                     'arm_max_deg': float(deg(p['th0'].max())), 'max_duty': float(np.max(np.abs(p['app'])))})
        print(f"{label:16s} {rows[-1]['role']:10s} arm {r['fit']['arm angle']:7.1f} pend {r['fit']['pendulum angle']:6.1f} "
              f"w {r['fit']['arm speed']:6.1f} i {r['fit']['current']:6.1f} | 2s: {r2['fit']['arm angle']:6.1f} {r2['fit']['pendulum angle']:6.1f}")
    return out, rows


def overlay(name, labels, sims, rows):
    n = len(labels); breakaway = name == 'breakaway'
    nrow = 3
    fig, axs = plt.subplots(nrow, n, figsize=(TW, 4.3 if not breakaway else 3.8), sharex='col', squeeze=False)
    for j, lab in enumerate(labels):
        p, r = sims[lab]; t = p['t']; f = r['fit']; sim = r['sim']; meas = r['meas']
        row = next(x for x in rows if x['run'] == lab)
        off = 2 * np.pi * np.round(np.median(meas['th1'] - np.pi) / (2 * np.pi))   # unwrapped angle: drop the 2 pi multiple
        series = [(deg(meas['th0']), deg(sim['th0']), r'Arm $\theta_0$ [deg]'),
                  (deg(meas['th1'] - np.pi - off), deg(sim['th1'] - np.pi - off), r'Pendulum $\theta_1-\pi$ [deg]'),
                  (meas['i'], sim['i'], 'Current [A]')]
        if breakaway:
            series[1] = (100 * p['app'], None, 'Applied duty [%]')
        for i, (m, s, yl) in enumerate(series):
            ax = axs[i, j]
            ax.plot(t, m, color=NAVY, lw=.8, label='measured')
            if s is not None:
                ax.plot(t, s, color=TERRA, lw=.8, label='simulation')
            if j == 0:
                ax.set_ylabel(yl, fontsize=8)
            if i == nrow - 1:
                ax.set_xlabel('Time [s]')
        title = lab.replace('_', ' ').replace('#', ' ')
        axs[0, j].set_title(f"{title}\narm {f['arm angle']:.0f} | pend {f['pendulum angle']:.0f} | i {f['current']:.0f} %", fontsize=7)
    axs[0, 0].legend(frameon=False, fontsize=7, loc='best')
    fig.align_ylabels(axs[:, 0]); fig.tight_layout(h_pad=.3, w_pad=.5)
    fig.savefig(HERE / f'fig_val_{name}.pdf'); plt.close(fig)


def main():
    fs = free_swing()
    print('free swing:', json.dumps({k: v for k, v in fs.items() if k != 'windows'}, indent=1))
    for w in fs['windows']:
        print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in w.items() if k not in ('peaks_t', 'peaks_deg')})
    sims, rows = simulate_runs()
    for name, labels in GROUPS.items():
        overlay(name, labels, sims, rows)
    (HERE / 'val_fits.json').write_text(json.dumps({'model': 'data/estimation/suite_20261120/export/parameters.json',
                                                    'pend_delay_ms': DELAY * 1e3, 'free_swing': fs, 'runs': rows}, indent=1))
    with (HERE / 'val_fits.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in r.items()})
    val = [r for r in rows if r['run'] in ('validation', 'validation2', 'validation3')]
    print('mean of validation 1-3:', {k: round(float(np.mean([r[k] for r in val])), 1) for k in
                                       ('fit_arm_angle', 'fit_pendulum_angle', 'fit_arm_speed', 'fit_current')})


if __name__ == '__main__':
    main()
