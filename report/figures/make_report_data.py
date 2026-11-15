"""Numbers and figures for the parameter-estimation, state-space and stability sections.

Recomputes everything from the raw recordings in data/runs with the Furuta Lab estimation code,
so the report and the data cannot disagree.  Run from the repository root with a Python that has
numpy, scipy and matplotlib:
    python report/figures/make_report_data.py
Outputs: numbers.json, numbers.tex (LaTeX macros) and the fig_*.pdf files in this folder.
"""
import json, math, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / 'desktop'))
from furuta_lab import estimation as E          # noqa: E402
from furuta_lab.backend import load_run          # noqa: E402

NAVY, STEEL, LIGHT, TERRA, GREY = '#143C6E', '#43638B', '#4C87AD', '#CC7451', '#888888'
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['CMU Serif', 'Latin Modern Roman', 'Computer Modern Roman', 'DejaVu Serif'],
                     'mathtext.fontset': 'cm', 'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
                     'ytick.labelsize': 8, 'axes.grid': True, 'grid.color': '#dddddd', 'grid.linewidth': .6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.prop_cycle': matplotlib.cycler(color=[NAVY, TERRA, STEEL, LIGHT]),
                     'savefig.bbox': 'tight', 'savefig.pad_inches': .02, 'lines.linewidth': 1.1})

RUN = lambda s: f'data/runs/{s}'
SWING = [RUN('20261115_100123_523700'), RUN('20261115_100214_541600')]
FIT = [RUN('20261115_100342_554200'), RUN('20261115_093503_954800'), RUN('20261115_093538_842000')]
VAL = [RUN('20261115_100519_238300'), RUN('20261115_101315_017000'), RUN('20261115_104740_966100')]
OTHER = {'Run B (near resonance)': RUN('20261115_100458_924000'), 'Fit C (30 %)': RUN('20261115_104654_672800'), 'Fit D (15 %)': RUN('20261115_104715_495700')}
STALL = RUN('20261115_103729_467500')
DELAY = .007; FP = 10.
_cache = {}


def prep(path, cur=True, delay=DELAY):
    key = (path, cur, delay)
    if key not in _cache:
        rows, m = load_run(REPO / path)
        _cache[key] = E.prepare(rows, m['config'], 12., None, 1, cur, Path(path).name, m, 1496, -1, delay)
    return _cache[key]


def meta(path):
    return load_run(REPO / path)[1]


N = {}
# ---------------------------------------------------------------- free swing
sw_each = [E.fit_freeswing([prep(p, False)], FP) for p in SWING]
sw = E.fit_freeswing([prep(p, False) for p in SWING], FP)
sw_visc = E.fit_freeswing([prep(p, False) for p in SWING], FP, coulomb=False)
N['swing'] = {'wn': sw['wn'], 'fn': sw['freq_hz'], 'zeta': sw['zeta'], 'rJ': sw['rJ'], 'rb': sw['rb'], 'rc': sw['rc'], 'rJ_se': sw['rJ_se'],
              'rb_se': sw['rb_se'], 'rc_se': sw['rc_se'], 'r2': sw['r2'], 'r2_viscous': sw_visc['r2'], 'length': sw['length'],
              'each': [{'wn': s['wn'], 'amp': s['amplitude_deg'], 'rJ': s['rJ'], 'rc_deg': math.degrees(s['rc']), 'arm_counts': s['arm_counts']} for s in sw_each]}
fixed = {k: sw[k] for k in ('rJ', 'rb', 'rc', 'rJ_se', 'rb_se', 'rc_se')}

# ---------------------------------------------------------------- timing scan
best, scan = E.estimate_pend_delay(lambda d: [prep(p, True, d) for p in FIT], fixed, range(0, 16), FP)
N['delay'] = {'best_ms': best, 'scan': scan}

# ---------------------------------------------------------------- least squares
pp = [prep(p) for p in FIT]
el = E.fit_electrical(pp, FP); pe = E.fit_pendulum(pp, FP, fixed); ar = E.fit_arm(pp, pe, FP); P_ls, SE = E.assemble(el, pe, ar)
N['electrical'] = {k: el[k] for k in ('Rm', 'Kb', 'Rm_se', 'Kb_se', 'r2', 'rows')} | {'eq92': el['eq92'],
                   'per_run': [{'Rm': E.fit_electrical([p], FP)['Rm'], 'Kb': E.fit_electrical([p], FP)['Kb']} for p in pp]}
N['pendulum'] = {k: pe[k] for k in ('rh', 'rh_se', 'Larm', 'r2', 'rows')}
N['arm'] = {k: ar[k] for k in ('Ja', 'G', 'Beq', 'tcp', 'tcn', 'Ja_se', 'G_se', 'Beq_se', 'tcp_se', 'tcn_se', 'r2', 'rows')}
N['P_ls'] = P_ls

# ---------------------------------------------------------------- refinement variants and validation
def validate(P):
    out = []
    for v in VAL:
        p = prep(v); o = E.simulate(P, p); r = E.simulate(P, p, 2.)
        out.append({'open': o['fit'], 'resync': r['fit'], 'rms': o['rms']})
    return out


ref_h = E.refine(P_ls, pp, 4., ('h',)); ref = E.refine(P_ls, pp, 4., ('h', 'b1', 'tp'))
ref_all = E.refine(P_ls, pp, 2., ('h', 'Ja', 'Beq', 'tcp', 'tcn', 'b1', 'tp', 'Rm', 'Kb'))
P = ref['params']
N['refine'] = {'change': ref['change_pct'], 'cost_before': ref['cost_before'], 'cost_after': ref['cost_after'], 'windows': ref['windows']}
N['P'] = P; N['SE'] = SE
N['variants'] = {'Least squares only': validate(P_ls), 'Refined: h': validate(ref_h['params']),
                 'Refined: h, b1, tp (final)': validate(P), 'Refined: nine parameters': validate(ref_all['params'])}
N['validation'] = N['variants']['Refined: h, b1, tp (final)']

# ---------------------------------------------------------------- runs table
runs = []
for label, path in [('Free swing 1', SWING[0]), ('Free swing 2', SWING[1])] + [(f'Fit {k}', p) for k, p in zip('123', FIT)] + \
                   [(f'Validation {k}', p) for k, p in zip('123', VAL)] + list(OTHER.items()):
    m = meta(path); md = m['metadata']; p = prep(path, 'swing' not in md.get('estimation_step', ''))
    runs.append({'label': label, 'name': Path(path).name, 'amp': md.get('amplitude_pct'), 'freqs': md.get('freqs_hz'),
                 'pend_max': math.degrees(np.max(np.abs(p['th1'] - np.pi))), 'arm_min': math.degrees(p['th0'].min()), 'arm_max': math.degrees(p['th0'].max()),
                 'i_p99': float(np.percentile(np.abs(p['i']), 99)) if 'swing' not in md.get('estimation_step', '') else 0., 'duration': float(p['t'][-1])})
N['runs'] = runs

# ---------------------------------------------------------------- stall test
rows, m = load_run(REPO / STALL); c = m['config']
i_f = (rows['filtered_mv'] - c['currentZeroMv']) / c['sensitivityMvA']; app = rows['applied'].astype(float); cnt = rows['count'].astype(np.int64)
d = np.diff(np.r_[0, (app != 0).astype(int), 0]); st = []
for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)):
    seg = slice(a + (b - a) // 2, b); V = float(np.median(app[a:b])) * 12; I = float(np.mean(i_f[seg]))
    st.append({'duty': float(np.median(app[a:b])), 'V': V, 'i': I, 'shaft_deg': float((cnt[seg].mean() - cnt[max(0, a - 20):a].mean()) * 360 / c['countsPerRev'])})
Vs_ = np.array([s['V'] for s in st]); Is_ = np.array([s['i'] for s in st])
N['stall'] = {'steps': st, 'R': float(np.linalg.lstsq(Is_[:, None], Vs_, rcond=None)[0][0])}

# ---------------------------------------------------------------- linear models and stability
lin = E.linear_models(P, 12., 1e-3)
def sysd(s):
    return {'A': s['A'].tolist(), 'B': s['B'][:, 0].tolist(), 'Bd': s['Bd'][:, 0].tolist(), 'C': s['C'].tolist(), 'D': s['D'][:, 0].tolist(),
            'poles': [[float(np.real(z)), float(np.imag(z))] for z in s['poles']],
            'tf': {k: {'num': t['num'].tolist(), 'den': t['den'].tolist(), 'zeros': [[float(np.real(z)), float(np.imag(z))] for z in t['zeros']],
                       'poles': [[float(np.real(z)), float(np.imag(z))] for z in t['poles']], 'gain': t['gain']} for k, t in s['tf'].items()},
            'charpoly': np.real(np.poly(s['A'])).tolist(), 'ctrb_rank': s['ctrb_rank'], 'ctrb_cond': s['ctrb_cond'],
            'poles_d': [[float(np.real(z)), float(np.imag(z))] for z in s['poles_d']], 'Ad': s['Ad'].tolist(), 'Bd_v': s['Bd_v'][:, 0].tolist()}
N['upright'] = sysd(lin['upright']); N['hanging'] = sysd(lin['hanging'])
N['friction_duty'] = lin['friction_duty']; N['Beq_prime'] = lin['Beq_prime']; N['Delta'] = lin['Delta']
# Observability with the encoders only (theta0, theta1)
A = lin['upright']['A']; Cm = lin['upright']['C'][:2]
N['obsv_rank'] = int(np.linalg.matrix_rank(np.vstack([Cm @ np.linalg.matrix_power(A, k) for k in range(4)])))

# ---------------------------------------------------------------- friction curve
sw0, = [None]; Ja, G = ar['Ja'], ar['G']; bins = np.array([.3, .6, 1., 1.5, 2., 3., 4., 5., 6.])
allw, allf = [], []
for path in FIT + VAL + list(OTHER.values()):
    p = prep(path); mk = E.masks(p)['moving'] & E.masks(p)['driven']; s_ = np.sin(p['th1']); c_ = np.cos(p['th1']); w0 = p['w0']
    Tr = pe['rJ'] * (s_ * s_ * p['a0'] + 2 * s_ * c_ * w0 * p['w1']) + pe['rh'] * (c_ * p['a1'] - s_ * p['w1'] ** 2)
    f = E.lp(p['i'] - Ja * p['a0'] - G * Tr, p['fs'], FP)
    if 'Fit D' in path or path == OTHER['Fit D (15 %)']:
        pass
    allw.append(w0[mk]); allf.append(f[mk])
w = np.concatenate(allw); f = np.concatenate(allf)
cent, med, q1, q3 = [], [], [], []
for lo, hi in zip(bins[:-1], bins[1:]):
    sel = (np.abs(w) >= lo) & (np.abs(w) < hi)
    if sel.sum() > 30:
        v = f[sel] * np.sign(w[sel]); cent.append((lo + hi) / 2); med.append(np.median(v)); q1.append(np.percentile(v, 25)); q3.append(np.percentile(v, 75))
N['friction_curve'] = {'speed': cent, 'median': med}

# ---------------------------------------------------------------- figures
TW = 6.3   # text width in inches (A4, 2.5 cm margins)
# 1. free swing
p = prep(SWING[0], False); r = sw_each[0]; run = r['runs'][0]; t = p['t']; mk = run['mask']
fig, ax = plt.subplots(figsize=(TW, 2.3))
ax.plot(t, np.degrees(run['phi']), color=NAVY, label='Measured')
ax.plot(t, np.degrees(E.simulate_swing(sw['rJ'], sw['rb'], p, FP, sw['rc'])), '--', color=TERRA, label='Model (viscous + dry friction)')
ax.axvspan(t[0], t[np.flatnonzero(mk)[0]], color='#CCCCFF', alpha=.35, lw=0, label='Not used (held, released)')
ax.set_xlabel('Time [s]'); ax.set_ylabel(r'$\varphi = \theta_1-\pi$ [deg]'); ax.legend(loc='upper right', ncol=3, frameon=False)
ax.set_xlim(t[0], t[np.flatnonzero(mk)[-1]]); fig.savefig(HERE / 'fig_swing.pdf'); plt.close(fig)
# 2. timing scan
fig, ax = plt.subplots(figsize=(TW * .55, 2.3))
ax.plot([s[0] for s in scan], [s[1] for s in scan], 'o-', color=NAVY, ms=3.5)
ax.axvline(best, color=TERRA, ls='--', lw=1); ax.text(best + .3, min(s[1] for s in scan), f'{best:.1f} ms', color=TERRA, fontsize=8, va='bottom')
ax.set_xlabel('Pendulum timing correction [ms]'); ax.set_ylabel(r'$R^2$ [-]'); fig.savefig(HERE / 'fig_delay.pdf'); plt.close(fig)
# 3. friction curve
fig, ax = plt.subplots(figsize=(TW * .55, 2.1))
ax.fill_between(cent, q1, q3, color=LIGHT, alpha=.25, lw=0, label='Interquartile range')
ax.plot(cent, med, 'o-', color=NAVY, ms=3.5, label='Median')
ws = np.linspace(.3, 6, 100); ax.plot(ws, P_ls['tcp'] + P_ls['Beq'] * ws, '--', color=TERRA, label=r'Model $\tau_c+B_{eq}\dot\theta_0$')
ax.set_xlabel(r'Arm speed $|\dot\theta_0|$ [rad/s]'); ax.set_ylabel('Friction current [A]'); ax.legend(frameon=False, loc='lower right')
fig.savefig(HERE / 'fig_friction.pdf'); plt.close(fig)
# 4. validation overlays (validation 1 and 3)
fig, axs = plt.subplots(3, 2, figsize=(TW, 5.0), sharex='col')
for col, (k, title) in enumerate(((0, 'Validation 1'), (2, 'Validation 3'))):
    p = prep(VAL[k]); s = E.simulate(P, p); t = p['t']; ok = np.isfinite(s['sim']['th0'])
    for row, (key, scale, lab) in enumerate((('th0', 180 / math.pi, r'$\theta_0$ [deg]'), ('th1', 180 / math.pi, r'$\theta_1$ [deg]'), ('i', 1., r'$i$ [A]'))):
        ax = axs[row, col]; meas = p[key] * scale; sim = s['sim'][key] * scale
        ax.plot(t[ok], meas[ok], color=NAVY, label='Measured'); ax.plot(t[ok], sim[ok], '--', color=TERRA, label='Model (open loop)')
        if col == 0:
            ax.set_ylabel(lab)
        if row == 0:
            ax.set_title(f'{title}: fit {s["fit"]["arm angle"]:.0f} / {s["fit"]["pendulum angle"]:.0f} / {s["fit"]["current"]:.0f} %', fontsize=9)
    axs[2, col].set_xlabel('Time [s]')
h_, l_ = axs[0, 0].get_legend_handles_labels(); fig.legend(h_, l_, loc='upper center', ncol=2, frameon=False, bbox_to_anchor=(.5, 1.03))
fig.align_ylabels(axs[:, 0]); fig.tight_layout(h_pad=.4, rect=(0, 0, 1, .97)); fig.savefig(HERE / 'fig_validation.pdf'); plt.close(fig)
# 5. pole-zero map
fig, axs = plt.subplots(1, 2, figsize=(TW, 2.5))
for ax, key, title in ((axs[0], 'upright', r'Upright ($\theta_1=0$)'), (axs[1], 'hanging', r'Hanging ($\theta_1=\pi$)')):
    s = lin[key]; pl = s['poles']; z = s['tf']['theta0/V']['zeros']
    ax.axvline(0, color=GREY, lw=.8); ax.axhline(0, color=GREY, lw=.8)
    ax.plot(np.real(pl), np.imag(pl), 'x', color=NAVY, ms=8, mew=1.8, label='Poles of $A$')
    ax.plot(np.real(z), np.imag(z), 'o', mfc='none', color=TERRA, ms=7, mew=1.4, label=r'Zeros of $\theta_0/V$')
    ax.set_title(title, fontsize=9); ax.set_xlabel(r'Re($s$) [rad/s]'); ax.set_xlim(-34, 16); ax.set_ylim(-16, 16)
axs[0].set_ylabel(r'Im($s$) [rad/s]'); axs[0].legend(frameon=False, loc='upper left', fontsize=7.5); fig.tight_layout(); fig.savefig(HERE / 'fig_pz.pdf'); plt.close(fig)

(HERE / 'numbers.json').write_text(json.dumps(N, indent=1, default=float), encoding='utf-8')
print('best delay %.2f ms; final params' % best, {k: '%.4g' % v for k, v in P.items()})
print('validation', [{'open': {k: round(v, 1) for k, v in x['open'].items()}, 'rs2': {k: round(v, 1) for k, v in x['resync'].items()}, 'rms': {k: round(v, 3) for k, v in x['rms'].items()}} for x in N['validation']])
for k, v in N['variants'].items():
    print(k, [(round(x['open']['pendulum angle']), round(x['resync']['arm angle']), round(x['resync']['pendulum angle']), round(x['open']['arm angle'])) for x in v])
print('upright poles', N['upright']['poles']); print('hanging poles', N['hanging']['poles'])
print('zeros theta0/V up', N['upright']['tf']['theta0/V']['zeros'], 'charpoly', N['upright']['charpoly'])
print('stall', N['stall']); print('obsv rank', N['obsv_rank'], 'ctrb', N['upright']['ctrb_rank'], N['upright']['ctrb_cond'])
print('electrical', N['electrical']); print('pendulum', N['pendulum']); print('arm', N['arm']); print('swing', {k: v for k, v in N['swing'].items() if k != 'each'}, N['swing']['each'])
print('runs', [(r['label'], r['amp'], r['freqs'], round(r['pend_max']), round(r['arm_min']), round(r['arm_max']), round(r['i_p99'], 2)) for r in runs])
print('refine', N['refine']); print('friction curve', [(round(a, 2), round(b, 3)) for a, b in zip(cent, med)])
