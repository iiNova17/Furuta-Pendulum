"""Headless re-identification with the Furuta Lab estimation chain (furuta_lab/estimation.py), same steps and
settings as the app's session data/estimation/claude_loop (the final model of the report):

    Step 1 free swing -> Step 3 electrical -> Step 4 pendulum equation (Jp/G, b1/G, tp/G from the swing)
    -> Step 5 arm equation -> Step 6 simulation refinement of (h, b1, tp), 4 s windows -> validation runs.
    Vs 12 V, polarity +1, pendulum sign -1, pendulum timing correction 7 ms, fp 10 Hz, wmin 0.3 rad/s.

    python desktop/reidentify.py check        reruns the chain on claude_loop's own recordings (must reproduce it)
    python desktop/reidentify.py today        2026-11-20 suite recordings -> data/estimation/suite_20261120/
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab import estimation as E  # noqa: E402
from furuta_lab.backend import load_run  # noqa: E402

OPTS = dict(Vs=12.0, polarity=1, fd=None, fp=10.0, wmin=0.3, cpr=1496.0, pend_sign=-1, pend_delay=0.007)
RUNS = ROOT / 'data' / 'runs'


def prep(path, need_current=True, t_window=None, pend_delay=None):
    rows, man = load_run(path)
    if t_window is not None:
        t = (rows['device_us'] - rows['device_us'][0]) / 1e6
        rows = rows[(t >= t_window[0]) & (t < t_window[1])]
    o = OPTS
    return E.prepare(rows, man.get('config', {}), o['Vs'], o['fd'], o['polarity'], need_current, Path(path).name
                     + (f'[{t_window[0]:.1f}-{t_window[1]:.1f}s]' if t_window else ''), man, o['cpr'], o['pend_sign'],
                     o['pend_delay'] if pend_delay is None else pend_delay)


def identify(swing, fit, val, breakaway_pct=None, refine_free=E.REFINE_FREE, log=print):
    """swing: list of (path, t_window or None); fit, val: lists of paths. Returns a result dict."""
    o = OPTS
    sw_p = [prep(p, False, w) for p, w in swing]
    fit_p = [prep(p) for p in fit]
    sw = E.fit_freeswing(sw_p, o['fp'])
    log(f"step 1 free swing: Jp/G {sw['rJ']:.5g}  b1/G {sw['rb']:.4g}  tp/G {sw['rc']:.4g}  (f {sw['freq_hz']:.4f} Hz, R2 {sw['r2']:.3f})")
    el = E.fit_electrical(fit_p, o['fp'])
    log(f"step 3 electrical: Rm {el['Rm']:.4g} ohm  Kb {el['Kb']:.4g} V s/rad  (R2 {el['r2']:.4f})")
    fixed = {k: sw.get(k, 0.) for k in ('rJ', 'rb', 'rc', 'rJ_se', 'rb_se', 'rc_se')}
    pe = E.fit_pendulum(fit_p, o['fp'], fixed)
    log(f"step 4 pendulum: h/G {pe['rh']:.5g}  Larm {pe['Larm'] * 100:.2f} cm  (R2 {pe['r2']:.3f})")
    ar = E.fit_arm(fit_p, pe, o['fp'], o['wmin'], False)
    log(f"step 5 arm: Ja {ar['Ja']:.4g}  G {ar['G']:.4g}  Beq {ar['Beq']:.4g}  tc+ {ar['tcp']:.4g}  tc- {ar['tcn']:.4g}  (R2 {ar['r2']:.4f})")
    ls, se = E.assemble(el, pe, ar)
    if breakaway_pct:
        ls['tsp'] = breakaway_pct[0] / 100 * o['Vs'] / ls['Rm']
        ls['tsn'] = breakaway_pct[1] / 100 * o['Vs'] / ls['Rm']
    # estimation._windows requires equal stream rates to 1 ppm, but prepare() takes the rate from the MEDIAN timestamp
    # step, which flips between 4000 and 4001 us on this firmware (fitA of 2026-11-20: 249.94 vs 250.00 Hz). Rates
    # within 0.1 % are the same stream rate: use the common one (time-scale error 0.025 %, ~1 ms per 4 s window).
    fs0 = float(np.median([q['fs'] for q in fit_p]))
    for q in fit_p:
        if abs(q['fs'] / fs0 - 1) < 1e-3:
            q['fs'] = fs0
    rf = E.refine(ls, fit_p, 4.0, refine_free)
    log(f"step 6 refine {rf['free']}: cost {rf['cost_before']:.4g} -> {rf['cost_after']:.4g}, change "
        + ', '.join(f'{k} {v:+.1f} %' for k, v in rf['change_pct'].items()) + (' (AT BOUND)' if rf['at_bound'] else ''))
    model = rf['params']
    vals = []
    for p in val:
        vp = prep(p)
        for lab, par in (('least squares', ls), ('refined', model)):
            r = E.simulate(par, vp)
            vals.append(dict(run=Path(p).name, model=lab, fit={k: round(v, 1) for k, v in r['fit'].items()},
                             rms={k: round(v, 4) for k, v in r['rms'].items()}))
            log(f"  validation {Path(p).name[:22]} {lab:13s} fit " + '  '.join(f'{k} {v:.1f}' for k, v in r['fit'].items()))
    return dict(swing=sw, electrical=el, pendulum=pe, arm=ar, ls=ls, se=se, refine=rf, model=model, validation=vals)


def clean(x):
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items() if not isinstance(v, np.ndarray) and k not in ('runs',)}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def claude_loop_inputs():
    s = json.loads((ROOT / 'data/estimation/claude_loop/session.json').read_text())
    import re
    ids = lambda key: [RUNS / re.search(r'(\d{8}_\d{6}_\d+)', r['path']).group(1)  # noqa: E731
                       for r in s['recordings'][key] if r.get('use', True)]          # ticked recordings only, as the app
    return [(p, None) for p in ids('swing')], ids('fit'), ids('validation'), s


def cmd_check():
    swing, fit, val, s = claude_loop_inputs()
    r = identify(swing, fit, val, breakaway_pct=(13, 14))
    ref = s['model']['params']
    print('\nparameter   claude_loop      rerun        diff')
    for k in E.PARAMS + E.STICK:
        print(f'{k:6s}  {ref.get(k, 0):12.6g}  {r["model"].get(k, 0):12.6g}  {100 * (r["model"].get(k, 0) / ref[k] - 1) if ref.get(k) else 0:+8.3f} %')


def suite_runs(tag):
    return sorted(p for p in RUNS.glob(f'20261120_*_{tag}') if p.is_dir())


def cmd_today():
    # free swing: the two releases of b0_freeswing1 (the hand held the pendulum before each release)
    fs = suite_runs('b0_freeswing1')[0]
    swing = [(fs, (10.0, 15.6)), (fs, (29.8, 35.4))]
    fit = [suite_runs(f'b1_fit{x}')[0] for x in 'ABCD']
    val = [suite_runs(t)[0] for t in ('b1_validation', 'b1_validation2', 'b1_validation3')]
    # breakaway: first duty that moved the arm, median over arm positions +2/-24/+31 deg (b1_breakaway_low runs)
    breakaway = (6.0, 6.0)
    print('inputs:', [p.name for p, _ in swing], [p.name for p in fit], [p.name for p in val], f'breakaway {breakaway} %')
    r = identify(swing, fit, val, breakaway_pct=breakaway)
    # 15 Nov with the SAME experiments (fit RunA 25 %, RunB 20 %, FitC, FitD; validations 1-3), breakaway 13/14 %
    print('\n15 Nov, same experiments:')
    o_swing, _, o_val, _ = claude_loop_inputs()
    o_fit = [RUNS / x for x in ('20261115_100342_554200', '20261115_100458_924000', '20261115_104654_672800',
                                '20261115_104715_495700')]
    r_old = identify(o_swing, o_fit, o_val, breakaway_pct=(13, 14))
    old = json.loads((ROOT / 'data/estimation/claude_loop/export/parameters.json').read_text())['params']
    print('\nparameter   claude_loop     15 Nov same set   20 Nov same set   change (same set)')
    for k in E.PARAMS + E.STICK:
        a, b = r_old['model'].get(k, 0), r['model'].get(k, 0)
        print(f'{k:6s}  {old.get(k, 0):13.6g}  {a:15.6g}  {b:15.6g}   {100 * (b / a - 1) if a else 0:+8.1f} %')
    print('\nvalidation (refined model on its own day):')
    for lab, rr in (('15 Nov', r_old), ('20 Nov', r)):
        for v in rr['validation']:
            if v['model'] == 'refined':
                print(f"  {lab} {v['run'][:22]}  " + '  '.join(f'{k} {x:.1f}' for k, x in v['fit'].items()))
    r['same_set_15nov'] = r_old
    out = ROOT / 'data/estimation/suite_20261120'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'result.json').write_text(json.dumps(clean(dict(
        created=time.strftime('%Y-%m-%d %H:%M'), method='desktop/reidentify.py (claude_loop chain and settings)',
        options=OPTS, inputs=dict(swing=[(p.name, w) for p, w in swing], fit=[p.name for p in fit], validation=[p.name for p in val],
                                  breakaway_pct=breakaway), **r)), indent=2))
    print('saved', (out / 'result.json').relative_to(ROOT))


if __name__ == '__main__':
    {'check': cmd_check, 'today': cmd_today}[sys.argv[1]]()
