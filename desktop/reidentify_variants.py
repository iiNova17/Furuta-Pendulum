"""Compare identification variants on the 2026-11-20 suite, judged ONLY on the three validation runs (never fitted).

    python desktop/reidentify_variants.py
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reidentify as R  # noqa: E402
from furuta_lab import estimation as E  # noqa: E402

fs_run = R.suite_runs('b0_freeswing1')[0]
SWING = [(fs_run, (10.0, 15.6)), (fs_run, (29.8, 35.4))]
FIT_ALL = {x: R.suite_runs(f'b1_fit{x}')[0] for x in 'ABCD'}
VAL = [R.suite_runs(t)[0] for t in ('b1_validation', 'b1_validation2', 'b1_validation3')]
BREAKAWAY = (6.0, 6.0)


def score(vals):
    """Mean refined-model fit over the validation runs, per signal, and the mean of the four."""
    rows = [v['fit'] for v in vals if v['model'] == 'refined']
    m = {k: float(np.mean([r[k] for r in rows])) for k in rows[0]}
    m['mean'] = float(np.mean(list(m.values())))
    return m


def run(label, fit_keys, free, delay_ms=7.0):
    R.OPTS['pend_delay'] = delay_ms / 1000
    quiet = lambda *a, **k: None  # noqa: E731
    r = R.identify(SWING, [FIT_ALL[k] for k in fit_keys], VAL, BREAKAWAY, free, log=quiet)
    s = score(r['validation'])
    print(f"{label:46s} " + '  '.join(f"{k[:9]} {v:6.1f}" for k, v in s.items()))
    return r, s


if __name__ == '__main__':
    # pendulum timing correction for today's rig (estimation.estimate_pend_delay, fit runs A-D, free-swing ratios)
    sw = E.fit_freeswing([R.prep(p, False, w) for p, w in SWING], R.OPTS['fp'])
    fixed = {k: sw.get(k, 0.) for k in ('rJ', 'rb', 'rc', 'rJ_se', 'rb_se', 'rc_se')}
    best_ms, curve = E.estimate_pend_delay(lambda dly: [R.prep(FIT_ALL[k], True, None, dly) for k in 'ABCD'], fixed,
                                           range(0, 21), R.OPTS['fp'])
    print(f'pendulum timing correction: best {best_ms:.1f} ms (15 Nov: 7 ms); R2 by ms: '
          + ' '.join(f'{int(ms)}:{r2:.3f}' for ms, r2 in curve[::2]))
    print()
    arm = ('Ja', 'Beq', 'tcp', 'tcn')
    results = {}
    for label, keys, free, dly in [
        ('A: claude_loop recipe (refine h b1 tp), 7 ms', 'ABCD', E.REFINE_FREE, 7.0),
        (f'B: as A, timing {best_ms:.1f} ms', 'ABCD', E.REFINE_FREE, best_ms),
        ('C: refine h b1 tp + Ja Beq tc+ tc-, 7 ms', 'ABCD', E.REFINE_FREE + arm, 7.0),
        (f'D: as C, timing {best_ms:.1f} ms', 'ABCD', E.REFINE_FREE + arm, best_ms),
        (f'E: as D without FitD (15 %)', 'ABC', E.REFINE_FREE + arm, best_ms),
        (f'F: as D, also refine G Jp', 'ABCD', E.REFINE_FREE + arm + ('G', 'Jp'), best_ms),
    ]:
        results[label] = run(label, keys, free, dly)
    best = max(results, key=lambda k: results[k][1]['mean'])
    print('\nbest on validation:', best)
    r = results[best][0]
    old = json.loads((R.ROOT / 'data/estimation/claude_loop/export/parameters.json').read_text())['params']
    for k in E.PARAMS + E.STICK:
        print(f'  {k:5s} {old.get(k, 0):12.5g} -> {r["model"].get(k, 0):12.5g}  ({100 * (r["model"].get(k, 0) / old[k] - 1):+6.1f} %)')
    out = R.ROOT / 'data/estimation/suite_20261120/variants.json'
    out.write_text(json.dumps(R.clean({'pend_delay_ms': best_ms, 'delay_curve': curve,
                                       'scores': {k: v[1] for k, v in results.items()}, 'best': best,
                                       'best_model': r['model'], 'best_refine': r['refine']}), indent=2))
    print('saved', out.relative_to(R.ROOT))
