"""Export the hardware runs used by the final model to .mat files for the MATLAB harness.

Only the recordings the final estimation actually used (data/estimation/model_20261115/session.json,
"use": true) are exported: free swings 1-2, fits 1-3, validations 1-3 and the stall test. Excluded
runs (near-resonance Run B, Fit C, Fit D) and older recordings are refused on purpose.

Each run is read from data/runs/<run>/ (read-only) and converted with the team's own code, exactly as
the final session did: desktop/furuta_lab/backend.load_run + estimation.prepare with the session's
settings (Vs, polarity, pendulum sign, 7 ms pendulum timing correction, automatic 40 Hz zero-phase
filter). One .mat per run in replay/runs/.

    python replay/export_runs.py              # all runs used by the final model
    python replay/export_runs.py <run> ...    # a subset of them

Fields: name, role (swing / fit / validation / stall), t [s], fs [Hz], th0, th1 [rad; th1 = pi
hanging, 0 upright], w0, w1 [rad/s], i [A], app (applied duty, as recorded), Vs, pend_delay_ms, fd.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.io import savemat

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'desktop'))
from furuta_lab import estimation as E  # noqa: E402
from furuta_lab.backend import load_run  # noqa: E402

SESSION = ROOT / 'data' / 'estimation' / 'model_20261115' / 'session.json'
OUT = Path(__file__).resolve().parent / 'runs'


def used_runs(session):
    roles = {}
    for role, recs in session['recordings'].items():
        for r in recs:
            if r.get('use'):
                roles[Path(r['path']).name] = role
    return roles


def main(names):
    s = json.loads(SESSION.read_text(encoding='utf-8'))
    roles = used_runs(s)
    bad = [n for n in names if n not in roles]
    if bad:
        sys.exit('Not used by the final model, refusing: ' + ', '.join(bad))
    names = names or list(roles)
    OUT.mkdir(exist_ok=True)
    fd = float(s['fd']) or None                       # 0 = automatic (40 Hz at 250 Hz)
    pend_sign = int(s['pend_sign']) or None
    for name in names:
        rows, manifest = load_run(ROOT / 'data' / 'runs' / name)
        p = E.prepare(rows, manifest.get('config', {}), float(s['Vs']), fd, int(s['polarity']), True,
                      name, manifest, pend_sign=pend_sign, pend_delay=float(s['pend_delay_ms']) * 1e-3)
        out = {k: np.asarray(p[k], float) for k in ('t', 'th0', 'th1', 'w0', 'w1', 'i', 'app')}
        out.update(name=name, role=roles[name], fs=float(p['fs']), Vs=float(p['Vs']),
                   pend_delay_ms=float(s['pend_delay_ms']), fd=float(p['fd']))
        savemat(OUT / f'{name}.mat', out)
        print(f'{name}: {roles[name]:10s} {p["t"][-1]:6.1f} s at {p["fs"]:.0f} Hz '
              f'-> {OUT.relative_to(ROOT) / (name + ".mat")}')


if __name__ == '__main__':
    main(sys.argv[1:])
