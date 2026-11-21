"""Dry run of the onboard controller: armed with the 12 V OFF and the centre rod IN (nothing can move).

    python desktop/rig_suite_dryrun.py --confirmed-12v-off-rod-in [seconds]

Purpose: firmware 1.12's model observer runs inside the 1 ms firmware loop; ONBOARD_BENCH measured 0.55 ms mean,
0.73-0.80 ms worst per step. This checks the real loop: STATUS controlOverruns must not grow during the run.
Swing-up + LQR from the hanging rest (an LQR-only start needs the pendulum upright); with no motor power the
controller stays in swing-up, the heavier law. Arms the driver: only with the confirmation above.
"""
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import rig_suite as rs  # noqa: E402
from furuta_lab.backend import load_run  # noqa: E402


def main(seconds, confirmed):
    if not confirmed:
        raise SystemExit('needs --confirmed-12v-off-rod-in')
    d = rs.connect()
    profile = copy.deepcopy(d.config); path = None
    try:
        time.sleep(1.0)
        st = d.status
        if st.get('state') != 'DISARMED' or st.get('fault') or not st.get('centered'):
            raise RuntimeError(f'not ready: {st}')
        knobs = d.onboard_params()
        print(f"firmware {d.identity.get('firmware')}, estKind {knobs['estKind']}, Ts {knobs['Ts']}, K1 {knobs['K1']:.4f}")
        before = int(d.rpc('STATUS')['status']['controlOverruns'])
        run = copy.deepcopy(profile); run['maxDuration'] = float(seconds); d.apply(run)
        path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S_') + 'dry_observer_timing')
        d.start_record(path, {'purpose': 'dry_run', 'suite': rs.SUITE, 'run': 'dry_observer_timing',
                              'note': 'firmware 1.12 observer timing in the 1 ms loop: armed with 12 V OFF and the centre rod IN',
                              'controller': 'swingup_lqr', 'onboard_params': knobs, '12v_declared': False,
                              'rig_confirmed': '12 V off, centre rod in (person at the rig)'})
        d.inhibit = False
        r = d.run_onboard(swing_up=True)
        print('started:', r.get('mode'), r.get('seconds'), 's')
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds + 2:
            if time.monotonic() - t0 > 0.5 and d.status.get('state') not in ('RUNNING', 'ARMED_IDLE', 'STOPPING'):
                break
            time.sleep(0.05)
    except BaseException:
        d.stop(); raise
    finally:
        d.inhibit = True
        if d.status.get('state') in ('RUNNING', 'ARMED_IDLE'):
            d.stop()
        p = d.finish_record() if path else None
        time.sleep(0.5)
        st = d.rpc('STATUS')['status']
        after = int(st['controlOverruns'])
        if d.status.get('state') == 'DISARMED':
            d.apply(profile)
        print(f"end: state {st['state']} fault '{st.get('fault', '')}'; controlOverruns {before} -> {after} "
              f"({after - before} during the run); maxDuration restored to {d.config.get('maxDuration')}")
        d.close()
    if p:
        a, m = load_run(p)
        ctrl = a['ctrl']; t = (a['device_us'] - a['device_us'][0]) / 1e6
        print(f"recorded {len(a)} rows over {t[-1]:.1f} s; controller states {sorted(set(int(c) for c in ctrl))} "
              f"(2 = swing-up); |requested duty| max {np.abs(a['requested']).max():.2f}; arm moved {np.ptp(a['count'])} counts")


if __name__ == '__main__':
    args = [x for x in sys.argv[1:] if not x.startswith('--')]
    main(float(args[0]) if args else 10.0, '--confirmed-12v-off-rod-in' in sys.argv)
