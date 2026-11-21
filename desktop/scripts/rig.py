"""Command-line access to the rig for the estimation loop (scriptable version of what the GUI does).

    rig.py status
    rig.py setup [--trip 4] [--travel-deg 120] [--max-duty 50] [--max-s 60] [--slew 1000]
    rig.py swing SECONDS [--tag swing|loaded_swing]                     passive, motor never driven
    rig.py run NAME AMP_PCT F1,F2,... [--each 4] [--tag fit|validation] MOVES THE MOTOR
    rig.py restore                                                       100 Hz stream, bypass filter

Recordings go to data/runs and are added to the estimation session given by --session.
Close the Furuta Lab app first (one program can own COM11).
"""
import argparse
import copy
import json
import sys
import time
from pathlib import Path
import numpy as np
from scipy import signal

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from furuta_lab.device import Device
from furuta_lab import estimation as E
from furuta_lab.backend import ROOT


def session_add(name, tag, path):
    f = ROOT / 'data' / 'estimation' / name / 'session.json'
    if not f.exists():
        return
    d = json.loads(f.read_text(encoding='utf-8')); rel = Path(path).resolve().relative_to(ROOT).as_posix()
    lst = d['recordings'].setdefault(tag, [])
    if all(r['path'] != rel for r in lst):
        lst.append({'path': rel, 'use': True})
        f.write_text(json.dumps(d, indent=2), encoding='utf-8')


def connect(port):
    d = Device(); d.connect(port); time.sleep(.3); return d


def summary(d):
    c = d.config; st = d.status
    keys = ['streamHz', 'adcHz', 'countsPerRev', 'armSign', 'pendSign', 'pendDownRaw', 'currentZeroMv', 'maxDuty', 'travelCounts',
            'maxDuration', 'slewPerSecond', 'currentTripA', 'velocityCountsPerSecond', 'deadzonePos', 'deadzoneNeg']
    out = {k: c.get(k) for k in keys}; out['filter'] = (c.get('filter') or {}).get('type')
    a = d.snapshot(50)
    if len(a):
        cpr = c.get('countsPerRev') or 1496
        out['arm_deg_now'] = round(float((int(a['count'][-1]) - int(st.get('centerCount', 0) or 0)) * 360 / cpr), 2)
        if c.get('pendDownRaw') is not None:
            out['pend_from_hanging_deg'] = round(float(((int(a['pend_raw'][-1]) - c['pendDownRaw'] + 2048) % 4096 - 2048) * 360 / 4096), 2)
        out['current_A_now'] = round(float(np.mean((a['current_mv'] - (c.get('currentZeroMv') or 2500)) / c.get('sensitivityMvA', 185))), 3)
    out.update(firmware=d.identity.get('firmware'), state=st.get('state'), fault=st.get('fault'), centered=st.get('centered'))
    return out


def record(d, seconds, meta, tag, session):
    path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000:06d}')
    d.start_record(path, meta); t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(.1)
    p = d.finish_record(); session_add(session, tag, p); return p


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('cmd'); ap.add_argument('args', nargs='*')
    ap.add_argument('--port', default='COM11'); ap.add_argument('--session', default='model_20261115')
    ap.add_argument('--tag', default=None); ap.add_argument('--each', type=float, default=4.)
    ap.add_argument('--trip', type=float, default=4.); ap.add_argument('--travel-deg', type=float, default=120.)
    ap.add_argument('--max-duty', type=float, default=50.); ap.add_argument('--max-s', type=float, default=60.)
    ap.add_argument('--slew', type=float, default=1000.); ap.add_argument('--yes-motor', action='store_true')
    a = ap.parse_args()
    d = connect(a.port)
    try:
        if a.cmd == 'status':
            print(json.dumps(summary(d), indent=1))
        elif a.cmd == 'setup':
            c = copy.deepcopy(d.config); hz = int(c.get('adcHz') or 2000)
            c['streamHz'] = 250; c['filter'] = dict(c.get('filter') or {}, type='biquad', sos=[float(x) for x in signal.butter(2, 40, fs=hz, output='sos')[0]])
            cpr = c.get('countsPerRev') or 1496
            c.update(maxDuty=a.max_duty / 100, travelCounts=int(a.travel_deg / 360 * cpr), maxDuration=a.max_s, slewPerSecond=a.slew / 100,
                     currentTripA=a.trip, assumedBusV=12.)
            d.apply(c); print(json.dumps(summary(d), indent=1))
        elif a.cmd == 'restore':
            c = copy.deepcopy(d.config); c['streamHz'] = 100; c['filter'] = dict(c.get('filter') or {}, type='bypass'); d.apply(c)
            print(json.dumps(summary(d), indent=1))
        elif a.cmd == 'swing':
            seconds = float(a.args[0]); tag = a.tag or 'swing'
            if d.status.get('state') != 'DISARMED':
                raise SystemExit(f'Device is {d.status.get("state")}, not DISARMED')
            p = record(d, seconds, {'purpose': 'free_swing', 'estimation_step': tag, 'source_tool': 'rig.py'}, tag, a.session)
            print('saved', p)
        elif a.cmd in ('run', 'steps'):
            if not a.yes_motor:
                raise SystemExit('Refusing to move the motor without --yes-motor')
            if a.cmd == 'run':
                name, amp, freqs = a.args[0], float(a.args[1]), [float(x) for x in a.args[2].split(',')]
                tag = a.tag or ('validation' if name.lower().startswith('val') else 'fit')
                seg = E.square_sweep(amp, freqs, a.each)
            else:    # steps NAME d1,d2,... : +d hold, rest, -d hold, rest for each level (e.g. arm locked by the rod)
                name, levels = a.args[0], [float(x) for x in a.args[1].split(',')]; freqs = []; amp = max(levels); tag = a.tag or 'stall'
                hold = lambda d, v: {'kind': 'hold', 'duration': d, 'amplitude': v, 'offset': 0., 'f0': 1., 'f1': 3.}
                seg = [hold(.5, 0.)]
                for lv in levels:
                    seg += [hold(a.each, lv / 100), hold(.4, 0.), hold(a.each, -lv / 100), hold(.4, 0.)]
            total = sum(s['duration'] for s in seg)
            st = d.status; c = d.config
            if st.get('state') != 'DISARMED' or not st.get('centered'):
                raise SystemExit(f'Not ready: state {st.get("state")}, centered {st.get("centered")}')
            if (c['maxDuration'] > 0 and total > c['maxDuration']) or amp / 100 > c['maxDuty'] + 1e-9 or not c.get('travelCounts'):
                raise SystemExit('Limits too small: run setup first')
            meta = {'purpose': 'estimation', 'estimation_step': tag, 'preset': name, 'amplitude_pct': amp, 'freqs_hz': freqs,
                    'segments': seg, 'source_tool': 'rig.py', 'setup': 'full_assembly', 'gear': 34, '12v_declared': True}
            path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000:06d}')
            d.start_record(path, meta)
            try:
                time.sleep(.3); d.inhibit = False; d.run(seg, meta)
                t0 = time.monotonic(); seen = False
                while time.monotonic() - t0 < total + 5:
                    s = d.status.get('state')
                    seen = seen or s == 'RUNNING'
                    if seen and s in ('DISARMED', 'FAULT'):
                        break
                    time.sleep(.05)
                time.sleep(.6)
            finally:
                d.stop(); p = d.finish_record()
            session_add(a.session, tag, p)
            print('saved', p, 'final', d.status.get('state'), d.status.get('fault') or '')
            print(json.dumps(summary(d), indent=1))
        else:
            raise SystemExit(__doc__)
    finally:
        d.close()


if __name__ == '__main__':
    main()
