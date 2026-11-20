"""Batch 1 of the 2026-11-20 rig suite: open-loop powered experiments (pendulum hanging).

    python desktop/rig_suite_batch1.py list                 sequences, model prediction (no device)
    python desktop/rig_suite_batch1.py validate             upload + firmware VALIDATE of every sequence (never arms)
    python desktop/rig_suite_batch1.py run NAME --confirmed-rod-out-12v-on-area-clear
    python desktop/rig_suite_batch1.py chain NAME1 NAME2 ... --confirmed-rod-out-12v-on-area-clear
        (waits for a still pendulum before each run; stops on a fault, a failed power check or a run beyond the model)

`run` arms the motor. Only after the person at the rig confirmed, for this batch, that the centre rod is
removed, 12 V is on and the area is clear (CLAUDE.md). Firmware limits stay as commissioned (travel, max duty,
current trip with current checks on, heartbeat); the script stops the motor on any exception or Ctrl+C.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab import estimation as est  # noqa: E402
from furuta_lab.analysis import deadzone_sequence  # noqa: E402
from furuta_lab.backend import save_capture  # noqa: E402
import rig_suite as rs  # noqa: E402

PARAMS = json.loads((ROOT / 'data/estimation/claude_loop/export/parameters.json').read_text())['params']
SLEW = 100.0          # duty per second (profile slewPerSecond)
TRAVEL_DEG = 510 / 1496 * 360


def seg(kind='hold', duration=0.5, amplitude=0.0, f0=1.0, f1=None, phase=0.0, duty=0.5, **kw):
    s = {'kind': kind, 'duration': float(duration), 'amplitude': float(amplitude), 'offset': 0.0,
         'f0': float(f0), 'f1': float(f0 if f1 is None else f1), 'phase': float(phase), 'duty': float(duty)}
    s.update(kw)
    return s


def sequences():
    S = {}
    bk, _ = deadzone_sequence(16, 0.5, start_pct=10)
    S['b1_breakaway'] = dict(segments=bk, note='breakaway map: alternating 0.25 s pulses 10..16 % in 0.5 % steps, '
                             '0.5 s rest; arm position as left by the person')
    bl, _ = deadzone_sequence(10, 0.5, start_pct=3)
    S['b1_breakaway_low'] = dict(segments=bl, note='breakaway map, low range: alternating 0.25 s pulses 3..10 %, 0.5 s rest '
                                 '(on 2026-11-20 every pulse from 10 % moved the arm)')
    S['b1_chirp'] = dict(segments=[seg(), seg('chirp', 20, 0.25, 1.5, 15, logarithmic=True), seg(duration=1.0)],
                         note='log chirp 1.5 -> 15 Hz, 25 % duty, 20 s: duty->arm and arm->pendulum response')
    S['b1_sine6_small'] = dict(segments=[seg(), seg('sine', 8, 0.18, 6.0), seg(duration=1.0)],
                               note='6 Hz sine 18 % duty: lag source test (small arm motion)')
    S['b1_sine6_large'] = dict(segments=[seg(), seg('sine', 8, 0.35, 6.0), seg(duration=1.0)],
                               note='6 Hz sine 35 % duty: lag source test (large arm motion)')
    S['b1_slowsine'] = dict(segments=[seg(), seg('sine', 8, 0.18, 1.0), seg(duration=1.0)],
                            note='1 Hz sine 18 % duty: slow reversals, gear play')
    S['b1_validation'] = dict(segments=[seg(f0=1.0, f1=3.0),
                                        seg('square', 4 / 1.1, 0.22, 1.1, phase=math.pi / 2),
                                        seg('square', 4.0, 0.22, 3.0, phase=math.pi / 2),
                                        seg('square', 4.0, 0.22, 4.0, phase=math.pi / 2),
                                        seg(duration=1.0, f0=1.0, f1=3.0)],
                              note='repeat of the report validation run 20261115_100519_238300 (Table 4)')
    # exact copies of the experiments the final model (data/estimation/claude_loop) was fitted / validated with
    for name, rid in [('b1_fitA', '20261115_100342_554200'), ('b1_fitB', '20261115_100458_924000'),
                      ('b1_fitC', '20261115_104654_672800'), ('b1_fitD', '20261115_104715_495700'),
                      ('b1_validation2', '20261115_101315_017000'), ('b1_validation3', '20261115_104740_966100')]:
        md = json.loads((ROOT / 'data/runs' / rid / 'manifest.json').read_text())['metadata']
        segs = md['segments']
        if isinstance(segs, str):
            import ast
            segs = ast.literal_eval(segs.replace('nan', 'None'))
        S[name] = dict(segments=segs, note=f'repeat of {md.get("preset")} ({md.get("estimation_step")}, run {rid}) '
                       f'used for the final model: {md.get("amplitude_pct")} % at {md.get("freqs_hz")} Hz')
    return S


LOW_FRICTION = dict(tcp=0.32, tcn=0.32, tsp=0.55, tsn=0.5)   # 2026-11-20 estimate (b1_validation fit, breakaway 6-7.5 %)


def predict_worst(segments):
    """Prediction with Table 6 and with today's lower friction; keeps the larger arm range and current."""
    global PARAMS
    a = predict(segments)
    keep = PARAMS
    PARAMS = {k: v * LOW_FRICTION.get(k, 1.0) for k, v in keep.items()}
    try:
        b = predict(segments)
    finally:
        PARAMS = keep
    w = dict(a)
    w['arm'] = (min(a['arm'][0], b['arm'][0]), max(a['arm'][1], b['arm'][1]))
    for k in ('pend', 'ipk', 'irms'):
        w[k] = max(a[k], b[k])
    return w


def value(s, t):
    """engine.h Segment::value"""
    k, A, f0 = s['kind'], s['amplitude'], s['f0']
    p = 2 * math.pi * f0 * t + s.get('phase', 0)
    if k in ('hold', 'step'):
        v = A
    elif k == 'sine':
        v = A * math.sin(p)
    elif k == 'square':
        z = f0 * t + s.get('phase', 0) / (2 * math.pi); z -= math.floor(z)
        v = A * (1 if z < s.get('duty', 0.5) else -1)
    elif k == 'chirp':
        T, f1 = s['duration'], s['f1']
        cyc = f0 * t + (f1 - f0) * t * t / (2 * T)
        if s.get('logarithmic') and f0 != f1:
            cyc = f0 * T / math.log(f1 / f0) * ((f1 / f0) ** (t / T) - 1)
        v = A * math.sin(2 * math.pi * cyc + s.get('phase', 0))
    else:
        raise ValueError(k)
    return s.get('offset', 0) + v


def render(segments, dt=0.001):
    req = []
    for s in segments:
        n = int(round(s['duration'] / dt))
        req += [value(s, k * dt) for k in range(n)]
    req = np.clip(np.array(req), -0.5, 0.5)
    out = np.zeros_like(req); d = 0.0
    for k, r in enumerate(req):                       # firmware slew limit
        d += np.clip(r - d, -SLEW * dt, SLEW * dt); out[k] = d
    return out


def predict(segments):
    dt = 0.001
    u = render(segments, dt)
    th = est._vec(PARAMS)[None, :]
    X, I = est.sim_batch(th, u[None, :], np.array([[0.0, math.pi, 0.0, 0.0]]), dt, 12.0)
    X, I = X[0, 0], I[0, 0]
    return dict(T=len(u) * dt, arm=(np.degrees(X[:, 0].min()), np.degrees(X[:, 0].max())),
                pend=np.degrees(np.abs(np.angle(np.exp(1j * (X[:, 1] - math.pi))))).max(),
                ipk=np.abs(I).max(), irms=np.sqrt(np.mean(I ** 2)), dmax=np.abs(u).max(),
                iabs_driven=np.abs(I[np.abs(u) > 0.05]).mean() if (np.abs(u) > 0.05).any() else 0.0)


def cmd_list():
    for name, S in sequences().items():
        p = predict_worst(S['segments'])
        print(f"{name:16s} {len(S['segments']):3d} seg {p['T']:5.1f} s | model: arm {p['arm'][0]:+6.1f}..{p['arm'][1]:+6.1f} deg"
              f" (trip at +-{TRAVEL_DEG:.0f}), pendulum swing {p['pend']:5.1f} deg, current peak {p['ipk']:.2f} A"
              f" rms {p['irms']:.2f} A, duty max {p['dmax']:.2f} | {S['note']}")


def upload(d, segments):
    d.rpc('UPLOAD_BEGIN')
    for i, s in enumerate(segments):
        d.rpc('SET_SEGMENT', index=i, segment=s)
    return d.rpc('VALIDATE')


def cmd_validate():
    d = rs.connect()
    try:
        for name, S in sequences().items():
            r = upload(d, S['segments'])
            print(f'{name:16s} firmware VALIDATE ok, duration {r.get("duration"):.2f} s; state {d.status.get("state")}')
    finally:
        d.close()


def cmd_run(name, confirmed):
    if not confirmed:
        raise SystemExit('Powered run needs --confirmed-rod-out-12v-on-area-clear (person at the rig confirmed this batch)')
    S = sequences()[name]
    pred = predict_worst(S['segments'])
    d = rs.connect()
    path = None
    try:
        st = d.status
        if st.get('state') != 'DISARMED' or st.get('fault') or not st.get('centered'):
            raise RuntimeError(f'not ready: {st.get("state")} fault "{st.get("fault")}" centered {st.get("centered")}')
        c = d.config
        assert c['currentChecks'] and c['currentTripA'] <= 4 and c['travelCounts'] > 0 and c['maxDuty'] <= 0.5
        rows = d.snapshot(25)
        th0, th1 = rs.angles(rows, c)
        print(f'start: arm {th0[-1]:+.1f} deg, pendulum {th1[-1]:+.1f} deg')
        r = upload(d, S['segments'])
        T = float(r['duration'])
        d.arm_burst(min(T + 1.0, 35.0))
        path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S_') + name)
        meta = {'purpose': 'open_loop', 'suite': rs.SUITE, 'run': name, 'note': S['note'], 'segments': S['segments'],
                '12v_declared': True, 'rig_confirmed': 'centre rod out, 12 V on, area clear (person at the rig, batch 1)',
                'gear': 34.0, 'motor_cpr': 44.0, 'model_prediction': {k: (list(v) if isinstance(v, tuple) else v)
                                                                      for k, v in pred.items()}}
        d.start_record(path, meta)
        d.inhibit = False                       # enables ARM/START for this run only
        d.run(S['segments'], meta)
        t0 = time.monotonic(); seen_running = False
        while time.monotonic() - t0 < T + 4:
            s = d.status.get('state')
            seen_running |= s in ('RUNNING', 'ARMED_IDLE')
            if seen_running and s not in ('RUNNING', 'ARMED_IDLE', 'STOPPING'):
                break
            if d.status.get('fault'):
                break
            time.sleep(0.05)
        print(f'end: state {d.status.get("state")} fault "{d.status.get("fault", "")}" after {time.monotonic() - t0:.1f} s')
    except BaseException:
        d.stop()
        raise
    finally:
        d.inhibit = True
        d.stop() if d.status.get('state') in ('RUNNING', 'ARMED_IDLE') else None
        p = d.finish_record() if path else None
    try:
        if p:
            cap = d.download_burst(timeout=10)
            save_capture(p / 'burst', cap, {'purpose': 'open_loop_burst', 'run': name, 'suite': rs.SUITE})
            print('high-rate capture saved')
    except Exception as e:  # noqa: BLE001
        print('high-rate capture not saved:', e)
        try:
            d.cancel_burst()
        except Exception:  # noqa: BLE001
            pass
    finally:
        d.close()
    return summarize(p, pred) if p else None


def summarize(p, pred):
    from furuta_lab.backend import load_run
    a, m = load_run(p)
    c = m['config']
    th0, th1 = rs.angles(a, c)
    dev = np.abs(np.angle(np.exp(1j * np.radians(th1 - 180))))
    i = rs.current_a(a, c)
    print(f'saved {p.relative_to(ROOT)}: {len(a)} rows; final {m["final_status"].get("state")} '
          f'fault "{m["final_status"].get("fault", "")}"')
    print(f'  measured: arm {th0.min():+.1f}..{th0.max():+.1f} deg, pendulum swing {np.degrees(dev.max()):.1f} deg, '
          f'current peak {np.abs(i).max():.2f} A rms {np.sqrt(np.mean(i ** 2)):.2f} A, applied max {np.abs(a["applied"]).max():.2f}')
    print(f'  model   : arm {pred["arm"][0]:+.1f}..{pred["arm"][1]:+.1f} deg, pendulum swing {pred["pend"]:.1f} deg, '
          f'current peak {pred["ipk"]:.2f} A rms {pred["irms"]:.2f} A')
    # power check: the firmware cannot sense the 12 V bus. With the supply off the duty is applied but no
    # current flows (run 20261120_195116, supply forgotten). Compare the current in the second half of the
    # run with the model's rms current for the same duty.
    half = slice(len(a) // 2, None)
    driven = np.abs(a['applied'][half]) > 0.05
    power_ok = True
    if driven.any():
        i_f = (a['filtered_mv'][half] - c['currentZeroMv']) / c['sensitivityMvA']   # MCU-filtered, noise ~0.03 A
        i_abs = np.abs(i_f[driven]).mean()
        if i_abs < max(0.08, 0.3 * pred['iabs_driven']):
            print(f'  !!! POWER CHECK FAILED: mean |current| {i_abs:.3f} A while driving (model {pred["iabs_driven"]:.2f} A). '
                  f'Is the 12 V supply on? Treat this run as invalid.')
            power_ok = False
        else:
            print(f'  power check ok (mean |current| while driving {i_abs:.2f} A, model {pred["iabs_driven"]:.2f} A)')

    over = max(pred['arm'][0] - th0.min(), th0.max() - pred['arm'][1])
    return dict(path=p, fault=m['final_status'].get('fault', ''), power_ok=power_ok, arm_over_model_deg=float(over))


def settle(max_wait=60.0, arm_limit=10.0):
    """Wait until the pendulum hangs still (< 1 deg swing over 2 s). Returns (ok, message)."""
    d = rs.connect()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_wait:
            rows = d.snapshot(500)                   # 2 s at 250 Hz
            th0, th1 = rs.angles(rows, d.config)
            swing = np.degrees(np.abs(np.angle(np.exp(1j * np.radians(th1 - 180))))).max()
            if len(rows) >= 500 and swing < 1.0:
                if abs(th0[-1]) > arm_limit:
                    return False, f'arm at {th0[-1]:+.1f} deg: put it back near the centre by hand'
                return True, f'settled: pendulum swing {swing:.2f} deg, arm {th0[-1]:+.1f} deg'
            time.sleep(0.5)
        return False, 'pendulum still swinging after 60 s'
    finally:
        d.close()


def cmd_chain(names, confirmed):
    if not confirmed:
        raise SystemExit('Powered runs need --confirmed-rod-out-12v-on-area-clear')
    for name in names:
        ok, msg = settle()
        print(f'[{name}] {msg}')
        if not ok:
            print('CHAIN STOPPED before', name); return
        r = cmd_run(name, True)
        if r is None or r['fault'] or not r['power_ok'] or r['arm_over_model_deg'] > 25:
            print(f'CHAIN STOPPED after {name}: {r}'); return
    print('CHAIN COMPLETE')


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a or a[0] == 'list':
        cmd_list()
    elif a[0] == 'validate':
        cmd_validate()
    elif a[0] == 'chain':
        cmd_chain([x for x in a[1:] if not x.startswith('--')], '--confirmed-rod-out-12v-on-area-clear' in a)
    elif a[0] == 'run':
        cmd_run(a[1], '--confirmed-rod-out-12v-on-area-clear' in a)
