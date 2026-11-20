"""Batch 2 of the 2026-11-20 rig suite: closed-loop baselines of the onboard controller (firmware 1.11).

    python desktop/rig_suite_batch2.py list
    python desktop/rig_suite_batch2.py dry NAME                     knobs + profile check, nothing armed
    python desktop/rig_suite_batch2.py run NAME --confirmed-rod-out-12v-on-area-clear

Swing-up + LQR from the hanging rest, 15 s (profile maxDuration 15 s and a host timer). Knob overrides are RAM
only (ONBOARD_CONFIG); the design defaults and maxDuration 0 are restored after every run, also on errors.
Gains are never changed here. `run` arms the motor: only after the person at the rig confirmed this batch.
"""
import copy
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab.backend import save_capture, load_run  # noqa: E402
import rig_suite as rs  # noqa: E402
from rig_suite_batch1 import settle  # noqa: E402

RUN_S = 15.0
RUNS = {
    'b2_default': dict(knobs={}, note='design defaults: compensation fricGain 1, fricStick 1 (breakaway 13/14 %), 4 ms'),
    'b2_coulomb': dict(knobs={'fricStick': 0}, note='Coulomb-only compensation (7.4 %, about the breakaway measured today)'),
    'b2_nocomp': dict(knobs={'fricGain': 0}, note='no friction compensation'),
    # firmware 1.12 (retune 2026-11-20): model observer (estKind 1, 12 ms), LQR on the 20 Nov model, compensation 6 %
    'b3_new': dict(knobs={}, note='firmware 1.12 defaults: model observer, LQR [30 8 20 30]/0.10 on the 20 Nov model'),
    'b3_new_nocomp': dict(knobs={'fricGain': 0}, note='firmware 1.12, friction compensation off'),
    'b3_new_fric05': dict(knobs={'fricGain': 0.5}, note='firmware 1.12, half friction compensation (on: 2.9 Hz wobble; off: arm wander)'),
}


def onboard_summary(p):
    a, m = load_run(p)
    c = m['config']
    t = (a['device_us'] - a['device_us'][0]) / 1e6
    th0, th1 = rs.angles(a, c)
    i = rs.current_a(a, c)
    ctrl = a['ctrl']
    trans = [(round(float(t[k]), 2), int(ctrl[k])) for k in range(len(a)) if k == 0 or ctrl[k] != ctrl[k - 1]]
    names = {0: 'off', 1: 'SAFE_STOP', 2: 'SWING_UP', 3: 'STABILIZE'}
    print(f'saved {p.relative_to(ROOT)}: {len(a)} rows over {t[-1]:.1f} s; final {m["final_status"].get("state")} '
          f'fault "{m["final_status"].get("fault", "")}"')
    print('  states: ' + ', '.join(f'{tt:.2f} s {names.get(s, s)}' for tt, s in trans))
    driven = np.abs(a['applied']) > 0.1
    if driven.any():
        i_f = (a['filtered_mv'] - c['currentZeroMv']) / c['sensitivityMvA']
        late = driven & (t > 1.0)
        i_abs = np.abs(i_f[late]).mean() if late.any() else np.abs(i_f[driven]).mean()
        print(('  power check ok' if i_abs > 0.1 else '  !!! POWER CHECK FAILED (no motor current; run invalid)')
              + f': mean |current| while driving {i_abs:.2f} A')
    sw = ctrl == 2
    if sw.any():
        print(f'  swing-up: arm {th0[sw].min():+.0f}..{th0[sw].max():+.0f} deg, current peak {np.abs(i[sw]).max():.2f} A')
    st = np.where(ctrl == 3)[0]
    if len(st) > 50:
        k = st[min(len(st) - 1, 250):]           # skip the first second after the catch
        th1u = np.degrees(np.angle(np.exp(1j * np.radians(th1[k]))))
        print(f'  stabilize {t[st[-1]] - t[st[0]]:.1f} s (caught at {t[st[0]]:.2f} s): pendulum rms {th1u.std():.2f} deg, '
              f'mean {th1u.mean():+.2f}; arm mean {th0[k].mean():+.1f} [{th0[k].min():+.0f}, {th0[k].max():+.0f}] deg; '
              f'duty at limit {100 * np.mean(np.abs(a["applied"][k]) >= 0.49):.0f} %; current rms {np.sqrt(np.mean(i[k] ** 2)):.2f} A')
    return m


def restore(d, profile):
    try:
        d.onboard_params(defaults=True)
    except Exception as e:  # noqa: BLE001
        print('  !! could not restore onboard defaults:', e)
    try:
        if d.config.get('maxDuration') != profile.get('maxDuration'):
            if d.status.get('state') == 'DISARMED':
                d.apply(profile)
            else:
                print(f'  !! profile NOT restored (state {d.status.get("state")} {d.status.get("fault", "")}): maxDuration is '
                      f'still {d.config.get("maxDuration")}; clear the fault, then restore maxDuration {profile.get("maxDuration")}')
    except Exception as e:  # noqa: BLE001
        print('  !! could not restore the profile:', e)


def prepare(d, spec):
    st = d.status
    if st.get('state') != 'DISARMED' or st.get('fault') or not st.get('centered'):
        raise RuntimeError(f'not ready: {st.get("state")} fault "{st.get("fault")}" centered {st.get("centered")}')
    c = d.config
    assert c['currentChecks'] and c['currentTripA'] <= 4 and c['travelCounts'] > 0 and c['maxDuty'] <= 0.5
    profile = copy.deepcopy(c)
    d.onboard_params(defaults=True)
    knobs = d.onboard_params(spec['knobs']) if spec['knobs'] else d.onboard_params()
    run_profile = copy.deepcopy(c)
    run_profile['maxDuration'] = RUN_S
    d.apply(run_profile)
    return profile, knobs


def cmd_dry(name):
    d = rs.connect()
    try:
        profile, knobs = prepare(d, RUNS[name])
        print(f'{name}: knobs ok ({ {k: knobs[k] for k in ["Ts", "fricGain", "fricStick", "stickGain", "K1"]} }), '
              f'maxDuration now {d.config["maxDuration"]}')
    finally:
        restore(d, profile if 'profile' in dir() else d.config)
        print('restored: maxDuration', d.config.get('maxDuration'), 'fricStick', d.onboard_params().get('fricStick'))
        d.close()


def power_probe():
    """4 % duty for 0.15 s (below the 5-7.5 % breakaway: the arm does not move) and check that the motor draws
    current. The firmware cannot sense the 12 V bus; twice on 2026-11-20 a run started without motor power."""
    from rig_suite_batch1 import seg, upload
    d = rs.connect()
    try:
        segs = [seg(duration=0.1), seg(duration=0.15, amplitude=0.04), seg(duration=0.1)]
        upload(d, segs)
        d.arm_burst(0.6)
        d.inhibit = False
        d.run(segs, {})
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0 and d.status.get('state') in ('RUNNING', 'ARMED_IDLE', 'STOPPING', 'ARMING'):
            time.sleep(0.05)
        d.inhibit = True
        cap = d.download_burst(timeout=5)
        ad, co = cap['adc'], cap['core']
        c = d.config
        k = np.argmax(co['applied'] != 0)
        if not np.any(co['applied'] != 0):
            return False, 'probe pulse not applied'
        t_on = co['device_us'][k]
        s = (ad['device_us'] >= t_on + 30_000) & (ad['device_us'] < t_on + 140_000)
        i = ((ad['current_mv'][s] - c['currentZeroMv']) / c['sensitivityMvA']).mean()
        moved = int(np.ptp(co['count']))
        expect = 0.04 * 12 / 3.4254
        ok = i > 0.5 * expect
        return ok, f'power probe: {i:.3f} A at 4 % duty (expected about {expect:.2f} A at 12 V), arm moved {moved} counts'
    finally:
        d.inhibit = True
        d.stop() if d.status.get('state') in ('RUNNING', 'ARMED_IDLE') else None
        d.close()


def cmd_run(name, confirmed):
    if not confirmed:
        raise SystemExit('Powered run needs --confirmed-rod-out-12v-on-area-clear')
    spec = RUNS[name]
    ok, msg = settle()
    print(msg)
    if not ok:
        raise SystemExit('NOT STARTED: ' + msg)
    ok, msg = power_probe()
    print(msg)
    if not ok:
        raise SystemExit('NOT STARTED: no motor power. Is the 12 V supply on? ' + msg)
    d = rs.connect()
    path = None; p = None; profile = copy.deepcopy(d.config)
    try:
        profile, knobs = prepare(d, spec)
        d.arm_burst(RUN_S + 0.5)
        path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S_') + name)
        meta = {'purpose': 'onboard', 'suite': rs.SUITE, 'run': name, 'note': spec['note'], 'controller': 'swingup_lqr',
                'start': {'swingUp': True}, 'onboard_params': knobs, 'controller_period_s': knobs['Ts'],
                '12v_declared': True, 'rig_confirmed': 'centre rod out, 12 V on, area clear (person at the rig, batch 2)',
                'gear': 34.0, 'motor_cpr': 44.0}
        d.start_record(path, meta)
        d.inhibit = False
        r = d.run_onboard(swing_up=True)
        print('started:', r.get('mode'), 'for', r.get('seconds'), 's')
        t0 = time.monotonic()
        while time.monotonic() - t0 < RUN_S + 2:
            if d.status.get('state') not in ('RUNNING', 'ARMED_IDLE', 'STOPPING') and time.monotonic() - t0 > 0.5:
                break
            time.sleep(0.05)
        print(f'end: state {d.status.get("state")} fault "{d.status.get("fault", "")}" after {time.monotonic() - t0:.1f} s;'
              f' last event {d.last_event}')
    except BaseException:
        d.stop()
        raise
    finally:
        d.inhibit = True
        if d.status.get('state') in ('RUNNING', 'ARMED_IDLE'):
            d.stop()
        p = d.finish_record() if path else None
        try:
            if p:
                cap = d.download_burst(timeout=10)
                save_capture(p / 'burst', cap, {'purpose': 'onboard_burst', 'run': name, 'suite': rs.SUITE})
                print('high-rate capture saved')
        except Exception as e:  # noqa: BLE001
            print('high-rate capture not saved:', e)
            try:
                d.cancel_burst()
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.3)
        restore(d, profile)
        d.close()
    if p:
        onboard_summary(p)


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a or a[0] == 'list':
        for k, v in RUNS.items():
            print(f'{k:12s} {v["knobs"]} {v["note"]}')
    elif a[0] == 'dry':
        cmd_dry(a[1])
    elif a[0] == 'run':
        cmd_run(a[1], '--confirmed-rod-out-12v-on-area-clear' in a)
