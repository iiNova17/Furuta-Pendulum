"""Rig acquisition suite (2026-11-20): one step per call, so the person at the rig acts between steps.

    python desktop/rig_suite.py info                      read-only: identity, profile, live angles
    python desktop/rig_suite.py ref arm|down|current      store a reference on the MCU (motor unpowered)
    python desktop/rig_suite.py watch SECONDS             print live angles (positioning aid)
    python desktop/rig_suite.py record NAME SECONDS NOTE  passive recording (motor never armed) into data/runs
    python desktop/rig_suite.py swing NAME SECONDS NOTE   same, but waits for motion (> 3 deg) and records SECONDS from it

Passive steps never arm the driver: Device.inhibit stays True, so ARM/START/START_ONBOARD are refused
by Device._send itself. Powered steps are added separately and need the rig confirmation per batch.
Close the Furuta Lab app first (one owner of the serial port).
"""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from furuta_lab.device import Device  # noqa: E402

PORT = os.environ.get('FURUTA_PORT', 'COM9')   # FTDI FT232R (the ESP32's own USB port does not carry the protocol)
SUITE = 'suite_20261120'


def angles(rows, cfg):
    """Arm angle and pendulum angle (0 = upright, +-180 hanging), degrees, as the controller sees them."""
    th0 = np.degrees(cfg['armSign'] * rows['count'].astype(float) / cfg['countsPerRev'] * 2 * np.pi)
    raw = rows['pend_raw'].astype(float)
    if cfg.get('pendDownRaw') is None or cfg['pendDownRaw'] < 0:
        return th0, np.full(len(rows), np.nan)
    d = (raw - cfg['pendDownRaw']) * 2 * np.pi / 4096 * cfg['pendSign']
    return th0, np.degrees(np.angle(np.exp(1j * (d + np.pi))))


def current_a(rows, cfg):
    return (rows['current_mv'] - cfg['currentZeroMv']) / cfg['sensitivityMvA']


def connect():
    d = Device()
    d.connect(PORT)
    assert d.inhibit, 'motion must stay inhibited in passive steps'
    time.sleep(0.5)
    return d


def show(d, n=250):
    rows = d.snapshot(n)
    c = d.config
    th0, th1 = angles(rows, c)
    i = current_a(rows, c)
    print(f"  state {d.status.get('state')} fault '{d.status.get('fault', '')}' centered {d.status.get('centered')}"
          f"  magnet status {int(rows['mag_status'][-1]) if len(rows) else '-'}")
    if len(rows):
        print(f"  arm {th0[-1]:+7.2f} deg (count {int(rows['count'][-1])})   pendulum {th1[-1]:+7.2f} deg (raw {int(rows['pend_raw'][-1])})"
              f"   current {np.mean(i):+.3f} A   applied duty {float(rows['applied'][-1]):+.3f}")


def cmd_info(d):
    h = d.identity
    keys = ['firmware', 'protocol', 'onboardControl', 'pendHz', 'as5600Conf', 'as5600SlowFilter', 'burstCancel']
    print('identity:', {k: h.get(k) for k in keys})
    c = d.config
    keys = ['countsPerRev', 'armSign', 'pendSign', 'pendDownRaw', 'maxDuty', 'slewPerSecond', 'travelCounts',
            'currentTripA', 'currentTripMs', 'currentChecks', 'currentZeroMv', 'sensitivityMvA', 'heartbeatMs',
            'maxDuration', 'streamHz', 'adcHz']
    print('profile :', {k: c.get(k) for k in keys})
    try:
        p = d.onboard_params()
        print('onboard :', {k: p.get(k) for k in ['Ts', 'K0', 'K1', 'K2', 'K3', 'fricGain', 'fricStick', 'stickGain',
                                                  'aPush', 'wc', 'E0', 'uMax', 'velFc']})
    except Exception as e:  # noqa: BLE001
        print('onboard : not available:', e)
    show(d)


def cmd_ref(d, target):
    assert target in ('arm', 'down', 'current')
    d.calibrate(target)
    if target == 'current':
        time.sleep(1.8)
        d.config = d.rpc('GET_CONFIG')['config']
    time.sleep(0.4)
    print(f'reference {target} stored; currentZeroMv {d.config.get("currentZeroMv")}, pendDownRaw {d.config.get("pendDownRaw")}')
    show(d)


def cmd_watch(d, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        show(d, 50)
        time.sleep(1.0)


def wait_for_motion(d, threshold_deg=3.0, max_wait=120.0):
    """Block until the pendulum or the arm leaves its position at the call by more than threshold_deg."""
    c = d.config
    time.sleep(0.3)
    r0 = d.snapshot(25)
    a0, p0 = angles(r0, c)
    a0, p0 = np.median(a0), np.median(p0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_wait:
        r = d.snapshot(5)
        a, p = angles(r, c)
        dp = np.degrees(np.angle(np.exp(1j * np.radians(p - p0))))
        if np.max(np.abs(dp)) > threshold_deg or np.max(np.abs(a - a0)) > threshold_deg:
            print(f'  motion detected after {time.monotonic() - t0:.1f} s')
            return True
        time.sleep(0.02)
    print('  no motion within the waiting time')
    return False


def cmd_record(d, name, seconds, note, trigger=False):
    if trigger:
        # record from the start (keeps the moments before the motion), stop `seconds` after motion begins
        path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S_') + name)
        meta = {'purpose': 'passive', 'suite': SUITE, 'run': name, 'note': note, 'motor': 'unpowered (never armed)',
                '12v_declared': False, 'gear': 34.0, 'motor_cpr': 44.0, 'trigger': 'pendulum or arm moves > 3 deg'}
        d.start_record(path, meta)
        try:
            print('  recording; waiting for motion (up to 120 s)')
            if wait_for_motion(d):
                time.sleep(seconds)
        finally:
            p = d.finish_record()
        summarize(p)
        return
    _record_fixed(d, name, seconds, note)


def summarize(p):
    m = json.loads((p / 'manifest.json').read_text())
    print(f'saved {p.relative_to(ROOT)}: {m["samples"]} samples, link errors {m["link_errors"]}')
    from furuta_lab.backend import load_run
    rows, man = load_run(p)
    c = man['config']
    th0, th1 = angles(rows, c)
    th1 = np.degrees(np.unwrap(np.radians(th1)))
    t = (rows['device_us'] - rows['device_us'][0]) / 1e6
    print(f'  {len(rows)} rows over {t[-1]:.1f} s; arm [{th0.min():+.1f}, {th0.max():+.1f}] deg;'
          f' pendulum [{th1.min():+.1f}, {th1.max():+.1f}] deg; |applied| max {np.abs(rows["applied"]).max():.3f}')


def _record_fixed(d, name, seconds, note):
    path = ROOT / 'data' / 'runs' / (time.strftime('%Y%m%d_%H%M%S_') + name)
    meta = {'purpose': 'passive', 'suite': SUITE, 'run': name, 'note': note, 'motor': 'unpowered (never armed)',
            '12v_declared': False, 'gear': 34.0, 'motor_cpr': 44.0}
    d.start_record(path, meta)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            time.sleep(min(2.0, seconds - (time.monotonic() - t0)) if seconds - (time.monotonic() - t0) > 0 else 0)
            el = time.monotonic() - t0
            print(f'  {el:5.1f}/{seconds} s')
    finally:
        p = d.finish_record()
    m = json.loads((p / 'manifest.json').read_text())
    print(f'saved {p.relative_to(ROOT)}: {m["samples"]} samples, link errors {m["link_errors"]}')
    sys.path.insert(0, str(ROOT / 'desktop'))
    from furuta_lab.backend import load_run
    rows, man = load_run(p)
    c = man['config']
    th0, th1 = angles(rows, c)
    th1 = np.degrees(np.unwrap(np.radians(th1)))      # no false spread when hanging at the +-180 wrap
    t = (rows['device_us'] - rows['device_us'][0]) / 1e6
    print(f'  {len(rows)} rows over {t[-1]:.1f} s ({len(rows) / max(t[-1], 1e-9):.0f}/s); arm [{th0.min():+.1f}, {th0.max():+.1f}] deg'
          f' std {th0.std():.3f}; pendulum [{np.nanmin(th1):+.1f}, {np.nanmax(th1):+.1f}] deg std {np.nanstd(th1):.3f};'
          f' current mean {current_a(rows, c).mean():+.3f} A std {current_a(rows, c).std():.3f};'
          f' |applied| max {np.abs(rows["applied"]).max():.3f}')


def main(argv):
    if not argv:
        print(__doc__)
        return
    d = connect()
    try:
        if argv[0] == 'info':
            cmd_info(d)
        elif argv[0] == 'ref':
            cmd_ref(d, argv[1])
        elif argv[0] == 'watch':
            cmd_watch(d, float(argv[1]))
        elif argv[0] == 'record':
            cmd_record(d, argv[1], float(argv[2]), ' '.join(argv[3:]))
        elif argv[0] == 'swing':                     # wait for the person to start the motion, then record
            cmd_record(d, argv[1], float(argv[2]), ' '.join(argv[3:]), trigger=True)
        else:
            print(__doc__)
    finally:
        d.close()


if __name__ == '__main__':
    main(sys.argv[1:])
