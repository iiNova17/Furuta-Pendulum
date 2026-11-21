"""Tests of the extra twin ghosts (furuta_lab.twin_core): FastResyncGhost, ObserverGhost, ClosedLoopGhost.

    .venv/Scripts/python.exe desktop/test_twin_ghosts.py      (no Qt needed)
"""
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab import twin_core as T  # noqa: E402
from furuta_lab.backend import load_run  # noqa: E402

RUN = ROOT / 'data' / 'runs' / '20261121_051144_b5_lqr'


def run_angles(path):
    """(t [s], applied duty, theta0 [rad], theta1 [rad, 0 up], ctrl) of a recorded run, as the firmware computes them."""
    a, m = load_run(path)
    c = m['config']; st = m.get('status', {})
    t = (a['device_us'] - a['device_us'][0]) * 1e-6
    th0 = c.get('armSign', 1) * (a['count'].astype(float) - float(st.get('centerCount', 0))) * 2 * math.pi / c['countsPerRev']
    th1 = (a['pend_raw'].astype(float) - c['pendDownRaw']) * 2 * math.pi / 4096 * c.get('pendSign', 1) + math.pi
    th1 = (th1 + math.pi) % (2 * math.pi) - math.pi
    return t, a['applied'].astype(float), th0, th1, a['ctrl']


def feed_const(g, th0, th1, seconds, dt=.004, duty=0.):
    out = []
    for k in range(int(seconds / dt)):
        out.append(g.feed(k * dt, duty, th0, th1))
    return out


class GhostTests(unittest.TestCase):
    def test_interface_and_sane_poses(self):
        for g in (T.FastResyncGhost(), T.ObserverGhost(), T.ClosedLoopGhost()):
            self.assertIsNone(g.feed(0., 0., None, None))
            x = g.feed(0., 0., 0., math.pi)
            self.assertEqual(len(x), 4)
            for k in range(1, 50):
                x = g.feed(k * .004, 0., 0., math.pi)
            self.assertTrue(all(math.isfinite(v) for v in x))
            if not isinstance(g, T.ClosedLoopGhost):                 # (the closed-loop ghost starts its own swing-up)
                self.assertLess(abs(T._wrap(x[1] - math.pi)), math.radians(5))  # hanging still stays hanging
            g.reset(); self.assertIsNone(g.x)

    def test_fast_resync(self):
        g = T.FastResyncGhost()
        self.assertAlmostEqual(g.resync_s, .2)
        # measured: pendulum held at 3 deg off upright; the open-loop model falls, but is put back every 0.2 s
        xs = feed_const(g, 0., math.radians(3), 2.)
        dev = [abs(math.degrees(T._wrap(x[1]))) for x in xs]
        self.assertLess(max(dev), 30.)
        slow = T.ModelGhost(resync_s=2.); xs2 = feed_const(slow, 0., math.radians(3), 1.9)
        self.assertGreater(max(abs(math.degrees(T._wrap(x[1]))) for x in xs2), max(dev))

    def test_observer_tracks_recorded_run(self):
        t, d, th0, th1, ctrl = run_angles(RUN)
        g = T.ObserverGhost(); est = []
        for k in range(len(t)):
            est.append(g.feed(t[k], d[k], th0[k], th1[k]))
        est = np.array([e if e is not None else [np.nan] * 4 for e in est])
        bal = np.where(ctrl == 3)[0][250:]
        err1 = np.degrees(np.angle(np.exp(1j * (est[bal, 1] - th1[bal]))))
        err0 = np.degrees(est[bal, 0] - th0[bal])
        rms1 = float(np.sqrt(np.mean(err1 ** 2))); rms0 = float(np.sqrt(np.mean(err0 ** 2)))
        print(f'\nObserverGhost on {RUN.name} while balancing: estimate - measured rms pendulum {rms1:.2f} deg, '
              f'arm {rms0:.2f} deg ({len(bal)} records)')
        self.assertLess(rms1, 3.)
        self.assertLess(rms0, 5.)

    def test_closed_loop_balances_near_upright(self):
        g = T.ClosedLoopGhost(stab=3, resync_s=0)
        g.feed(0., 0., 0., math.radians(3)); g.feed(.004, 0., 0., math.radians(3))
        dev = []
        for k in range(2, 750):                                    # 3 s; measurement no longer used (no re-sync)
            x = g.feed(k * .004, 0., 0., math.radians(3)); dev.append(abs(math.degrees(T._wrap(x[1]))))
        print(f'ClosedLoopGhost LQR from 3 deg: max |theta1| {max(dev):.2f} deg over 3 s, mode {g.mode}')
        self.assertEqual(g.mode, T.ClosedLoopGhost.STAB)
        self.assertLess(max(dev), 10.)

    def test_closed_loop_swings_up_and_catches(self):
        for stab in (3, 1):
            g = T.ClosedLoopGhost(stab=stab, resync_s=0)
            t_catch = None
            for k in range(0, 1500):                               # up to 6 s
                g.feed(k * .004, 0., 0., math.pi)
                if g.mode == T.ClosedLoopGhost.STAB and t_catch is None:
                    t_catch = k * .004
            up = abs(math.degrees(T._wrap(g.x[1])))
            print(f'ClosedLoopGhost stab {stab} from hanging: caught at {t_catch} s, final |theta1| {up:.2f} deg, '
                  f'arm {math.degrees(g.x[0]):+.1f} deg')
            self.assertIsNotNone(t_catch)
            self.assertLess(t_catch, 4.)
            self.assertLess(up, 10.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
