"""Parameter-estimation tests on the simulated bench (furuta_lab/simbench.py). Run through test_lab.py."""
import json
import math
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from scipy import signal

sys.path.insert(0, str(Path(__file__).parent))
from furuta_lab import estimation as E, simbench as B, wire
from furuta_lab.backend import load_run

TRUE = B.true_params()


def err(a, b):
    return abs(a / b - 1)


class EstimationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp()); cls.paths = B.synthetic_session(cls.tmp / 'runs', seed=0)
        cls.prep = {k: cls.load(k) for k in cls.paths}
        fit = [cls.prep['fit_a'], cls.prep['fit_b']]
        cls.swing = E.fit_freeswing([cls.prep['swing']])
        cls.elec = E.fit_electrical(fit); cls.pend = E.fit_pendulum(fit); cls.arm = E.fit_arm(fit, cls.pend)
        cls.params, cls.se = E.assemble(cls.elec, cls.pend, cls.arm)
        cls._refined = None

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    @classmethod
    def load(cls, name, **kw):
        rows, m = load_run(cls.paths[name])
        cfg = dict(m['config'], **kw.pop('config', {}))
        return E.prepare(rows, cfg, 12., need_current=name != 'swing', name=name, manifest=m, **kw)

    @classmethod
    def refined(cls):
        if cls._refined is None:
            cls._refined = E.refine(cls.params, [cls.prep['fit_a'], cls.prep['fit_b']], free=('Ja', 'Beq', 'tcp', 'tcn'))
        return cls._refined

    def test_free_swing(self):
        L = B.lumped(); s = self.swing
        self.assertLess(err(s['rJ'], L['Jp'] / L['G']), .01); self.assertLess(err(s['rb'], L['b1'] / L['G']), .10)
        self.assertLess(s['arm_counts'], 1); self.assertEqual(E.check_swing(s), [])

    def test_free_swing_ignores_hand_before_release(self):
        L = B.lumped(); dt = 1e-3
        t1 = np.arange(0, 1, dt); t2 = np.arange(0, 1, dt); t3 = np.arange(0, 2, dt); t4 = np.arange(0, 18, dt)
        phi = np.r_[np.zeros(len(t1)), .6 * (1 - np.cos(np.pi * t2)) / 2, np.full(len(t3), .6) + np.random.default_rng(5).normal(0, .002, len(t3)),
                    B.sim_swing(t4, B.TRUE, .6) - np.pi]
        t = np.arange(len(phi)) * dt; z = np.zeros(len(t))
        p = E.prepare(B.to_rows(t, z, phi + np.pi, z, z, 250, .12, 3), B.default_config(), 12., need_current=False)
        r = E.fit_freeswing([p]); m = E.masks(p)['swing']
        self.assertLess(err(r['rJ'], L['Jp'] / L['G']), .01); self.assertLess(err(r['rb'], L['b1'] / L['G']), .15)
        self.assertGreater(p['t'][np.flatnonzero(m)[0]], 4.0); self.assertLess(p['t'][np.flatnonzero(m)[0]], 4.6)

    def test_free_swing_dry_friction(self):
        P = dict(B.TRUE, tp=7.4e-4); L = B.lumped(P); t = np.arange(0, 12, 1e-3); z = np.zeros(len(t))
        rows = B.to_rows(t, z, B.sim_swing(t, P, .6), z, z, 250, .12, 4)
        r = E.fit_freeswing([E.prepare(rows, B.default_config(), 12., need_current=False)])
        self.assertLess(err(r['rJ'], L['Jp'] / L['G']), .01); self.assertLess(err(r['rc'], 7.4e-4 / L['G']), .15)
        self.assertLess(err(r['rb'], L['b1'] / L['G']), .5)

    def test_pendulum_timing_estimate_and_fixed_fit(self):
        P = dict(B.TRUE, tp=7.4e-4); L = B.lumped(P); runs = []
        for k, (amp, fr) in enumerate(((30, (1.5, 2., 3., 4., 2.5)), (40, (2., 3., 4., 5., 2.5)))):
            seg = E.square_sweep(amp, fr, 4.); t, app = B.firmware_segments(seg); X, cur = B.sim_furuta(t, app, P)
            runs.append(B.to_rows(t, X[:, 0], X[:, 1], cur, app, 250, .12, k + 7, pend_lag=.006))
        fixed = {'rJ': L['Jp'] / L['G'], 'rb': L['b1'] / L['G'], 'rc': 7.4e-4 / L['G']}
        make = lambda d: [E.prepare(r, B.default_config(), 12., pend_delay=d) for r in runs]
        best, rows = E.estimate_pend_delay(make, fixed)
        self.assertLess(abs(best - 6), 1.5, rows)
        q = E.fit_pendulum(make(best / 1000), fixed=fixed); self.assertEqual(q['source'], 'free swing')
        self.assertLess(err(q['Larm'], B.TRUE['Larm']), .03); self.assertGreater(q['r2'], .95)
        a = E.fit_arm(make(best / 1000), q); params, _ = E.assemble(E.fit_electrical(make(best / 1000)), q, a)
        self.assertLess(err(params['tp'], P['tp'] / P['Kt']), .1)
        self.assertGreater(E.simulate(params, make(best / 1000)[0])['fit']['pendulum angle'], 85)

    def test_assembled_least_squares(self):
        p = self.params
        for k, tol in (('Rm', .03), ('Kb', .03), ('Ja', .08), ('Jp', .06), ('h', .06), ('G', .06), ('b1', .10), ('tcp', .12), ('tcn', .12)):
            self.assertLess(err(p[k], TRUE[k]), tol, k)
        self.assertLess(err(self.pend['Larm'], B.TRUE['Larm']), .02)
        self.assertEqual(E.check_electrical(self.elec), []); self.assertEqual(E.check_pendulum(self.pend, self.swing), [])

    def test_open_circuit_excluded(self):
        p = self.prep['fit_a']; m = E.masks(p)
        self.assertTrue(np.any(p['app'] == 0)); self.assertFalse(np.any(m['driven'] & (p['app'] == 0)))
        self.assertFalse(np.any(m['moving'] & m['resting']))

    def test_refinement_improves_validation(self):
        r = self.refined(); v = self.prep['validation']
        before = E.simulate(self.params, v)['fit']['arm angle']; after = E.simulate(r['params'], v)['fit']['arm angle']
        self.assertGreater(after, before); self.assertGreaterEqual(after, 80)
        self.assertLess(err(r['params']['Ja'], TRUE['Ja']), .04)
        self.assertLess(err(r['params']['tcp'], TRUE['tcp']), .05); self.assertLess(err(r['params']['tcn'], TRUE['tcn']), .05)
        un = lambda q: max(np.real(E.linear_models(q)['upright']['poles']))
        self.assertLess(err(un(r['params']), un(TRUE)), .03)
        self.assertLess(r['cost_after'], r['cost_before'])

    def test_simulator_matches_true_plant(self):
        seg = E.square_sweep(35, (2., 3.), 3.); t, app = B.firmware_segments(seg); X, cur = B.sim_furuta(t, app)
        rows = B.to_rows(t, X[:, 0], X[:, 1], cur, app, 250, sigma_i=0., seed=1)
        p = E.prepare(rows, B.default_config(), 12.)
        for resync in (0., 1.):
            f = E.simulate(TRUE, p, resync)['fit']
            self.assertGreater(f['arm angle'], 95, resync); self.assertGreater(f['pendulum angle'], 95, resync)

    def test_mcu_filter_delay_compensated(self):
        hz = 1000.; sos = signal.butter(2, 40, fs=hz, output='sos')[0]
        cfg = dict(B.default_config(), adcHz=int(hz), filter={'type': 'biquad', 'sos': [float(x) for x in sos], 'alpha': .1, 'window': 8})
        preps = []
        for k, (amp, fr) in enumerate(((30, (1.5, 2., 3., 4., 2.5)), (40, (2., 3., 4., 5., 2.5)))):
            seg = E.square_sweep(amp, fr, 4.); t, app = B.firmware_segments(seg); X, cur = B.sim_furuta(t, app)
            rows = B.to_rows(t, X[:, 0], X[:, 1], cur, app, 250, .12, k + 1, cfg, mcu_filter=sos, adc_hz=hz)
            preps.append(E.prepare(rows, cfg, 12.))
        self.assertIn('MCU filter', preps[0]['current_source']); self.assertGreater(preps[0]['delay_ms'], 4)
        pend = E.fit_pendulum(preps); arm = E.fit_arm(preps, pend)
        self.assertLess(err(arm['Ja'], self.arm['Ja']), .03)

    def test_sign_checks(self):
        p = self.load('fit_a', polarity=-1)
        self.assertTrue(any('polarity' in t for _, t in E.check_polarity([p])))
        q = self.load('fit_a', config={'pendSign': -1}); r = E.fit_pendulum([q])
        self.assertTrue(any('Pendulum sign' in t for _, t in E.check_pendulum(r)))
        q2 = self.load('fit_a', config={'pendSign': -1}, pend_sign=1); self.assertGreater(E.fit_pendulum([q2])['rh'], 0)

    def test_linear_models(self):
        q = dict(TRUE, tcp=0., tcn=0.); lin = E.linear_models(q, 12., 1e-3, 2e-3)
        for key, x0 in (('upright', [0, 0, 0, 0]), ('hanging', [0, math.pi, 0, 0])):
            A = np.zeros((4, 4)); e = 1e-6
            for k in range(4):
                d = np.zeros(4); d[k] = e
                A[:, k] = (np.array(E.rhs(np.add(x0, d), .5, q, 12.)[0]) - np.array(E.rhs(np.subtract(x0, d), .5, q, 12.)[0])) / (2 * e)
            Bn = (np.array(E.rhs(x0, .5 + e, q, 12.)[0]) - np.array(E.rhs(x0, .5 - e, q, 12.)[0])) / (2 * e)
            # rhs input is duty: Bd = Vs * B. Linearise at w0 = 0 with nonzero duty (open-circuit rule off).
            np.testing.assert_allclose(A, lin[key]['A'], rtol=1e-5, atol=1e-6 * np.abs(A).max())
            np.testing.assert_allclose(Bn, lin[key]['Bd'][:, 0], rtol=1e-5, atol=1e-9)
            s = lin[key]
            den = s['tf']['theta0/V']['full_den']
            np.testing.assert_allclose(np.sort_complex(np.roots(den)), np.sort_complex(s['poles']), atol=1e-6)
            Ad, Bd, *_ = signal.cont2discrete((s['A'], s['B'], s['C'], s['D']), 1e-3, 'zoh')
            np.testing.assert_allclose(Ad, s['Ad']); np.testing.assert_allclose(Bd, s['Bd_v'])
        self.assertEqual(lin['upright']['ctrb_rank'], 4); self.assertIn('five_state', lin)
        self.assertEqual(sum(np.real(lin['upright']['poles']) > 1e-9), 1)
        text = E.model_markdown(TRUE, {}, lin, E.scale(TRUE, mode='eta', eta=.7), .12)
        self.assertIn('θ1(s)/V(s)', text); self.assertIn('Unstable pole', text)

    def test_square_sweep(self):
        seg = E.square_sweep(30, (1.5, 2.5, 4.), 4.)
        self.assertEqual(len(seg), 5); self.assertEqual([s['kind'] for s in seg], ['hold', 'square', 'square', 'square', 'hold'])
        for s in seg[1:-1]:
            self.assertAlmostEqual(s['phase'], math.pi / 2); self.assertAlmostEqual(s['duration'] * s['f0'] % 1, 0, places=9)
            self.assertEqual(s['amplitude'], .3)
        self.assertAlmostEqual(sum(s['duration'] for s in seg), 1.5 + 6 / 1.5 + 10 / 2.5 + 16 / 4.)
        t, app = B.firmware_segments(seg, slew=1e9)
        self.assertAlmostEqual(np.mean(app[(t > .5) & (t < .5 + 4)]), 0, places=3)   # centred: no net drive

    def test_added_mass_scale(self):
        L = B.lumped(); t = np.arange(0, 20, 1e-3); z = np.zeros(len(t)); dm, d = .0085, .19
        def rJ(extra, seed):
            th1 = B.sim_swing(t, B.TRUE, .6, extra)
            rows = B.to_rows(t, z, th1, z, z, 250, .12, seed)
            return E.fit_freeswing([E.prepare(rows, B.default_config(), 12., need_current=False)])
        r1 = rJ(None, 1); r2 = rJ((dm, d), 2)
        a = E.added_mass(r1['rJ'], r2['rJ'], dm, d)
        self.assertLess(err(a['G'], L['G']), .03); self.assertLess(err(a['Jp'], L['Jp']), .03)
        s = E.scale(TRUE, {}, 'mass', rJ_plain=r1['rJ'], rJ_loaded=r2['rJ'], dm=dm, d=d)
        self.assertLess(err(s['Kt'], B.TRUE['Kt']), .03); self.assertLess(err(s['eta'], B.TRUE['eta']), .03)
        deq = 9.80665 * r1['rJ']    # mass at the centre of oscillation does not change the frequency
        r3 = rJ((dm, deq), 3)
        s3 = E.scale(TRUE, {}, 'mass', rJ_plain=r1['rJ'], rJ_loaded=r3['rJ'], dm=dm, d=deq)
        self.assertTrue(any('centre of oscillation' in n for n in s3['notes']))
        w = E.scale(TRUE, {}, 'weighed', m=B.TRUE['m'], Lp=B.TRUE['Lp']); self.assertLess(err(w['eta'], B.TRUE['eta']), 1e-9)
        e = E.scale(TRUE, {}, 'eta', eta=B.TRUE['eta']); self.assertLess(err(e['Kt'], w['Kt']), 1e-9)

    def test_export_files(self):
        lin = E.linear_models(self.params); out = self.tmp / 'export'
        paths = E.export(out, self.params, self.se, lin, E.scale(self.params, self.se), self.pend['Larm'])
        self.assertEqual(sorted(p.name for p in paths), ['furuta_model.m', 'furuta_rhs.m', 'parameters.json', 'parameters.tex'])
        j = json.loads((out / 'parameters.json').read_text(encoding='utf-8')); self.assertAlmostEqual(j['params']['Rm'], self.params['Rm'])
        self.assertIn("ss(A_up, B_up", (out / 'furuta_model.m').read_text(encoding='utf-8'))

    def test_session_round_trip_and_stale(self):
        from furuta_lab.estimate_page import Session
        s = Session(self.tmp / 'sess'); s.add('fit', self.paths['fit_a']); s.set_result('electrical', {'Rm': 1.})
        s.set_result('pendulum', {'rh': 1.}); self.assertFalse(s.data['stale'].get('pendulum'))
        s.set_result('electrical', {'Rm': 2.}); self.assertTrue(s.data['stale']['pendulum'])
        s.add('fit', self.paths['fit_b']); self.assertTrue(s.data['stale']['electrical'])
        t = Session(self.tmp / 'sess'); self.assertEqual(t.data['results']['electrical']['Rm'], 2.)
        self.assertEqual(len(t.recordings('fit')), 2); self.assertTrue(t.recordings('fit')[0].exists())

    def test_gui_estimation_walkthrough(self):
        from PySide6.QtWidgets import QApplication, QFileDialog
        from furuta_lab.app import LabApp
        app = QApplication.instance() or QApplication([]); w = LabApp(estimation_root=self.tmp / 'estimation'); page = w.estimate
        def wait():
            t0 = time.time()
            while w.busy and time.time() - t0 < 120:
                app.processEvents(); time.sleep(.01)
            app.processEvents()
        def ok(step):
            text = w.output.toPlainText(); self.assertNotIn('needs attention', text, f'{step}: {text[:400]}')
        w.nav.setCurrentIndex(3)
        page.add_path('swing', self.paths['swing']); page.add_path('fit', self.paths['fit_a']); page.add_path('fit', self.paths['fit_b'])
        page.add_path('validation', self.paths['validation'])
        page.show_prepared('fit', 'driven'); wait(); ok('prepared'); self.assertEqual(w.right_stack.currentIndex(), 2)
        self.assertEqual(len(w.fitview.plots), 6)
        page.fit_swing(); wait(); ok('swing'); self.assertIn('swing', page.session.data['results'])
        page.fit_electrical(); wait(); ok('electrical'); page.fit_pendulum(); wait(); ok('pendulum'); page.fit_arm(); wait(); ok('arm')
        self.assertEqual(page.session.data['model']['source'], 'least squares')
        self.assertIn('Larm', page.status['pendulum'].text())
        page.update_predictions(); self.assertIn('predicted arm swing', page.preset_widgets['A']['info'].text())
        page.compare(); wait(); ok('compare'); self.assertIn('Fit %', w.output.toPlainText())
        self.assertGreater(page.session.data['results']['validate']['fit']['pendulum angle'], 80)
        page.show_model(); ok('model'); self.assertIn('θ0(s)/V(s)', w.output.toPlainText())
        out = self.tmp / 'gui_export'; out.mkdir()
        orig = QFileDialog.getExistingDirectory; QFileDialog.getExistingDirectory = staticmethod(lambda *a, **k: str(out))
        try:
            page.export()
        finally:
            QFileDialog.getExistingDirectory = orig
        self.assertTrue((out / 'furuta_model.m').exists())
        page.scale_mode['eta'].setChecked(True); page.apply_scale(); ok('scale'); self.assertIn('ηg', w.output.toPlainText())
        page.preview('A'); self.assertEqual(len(w.fitview.plots), 1)
        # Refit electrical: later steps become stale.
        page.fit_electrical(); wait(); self.assertTrue(page.session.data['stale']['arm']); self.assertIn('stale', page.status['arm'].text())
        # G1: the dead-zone flag belongs to one run; STOP clears it and other runs never trigger it.
        w.auto_deadzone_path = self.paths['fit_a']; w.stop(); self.assertIsNone(w.auto_deadzone_path)
        rows, m = load_run(self.paths['fit_b']); w.auto_deadzone_path = self.paths['fit_a']
        w.run_loaded((self.paths['fit_b'], (rows, m))); self.assertIsNotNone(w.auto_deadzone_path)
        self.assertEqual(w.run_path, self.paths['fit_b'])
        w.close()

    def test_failed_start_closes_recording(self):
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        app = QApplication.instance() or QApplication([]); w = LabApp(estimation_root=self.tmp / 'estimation2')
        w.device.status = {'state': 'DISARMED'}; w.device.config = dict(B.default_config(), maxDuration=100)
        w.busy = True
        with self.assertRaises(RuntimeError):
            w.start_sequence(E.square_sweep(10, (2.,), 2.), 'estimation')
        self.assertIsNone(w.device.record); w.busy = False; w.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
