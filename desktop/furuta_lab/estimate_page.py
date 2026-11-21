"""Parameter estimation tab: guided steps 0-9 (method: report sections 4 to 7).

Every motor run goes through LabApp.start_sequence and its safety gates. Fits run in the
LabApp worker thread; plots go to the FitView, tables and checks to the Messages pane.
"""
import copy
import json
import math
import time
from pathlib import Path
import numpy as np
from scipy import signal
from PySide6 import QtCore, QtWidgets as W

from . import estimation as E
from .backend import ROOT, jsonable, load_run
from .simbench import firmware_segments

FIT_STEPS = ['swing', 'electrical', 'pendulum', 'arm', 'refine', 'model']
STEP_TITLES = {'swing': 'Step 1', 'electrical': 'Step 3', 'pendulum': 'Step 4', 'arm': 'Step 5', 'refine': 'Step 6', 'model': 'Model'}
LIST_DEPENDS = {'swing': ['swing'], 'fit': ['electrical', 'pendulum', 'arm', 'refine', 'model'], 'validation': [], 'loaded_swing': []}
PRESETS = [('A', 'Run A', 25., '1, 1.4, 3.5, 4.5', 'fit'), ('B', 'Run B', 20., '1.2, 2.7, 3.5, 5', 'fit'),
           ('V', 'Validation', 22., '1.1, 3, 4', 'validation')]


class Session:
    """data/estimation/<name>/session.json, written on every change."""

    def __init__(self, folder, on_save=None):
        self.folder = Path(folder); self.file = self.folder / 'session.json'; self.on_save = on_save
        self.data = {'name': self.folder.name, 'created': time.time(), 'Vs': 12., 'polarity': 1, 'pend_sign': 0, 'pend_delay_ms': 0., 'swing_s': 10., 'use_swing': True, 'fd': 0., 'fp': 10., 'wmin': .3,
                     'Lm_mH': 0., 'Ts_ms': 1., 'window_s': 4., 'resync_s': 0., 'symmetric': False,
                     'recordings': {'swing': [], 'fit': [], 'validation': [], 'loaded_swing': []},
                     'results': {}, 'stale': {}, 'model': None, 'scale': {'mode': 'nominal'}, 'saved_settings': None}
        if self.file.exists():
            stored = json.loads(self.file.read_text(encoding='utf-8'))
            self.data.update(stored)
            for k in ('swing', 'fit', 'validation', 'loaded_swing'):
                self.data['recordings'].setdefault(k, [])

    def save(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        self.file.write_text(json.dumps(jsonable(self.data), indent=2), encoding='utf-8')
        if self.on_save:
            self.on_save(self)

    @staticmethod
    def rel(path):
        p = Path(path).resolve()
        try:
            return p.relative_to(ROOT).as_posix()
        except ValueError:
            return str(p)

    @staticmethod
    def abs(rel):
        p = Path(rel)
        return p if p.is_absolute() else ROOT / p

    def recordings(self, key, used_only=True):
        return [Session.abs(r['path']) for r in self.data['recordings'][key] if r.get('use', True) or not used_only]

    def add(self, key, path):
        rel = self.rel(path)
        if any(r['path'] == rel for r in self.data['recordings'][key]):
            return False
        self.data['recordings'][key].append({'path': rel, 'use': True}); self.mark_stale(LIST_DEPENDS[key]); self.save(); return True

    def mark_stale(self, steps):
        for s in steps:
            if s in self.data['results'] or (s == 'model' and self.data['model']):
                self.data['stale'][s] = True

    def set_result(self, step, value):
        self.data['results'][step] = value; self.data['stale'][step] = False
        if step in FIT_STEPS:
            self.mark_stale(FIT_STEPS[FIT_STEPS.index(step) + 1:])
        self.save()


class EstimatePage(QtCore.QObject):
    def __init__(self, app, root=None):
        super().__init__()
        self.app = app; self.root = Path(root) if root else ROOT / 'data' / 'estimation'
        self.cache = {}; self.last = {}; self.passive = None; self.session = None; self.dz_applied = False
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.passive_tick)

    # ------------------------------------------------------------------ layout
    def build(self, v):
        a = self.app
        g = a.group(v, 'Session')
        self.session_label = a.text(g, 'Session: none')
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'New session…', self.new_session, action=False); a.button(row, 'Open session…', self.open_session, action=False)
        a.text(g, 'Everything is done on the assembled rig: free swing, square-wave runs, fits, refinement, comparison, model. '
                  'Only Vs and the encoder scale (gear ratio) are entered.', small=True)

        g = a.group(v, 'Step 0 · Setup')
        self.vs = a.number(g, 'Motor supply Vs (V)', 12, .1, 30, 2)
        row = W.QHBoxLayout(); row.addWidget(W.QLabel('Current polarity'), 1); self.polarity = W.QComboBox(); self.polarity.addItems(['+1', '−1'])
        self.polarity.setMaximumWidth(110); row.addWidget(self.polarity); g.addLayout(row)
        row = W.QHBoxLayout(); row.addWidget(W.QLabel('Pendulum sign used for fitting'), 1); self.pend_sign = W.QComboBox()
        self.pend_sign.addItems(['from each recording', '+1', '−1']); self.pend_sign.setMaximumWidth(170); row.addWidget(self.pend_sign); g.addLayout(row)
        a.button(g, 'Reverse the pendulum direction on the device (pendSign)', self.flip_device_pend_sign)
        self.pend_delay = a.number(g, 'Pendulum timing correction (ms): angle lags the arm by', 0, 0, 50, 1)
        a.button(g, 'Estimate the pendulum timing from the fit runs', self.estimate_delay)
        self.lm = a.number(g, 'Armature inductance Lm (mH, optional, 0 = none)', 0, 0, 1000, 3)
        self.scale_label = a.text(g, 'Encoder scale: —', small=True)
        self.checklist = a.text(g, 'Checklist: connect the device.', small=True)
        a.button(g, 'Apply estimation recording settings', self.apply_settings)
        a.button(g, 'Restore normal settings', self.restore_settings)
        a.text(g, 'Estimation settings: 250 Hz stream and a 40 Hz Butterworth anti-alias filter on the MCU current. '
                  'The overcurrent trip uses the filtered current.', small=True)
        a.text(g, 'Advanced (defaults are fine):', small=True)
        self.fd = a.number(g, 'Signal filter fd (Hz, 0 = auto: 40 at 250 Hz)', 0, 0, 200, 1)
        self.fp = a.number(g, 'Equation filter fp (Hz)', 10, 1, 100, 1)
        self.wmin = a.number(g, 'Moving threshold ω min (rad/s)', .3, .01, 10, 2)
        for w in (self.vs, self.lm, self.fd, self.fp, self.wmin, self.pend_delay):
            w.valueChanged.connect(self.options_changed)
        self.polarity.currentIndexChanged.connect(self.options_changed); self.pend_sign.currentIndexChanged.connect(self.options_changed)

        g = a.group(v, 'Step 1 · Free swing — arm held, motor off')
        self.swing_s = a.number(g, 'Free-swing record length (s)', 10, 2, 120, 0)
        self.swing_s.valueChanged.connect(lambda *_: self.save_ui())
        a.text(g, 'Hold the arm still by hand. Press Record, lift the pendulum 30–40° away from hanging straight down (θ1 ≈ 140–150°) '
                  'and let go within a few seconds; keep holding the arm until the recording ends. No motor power is used. '
                  'The part before you let go is left out of the fit automatically (grey in the plots).')
        self.swing_button = a.button(g, 'Record free swing (passive)', lambda: self.record_passive('swing', self.swing_s.value()))
        self.lists = {}
        self.rec_list(g, 'swing', 'Free-swing recordings', 'swing')
        a.button(g, 'Fit free swing', self.fit_swing)
        self.status = {}; self.status['swing'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 2 · Square-wave runs — pendulum hanging, MOVES MOTOR')
        a.text(g, 'Centred square waves (the arm swings about its start). Run A and B are used for fitting, the validation run only for Step 7. '
                  'Each run needs “Maximum run seconds” and “Maximum duty” (Motor tests step 2) at least as large as shown.')
        self.seconds_each = a.number(g, 'Seconds per frequency', 4, .5, 60, 1)
        self.preset_widgets = {}
        for key, title, amp, freqs, target in PRESETS:
            box = W.QGroupBox(title); bl = W.QVBoxLayout(box); g.addWidget(box)
            amp_w = a.number(bl, 'Amplitude (% duty)', amp, .1, 100, 1)
            row = W.QHBoxLayout(); row.addWidget(W.QLabel('Frequencies (Hz)'), 1); fr = W.QLineEdit(freqs); fr.setMaximumWidth(170); row.addWidget(fr); bl.addLayout(row)
            info = a.text(bl, '', small=True)
            row = W.QHBoxLayout(); bl.addLayout(row)
            a.button(row, 'Preview', lambda k=key: self.preview(k))
            run = a.button(row, f'{title} — MOVES MOTOR', lambda k=key: self.run_preset(k))
            self.preset_widgets[key] = dict(amp=amp_w, freqs=fr, info=info, run=run, target=target, title=title)
            amp_w.valueChanged.connect(self.update_predictions); fr.textChanged.connect(self.update_predictions)
        self.seconds_each.valueChanged.connect(self.update_predictions)
        self.rec_list(g, 'fit', 'Fit runs (A, B)', 'driven')
        self.rec_list(g, 'validation', 'Validation runs', 'driven')

        g = a.group(v, 'Step 3 · Electrical fit  i = V/Rm − (Kb/Rm)·θ̇0')
        a.text(g, 'Uses the fit runs, driven samples only (at zero duty the driver is open-circuit).', small=True)
        a.button(g, 'Fit Rm and Kb', self.fit_electrical); self.status['electrical'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 4 · Pendulum-equation fit')
        a.text(g, 'sin θ1 = (h/G)·cos θ1·θ̈0 + (Jp/G)·(θ̈1 − sin θ1·cos θ1·θ̇0²) + (b1/G)·θ̇1 + (τp/G)·sgn θ̇1  →  Larm = g·h/G', small=True)
        self.use_swing = W.QCheckBox('Use the free-swing Jp/G, damping and dry friction; fit only the coupling h/G (recommended)')
        self.use_swing.setChecked(True); g.addWidget(self.use_swing); self.use_swing.toggled.connect(lambda *_: self.save_ui())
        a.button(g, 'Fit pendulum equation', self.fit_pendulum); self.status['pendulum'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 5 · Arm-equation fit')
        a.text(g, 'i = Ja·θ̈0 + G·Tr + Beq·θ̇0 + τc±·sgn θ̇0 (per Kτ). Moving samples only.', small=True)
        self.symmetric = W.QCheckBox('One Coulomb friction for both directions'); g.addWidget(self.symmetric)
        a.button(g, 'Fit arm equation', self.fit_arm); self.status['arm'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 6 · Refine by simulation')
        a.text(g, 'Adjusts the ticked parameters so the simulated runs match the recordings (4 s windows from the measured state). '
                  'Default: the coupling h and the pendulum damping b1, τp (best on held-out runs). '
                  'Judge the result on the validation runs (Step 7), not on the fit runs.', small=True)
        row = W.QHBoxLayout(); g.addLayout(row); self.refine_free = {}
        for key, label, on in (('h', 'h', True), ('b1', 'b1', True), ('tp', 'τp', True), ('Ja', 'Ja', False), ('Beq', 'Beq', False), ('tcp', 'τc+', False), ('tcn', 'τc−', False)):
            c = W.QCheckBox(label); c.setChecked(on); row.addWidget(c); self.refine_free[key] = c
        row.addStretch()
        self.window_s = a.number(g, 'Window length (s)', 4, 1, 30, 1)
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Show least-squares model on fit runs', self.show_ls_sim); a.button(row, 'Refine', self.refine)
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Accept refined model', lambda: self.accept_model('refined')); a.button(row, 'Keep least-squares model', lambda: self.accept_model('least squares'))
        self.status['refine'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 7 · Model against hardware')
        a.text(g, 'The model is driven by the recorded duty and compared with the measured signals. Any run can be used, '
                  'including later closed-loop runs.', small=True)
        row = W.QHBoxLayout(); row.addWidget(W.QLabel('Recording'), 0); self.compare_combo = W.QComboBox(); row.addWidget(self.compare_combo, 1); g.addLayout(row)
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Add recording…', lambda: self.add_folder('validation')); a.button(row, 'Add the open recording', lambda: self.add_open('validation'))
        self.resync = a.number(g, 'Re-sync to the measured state every (s, 0 = whole run)', 0, 0, 600, 1)
        a.button(g, 'Compare model with hardware', self.compare); self.status['validate'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 8 · Model: state space and transfer functions')
        self.ts = a.number(g, 'Discrete sample time Ts (ms)', 1, .01, 1000, 3)
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Show model', self.show_model); a.button(row, 'Export model files…', self.export)
        self.status['model'] = a.text(g, '', small=True)

        g = a.group(v, 'Step 9 · Physical units (optional)')
        a.text(g, 'Models, poles and the simulation do not change. Only the SI values (kg·m², N·m) need a reference.', small=True)
        self.scale_mode = {}
        for key, label in (('nominal', 'Nominal: Kτ = Kb (ηg = 1)'), ('mass', 'Known added mass (second free swing)'),
                           ('weighed', 'Weighed pendulum (m and balance point Lp)'), ('eta', 'Assumed gearbox efficiency ηg')):
            r = W.QRadioButton(label); g.addWidget(r); self.scale_mode[key] = r
        self.scale_mode['nominal'].setChecked(True)
        self.dm = a.number(g, 'Added mass (g)', 10, .01, 10000, 3); self.dm_tol = a.number(g, '  tolerance (g)', .1, 0, 1000, 3)
        self.dpos = a.number(g, 'Mass distance from the pivot (cm)', 15, .1, 200, 2); self.dpos_tol = a.number(g, '  tolerance (cm)', .1, 0, 50, 2)
        self.mass_button = a.button(g, 'Record a free swing with the mass fitted (passive)', lambda: self.record_passive('loaded_swing', self.swing_s.value()))
        self.rec_list(g, 'loaded_swing', 'Free swings with the added mass', 'swing')
        self.wm = a.number(g, 'Weighed pendulum mass (g)', 50, .01, 100000, 2); self.wlp = a.number(g, 'Pivot to balance point Lp (cm)', 10, .01, 200, 2)
        self.eta = a.number(g, 'Assumed ηg', .8, .01, 1, 3)
        a.button(g, 'Apply scale', self.apply_scale); self.status['scale'] = a.text(g, '', small=True)

        self.open_last()

    def rec_list(self, parent, key, title, shade):
        a = self.app
        parent.addWidget(W.QLabel(title + ' (tick = use)'))
        lst = W.QListWidget(); lst.setMaximumHeight(86); parent.addWidget(lst); self.lists[key] = lst
        lst.itemChanged.connect(lambda item, k=key: self.list_changed(k))
        row = W.QHBoxLayout(); parent.addLayout(row)
        a.button(row, 'Add the open recording', lambda k=key: self.add_open(k), action=False)
        a.button(row, 'Add folder…', lambda k=key: self.add_folder(k), action=False)
        row = W.QHBoxLayout(); parent.addLayout(row)
        a.button(row, 'Remove selected', lambda k=key: self.remove_selected(k), action=False)
        a.button(row, 'Show prepared signals', lambda k=key, s=shade: self.show_prepared(k, s))

    # ------------------------------------------------------------------ session
    def open_last(self):
        last = self.root / 'last.txt'
        folder = None
        if last.exists():
            f = Session.abs(last.read_text(encoding='utf-8').strip())
            if (f / 'session.json').exists():
                folder = f
        self.load_session(folder or self.root / time.strftime('session_%Y%m%d'))

    def load_session(self, folder):
        self.session = Session(folder, self.remember); self.cache.clear(); self.last.clear()
        if self.session.file.exists():
            self.remember(self.session)
        d = self.session.data
        for w, key in ((self.vs, 'Vs'), (self.lm, 'Lm_mH'), (self.fd, 'fd'), (self.fp, 'fp'), (self.wmin, 'wmin'),
                       (self.pend_delay, 'pend_delay_ms'), (self.swing_s, 'swing_s'),
                       (self.window_s, 'window_s'), (self.resync, 'resync_s'), (self.ts, 'Ts_ms')):
            w.blockSignals(True); w.setValue(float(d.get(key, w.value()))); w.blockSignals(False)
        self.polarity.blockSignals(True); self.polarity.setCurrentIndex(0 if d.get('polarity', 1) == 1 else 1); self.polarity.blockSignals(False)
        self.pend_sign.blockSignals(True); self.pend_sign.setCurrentIndex({0: 0, 1: 1, -1: 2}.get(d.get('pend_sign', 0), 0)); self.pend_sign.blockSignals(False)
        self.symmetric.setChecked(bool(d.get('symmetric'))); self.use_swing.setChecked(bool(d.get('use_swing', True)))
        for key, w in self.preset_widgets.items():
            saved = (d.get('presets') or {}).get(key)
            if saved:
                w['amp'].blockSignals(True); w['freqs'].blockSignals(True)
                w['amp'].setValue(float(saved['amp'])); w['freqs'].setText(saved['freqs'])
                w['amp'].blockSignals(False); w['freqs'].blockSignals(False)
        if d.get('seconds_each'):
            self.seconds_each.blockSignals(True); self.seconds_each.setValue(float(d['seconds_each'])); self.seconds_each.blockSignals(False)
        self.scale_mode.get(d['scale'].get('mode', 'nominal'), self.scale_mode['nominal']).setChecked(True)
        for k in self.lists:
            self.fill_list(k)
        self.session_label.setText(f'Session: {self.session.folder.name}  ·  {Session.rel(self.session.folder)}')
        self.refresh_status(); self.update_predictions()

    def remember(self, session):
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / 'last.txt').write_text(Session.rel(session.folder), encoding='utf-8')

    def new_session(self):
        name, ok = W.QInputDialog.getText(self.app, 'New estimation session', 'Name', text=time.strftime('session_%Y%m%d_%H%M'))
        if not ok or not name.strip():
            return
        folder = self.root / ''.join(c if c.isalnum() or c in '-_' else '_' for c in name.strip())
        if (folder / 'session.json').exists():
            raise ValueError(f'Session {folder.name} already exists; use Open session…')
        self.load_session(folder); self.session.save()

    def open_session(self):
        path = W.QFileDialog.getExistingDirectory(self.app, 'Open an estimation session folder', str(self.root))
        if path:
            if not (Path(path) / 'session.json').exists():
                raise ValueError('That folder has no session.json')
            self.load_session(path)

    def options_changed(self, *_):
        if self.session is None:
            return
        d = self.session.data
        new = {'Vs': self.vs.value(), 'polarity': 1 if self.polarity.currentIndex() == 0 else -1,
               'pend_sign': [0, 1, -1][self.pend_sign.currentIndex()], 'pend_delay_ms': self.pend_delay.value(), 'fd': self.fd.value(),
               'fp': self.fp.value(), 'wmin': self.wmin.value(), 'Lm_mH': self.lm.value()}
        changed = [k for k in ('Vs', 'polarity', 'pend_sign', 'pend_delay_ms', 'fd', 'fp', 'wmin') if d.get(k) != new[k]]
        d.update(new)
        if changed:
            self.cache.clear()
            # The pendulum sign only enters the pendulum and arm equations.
            self.session.mark_stale(['pendulum', 'arm', 'refine', 'model'] if set(changed) <= {'pend_sign', 'pend_delay_ms'} else FIT_STEPS)
        self.session.save(); self.refresh_status(); self.update_predictions()

    def save_ui(self):
        d = self.session.data
        d.update(window_s=self.window_s.value(), resync_s=self.resync.value(), Ts_ms=self.ts.value(), symmetric=self.symmetric.isChecked(),
                 swing_s=self.swing_s.value(), use_swing=self.use_swing.isChecked())
        self.session.save()

    # ------------------------------------------------------------------ recordings lists
    def fill_list(self, key):
        lst = self.lists[key]; lst.blockSignals(True); lst.clear()
        for r in self.session.data['recordings'][key]:
            item = W.QListWidgetItem(Path(r['path']).name); item.setData(QtCore.Qt.UserRole, r['path']); item.setToolTip(r['path'])
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable); item.setCheckState(QtCore.Qt.Checked if r.get('use', True) else QtCore.Qt.Unchecked)
            lst.addItem(item)
        lst.blockSignals(False)
        if key in ('validation', 'fit'):
            self.fill_compare(key == 'validation')

    def fill_compare(self, newest_validation=False):
        current = self.compare_combo.currentData()
        vals = self.session.data['recordings']['validation']
        if newest_validation and vals:
            current = vals[-1]['path']
        self.compare_combo.clear()
        for key, tag in (('validation', 'validation'), ('fit', 'fit')):
            for r in self.session.data['recordings'][key]:
                self.compare_combo.addItem(f'{Path(r["path"]).name}  ({tag})', r['path'])
        if current is not None:
            k = self.compare_combo.findData(current)
            if k >= 0:
                self.compare_combo.setCurrentIndex(k)

    def list_changed(self, key):
        lst = self.lists[key]; recs = self.session.data['recordings'][key]
        for i in range(lst.count()):
            item = lst.item(i)
            for r in recs:
                if r['path'] == item.data(QtCore.Qt.UserRole):
                    r['use'] = item.checkState() == QtCore.Qt.Checked
        self.session.mark_stale(LIST_DEPENDS[key]); self.session.save(); self.refresh_status()

    def add_path(self, key, path):
        path = Path(path)
        if not (path / 'manifest.json').exists():
            raise ValueError(f'{path} is not a run folder (no manifest.json)')
        added = self.session.add(key, path); self.fill_list(key); self.refresh_status()
        return added

    def add_open(self, key):
        path = getattr(self.app, 'run_path', None)
        if not path:
            raise ValueError('No recording is open. Finish a run or use Open recording… (top bar) first.')
        if not self.add_path(key, path):
            self.app.statusBar().showMessage('Already in the list')

    def add_folder(self, key):
        path = W.QFileDialog.getExistingDirectory(self.app, 'Select a run folder (manifest.json)', str(ROOT / 'data' / 'runs'))
        if path:
            self.add_path(key, path)

    def remove_selected(self, key):
        lst = self.lists[key]; item = lst.currentItem()
        if item is None:
            raise ValueError('Select a recording in the list first')
        rel = item.data(QtCore.Qt.UserRole)
        self.session.data['recordings'][key] = [r for r in self.session.data['recordings'][key] if r['path'] != rel]
        self.session.mark_stale(LIST_DEPENDS[key]); self.session.save(); self.fill_list(key); self.refresh_status()

    def recording_finished(self, path, manifest):
        """Called by LabApp when a run or passive recording tagged with an estimation step is saved."""
        step = (manifest.get('metadata') or {}).get('estimation_step')
        if step in ('swing', 'fit', 'validation', 'loaded_swing') and self.session is not None:
            self.add_path(step, path)
            self.app.output.append(f'\nAdded **{Path(path).name}** to the {step.replace("_", " ")} list of the estimation session.')
            fin = manifest.get('final_status') or {}
            if fin.get('state') == 'FAULT':
                self.app.output.append(f'\n⚠ **The run ended in a fault: {fin.get("fault", "?")}.** The data before the fault is kept and usable. '
                                       'Run buttons stay disabled until you clear it: Setup & guide → “Clear a resolved fault”.')

    # ------------------------------------------------------------------ preparing
    def opts(self):
        d = self.session.data
        a = self.app; cpr = a.gear.value() * a.motor_ppr.value() * a.quadrature.value()
        return dict(Vs=float(d['Vs']), polarity=int(d['polarity']), fd=float(d['fd']) or None, fp=float(d['fp']), wmin=float(d['wmin']), cpr=cpr,
                    pend_sign=int(d.get('pend_sign', 0)) or None, pend_delay=float(d.get('pend_delay_ms', 0.)) / 1000)

    def prepared(self, path, need_current=True, o=None):
        o = o or self.opts(); key = (str(path), o['Vs'], o['polarity'], o['fd'], need_current, o['cpr'], o.get('pend_sign'), o.get('pend_delay', 0.))
        if key not in self.cache:
            rows, manifest = load_run(path)
            self.cache[key] = E.prepare(rows, manifest.get('config', {}), o['Vs'], o['fd'], o['polarity'], need_current,
                                        Path(path).name, manifest, o['cpr'], o.get('pend_sign'), o.get('pend_delay', 0.))
        return self.cache[key]

    def preps(self, key, need_current=True, o=None):
        paths = self.session.recordings(key)
        if not paths:
            raise ValueError(f'No ticked recordings in the {key.replace("_", " ")} list')
        return [self.prepared(p, need_current, o) for p in paths]

    def selected_name(self, key):
        item = self.lists[key].currentItem() if key in self.lists else None
        return Path(item.data(QtCore.Qt.UserRole)).name if item is not None else None

    def pick(self, runs, key):
        name = self.selected_name(key)
        return next((r for r in runs if r['name'] == name), runs[0])

    def show_prepared(self, key, shade):
        lst = self.lists[key]; item = lst.currentItem() or (lst.item(0) if lst.count() else None)
        if item is None:
            raise ValueError('The list is empty')
        path = Session.abs(item.data(QtCore.Qt.UserRole)); need = key not in ('swing', 'loaded_swing'); o = self.opts()
        self.app.submit(lambda: self.prepared(path, need, o), lambda p: self.draw_prepared(p, shade, o), 'Preparing signals…')

    def draw_prepared(self, p, shade, o):
        t = p['t']; raw = p['raw']; m = E.masks(p, o['fp'], o['wmin'])[shade]; deg = 180 / math.pi
        panels = [
            {'unit': 'θ0 (deg)', 'curves': [('arm angle raw', t, raw['th0'] * deg, 'raw', 1), (f'filtered {p["fd"]:g} Hz', t, p['th0'] * deg, 'filtered', 1.4)], 'shade': (t, m)},
            {'unit': 'θ̇0 (deg/s)', 'curves': [('arm speed', t, p['w0'] * deg, 'filtered', 1.3)], 'shade': (t, m)},
            {'unit': 'θ̈0 (deg/s²)', 'curves': [('arm acceleration', t, p['a0'] * deg, 'filtered', 1.1)], 'shade': (t, m)},
            {'unit': 'θ1 (deg)', 'curves': [('pendulum angle raw (180 = hanging)', t, raw['th1'] * deg, 'raw', 1), ('filtered', t, p['th1'] * deg, 'p3', 1.4)], 'shade': (t, m)},
            {'unit': 'i (A)', 'curves': [(f'current ({p["current_source"]})', t, raw['i'], 'raw', 1), ('filtered', t, p['i'], 'p1', 1.4)], 'shade': (t, m)},
            {'unit': 'V (V)', 'curves': [('duty × Vs', t, raw['app'] * p['Vs'], 'raw', 1), ('filtered', t, p['V'], 'duty', 1.3)], 'shade': (t, m)}]
        self.show_panels(panels, f'Prepared signals · {p["name"]} · {p["fs"]:.0f} Hz · grey = not used by this step ({shade})')
        self.app.report('Prepared signals', f'**{p["name"]}**: {len(t)} samples at {p["fs"]:.1f} Hz on a uniform grid '
                        f'(span {t[-1]:.1f} s). Every signal is low-passed at **{p["fd"]:g} Hz** (zero-phase), then differentiated.\n\n'
                        f'Current source: {p["current_source"]}. Grey bands: samples the {shade} mask excludes.\n\n'
                        + E.checks_markdown(E.check_prepared(p)))

    def show_panels(self, panels, title):
        self.app.fitview.show_panels(panels, title); self.app.show_fit_view()

    # ------------------------------------------------------------------ Step 0
    def update_status(self):
        """Called from LabApp.refresh (every 200 ms): cheap label updates only."""
        d = self.app.device
        if not d.ready:
            self.checklist.setText('Checklist: connect the device (USB) — recordings made earlier can still be fitted.'); return
        c = d.config; st = d.status
        items = [('current zero set', S_finite(c.get('currentZeroMv'))), ('arm centered', bool(st.get('centered'))),
                 ('pendulum-down set', S_finite(c.get('pendDownRaw')) and c.get('pendDownRaw', -1) >= 0),
                 ('encoder scale applied', S_finite(c.get('countsPerRev'))),
                 ('limits commissioned', c.get('maxDuty', 0) > 0 and c.get('travelCounts', 0) > 0),
                 ('stream 250 Hz', c.get('streamHz') == 250),
                 ('MCU current filter on', (c.get('filter') or {}).get('type', 'bypass') != 'bypass')]
        self.checklist.setText('Checklist: ' + ' · '.join(('✓ ' if ok else '✗ ') + name for name, ok in items))
        cpr = c.get('countsPerRev')
        self.scale_label.setText(f'Encoder scale: {cpr:g} counts per output revolution (gear × 11 × 4).' if S_finite(cpr) else
                                 'Encoder scale: not applied (Setup → Apply proposed encoder scale to RAM).')
        dz = max(c.get('deadzonePos') or 0, c.get('deadzoneNeg') or 0) if S_finite(c.get('deadzonePos')) or S_finite(c.get('deadzoneNeg')) else 0
        if dz and not self.dz_applied:
            self.dz_applied = True
            for w in self.preset_widgets.values():
                if w['amp'].value() < 200 * dz:
                    w['amp'].setValue(round(200 * dz, 1))
            self.update_predictions()

    def apply_settings(self):
        a = self.app; a.require_disarmed(); d = a.device; c = copy.deepcopy(d.config)
        hz = int(c.get('adcHz') or 2000); sos = signal.butter(2, 40, fs=hz, output='sos')[0]
        if not self.session.data.get('saved_settings'):
            self.session.data['saved_settings'] = {'streamHz': c.get('streamHz'), 'filter': copy.deepcopy(c.get('filter'))}; self.session.save()
        c['streamHz'] = 250; c['filter'] = dict(c.get('filter') or {}, type='biquad', sos=[float(x) for x in sos])
        a.submit(lambda: d.apply(c), lambda _: a.report('Estimation recording settings applied',
                 f'Stream 250 Hz; MCU current filter 2nd-order Butterworth 40 Hz at {hz} samples/s '
                 f'(delay ≈ {1000 * E.filter_delay_s(c):.1f} ms, removed during preparation). '
                 'The previous values are kept in the session for “Restore normal settings”. Not saved to NVS.'), 'Applying settings…')

    def estimate_delay(self):
        o = self.opts(); fixed = self.swing_fixed()
        paths = self.session.recordings('fit')
        if not paths:
            raise ValueError('Add fit runs (Step 2) first')

        def make(delay):
            oo = dict(o, pend_delay=delay)
            return [self.prepared(pth, True, oo) for pth in paths]

        def work():
            return E.estimate_pend_delay(make, fixed, range(0, 16), o['fp'])

        def done(res):
            best, rows = res
            self.pend_delay.setValue(round(best, 1))      # triggers options_changed: later steps become stale
            text = '\n'.join(f'| {ms:g} | {r2:.4f} |' for ms, r2 in rows)
            self.app.report('Pendulum timing estimate', f'Best fit of the pendulum equation at **{best:.1f} ms** '
                            '(the measured pendulum angle lags the arm and current by this much; it may be the AS5600 filter '
                            'or the arm lagging the motor through gearbox backlash). Set in Step 0; fit Steps 4 and 5 again.\n\n'
                            '| Lag (ms) | R² of the pendulum equation |\n|---|---|\n' + text)
            self.show_panels([{'unit': 'R²', 'xlabel': 'pendulum timing correction (ms)',
                               'curves': [('pendulum-equation R²', [x for x, _ in rows], [y for _, y in rows], 'model', 'dots'),
                                          ('', [x for x, _ in rows], [y for _, y in rows], 'model', 1.)]}], 'Pendulum timing scan')
        self.app.submit(work, done, 'Scanning pendulum timing (0–15 ms)…')

    def flip_device_pend_sign(self):
        a = self.app; a.require_disarmed(); c = copy.deepcopy(a.device.config)
        c['pendSign'] = -(c.get('pendSign', 1) or 1)

        def done(_):
            a.report('Pendulum direction reversed on the device',
                     f'pendSign is now {c["pendSign"]:+d} in device RAM. The live view and new recordings use it. '
                     'Press “Save current profile to device NVS” (Setup) to keep it after a restart. '
                     f'For this session, set “Pendulum sign used for fitting” to {c["pendSign"]:+d} so old and new recordings agree.')
        a.submit(lambda: a.device.apply(c), done, 'Reversing pendulum direction…')

    def restore_settings(self):
        a = self.app; a.require_disarmed(); saved = self.session.data.get('saved_settings')
        if not saved:
            raise ValueError('No saved settings in this session (apply the estimation settings first)')
        c = copy.deepcopy(a.device.config); c['streamHz'] = saved['streamHz'] or 100; c['filter'] = saved['filter'] or c['filter']

        def done(_):
            self.session.data['saved_settings'] = None; self.session.save()
            a.report('Normal settings restored', f'Stream {c["streamHz"]} Hz; MCU filter {c["filter"].get("type")}.')
        a.submit(lambda: a.device.apply(c), done, 'Restoring settings…')

    # ------------------------------------------------------------------ passive recordings
    def record_passive(self, key, seconds):
        a = self.app; a.require_disarmed()
        if a.device.record:
            raise ValueError('A recording is already running')
        if self.passive:
            raise ValueError('A passive recording is already running')
        path = a.new_path('runs')
        meta = a.metadata() | {'purpose': 'free_swing' if key == 'swing' else 'free_swing_added_mass', 'estimation_step': key}
        if key == 'loaded_swing':
            meta.update(added_mass_g=self.dm.value(), added_mass_distance_cm=self.dpos.value())
        a.device.start_record(path, meta)
        self.passive = (path, key, time.monotonic() + seconds); self.timer.start(200)
        a.report('Recording free swing', f'Recording for {seconds:g} s to {path}. Release the pendulum now. The motor is not driven.')

    def passive_tick(self):
        if not self.passive:
            self.timer.stop(); return
        path, key, until = self.passive; left = until - time.monotonic(); a = self.app
        if not a.device.record or a.device.record_path != path:
            self.passive = None; self.timer.stop(); return       # stopped elsewhere (STOP loads it)
        a.statusBar().showMessage(f'Free swing: {max(0, left):.1f} s left')
        if left <= 0:
            self.passive = None; self.timer.stop()
            p = a.device.finish_record()
            if p:
                a.load_recorded(p)

    # ------------------------------------------------------------------ Step 2
    def preset(self, key):
        w = self.preset_widgets[key]
        try:
            freqs = [float(x) for x in w['freqs'].text().replace(';', ',').split(',') if x.strip()]
        except ValueError:
            raise ValueError(f'{w["title"]}: frequencies must be numbers separated by commas')
        if not freqs or any(f <= 0 for f in freqs):
            raise ValueError(f'{w["title"]}: enter positive frequencies')
        return w['amp'].value(), freqs, E.square_sweep(w['amp'].value(), freqs, self.seconds_each.value())

    def current_kb(self):
        m = self.session.data.get('model') if self.session else None
        if m:
            return m['params']['Kb']
        r = self.session.data['results'].get('electrical') if self.session else None
        return r['Kb'] if r else None

    def update_predictions(self, *_):
        if self.session is None:
            return
        presets = {k: {'amp': w['amp'].value(), 'freqs': w['freqs'].text()} for k, w in self.preset_widgets.items()}
        if presets != self.session.data.get('presets') or self.seconds_each.value() != self.session.data.get('seconds_each'):
            self.session.data['presets'] = presets; self.session.data['seconds_each'] = self.seconds_each.value(); self.session.save()
        d = self.app.device; c = d.config if d.ready else {}
        kb = self.current_kb(); Vs = self.vs.value()
        dz = max(c.get('deadzonePos') or 0, c.get('deadzoneNeg') or 0)
        cpr = c.get('countsPerRev'); travel = c.get('travelCounts', 0)
        travel_deg = travel / cpr * 360 if S_finite(cpr) and cpr > 0 and travel else None
        for key, w in self.preset_widgets.items():
            try:
                amp, freqs, seg = self.preset(key)
            except ValueError as e:
                w['info'].setText(str(e)); continue
            total = sum(s['duration'] for s in seg); text = [f'{len(freqs)} frequencies · {total:.1f} s · peak duty {amp:g} %']
            if kb:
                swing = max(E.predict_arm_swing(amp, f, Vs, kb, dz) for f in freqs)
                text.append(f'predicted arm swing ≤ ±{swing:.0f}°' + (f' (travel ±{travel_deg:.0f}°)' if travel_deg else ''))
                if travel_deg and swing > travel_deg:
                    text.append('⚠ may exceed the commissioned travel (ARM_TRAVEL stops the run; data before it is usable)')
            else:
                text.append('arm-swing prediction after Step 3 (needs Kb): start with the lowest amplitude')
            sw = self.session.data['results'].get('swing')
            if sw:
                fn = sw['freq_hz']; near = [f for f in freqs if abs(f / fn - 1) < .25]
                if near:
                    text.append(f'⚠ {", ".join(f"{f:g}" for f in near)} Hz is near the pendulum natural frequency {fn:.2f} Hz: '
                                'it will resonate and swing very large; use frequencies at least 25 % away')
            rev, frac = E.reversal_note(amp, max(freqs), c.get('slewPerSecond'))
            if rev is not None:
                text.append(f'slew: each reversal takes {rev * 1000:.0f} ms ({frac * 100:.0f} % of the shortest half period)'
                            + (' ⚠ raise “Slew rate” for sharper edges' if frac > .5 else ''))
            if c:
                if c.get('maxDuration', 0) > 0 and total > c['maxDuration']:
                    text.append(f'⚠ needs “Maximum run seconds” ≥ {total:.1f}')
                if amp / 100 > c.get('maxDuty', 0) + 1e-9:
                    text.append(f'⚠ needs “Maximum duty” ≥ {amp:g} %')
            w['info'].setText(' · '.join(text))

    def preview(self, key):
        amp, freqs, seg = self.preset(key)
        slew = (self.app.device.config.get('slewPerSecond') if self.app.device.ready else None) or 10.
        t, app = firmware_segments(seg, 1e-3, slew)
        req = firmware_segments(seg, 1e-3, 1e9)[1]
        self.show_panels([{'unit': 'duty (%)', 'curves': [('requested', t, req * 100, 'raw', 1), (f'applied (slew {slew * 100:g} %/s)', t, app * 100, 'duty', 1.4)]}],
                         f'{self.preset_widgets[key]["title"]}: commanded duty')

    def run_preset(self, key):
        amp, freqs, seg = self.preset(key); w = self.preset_widgets[key]
        self.app.authorize_run()
        self.app.dead_windows = None
        self.app.start_sequence(seg, 'estimation', extra={'estimation_step': w['target'], 'preset': w['title'],
                                                          'amplitude_pct': amp, 'freqs_hz': freqs})

    # ------------------------------------------------------------------ fits
    def fit_swing(self):
        o = self.opts()

        def work():
            preps = self.preps('swing', False, o)
            return preps, E.fit_freeswing(preps, o['fp'])
        self.app.submit(work, self.swing_done, 'Fitting free swing…')

    def swing_done(self, res):
        preps, r = res; self.last['swing'] = r
        self.session.set_result('swing', {k: r[k] for k in ('rJ', 'rb', 'rc', 'rJ_se', 'rb_se', 'rc_se', 'wn', 'wn_se', 'zeta', 'zeta_se', 'length',
                                                             'freq_hz', 'period_s', 'amplitude_deg', 'arm_counts', 'r2', 'rows')})
        run = self.pick(r['runs'], 'swing'); t = run['t']; deg = 180 / math.pi
        self.show_panels([
            {'unit': 'φ (deg)', 'curves': [('measured φ = θ1 − 180°', t, run['phi'] * deg, 'measured', 1.3), ('simulated swing (fitted)', t, run['phi_sim'] * deg, 'model', 1.3)], 'shade': (t, run['mask'])},
            {'unit': 'sin φ', 'curves': [('sin φ measured', t, run['measured'], 'measured', 1.3), ('fit', t, run['predicted'], 'model', 1.3)], 'shade': (t, run['mask'])},
            {'unit': 'residual', 'curves': [('measured − fit', t, run['measured'] - run['predicted'], 'residual', 1)], 'shade': (t, run['mask'])}],
            f'Step 1 · free swing · {run["name"]}')
        period = f'{r["period_s"]:.4f} s' if r['period_s'] else '—'
        self.app.report('Step 1 · Free swing', f'sin φ = −(Jp/G)·φ̈ − (b1/G)·φ̇ − (τp/G)·sgn φ̇ fitted on {r["rows"]} samples (R² {r["r2"]:.4f}).\n\n'
                        '| Quantity | Value | ± | Unit |\n|---|---|---|---|\n'
                        f'| Jp/G | {r["rJ"]:.5g} | {pct(r["rJ_se"], r["rJ"])} | s² |\n| b1/G | {r["rb"]:.4g} | {pct(r["rb_se"], r["rb"])} | s |\n'
                        f'| ωn (small swings) | {r["wn"]:.4f} | {pct(r["wn_se"], r["wn"])} | rad/s ({r["freq_hz"]:.4f} Hz) |\n'
                        f'| ζ (viscous part) | {r["zeta"]:.4g} | {pct(r["zeta_se"], r["zeta"])} | — |\n'
                        f'| τp/G (dry friction) | {r["rc"]:.4g} | {pct(r["rc_se"], r["rc"])} | rad ({math.degrees(r["rc"]):.2f}°: amplitude lost per half swing ≈ {2 * math.degrees(r["rc"]):.1f}°) |\n'
                        f'| equivalent length g·Jp/G | {r["length"] * 100:.2f} | | cm |\n'
                        f'| period from zero crossings | {period} | | (amplitude {r["amplitude_deg"]:.0f}°) |\n\n'
                        + E.checks_markdown(E.check_swing(r)))
        self.refresh_status()

    def fit_electrical(self):
        o = self.opts()

        def work():
            preps = self.preps('fit', True, o)
            return preps, E.fit_electrical(preps, o['fp'])
        self.app.submit(work, self.electrical_done, 'Fitting Rm and Kb…')

    def electrical_done(self, res):
        preps, r = res; self.last['electrical'] = r
        self.session.set_result('electrical', {k: r[k] for k in ('Rm', 'Kb', 'Rm_se', 'Kb_se', 'r2', 'rows', 'cond')} | {'eq92': r['eq92']})
        run = self.pick(r['runs'], 'fit'); t = run['t']; p = next(q for q in preps if q['name'] == run['name']); deg = 180 / math.pi
        self.show_panels([
            {'unit': 'i (A)', 'curves': [('measured current', t, run['measured'], 'measured', 1.2), ('V/Rm − (Kb/Rm)·θ̇0', t, run['predicted'], 'model', 1.3)], 'shade': (t, run['mask'])},
            {'unit': 'residual (A)', 'curves': [('measured − model', t, run['measured'] - run['predicted'], 'residual', 1)], 'shade': (t, run['mask'])},
            {'unit': 'V (V)', 'curves': [('duty × Vs (filtered)', t, p['V'], 'duty', 1.2)], 'shade': (t, run['mask'])},
            {'unit': 'θ̇0 (deg/s)', 'curves': [('arm speed', t, p['w0'] * deg, 'filtered', 1.2)], 'shade': (t, run['mask'])}],
            f'Step 3 · electrical · {run["name"]} · grey = not driven')
        eq = r['eq92']
        cmp = f'\n\nReport Eq. 92 form (V on the left), for comparison: Rm = {eq["Rm"]:.4g} Ω, Kb = {eq["Kb"]:.4g} V·s/rad.' if eq else ''
        self.app.report('Step 3 · Electrical fit', f'{r["rows"]} driven samples from {len(preps)} run(s), R² {r["r2"]:.3f}.\n\n'
                        '| Parameter | Value | ± | Unit |\n|---|---|---|---|\n'
                        f'| Rm | {r["Rm"]:.4g} | {pct(r["Rm_se"], r["Rm"])} | Ω |\n| Kb | {r["Kb"]:.4g} | {pct(r["Kb_se"], r["Kb"])} | V·s/rad (output shaft) |'
                        + cmp + '\n\n' + E.checks_markdown(E.check_electrical(r) + E.check_polarity(preps) + sum((E.check_prepared(q) for q in preps), [])))
        self.refresh_status(); self.update_predictions()

    def swing_fixed(self):
        sw = self.session.data['results'].get('swing')
        if not self.use_swing.isChecked():
            return None
        if not sw:
            raise ValueError('Fit the free swing (Step 1) first, or untick “Use the free-swing …” in Step 4')
        return {k: sw.get(k, 0.) for k in ('rJ', 'rb', 'rc', 'rJ_se', 'rb_se', 'rc_se')}

    def fit_pendulum(self):
        o = self.opts(); fixed = self.swing_fixed(); self.save_ui()

        def work():
            preps = self.preps('fit', True, o)
            return preps, E.fit_pendulum(preps, o['fp'], fixed)
        self.app.submit(work, self.pendulum_done, 'Fitting pendulum equation…')

    def pendulum_done(self, res):
        preps, r = res; self.last['pendulum'] = r
        self.session.set_result('pendulum', {k: r[k] for k in ('rh', 'rJ', 'rb', 'rc', 'rh_se', 'rJ_se', 'rb_se', 'rc_se', 'Larm', 'Larm_se', 'r2', 'rows', 'source')})
        run = self.pick(r['runs'], 'fit'); t = run['t']
        parts = [(name, t, y, c, 1.1) for (name, y), c in zip(run['parts'].items(), ('p1', 'p2', 'p3', 'p4'))]
        self.show_panels([
            {'unit': 'sin θ1', 'curves': [('measured sin θ1', t, run['measured'], 'measured', 1.2), ('fit', t, run['predicted'], 'model', 1.3)], 'shade': (t, run['mask'])},
            {'unit': 'contributions', 'curves': parts, 'shade': (t, run['mask'])},
            {'unit': 'residual', 'curves': [('measured − fit', t, run['measured'] - run['predicted'], 'residual', 1)], 'shade': (t, run['mask'])}],
            f'Step 4 · pendulum equation · {run["name"]}')
        sw = self.session.data['results'].get('swing')
        cmp = lambda k: f'{sw[k]:.5g} ({100 * (r[k] / sw[k] - 1):+.1f} %)' if sw else '— (fit Step 1)'
        used = sorted({q['pend_sign'] for q in preps})
        hint = (f'\n\n**Fix:** the recordings were converted with pendulum sign {", ".join(f"{u:+d}" for u in used)}. '
                f'Set “Pendulum sign used for fitting” (Step 0) to **{-used[0]:+d}** and press Fit pendulum equation again.') if r['rh'] < 0 and len(used) == 1 else ''
        src = ('Jp/G, damping and dry friction taken from the free swing; only h/G fitted.' if r['source'] == 'free swing'
               else 'All pendulum ratios fitted from the driven runs (dry friction 0).')
        self.app.report('Step 4 · Pendulum-equation fit', f'{r["rows"]} samples, R² {r["r2"]:.4f} (whole equation). {src} '
                        f'Pendulum timing correction {self.session.data.get("pend_delay_ms", 0):g} ms.\n\n'
                        '| Quantity | Value | ± | Free swing (Step 1) | Unit |\n|---|---|---|---|---|\n'
                        f'| h/G | {r["rh"]:.5g} | {pct(r["rh_se"], r["rh"])} | — | s² |\n'
                        f'| Jp/G | {r["rJ"]:.5g} | {pct(r["rJ_se"], r["rJ"])} | {cmp("rJ")} | s² |\n'
                        f'| b1/G | {r["rb"]:.4g} | {pct(r["rb_se"], r["rb"])} | {cmp("rb")} | s |\n'
                        f'| τp/G | {r["rc"]:.4g} | {pct(r["rc_se"], r["rc"])} | {cmp("rc") if sw and "rc" in sw else "—"} | rad |\n'
                        f'| **Larm = g·h/G** | **{r["Larm"] * 100:.2f}** | {pct(r["Larm_se"], r["Larm"])} | — | cm |\n\n'
                        + E.checks_markdown(E.check_pendulum(r, sw)) + hint)
        self.refresh_status()

    def fit_arm(self):
        o = self.opts(); pend = self.session.data['results'].get('pendulum'); elec = self.session.data['results'].get('electrical')
        if not pend or not elec:
            raise ValueError('Fit Step 3 (electrical) and Step 4 (pendulum equation) first')
        sym = self.symmetric.isChecked(); self.save_ui()

        def work():
            preps = self.preps('fit', True, o)
            return preps, E.fit_arm(preps, pend, o['fp'], o['wmin'], sym)
        self.app.submit(work, self.arm_done, 'Fitting arm equation…')

    def arm_done(self, res):
        preps, r = res; self.last['arm'] = r; d = self.session.data
        self.session.set_result('arm', {k: r[k] for k in ('Ja', 'G', 'Beq', 'tcp', 'tcn', 'Ja_se', 'G_se', 'Beq_se', 'tcp_se', 'tcn_se', 'r2', 'rows', 'symmetric')})
        params, se = E.assemble(d['results']['electrical'], d['results']['pendulum'], r)
        d['ls_model'] = {'params': params, 'se': se}
        if not d.get('model') or d['model'].get('source') == 'least squares' or d['stale'].get('model'):
            d['model'] = {'params': params, 'se': se, 'source': 'least squares', 'Larm': d['results']['pendulum']['Larm']}
            d['stale']['model'] = False
        self.session.save()
        run = self.pick(r['runs'], 'fit'); t = run['t']
        parts = [(name, t, y, c, 1.1) for (name, y), c in zip(run['parts'].items(), ('p1', 'p2', 'p3', 'p4', 'p5'))]
        self.show_panels([
            {'unit': 'i (A)', 'curves': [('measured current', t, run['measured'], 'measured', 1.2), ('arm equation', t, run['predicted'], 'model', 1.3)], 'shade': (t, run['mask'])},
            {'unit': 'contributions (A)', 'curves': parts, 'shade': (t, run['mask'])},
            {'unit': 'residual (A)', 'curves': [('measured − fit', t, run['measured'] - run['predicted'], 'residual', 1)], 'shade': (t, run['mask'])}],
            f'Step 5 · arm equation · {run["name"]} · grey = arm at rest or slow')
        c = self.app.device.config if self.app.device.ready else {}
        Vs = d['Vs']; Rm = d['results']['electrical']['Rm']
        dz = f' Stored dead zone: +{100 * c["deadzonePos"]:.1f} % / −{100 * c["deadzoneNeg"]:.1f} % (breakaway is above sliding friction).' \
            if S_finite(c.get('deadzonePos')) and S_finite(c.get('deadzoneNeg')) else ''
        rows = '\n'.join(f'| {E.INFO[k][0]} | {r[k]:.4g} | {pct(r[k + "_se"], r[k])} | {E.INFO[k][1]} |' for k in ('Ja', 'G', 'Beq', 'tcp', 'tcn'))
        self.app.report('Step 5 · Arm-equation fit', f'{r["rows"]} moving samples, R² {r["r2"]:.3f}. Values are per Kτ (torque in amperes).\n\n'
                        '| Parameter | Value | ± | Unit |\n|---|---|---|---|\n' + rows +
                        f'\n\nSliding friction as duty: +{100 * Rm * r["tcp"] / Vs:.1f} % / −{100 * Rm * r["tcn"] / Vs:.1f} %.{dz}\n\n'
                        'The least-squares model (all ten parameters) is now available to Steps 6–8.\n\n'
                        + E.checks_markdown(E.check_arm(r, d['results']['pendulum'])))
        self.refresh_status()

    # ------------------------------------------------------------------ Step 6
    def ls_model(self):
        m = self.session.data.get('ls_model')
        if not m or self.session.data['stale'].get('arm'):
            raise ValueError('Fit Steps 3–5 first (or refit them: they are stale)')
        return m

    def show_ls_sim(self):
        m = self.ls_model(); o = self.opts()

        def work():
            preps = self.preps('fit', True, o)
            return [E.simulate(m['params'], p, 0., self.app.cancel) for p in preps]
        self.app.submit(work, lambda sims: self.draw_sim(sims, None, 'Step 6 · least-squares model on the fit runs'), 'Simulating…')

    def refine(self):
        m = self.ls_model(); o = self.opts(); win = self.window_s.value(); self.save_ui()
        free = tuple(k for k, c in self.refine_free.items() if c.isChecked())
        if not free:
            raise ValueError('Tick at least one parameter to refine')
        progress = self.app.signals.progress.emit; cancel = self.app.cancel

        def work():
            preps = self.preps('fit', True, o)
            r = E.refine(m['params'], preps, win, free, progress, cancel)
            before = [E.simulate(m['params'], p, 0., cancel) for p in preps]
            after = [E.simulate(r['params'], p, 0., cancel) for p in preps]
            return r, before, after
        self.app.submit(work, self.refine_done, 'Refining by simulation (about 10–30 s)…')

    def refine_done(self, res):
        r, before, after = res; self.last['refine'] = (r, before, after)
        self.session.set_result('refine', {k: r[k] for k in ('params', 'cost_before', 'cost_after', 'change_pct', 'nfev', 'windows', 'window_s', 'free', 'at_bound')}
                                | {'fit_before': [b['fit'] for b in before], 'fit_after': [a['fit'] for a in after], 'names': [b['name'] for b in before]})
        self.draw_sim(after, before, 'Step 6 · refined (red) and least-squares (dashed) model on the fit runs')
        rows = '\n'.join(f'| {b["name"]} | ' + ' | '.join(f'{b["fit"][k]:.0f} → {a["fit"][k]:.0f}' for k in ('arm angle', 'pendulum angle', 'arm speed', 'current')) + ' |'
                         for b, a in zip(before, after))
        ch = '\n'.join(f'| {E.INFO[k][0]} | {self.session.data["ls_model"]["params"][k]:.4g} | {r["params"][k]:.4g} | {v:+.1f} % |' for k, v in r['change_pct'].items())
        self.app.report('Step 6 · Refinement', f'{r["windows"]} windows of {r["window_s"]:g} s, {r["nfev"]} evaluations; '
                        f'simulation cost {r["cost_before"]:.4g} → {r["cost_after"]:.4g}.\n\n'
                        '**Fit % on the fit runs, whole-run simulation (least squares → refined)**\n\n'
                        '| Run | Arm angle | Pendulum angle | Arm speed | Current |\n|---|---|---|---|---|\n' + rows +
                        '\n\n| Parameter | Least squares | Refined | Change |\n|---|---|---|---|\n' + ch +
                        '\n\nChoose **Accept refined model** or **Keep least-squares model**, then check Step 7 on the validation run.\n\n'
                        + E.checks_markdown(E.check_refine(r)))
        self.refresh_status()

    def accept_model(self, source):
        d = self.session.data; ls = self.ls_model(); larm = d['results']['pendulum']['Larm']
        if source == 'refined':
            r = d['results'].get('refine')
            if not r or d['stale'].get('refine'):
                raise ValueError('Run Refine first (or again: it is stale)')
            d['model'] = {'params': r['params'], 'se': ls['se'], 'source': 'refined', 'Larm': larm}
        else:
            d['model'] = {'params': ls['params'], 'se': ls['se'], 'source': 'least squares', 'Larm': larm}
        d['stale']['model'] = False; self.session.save(); self.refresh_status(); self.update_predictions()
        self.app.output.append(f'\n**Model set:** {source}. Steps 7 and 8 use it.')

    def model(self):
        m = self.session.data.get('model') if self.session else None
        if not m:
            raise ValueError('No model yet: fit Steps 3–5 (and optionally refine in Step 6)')
        return m

    # ------------------------------------------------------------------ Step 7
    def compare(self):
        m = self.model(); o = self.opts(); rel = self.compare_combo.currentData()
        if rel is None:
            raise ValueError('Add a validation run (Step 2) or a recording first')
        path = Session.abs(rel); resync = self.resync.value(); self.save_ui()

        def work():
            p = self.prepared(path, True, o)
            return p, E.simulate(m['params'], p, resync, self.app.cancel)
        self.app.submit(work, self.compare_done, 'Simulating the model on the recording…')

    def compare_done(self, res):
        p, sim = res; self.last['validate'] = sim
        m = self.session.data['model']
        self.session.data['results']['validate'] = {'name': sim['name'], 'fit': sim['fit'], 'rms': sim['rms'], 'resync_s': sim['resync_s'], 'model': m['source']}
        self.session.save()
        self.draw_sim([sim], None, f'Step 7 · model ({m["source"]}) against hardware', full=True)
        fit = sim['fit']; rms = sim['rms']
        mode = 'whole run from the first state (open loop)' if not sim['resync_s'] else f're-synced to the measured state every {sim["resync_s"]:g} s'
        self.app.report('Step 7 · Model against hardware',
                        f'**{sim["name"]}**, {mode}. Measured signals are the prepared ones (low-passed at {p["fd"]:g} Hz).\n\n'
                        '| Signal | Fit % | RMS error |\n|---|---|---|\n'
                        f'| Arm angle | {fit["arm angle"]:.1f} | {rms["arm angle (deg)"]:.2f}° |\n'
                        f'| Pendulum angle | {fit["pendulum angle"]:.1f} | {rms["pendulum angle (deg)"]:.2f}° |\n'
                        f'| Arm speed | {fit["arm speed"]:.1f} | |\n| Current | {fit["current"]:.1f} | {rms["current (A)"]:.3f} A |\n\n'
                        'Fit % = 100·(1 − ‖y − ŷ‖/‖y − ȳ‖). In open loop the arm angle drifts slowly (friction and noise integrate); '
                        'a re-sync interval shows the short-term agreement. The current fit is limited by sensor noise.\n\n'
                        + E.checks_markdown(E.check_validation(sim) + E.check_prepared(p)))
        self.refresh_status()

    def draw_sim(self, sims, before=None, title='', full=False):
        s = sims[0]; b = before[0] if before else None; t = s['t']; deg = 180 / math.pi
        def curves(key, scale, label, unit):
            c = [(f'measured {label}', t, s['meas'][key] * scale, 'measured', 1.2), (f'model {label}', t, s['sim'][key] * scale, 'model', 1.3)]
            if b is not None:
                c.append(('least-squares model', t, b['sim'][key] * scale, 'p2', 'dash'))
            return {'unit': unit, 'curves': c}
        panels = [curves('th0', deg, 'arm angle', 'θ0 (deg)'), curves('th1', deg, 'pendulum angle', 'θ1 (deg)')]
        if full or b is None:
            panels.append(curves('w0', deg, 'arm speed', 'θ̇0 (deg/s)'))
        panels.append(curves('i', 1, 'current', 'i (A)'))
        panels.append({'unit': 'duty (%)', 'curves': [('applied duty', t, s['app'] * 100, 'duty', 1.1)]})
        fit = s['fit']
        self.show_panels(panels, f'{title} · {s["name"]} · fit {fit["arm angle"]:.0f} / {fit["pendulum angle"]:.0f} / {fit["current"]:.0f} %')

    # ------------------------------------------------------------------ Step 8 and 9
    def lin(self):
        m = self.model(); d = self.session.data; self.save_ui()
        return m, E.linear_models(m['params'], d['Vs'], self.ts.value() / 1000, (d['Lm_mH'] or 0) / 1000 or None)

    def scale_result(self, m):
        sc = self.session.data.get('scale') or {'mode': 'nominal'}
        try:
            return E.scale(m['params'], m['se'], **sc)
        except Exception:
            return E.scale(m['params'], m['se'], 'nominal')

    def show_model(self):
        m, lin = self.lin(); scl = self.scale_result(m)
        text = E.model_markdown(m['params'], m['se'], lin, scl, m.get('Larm'))
        stale = ' (⚠ earlier steps changed since this model was set; refit and accept again)' if self.session.data['stale'].get('model') else ''
        self.app.report(f'Step 8 · Model ({m["source"]}){stale}', text + '\n\n' + E.checks_markdown(E.check_params(m['params'])))
        up, hg = lin['upright'], lin['hanging']
        self.app.fitview.show_pz([('upright', up['poles'], up['tf']['theta0/V']['zeros'], 'model'),
                                  ('hanging', hg['poles'], hg['tf']['theta0/V']['zeros'], 'filtered')],
                                 'Poles (×) and zeros of θ0/V (○): upright and hanging')
        self.app.show_fit_view()
        self.status['model'].setText(f'Upright poles: {E.fmt_roots(up["poles"], 3)} rad/s')

    def export(self):
        m, lin = self.lin(); scl = self.scale_result(m)
        folder = W.QFileDialog.getExistingDirectory(self.app, 'Export model files to', str(self.session.folder))
        if not folder:
            return
        info = {'session': self.session.data['name'], 'source': m['source'], 'created': time.strftime('%Y-%m-%d %H:%M'),
                'fit_recordings': [r['path'] for r in self.session.data['recordings']['fit'] if r.get('use', True)],
                'validation': self.session.data['results'].get('validate')}
        paths = E.export(folder, m['params'], m['se'], lin, scl, m.get('Larm'), info)
        self.app.report('Model exported', '\n\n'.join(f'- `{p}`' for p in paths) +
                        '\n\nIn MATLAB: `run furuta_model.m` gives `p`, `sys_up`, `sys_up_duty`, `G_up`, `sysd_up` (and `_down` for hanging); '
                        '`furuta_rhs(x, duty, p)` is the nonlinear model.')

    def apply_scale(self):
        mode = next(k for k, r in self.scale_mode.items() if r.isChecked()); m = self.model(); d = self.session.data
        kw = {'mode': mode}
        if mode == 'mass':
            sw = d['results'].get('swing')
            if not sw:
                raise ValueError('Fit the plain free swing (Step 1) first')
            loaded = self.session.recordings('loaded_swing')
            if not loaded:
                raise ValueError('Record (or add) a free swing with the added mass first')
            o = self.opts(); r = E.fit_freeswing([self.prepared(p, False, o) for p in loaded], o['fp'])
            kw.update(rJ_plain=sw['rJ'], rJ_plain_se=sw['rJ_se'], rJ_loaded=r['rJ'], rJ_loaded_se=r['rJ_se'],
                      dm=self.dm.value() / 1000, dm_tol=self.dm_tol.value() / 1000, d=self.dpos.value() / 100, d_tol=self.dpos_tol.value() / 100)
        elif mode == 'weighed':
            kw.update(m=self.wm.value() / 1000, Lp=self.wlp.value() / 100)
        elif mode == 'eta':
            kw.update(eta=self.eta.value())
        scl = E.scale(m['params'], m['se'], **kw)
        d['scale'] = kw; self.session.save()
        rows = '\n'.join(f'| {E.INFO[k][0].replace("/Kτ", "")} | {scl["si"][k]:.4g} | {E.INFO[k][2]} |' for k in E.PARAMS)
        extra = f'\n\nAdded-mass swing: m·g·Lp = {scl["G_abs"]:.4g} ± {scl["G_abs_se"]:.2g} N·m, Jp = {scl["Jp_abs"]:.4g} kg·m², ' \
                f'frequency change {100 * scl["sensitivity"]:.1f} %.' if mode == 'mass' else ''
        self.app.report('Step 9 · Physical units', f'**Kτ = {scl["Kt"]:.4g} ± {scl["Kt_se"]:.2g} N·m/A · ηg = Kτ/Kb = {scl["eta"]:.3f} ± {scl["eta_se"]:.2g} · '
                        f'm·Lp = {scl["mLp"] * 1000:.2f} g·m**{extra}\n\n| Parameter | SI value | Unit |\n|---|---|---|\n' + rows +
                        '\n\n' + ' '.join(scl['notes']) + '\n\nm and Lp separately (and J0, J1) cannot be separated with these sensors; the model does not need them.')
        self.refresh_status()

    # ------------------------------------------------------------------ status lines
    def refresh_status(self):
        if self.session is None:
            return
        d = self.session.data; res = d['results']; stale = d['stale']
        def line(step, text):
            lab = self.status.get(step)
            if lab is None:
                return
            if step in stale and stale[step]:
                text = '⚠ stale — inputs changed, fit again · ' + text
            lab.setText(text)
        n_sw = len(self.session.recordings('swing')); n_fit = len(self.session.recordings('fit'))
        s = res.get('swing')
        line('swing', f'✓ ωn {s["wn"]:.3f} rad/s · ζ {s["zeta"]:.4f} · dry {math.degrees(s.get("rc", 0)):.2f}°' if s else f'{n_sw} recording(s) ticked; not fitted')
        e = res.get('electrical')
        line('electrical', f'✓ Rm {e["Rm"]:.4g} Ω · Kb {e["Kb"]:.4g} V·s/rad' if e else f'{n_fit} fit run(s) ticked; not fitted')
        p = res.get('pendulum')
        line('pendulum', f'✓ Larm {p["Larm"] * 100:.2f} cm · R² {p["r2"]:.3f} · {p.get("source", "driven runs")}' if p else 'not fitted')
        a = res.get('arm')
        line('arm', f'✓ Ja {a["Ja"]:.4g} · G {a["G"]:.4g} · τc± {a["tcp"]:.3g}/{a["tcn"]:.3g} (per Kτ)' if a else 'not fitted')
        r = res.get('refine'); m = d.get('model')
        text = (f'refined: cost {r["cost_before"]:.4g} → {r["cost_after"]:.4g}' if r else 'not refined')
        text += f' · model in use: {m["source"]}' if m else ' · no model yet'
        line('refine', text)
        v = res.get('validate')
        line('validate', f'last: {v["name"]} · fit arm {v["fit"]["arm angle"]:.0f} % · pendulum {v["fit"]["pendulum angle"]:.0f} % · current {v["fit"]["current"]:.0f} %' if v else 'not compared yet')
        if m:
            line('model', f'model: {m["source"]}' + (' — stale' if stale.get('model') else ''))
        sc = d.get('scale') or {}
        line('scale', f'scale mode: {sc.get("mode", "nominal")}')


def S_finite(x):
    return E.finite(x)


def pct(se, v):
    try:
        return f'{abs(se / v) * 100:.1f} %' if v else '—'
    except (TypeError, ZeroDivisionError):
        return '—'
