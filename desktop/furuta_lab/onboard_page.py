"""Onboard controller page (firmware 1.6+).

The ESP32 runs the controller itself (firmware/include/onboard.h, every 4 ms). This page only
starts it, keeps the heartbeat going (Device._loop), shows the controller state from the `ctrl`
byte of every core record, edits the controller knobs (RAM only) and records the run.

To add a controller (e.g. PID, pole placement): add an entry to CONTROLLERS. `start` is merged into
the START_ONBOARD command, so a firmware that understands e.g. {'controller': 'pid'} needs nothing
else here. Its knobs are whatever ONBOARD_GET returns; add descriptions to KNOBS.
"""
import copy
import math
import time
from pathlib import Path
import numpy as np
from PySide6 import QtCore, QtWidgets as W
from . import signals as S
from . import wire

CONTROLLERS = [
    {'key': 'swingup_lqr', 'label': 'Swing-up + LQR balance', 'start': {'swingUp': True},
     'how': 'Start with the pendulum <b>hanging at rest</b> and the arm centered. The energy law swings it up and '
            'the LQR catches it near the top (simulation: caught after about 1.5 s, arm within about ±50°, '
            'current below 1 A while swinging with one short peak near 1.5 A at the catch).'},
    {'key': 'lqr', 'label': 'LQR balance only', 'start': {'swingUp': False},
     'how': 'Hold the pendulum lightly <b>near upright</b> (the catch-region light below must be green), press Start, '
            'then let go. If the pendulum leaves the catch region the run ends (ONBOARD_PENDULUM_LOST). '
            'Expected: it stays up and the arm drifts slowly by about ±20° (friction limit cycle with fricGain 0).'},
]

# knob -> (unit, meaning). Unknown knobs from a newer firmware are still listed and editable.
KNOBS = {
    'K0': ('duty/rad', 'LQR gain on arm angle θ0 (duty = −K·(x − [r 0 0 0]))'),
    'K1': ('duty/rad', 'LQR gain on pendulum angle θ1'),
    'K2': ('duty·s/rad', 'LQR gain on arm speed'),
    'K3': ('duty·s/rad', 'LQR gain on pendulum speed'),
    'r': ('rad', 'arm reference angle'),
    'velFc': ('Hz', 'velocity estimate filter cut-off'),
    'fricGain': ('–', 'friction compensation in balance (0 = off, 1 = identified Coulomb duty)'),
    'fricBand': ('duty', 'width of the friction-compensation sign function'),
    'catchTh1': ('rad', 'hand over to LQR when |θ1| is below this …'),
    'catchW1': ('rad/s', '… and |θ̇1| is below this'),
    'exitTh1': ('rad', 'back to swing-up when |θ1| exceeds this'),
    'safeTh0': ('rad', 'SafeStop (motor off) when |θ0| exceeds this'),
    'E0': ('–', 'swing-up target energy, normalised E/(2G̃)'),
    'kE': ('–', 'energy error gain'),
    'aPush': ('rad/s²', 'swing-up arm acceleration amplitude'),
    'wc': ('rad/s', 'arm centring loop natural frequency during swing-up'),
    'zc': ('–', 'arm centring loop damping ratio'),
    'aKick': ('rad/s²', 'kick acceleration to start from rest'),
    'kickE': ('–', 'use the kick while the energy is within this of hanging'),
    'wSign': ('rad/s', 'smoothing of sign(θ̇1·cos θ1)'),
    'wFric': ('rad/s', 'smoothing of the arm friction sign'),
    'iMax': ('A', 'swing-up current limit'),
    'uMax': ('duty', 'swing-up duty limit'),
}
READ_ONLY = {'Ts', 'rearm', 'modelArmSign', 'modelPendSign'}
CTRL_STYLE = {0: ('NOT RUNNING', '#6b7785'), 1: ('SAFE STOP', '#7a1020'), 2: ('SWING-UP', '#c77700'), 3: ('STABILIZE', '#1f7a4d')}
# Firmware notices that end an onboard run, with a hint for the operator.
STOP_HINTS = {
    'ONBOARD_SAFE_STOP': 'the arm passed safeTh0 (|θ0| limit) and the supervisor stopped the motor',
    'ONBOARD_PENDULUM_LOST': 'balance-only mode and the pendulum left the catch region',
    'PEND_INVALID': 'the pendulum reading was older than 20 ms or the magnet status was bad',
}


def angles(row, config, status):
    """(theta0 deg or None, theta1 deg or None, catch region bool) from one core record, as the firmware computes them."""
    cpr = config.get('countsPerRev'); down = config.get('pendDownRaw')
    th0 = None
    if status.get('centered') and S.finite(cpr) and cpr > 0:
        th0 = (config.get('armSign', 1) or 1) * (int(row['count']) - int(status.get('centerCount', 0))) * 360. / cpr
    th1 = None
    if S.finite(down) and down >= 0 and (int(row['validity']) & 2):
        th1 = float(S.wrap_deg((float(row['pend_raw']) - down) * 360. / S.PEND_COUNTS * (config.get('pendSign', 1) or 1) + 180))
    return th0, th1


def summary(rows, config, manifest):
    """Markdown summary of a recorded onboard run."""
    if rows is None or not len(rows):
        return 'No samples were recorded.'
    t, series = S.compute(rows, config, center=(manifest.get('status') or {}).get('centerCount'),
                          centered=bool((manifest.get('status') or {}).get('centered')), smooth_hz=20.)
    running = rows['state'] == 3
    ctrl = rows['ctrl'].astype(int)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.
    on = np.flatnonzero(running)
    lines = []
    if len(on):
        t0 = t[on[0]]
        lines.append(f'Controller ran for **{t[on[-1]] - t0:.2f} s**.')
        stab = np.flatnonzero(ctrl == 3)
        if len(stab):
            lines.append(f'First STABILIZE (caught) at **{t[stab[0]] - t0:.2f} s** after start; '
                         f'time in STABILIZE {len(stab) * dt:.2f} s, in SWING-UP {np.sum(ctrl == 2) * dt:.2f} s.')
        else:
            lines.append(f'Never reached STABILIZE (time in SWING-UP {np.sum(ctrl == 2) * dt:.2f} s).')
        sel = on
    else:
        lines.append('The controller never entered RUNNING in this recording.'); sel = np.arange(len(t))
    arm = series['arm_deg']; pend = series['pend_deg']; cur = series['current_smooth']
    if arm.unit == 'deg' and np.isfinite(arm.values[sel]).any():
        lines.append(f'Arm angle range {np.nanmin(arm.values[sel]):.1f}° to {np.nanmax(arm.values[sel]):.1f}°'
                     + (f' ({arm.note})' if arm.note else '') + '.')
    if np.isfinite(cur.values[sel]).any():
        lines.append(f'Peak |current| {np.nanmax(np.abs(cur.values[sel])):.2f} {cur.unit} (20 Hz smoothed); '
                     f'peak |applied duty| {np.max(np.abs(rows["applied"][sel])) * 100:.1f} %.')
    fin = manifest.get('final_status') or {}
    if fin.get('state') == 'FAULT':
        lines.append(f'⚠ Ended in **FAULT: {fin.get("fault", "?")}**. Clear it on Setup & guide before the next run.')
    return '\n\n'.join(lines)


class OnboardPage:
    def __init__(self, app):
        self.app = app; self.params = None; self.active = False; self.t_start = None; self.event_mark = 0
        self.last_end = ''; self.seen_running = False

    # ------------------------------------------------------------------ layout
    def build(self, v):
        a = self.app
        a.text(v, 'The ESP32 runs the controller itself (firmware 1.6, every 4 ms, with all firmware trips active). '
                  'This page starts it, keeps the heartbeat, shows its state and records the run. '
                  'STOP / DISABLE, unticking the 12 V box or closing the app stops the motor (heartbeat within 300 ms).', small=True)

        g = a.group(v, '1 · Readiness')
        self.checklist = W.QLabel('Connect USB to check.'); self.checklist.setWordWrap(True); self.checklist.setTextFormat(QtCore.Qt.RichText)
        g.addWidget(self.checklist)
        self.rec_seconds = a.number(g, 'Run length — maxDuration (s)', 20, .5, 3600, 1)
        self.rec_duty = a.number(g, 'Maximum duty (%)', 50, 0, 100, 1)
        self.rec_slew = a.number(g, 'Slew rate (%/s)', 10000, .1, 100000, 0)
        self.rec_trip = a.number(g, 'Current trip A (0 = off)', 1.5, 0, 50, 2)
        a.button(g, 'Apply these onboard-run limits to RAM', self.apply_limits)
        a.text(g, 'Designed with 100 /s slew (10000 %/s) and 50 % duty; the firmware warns below 50 /s. '
                  'Travel stays as commissioned on Motor tests.', small=True)
        self.sign_button = a.button(g, 'Set profile signs to the model’s (armSign +1, pendSign −1)', self.set_signs)

        g = a.group(v, '2 · Run — MOVES MOTOR')
        self.controller = W.QComboBox(); self.controller.addItems([c['label'] for c in CONTROLLERS]); g.addWidget(self.controller)
        self.how = a.text(g, '', small=True); self.controller.currentIndexChanged.connect(self.controller_changed); self.controller_changed()
        self.record = W.QCheckBox('Record the run to data/runs'); self.record.setChecked(True); g.addWidget(self.record)
        self.start_button = a.button(g, 'Start onboard controller — MOVES MOTOR', self.start)
        self.start_button.setStyleSheet('font-weight:700;'); self.start_button.setEnabled(False)
        row = W.QHBoxLayout(); g.addLayout(row)
        self.state_pill = W.QLabel(); self.state_pill.setAlignment(QtCore.Qt.AlignCenter); self.state_pill.setMinimumWidth(130); row.addWidget(self.state_pill)
        self.catch_light = W.QLabel(); self.catch_light.setAlignment(QtCore.Qt.AlignCenter); row.addWidget(self.catch_light)
        row.addStretch()
        self.live = a.text(g, '—'); self.live.setStyleSheet('font-family:Consolas;font-size:13px;')
        self.end_label = a.text(g, '', small=True)
        self.set_pill(0)

        g = a.group(v, '3 · Controller knobs (device RAM; a restart restores the design values)')
        self.table = W.QTableWidget(0, 4); self.table.setHorizontalHeaderLabels(['knob', 'value', 'unit', 'meaning'])
        self.table.verticalHeader().setVisible(False); self.table.setMinimumHeight(300)
        self.table.horizontalHeader().setStretchLastSection(True); g.addWidget(self.table)
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Read from device', self.read_params)
        a.button(row, 'Apply edited values', self.apply_params)
        a.button(row, 'Restore design values', self.restore_params)
        a.text(g, 'Design values come from matlab/harness/controllers/hybrid/ctrl_hybrid_design.m (export_onboard). '
                  'Knobs can only be changed while DISARMED.', small=True)

    def controller_changed(self, *_):
        self.how.setText(CONTROLLERS[self.controller.currentIndex()]['how'])

    def set_pill(self, code, extra=''):
        text, color = CTRL_STYLE.get(int(code), (f'CTRL {code}', '#6b7785'))
        self.state_pill.setText(text + extra)
        self.state_pill.setStyleSheet(f'background:{color};color:white;font-weight:700;border-radius:11px;padding:6px 14px;')

    # --------------------------------------------------------------- readiness
    def supported(self):
        d = self.app.device
        return d.ready and bool(d.identity.get('onboardControl'))

    def checks(self):
        """List of (ok, text) for the readiness panel. ok None = advisory."""
        a = self.app; d = a.device
        if not d.ready:
            return [(False, 'Connect USB')]
        c = d.config; s = d.status; p = self.params or {}
        out = [(bool(d.identity.get('onboardControl')), f'Firmware with onboard control ({d.identity.get("firmware", "?")})')]
        out.append((s.get('state') == 'DISARMED', f'Device DISARMED (now {s.get("state", "?")})'))
        out.append((a.power.isChecked(), '12 V box ticked (motion allowed)'))
        out.append((bool(s.get('centered')), 'Arm centered (Live tab → References)'))
        out.append((S.finite(c.get('pendDownRaw')) and c.get('pendDownRaw', -1) >= 0, 'Pendulum-down reference set'))
        cpr = c.get('countsPerRev')
        out.append((S.finite(cpr) and cpr > 0, f'Encoder scale set ({cpr} counts/rev' + (', model used 1496)' if cpr != 1496 else ')')))
        ma, mp = p.get('modelArmSign'), p.get('modelPendSign')
        if ma is None:
            out.append((None, 'Profile signs: read the knobs to compare with the model'))
        else:
            out.append((c.get('armSign') == ma and c.get('pendSign') == mp,
                        f'Profile signs armSign {c.get("armSign")}, pendSign {c.get("pendSign")} (model {ma:+d}, {mp:+d})'))
        out.append((c.get('maxDuty', 0) > 0, f'Maximum duty {c.get("maxDuty", 0) * 100:g} %'))
        out.append((c.get('travelCounts', 0) > 0, f'Travel ±{c.get("travelCounts", 0)} counts' +
                    (f' (±{c["travelCounts"] * 360 / cpr:.0f}°)' if S.finite(cpr) and cpr > 0 else '')))
        slew = c.get('slewPerSecond', 0)
        out.append((True if slew >= 50 else None, f'Slew {slew:g} /s' + ('' if slew >= 50 else ' — below the 100 /s it was designed with')))
        trip = c.get('currentTripA', 0)
        out.append((True if trip > 0 else None, f'Current trip {trip:g} A' if trip > 0 else 'Current trip OFF (1.5 A recommended)'))
        out.append((True, f'Run length (maxDuration) {c.get("maxDuration", 0):g} s · heartbeat {c.get("heartbeatMs", "?")} ms'))
        return out

    def update_status(self):
        a = self.app; d = a.device
        items = self.checks()
        mark = {True: '<span style="color:#1f7a4d">✓</span>', False: '<span style="color:#b52931">✗</span>', None: '<span style="color:#c77700">!</span>'}
        self.checklist.setText('<br>'.join(f'{mark[ok]} {text}' for ok, text in items))
        p = self.params or {}
        rows = d.snapshot(1) if d.ready else []
        state = d.status.get('state')
        row_running = bool(len(rows)) and int(rows[-1]['state']) == 3      # streamed at 100 Hz (State::Running)
        if self.active and row_running:
            self.seen_running = True
        if len(rows):
            r = rows[-1]; code = int(r['ctrl'])
            th0, th1 = angles(r, d.config, d.status)
            catch = p.get('catchTh1', math.radians(20))
            inside = th1 is not None and abs(math.radians(th1)) < catch
            self.catch_light.setText(('● inside' if inside else '○ outside') + f' catch region (|θ1| < {math.degrees(catch):.0f}°)')
            self.catch_light.setStyleSheet(f'color:{"#1f7a4d" if inside else "#6b7785"};font-weight:600;')
            el = f' · {time.monotonic() - self.t_start:4.1f} s' if self.active and self.t_start else ''
            self.set_pill(code, el)
            cur = ''
            zero, sens = d.config.get('currentZeroMv'), d.config.get('sensitivityMvA')
            if S.finite(zero) and S.finite(sens) and sens:
                cur = f'  i {(float(r["filtered_mv"]) - zero) / sens:+5.2f} A'
            f = lambda x: '   —  ' if x is None else f'{x:+6.1f}°'
            self.live.setText(f'θ0 {f(th0)}  θ1 {f(th1)}  duty {float(r["applied"]) * 100:+5.1f} %{cur}  · {state}')
        else:
            self.set_pill(0); self.catch_light.setText(''); self.live.setText('—')
        # end of a run started here: the stream showed RUNNING and no longer does (or the link/fault ended it)
        if self.active and ((self.seen_running and not row_running) or state in ('FAULT', 'CONNECTION_ERROR', 'DISCONNECTED')
                            or (not self.seen_running and time.monotonic() - self.t_start > 2)):
            self.active = False; self.finished_run()
        if not a.busy:
            self.start_button.setEnabled(self.supported() and a.power.isChecked() and state == 'DISARMED')
            c = d.config if d.ready else {}
            self.sign_button.setVisible(bool(p) and (c.get('armSign') != p.get('modelArmSign') or c.get('pendSign') != p.get('modelPendSign')))

    def finished_run(self):
        d = self.app.device; s = d.status; reason = 'reached the run length (maxDuration) or STOP'
        if s.get('state') == 'FAULT':
            reason = f'FAULT {s.get("fault", "?")}'
        if d.event_count > self.event_mark and (d.last_event or {}).get('event') in STOP_HINTS:
            ev = d.last_event['event']; reason = f'{ev}: {STOP_HINTS[ev]}'
        self.last_end = reason
        self.end_label.setText(f'Last run ended: {reason}.')
        self.app.output.append(f'\n**Onboard run ended:** {reason}.')

    # ----------------------------------------------------------------- actions
    def apply_limits(self):
        a = self.app; a.require_disarmed(); c = copy.deepcopy(a.device.config)
        c.update(maxDuration=self.rec_seconds.value(), maxDuty=self.rec_duty.value() / 100,
                 slewPerSecond=self.rec_slew.value() / 100, currentTripA=self.rec_trip.value())
        a.submit(lambda: a.device.apply(c), lambda r: a.report('Onboard-run limits applied to RAM',
                 f'maxDuration {c["maxDuration"]:g} s · maxDuty {c["maxDuty"] * 100:g} % · slew {c["slewPerSecond"]:g} /s · '
                 f'current trip {c["currentTripA"]:g} A. Use “Save current profile to device NVS” (Setup) to keep them after a restart.'))

    def set_signs(self):
        a = self.app; a.require_disarmed(); p = self.params or {}
        if 'modelArmSign' not in p:
            raise ValueError('Read the knobs from the device first')
        answer = W.QMessageBox.question(a, 'Change profile signs',
                                        f'Set armSign {p["modelArmSign"]:+d} and pendSign {p["modelPendSign"]:+d}?\n\n'
                                        'Afterwards check by hand (motor unpowered) on the Live tab: pendulum upright reads about 0°, '
                                        'hanging ±180°, and turning the arm in its positive direction gives a positive arm angle.')
        if answer != W.QMessageBox.Yes:
            return
        c = copy.deepcopy(a.device.config); c.update(armSign=int(p['modelArmSign']), pendSign=int(p['modelPendSign']))
        a.submit(lambda: a.device.apply(c), lambda r: a.report('Signs changed', 'Do the passive sign check on the Live tab before any powered run.'))

    def read_params(self):
        a = self.app
        if not self.supported():
            raise RuntimeError('Connect to firmware 1.6 or newer (onboard control)')
        a.submit(lambda: a.device.onboard_params(), self.params_loaded, 'Reading controller knobs…')

    def params_loaded(self, params):
        self.params = dict(params)
        self.table.setRowCount(0)
        for name, value in self.params.items():
            r = self.table.rowCount(); self.table.insertRow(r)
            unit, meaning = KNOBS.get(name, ('', 'read-only' if name in READ_ONLY else ''))
            if name in READ_ONLY:
                meaning = {'Ts': 'controller period (fixed)', 'rearm': 'swing up again after a SafeStop (design file)',
                           'modelArmSign': 'arm sign the model was identified with', 'modelPendSign': 'pendulum sign the model was identified with'}.get(name, meaning)
            item = W.QTableWidgetItem(name); item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable); self.table.setItem(r, 0, item)
            val = W.QTableWidgetItem(str(value).lower() if isinstance(value, bool) else f'{value:.10g}')
            if name in READ_ONLY or name == 'swingUp':
                val.setFlags(val.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(r, 1, val)
            for col, text in ((2, unit), (3, meaning if name != 'swingUp' else 'set by the controller choice above')):
                it = W.QTableWidgetItem(text); it.setFlags(it.flags() & ~QtCore.Qt.ItemIsEditable); self.table.setItem(r, col, it)
        self.table.resizeColumnsToContents()
        self.app.report('Controller knobs', f'{len(self.params)} values read from the device.')

    def edited(self):
        out = {}
        for r in range(self.table.rowCount()):
            name = self.table.item(r, 0).text()
            if name in READ_ONLY or name == 'swingUp':
                continue
            text = self.table.item(r, 1).text().strip()
            try:
                value = float(text)
            except ValueError:
                raise ValueError(f'{name}: “{text}” is not a number')
            if not math.isfinite(value):
                raise ValueError(f'{name}: value must be finite')
            if self.params is None or abs(value - float(self.params.get(name, math.nan))) > 1e-12 * max(1, abs(value)) or name not in self.params:
                out[name] = value
        return out

    def apply_params(self):
        a = self.app; a.require_disarmed()
        changes = self.edited()
        if not changes:
            a.report('Controller knobs', 'No values were changed.'); return
        a.submit(lambda: a.device.onboard_params(changes), lambda p: (self.params_loaded(p), a.report(
            'Controller knobs applied', 'Changed in device RAM: ' + ', '.join(f'{k} = {v:g}' for k, v in changes.items()))), 'Applying knobs…')

    def restore_params(self):
        a = self.app; a.require_disarmed()
        a.submit(lambda: a.device.onboard_params(defaults=True), lambda p: (self.params_loaded(p), a.report(
            'Controller knobs', 'Design values restored in device RAM.')), 'Restoring design values…')

    def start(self):
        a = self.app; d = a.device
        if not self.supported():
            raise RuntimeError('Connect to firmware 1.6 or newer (onboard control)')
        entry = CONTROLLERS[self.controller.currentIndex()]
        a.authorize_run()                                  # DISARMED, 12 V box, centered, finite limits; enables motion
        if a.busy:
            d.inhibit = True; raise RuntimeError('Wait for the current task or cancel it')
        meta = a.metadata() | {'purpose': 'onboard', 'controller': entry['key'], 'controller_label': entry['label'],
                               'start': entry['start'], 'onboard_params': copy.deepcopy(self.params)}
        path = None
        if self.record.isChecked():
            path = a.new_path('runs'); d.start_record(path, meta)
        self.event_mark = d.event_count

        def run():
            try:
                return d.run_onboard(**entry['start'])
            except Exception:
                d.stop(); d.finish_record(); raise

        def done(r):
            a.running = True; a.run_start = None; a.run_metadata = meta
            self.active = True; self.seen_running = False; self.t_start = time.monotonic(); self.end_label.setText('')
            warn = f'\n\n⚠ {r["warning"]}' if r.get('warning') else ''
            a.report('Onboard controller running', f'**{r.get("mode", entry["label"])}**, up to {r.get("seconds", "?")} s. '
                     + (f'Recording to {path}. ' if path else 'Not recording. ') + 'STOP / DISABLE stops the motor.' + warn)
        try:
            a.submit(run, done, 'Arming and starting the onboard controller…')
        except Exception:
            d.inhibit = True; d.finish_record(); raise

    def recording_finished(self, path, rows, manifest):
        """Called by LabApp when a run is saved or loaded; summarises onboard runs."""
        if (manifest.get('metadata') or {}).get('purpose') != 'onboard':
            return
        m = manifest.get('metadata') or {}
        text = summary(rows, manifest.get('config') or {}, manifest)
        end = f'\n\nStop reason: {self.last_end}.' if self.last_end else ''
        self.app.report(f'Onboard run: {m.get("controller_label", m.get("controller", ""))}', f'{text}{end}\n\n{Path(path).name}')
