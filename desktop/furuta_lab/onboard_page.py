"""Onboard controller page (firmware 1.6+; controller period choice and timing benchmark need 1.7).

The ESP32 runs the controller itself (firmware/include/onboard.h, every 1, 2 or 4 ms). This page only
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
from .backend import save_capture

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
    'fricGain': ('–', 'friction compensation in balance (1 = on, the 1.9 default; 0 = off)'),
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
    'tsp': ('A', 'arm breakaway friction, positive (measured dead zone 13 %)'),
    'tsn': ('A', 'arm breakaway friction, negative (measured dead zone 14 %)'),
    'stickGain': ('–', 'breakaway feedforward in the swing-up (1 = on, 0 = the 1.7 law)'),
    'wStick': ('rad/s', 'arm speed over which the breakaway part fades out'),
    'aStick': ('rad/s²', 'smoothing of the sign of the wanted arm acceleration'),
    'fricStick': ('–', 'balance: breakaway duty at rest, fading to Coulomb as the arm moves (1 = on, 0 = Coulomb only)'),
}
READ_ONLY = {'Ts', 'rearm', 'modelArmSign', 'modelPendSign'}
# Controller periods (firmware 1.7: onboard_rates.h). The LQR gain for each comes from the design's Q and R.
# On the rig (2026-11-19) 1 ms never swung up: the speed estimates are noisier at 1 ms, the duty request
# jumpier (slew-clipped 7 % of the time vs 2 %) and the pump weaker, so 4 ms (the Simulink design) is the
# default and 2 ms the alternative. The first entry is the default.
PERIODS = [(0.004, '4 ms — as designed and validated in Simulink (default)'), (0.002, '2 ms'),
           (0.001, '1 ms — did not swing up on the rig (noisier speed estimates)')]
STREAM_HZ = 250            # fastest stream that fits 460800 baud with margin (500 Hz records / 2)
BURST_BUDGET = 4194304     # bytes the firmware allows for a capture (HELLO maxBurstBytes)
CTRL_STYLE = {0: ('NOT RUNNING', '#6b7785'), 1: ('SAFE STOP', '#7a1020'), 2: ('SWING-UP', '#c77700'), 3: ('STABILIZE', '#1f7a4d')}
# Firmware notices that end an onboard run, with a hint for the operator.
STOP_HINTS = {
    'ONBOARD_SAFE_STOP': 'the arm passed safeTh0 (|θ0| limit) and the supervisor stopped the motor',
    'ONBOARD_PENDULUM_LOST': 'balance-only mode and the pendulum left the catch region',
    'PEND_INVALID': 'the pendulum reading was older than 20 ms or the magnet status was bad',
}


def run_length(config):
    m = config.get('maxDuration', 0)
    return 'no limit' if not m else f'{m:g} s'


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
    arm = series['arm_deg']; cur = series['current_smooth']
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
        self.burst_armed = None; self.pending_burst = None; self.run_path = None; self.run_meta = {}

    # ------------------------------------------------------------------ layout
    def build(self, v):
        a = self.app
        a.text(v, 'The ESP32 runs the controller itself (firmware 1.11: every 4, 2 or 1 ms, pendulum read at 1 kHz, all firmware trips active). '
                  'This page starts it, keeps the heartbeat, shows its state and records the run. '
                  'STOP / DISABLE, unticking the 12 V box or closing the app stops the motor (heartbeat within 300 ms).', small=True)

        g = a.group(v, '1 · Readiness')
        self.checklist = W.QLabel('Connect USB to check.'); self.checklist.setWordWrap(True); self.checklist.setTextFormat(QtCore.Qt.RichText)
        g.addWidget(self.checklist)
        self.rec_seconds = a.number(g, 'Run length — maxDuration (s)', 20, .5, 3600, 1)
        self.no_limit = W.QCheckBox('No run-length limit (runs until STOP, a trip or a fault; firmware 1.8)'); g.addWidget(self.no_limit)
        self.no_limit.toggled.connect(lambda on: self.rec_seconds.setEnabled(not on))
        self.rec_duty = a.number(g, 'Maximum duty (%)', 50, 0, 100, 1)
        self.rec_slew = a.number(g, 'Slew rate (%/s)', 10000, .1, 100000, 0)
        self.rec_trip = a.number(g, 'Current trip A (0 = off)', 4, 0, 50, 2)
        self.ignore_current = W.QCheckBox('Ignore the current sensor during runs (no ADC_INVALID_OR_CLIPPED or overcurrent trip; firmware 1.10)')
        self.ignore_current.setToolTip('The current is still measured and recorded, but no longer stops a run. Positive currents above about '
                                       '3.2 A clip the sensor at the top of the ADC; with this ticked nothing limits the current except the '
                                       'duty limit, so keep max duty moderate (the TB6612 is rated 1.2 A continuous / 3.2 A peak).')
        g.addWidget(self.ignore_current)
        a.button(g, 'Apply these onboard-run limits to RAM', self.apply_limits)
        a.text(g, 'Designed with 100 /s slew (10000 %/s) and 50 % duty; the firmware warns below 50 /s. '
                  'Travel stays as commissioned on Motor tests. Current trip: ' + 'Default 4 A. Note: the current sensor reaches the top of the ESP32 ADC at about +3.2 A, where ADC_INVALID_OR_CLIPPED stops the motor first; the TB6612 is rated 1.2 A continuous / 3.2 A peak.', small=True)
        self.sign_button = a.button(g, 'Set profile signs to the model’s (armSign +1, pendSign −1)', self.set_signs)

        g = a.group(v, '2 · Run — MOVES MOTOR')
        self.controller = W.QComboBox(); self.controller.addItems([c['label'] for c in CONTROLLERS]); g.addWidget(self.controller)
        self.how = a.text(g, '', small=True); self.controller.currentIndexChanged.connect(self.controller_changed); self.controller_changed()
        self.period = W.QComboBox(); g.addWidget(W.QLabel('Controller period'))
        for ts, label in PERIODS:
            self.period.addItem(label, ts)
        g.addWidget(self.period)
        self.record = W.QCheckBox('Record the run to data/runs (stream raised to 250 Hz for the run)'); self.record.setChecked(True); g.addWidget(self.record)
        self.burst = W.QCheckBox('High-rate capture on the MCU (500 Hz records, current at 2 kHz), downloaded after the run')
        self.burst.setChecked(True); g.addWidget(self.burst)
        self.start_button = a.button(g, 'Start onboard controller — MOVES MOTOR', self.start)
        self.start_button.setStyleSheet('font-weight:700;'); self.start_button.setEnabled(False)
        self.clear_button = a.button(g, 'Clear fault', lambda: a.submit(lambda: a.device.rpc('CLEAR_FAULT'),
                                                                        lambda r: a.report('Fault cleared', 'The device is DISARMED again.')))
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
        a.button(g, 'Measure controller CPU time on the MCU', self.bench)
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
        if c.get('currentChecks') is False:
            out.append((None, 'Current sensor IGNORED in runs: no ADC clipping or overcurrent trip (only the duty limit bounds the current)'))
        else:
            out.append((True if trip > 0 else None, f'Current trip {trip:g} A' if trip > 0 else 'Current trip OFF (4 A is the default)'))
        if s.get('burstActive') or s.get('burstPending'):
            out.append((None, 'A high-rate capture is ' + ('recording' if s.get('burstActive') else 'armed') + ' on the device'
                        + ('' if self.pending_burst or self.active else ' (Setup → “Cancel device capture” if it is left over)')))
        if s.get('state') == 'FAULT':
            out.append((False, f'FAULT {s.get("fault", "")}: press “Clear fault” below'))
        out.append((True, f'Run length {run_length(c)} · heartbeat {c.get("heartbeatMs", "?")} ms'))
        periods = d.identity.get('onboardPeriods') or [0.004]
        ts = self.period.currentData()
        out.append((any(abs(ts - p) < 1e-9 for p in periods),
                    f'Controller period {ts * 1e3:g} ms (firmware offers {", ".join(f"{p * 1e3:g}" for p in periods)} ms; '
                    f'pendulum read at {d.identity.get("pendHz", 500)} Hz)'))
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
        if self.pending_burst and not a.busy and not a.pending_load and not self.active:
            seconds, path = self.pending_burst; self.pending_burst = None
            try:
                self.download(seconds, path)
            except Exception as e:
                a.failed(f'High-rate capture not downloaded: {e}', release=False)
        if not a.busy:
            self.start_button.setEnabled(self.supported() and a.power.isChecked() and state == 'DISARMED')
            self.clear_button.setEnabled(d.ready and state == 'FAULT')
            c = d.config if d.ready else {}
            self.sign_button.setVisible(bool(p) and (c.get('armSign') != p.get('modelArmSign') or c.get('pendSign') != p.get('modelPendSign')))

    def finished_run(self):
        d = self.app.device; s = d.status
        reason = 'STOP / DISABLE' if not d.config.get('maxDuration') else 'reached the run length (maxDuration) or STOP'
        if s.get('state') == 'FAULT':
            reason = f'FAULT {s.get("fault", "?")}'
        if d.event_count > self.event_mark and (d.last_event or {}).get('event') in STOP_HINTS:
            ev = d.last_event['event']; reason = f'{ev}: {STOP_HINTS[ev]}'
        self.last_end = reason
        self.end_label.setText(f'Last run ended: {reason}.')
        self.app.output.append(f'\n**Onboard run ended:** {reason}.')
        if self.burst_armed:
            self.pending_burst = (self.burst_armed, self.run_path); self.burst_armed = None

    def download(self, seconds, run_path):
        a = self.app; d = a.device; meta = dict(self.run_meta) | {'purpose': 'onboard_burst'}
        target = Path(run_path) / 'burst' if run_path else a.new_path('captures')

        def job():
            r = d.download_burst(seconds + 5, a.signals.progress.emit, a.cancel, 'Waiting for the high-rate capture to finish')
            save_capture(target, r, meta)
            return r
        a.submit(job, lambda r: self.burst_done(r, target), 'Downloading the high-rate capture…')

    def burst_done(self, r, target):
        core, adc, c = r['core'], r['adc'], r.get('config') or {}
        text = summary(core, c, {'status': r.get('status') or {}})
        zero, sens = c.get('currentZeroMv'), c.get('sensitivityMvA')
        if len(adc) and S.finite(zero) and S.finite(sens) and sens:
            i = (adc['current_mv'].astype(float) - zero) / sens
            text += f'\n\nCurrent at {r["info"].get("adcHz", c.get("adcHz", "?"))} Hz: peak |i| {np.max(np.abs(i)):.2f} A (unfiltered samples).'
        self.app.output.append(f'\n\n### High-rate capture: {len(core)} records (500 Hz), {len(adc)} current samples\n\n{text}\n\nSaved to {target}')

    # ----------------------------------------------------------------- actions
    def apply_limits(self):
        a = self.app; a.require_disarmed(); c = copy.deepcopy(a.device.config)
        if self.no_limit.isChecked() and not a.device.identity.get('runLengthUnlimited'):
            raise RuntimeError('“No run-length limit” needs firmware 1.8 or newer')
        if self.ignore_current.isChecked() and not a.device.identity.get('currentChecksOption'):
            raise RuntimeError('“Ignore the current sensor” needs firmware 1.10 or newer')
        if a.device.identity.get('currentChecksOption'):
            c['currentChecks'] = not self.ignore_current.isChecked()
        c.update(maxDuration=0. if self.no_limit.isChecked() else self.rec_seconds.value(), maxDuty=self.rec_duty.value() / 100,
                 slewPerSecond=self.rec_slew.value() / 100, currentTripA=self.rec_trip.value())
        a.submit(lambda: a.device.apply(c), lambda r: a.report('Onboard-run limits applied to RAM',
                 ('⚠ current sensor ignored in runs (no clipping or overcurrent trip) · ' if c.get('currentChecks') is False else '') +
                 f'run length {run_length(c)} · maxDuty {c["maxDuty"] * 100:g} % · slew {c["slewPerSecond"]:g} /s · '
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
        periods = d.identity.get('onboardPeriods') or [0.004]
        ts = self.period.currentData()
        if not any(abs(ts - p) < 1e-9 for p in periods):
            d.inhibit = True; raise ValueError(f'This firmware supports only {", ".join(f"{p * 1e3:g} ms" for p in periods)}; flash 1.7 for faster periods')
        if d.record:
            d.inhibit = True; raise RuntimeError('Finish the live recording (Live tab) before starting an onboard run')
        if self.pending_burst:
            d.inhibit = True; raise RuntimeError('The previous run’s high-rate capture has not been downloaded yet; wait a moment')
        record = self.record.isChecked(); burst = self.burst.isChecked()
        path = a.new_path('runs') if record else None
        self.event_mark = d.event_count; self.burst_armed = None; self.run_path = path

        def run():
            notes = []; armed = False
            try:
                info = d.rpc('BURST_INFO')
                if (info.get('active') or info.get('pending')) and d.cancel_burst() is not None:
                    notes.append('a left-over device capture was cancelled')
                # Profile changes must happen before recording starts (the MCU refuses them while recording).
                if record and d.config.get('streamHz') != STREAM_HZ:
                    c = copy.deepcopy(d.config); c['streamHz'] = STREAM_HZ; d.apply(c); notes.append(f'stream {STREAM_HZ} Hz')
                params = d.onboard_params()
                if abs(params.get('Ts', 0.004) - ts) > 1e-9:
                    params = d.onboard_params({'Ts': ts}); notes.append(f'controller period {ts * 1e3:g} ms (design LQR gain for it)')
                armed_s = None
                if burst:
                    c = d.config; per_s = c.get('adcHz', 2000) * 24 + 500 * 128     # bytes per second (record sizes rounded up)
                    limit = c.get('maxDuration', 20) + .5 if c.get('maxDuration', 0) > 0 else math.inf
                    armed_s = round(min(limit, .95 * BURST_BUDGET / per_s), 1)
                    d.arm_burst(armed_s); armed = True; notes.append(f'high-rate capture armed for {armed_s:g} s')
                meta = a.metadata() | {'purpose': 'onboard', 'controller': entry['key'], 'controller_label': entry['label'],
                                       'start': entry['start'], 'controller_period_s': ts, 'onboard_params': copy.deepcopy(params),
                                       'burst_seconds': armed_s}
                if path:
                    d.start_record(path, meta)
                r = d.run_onboard(**entry['start'])
                return r, params, meta, armed_s, notes
            except Exception as e:
                d.stop(); d.finish_record()
                if armed:
                    try:
                        d.cancel_burst()          # otherwise it stays pending and blocks every profile change
                    except Exception:
                        pass
                if 'Capture/calibration in progress' in str(e):
                    raise RuntimeError(d.busy_text()) from e
                raise

        def done(result):
            r, params, meta, armed_s, notes = result
            self.params_loaded(params)
            a.running = True; a.run_start = None; a.run_metadata = meta; self.run_meta = meta; self.burst_armed = armed_s
            self.active = True; self.seen_running = False; self.t_start = time.monotonic(); self.end_label.setText('')
            warn = f'\n\n⚠ {r["warning"]}' if r.get('warning') else ''
            a.report('Onboard controller running', f'**{r.get("mode", entry["label"])}** every {r.get("Ts", ts) * 1e3:g} ms, '
                     + (f'up to {r["seconds"]:g} s. ' if r.get('seconds') else 'no run-length limit: STOP / DISABLE ends it. ')
                     + (f'Recording to {path}. ' if path else 'Not recording. ') + 'STOP / DISABLE stops the motor.'
                     + (f'\n\nSet for this run: {"; ".join(notes)}.' if notes else '') + warn)
        try:
            a.submit(run, done, 'Preparing, arming and starting the onboard controller…')
        except Exception:
            d.inhibit = True; d.finish_record(); raise

    def bench(self):
        a = self.app
        if not self.supported():
            raise RuntimeError('Connect to firmware with onboard control')
        if not a.device.identity.get('onboardPeriods'):
            raise RuntimeError('The CPU-time measurement needs firmware 1.7')

        def done(r):
            us = r['usPerStep']
            rows = ' · '.join(f'{p * 1e3:g} ms period: {100 * us / (p * 1e6):.1f} %' for p, _ in PERIODS)
            a.report('Controller CPU time', f'**{us:.1f} µs per controller step** ({r["steps"]} steps, current knobs). '
                     f'Share of the period — {rows}. The 1 ms firmware loop also runs the safety checks and the PWM; '
                     'an overrun trips CONTROL_OVERRUN.')
        a.submit(lambda: a.device.onboard_bench(2000), done, 'Timing the controller on the MCU…')

    def recording_finished(self, path, rows, manifest):
        """Called by LabApp when a run is saved or loaded; summarises onboard runs."""
        if (manifest.get('metadata') or {}).get('purpose') != 'onboard':
            return
        m = manifest.get('metadata') or {}
        text = summary(rows, manifest.get('config') or {}, manifest)
        end = f'\n\nStop reason: {self.last_end}.' if self.last_end else ''
        self.app.report(f'Onboard run: {m.get("controller_label", m.get("controller", ""))}', f'{text}{end}\n\n{Path(path).name}')
