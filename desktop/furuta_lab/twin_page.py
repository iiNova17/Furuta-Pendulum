"""Digital twin tab: the rig in 3D, posed live from the stream (or from an opened recording), with the identified model
as a translucent ghost next to it. The 3D view (twin_view.TwinView) sits in the app's right-hand panel; it can also be
opened in its own window and shown on the Onboard control tab.

Rig pose: newest stream record -> theta0, theta1 exactly as the firmware computes them (onboard_page.angles).
Model ghost: twin_core.ModelGhost (identified model with breakaway, driven by the applied duty, re-synced to the
measured state every N s), or poses received over UDP (RIP1, e.g. MATLAB twin_send / twin_play).
Optional: forward both poses over UDP to the Unreal twin (same RIP1 packet).
"""
import math
import time
import numpy as np
from PySide6 import QtCore, QtWidgets as W
from . import twin_core as T
import copy
from .onboard_page import angles, STREAM_HZ

GHOST_MODES = ['Identified model (driven by the applied duty)', 'UDP packets (MATLAB twin_send / twin_play)', 'Off']


class TwinPage:
    def __init__(self, app):
        self.app = app; self.view = None; self.popout = None; self.error = ''
        self.ghost = None; self.link = T.UdpLink(); self.last_us = None
        self.play = None; self.frames = []; self.hud_at = 0.; self.fps_t = []

    # ------------------------------------------------------------------ layout
    def build(self, v):
        a = self.app
        a.text(v, 'The rig in 3D, posed on every display frame from the newest stream record (no network hop: the view runs '
                  'in the app). Solid = rig, translucent cyan = the identified model. Mouse: drag to orbit, wheel to zoom.', small=True)
        try:
            from .twin_view import TwinView
            self.view = TwinView()
            if not self.view.ok:
                self.error = self.view.error; self.view = None
        except Exception as e:                                   # e.g. PySide6-Addons (QtQuick3D) not installed
            self.error = f'{type(e).__name__}: {e}'; self.view = None
        if self.error:
            a.text(v, f'⚠ The 3D view is not available: {self.error}\n\nInstall the PySide6-Addons wheel '
                      '(Setup-Furuta-Lab.ps1 installs desktop/requirements.txt).', small=False)

        g = a.group(v, '1 · Source')
        self.src_live = W.QRadioButton('Live: the connected rig'); self.src_live.setChecked(True); g.addWidget(self.src_live)
        self.src_rec = W.QRadioButton('Playback: the open recording'); g.addWidget(self.src_rec)
        row = W.QHBoxLayout(); g.addLayout(row)
        self.play_button = a.button(row, 'Play / pause', self.toggle_play, action=False)
        a.button(row, 'Restart', self.restart, action=False)
        self.speed = W.QComboBox(); self.speed.addItems(['0.25×', '0.5×', '1×', '2×']); self.speed.setCurrentIndex(2); row.addWidget(self.speed)
        self.slider = W.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(0, 1000); g.addWidget(self.slider)
        self.slider.sliderMoved.connect(self.seek)
        self.rec_label = a.text(g, 'No recording open (Open recording… or record a run).', small=True)
        self.src_live.toggled.connect(lambda on: self.source_changed())
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, f'Live stream at {STREAM_HZ} Hz (smoother twin)', self.fast_stream)
        self.stream_label = W.QLabel(''); self.stream_label.setStyleSheet('color:#5d6d7e;font-size:12px;'); row.addWidget(self.stream_label, 1)

        g = a.group(v, '2 · Model ghost')
        self.ghost_mode = W.QComboBox(); self.ghost_mode.addItems(GHOST_MODES); g.addWidget(self.ghost_mode)
        self.resync = a.number(g, 'Re-sync the model to the rig every (s, 0 = never)', 2, 0, 600, 1)
        self.udp_port = a.number(g, 'UDP port to listen on', T.UDP_PORT, 1, 65535, 0)
        a.text(g, 'The model integrates the identified equations (with the measured breakaway friction) from the duty the '
                  'firmware applied, starting from the measured state; the re-sync shows how far it drifts over that horizon.',
               small=True)
        self.ghost_mode.currentIndexChanged.connect(lambda *_: self.ghost_changed())
        self.resync.valueChanged.connect(lambda *_: self.ghost_changed())

        g = a.group(v, '3 · Forward to the Unreal twin (optional)')
        self.fwd = W.QCheckBox('Send both poses as RIP1 UDP packets'); g.addWidget(self.fwd)
        row = W.QHBoxLayout(); g.addLayout(row)
        self.fwd_host = W.QLineEdit('127.0.0.1'); row.addWidget(self.fwd_host)
        self.fwd_port = W.QSpinBox(); self.fwd_port.setRange(1, 65535); self.fwd_port.setValue(T.UDP_PORT + 1); row.addWidget(self.fwd_port)
        a.text(g, 'Same packet as twin/README.md (UE 5 twin on main). Use a port other than the one listened on above.', small=True)
        self.fwd.toggled.connect(lambda on: self.forward_changed())

        g = a.group(v, '4 · View')
        row = W.QHBoxLayout(); g.addLayout(row)
        a.button(row, 'Reset camera', self.reset_camera, action=False)
        a.button(row, 'Open in its own window', self.open_window, action=False)
        self.on_onboard = W.QCheckBox('Show the twin instead of the plots on the Onboard control tab'); g.addWidget(self.on_onboard)
        self.on_onboard.toggled.connect(lambda on: self.app.page_changed(self.app.nav.currentIndex()))
        self.latency = a.text(g, '', small=True)
        a.text(g, 'Latency: the firmware (1.11) sends every record as soon as it is taken (250 records/s during onboard runs). '
                  'The rig\'s USB bridge is an FTDI FT232R, whose driver holds data up to its latency timer (16 ms by default): '
                  'Device Manager → Ports → USB Serial Port → Properties → Port Settings → Advanced → Latency Timer = 1 ms '
                  'removes most of the remaining delay.', small=True)

        self.timer = QtCore.QTimer(self.app); self.timer.setTimerType(QtCore.Qt.PreciseTimer); self.timer.setInterval(8)
        self.timer.timeout.connect(self.tick)
        self.ghost_changed()

    # ------------------------------------------------------------------ visibility
    def views(self):
        return [x for x in (self.view, self.popout) if x is not None]

    def set_active(self, on):
        """Run the frame timer only while a twin view is visible."""
        on = on and bool(self.views())
        if on and not self.timer.isActive():
            self.timer.start()
        elif not on and self.timer.isActive() and not (self.popout and self.popout.isVisible()):
            self.timer.stop()

    def open_window(self):
        if self.error:
            raise RuntimeError(f'3D view not available: {self.error}')
        if self.popout is None:
            from .twin_view import TwinView
            self.popout = TwinView(); self.popout.setWindowTitle('Furuta Lab — digital twin'); self.popout.resize(900, 700)
        self.popout.show(); self.popout.raise_(); self.timer.start()

    def reset_camera(self):
        for v in self.views():
            v.reset_camera()

    # ------------------------------------------------------------------ settings
    def ghost_changed(self):
        mode = self.ghost_mode.currentIndex()
        self.ghost = T.ModelGhost(resync_s=self.resync.value()) if mode == 0 else None
        self.last_us = None
        if mode == 1:
            try:
                self.link.listen(int(self.udp_port.value()))
            except OSError as e:
                self.app.report('Digital twin', f'Cannot listen on UDP port {int(self.udp_port.value())}: {e}. '
                                                '(The Unreal twin may be using it; close it or choose another port.)')
        else:
            self.link.close_rx()
        for v in self.views():
            v.show_ghost(mode != 2)

    def forward_changed(self):
        if self.fwd.isChecked():
            self.link.forward(self.fwd_host.text().strip() or '127.0.0.1', self.fwd_port.value())
        else:
            self.link.stop_forward()

    def fast_stream(self):
        """Raise the display stream (RAM setting, like the onboard run does); the twin then gets every 4 ms record."""
        a = self.app; d = a.device
        if not d.ready:
            raise RuntimeError('Connect first')
        if d.config.get('streamHz') == STREAM_HZ:
            a.statusBar().showMessage(f'Stream is already {STREAM_HZ} Hz'); return
        a.require_disarmed(); c = copy.deepcopy(d.config); c['streamHz'] = STREAM_HZ
        a.submit(lambda: d.apply(c), lambda _: a.statusBar().showMessage(f'Stream {STREAM_HZ} Hz (device RAM)'), 'Raising the stream rate…')

    def run_changed(self):
        """LabApp.set_run: a recording was opened or saved."""
        if self.src_rec.isChecked():
            self.load_recording()
        elif self.app.run_data is not None:
            name = self.app.run_path.name if self.app.run_path else 'recording'
            self.rec_label.setText(f'{name} is open: choose Playback to replay it.')

    # ------------------------------------------------------------------ playback
    def source_changed(self):
        self.play = None; self.last_us = None
        if self.ghost:
            self.ghost.reset()
        if self.src_rec.isChecked():
            self.load_recording()

    def load_recording(self):
        a = self.app
        if a.run_data is None or not len(a.run_data):
            self.rec_label.setText('No recording open (Open recording… or record a run).'); self.frames = []; return
        rows = a.run_data; m = a.run_manifest or {}
        t = (rows['device_us'].astype(np.int64) - int(rows['device_us'][0])) * 1e-6
        self.frames = (rows, m.get('config') or {}, m.get('status') or {}, t)
        name = a.run_path.name if a.run_path else 'recording'
        self.rec_label.setText(f'{name}: {t[-1]:.1f} s, {len(rows)} records ({(m.get("metadata") or {}).get("purpose", "run")})')
        self.play = {'t': 0., 'running': True, 'wall': time.perf_counter(), 'k': 0}
        if self.ghost:
            self.ghost.reset()

    def toggle_play(self):
        if not self.src_rec.isChecked():
            self.src_rec.setChecked(True); return
        if self.play is None:
            self.load_recording(); return
        self.play['running'] = not self.play['running']; self.play['wall'] = time.perf_counter()

    def restart(self):
        if self.play:
            self.play.update(t=0., k=0, wall=time.perf_counter(), running=True)
            if self.ghost:
                self.ghost.reset()

    def seek(self, pos):
        if self.play and self.frames:
            self.play.update(t=pos / 1000 * self.frames[3][-1], wall=time.perf_counter())
            self.play['k'] = int(np.searchsorted(self.frames[3], self.play['t']))
            if self.ghost:
                self.ghost.reset()

    # ------------------------------------------------------------------ frame
    def tick(self):
        views = [v for v in self.views() if v.isVisible()]
        if not views:
            return
        now = time.perf_counter(); self.fps_t.append(now); self.fps_t = [x for x in self.fps_t if now - x < 1.]
        rig = ghost = None; info = []; age_ms = None; ctrl = 0
        if self.src_rec.isChecked() and self.frames:
            rows, cfg, st, t = self.frames; p = self.play
            if p['running']:
                p['t'] += (now - p['wall']) * [0.25, 0.5, 1., 2.][self.speed.currentIndex()]
                if p['t'] > t[-1]:
                    p['t'] = t[-1]; p['running'] = False
            p['wall'] = now
            k1 = max(1, int(np.searchsorted(t, p['t'], side='right')))
            new = range(p['k'], k1) if k1 > p['k'] else range(k1 - 1, k1)
            p['k'] = k1; self.slider.blockSignals(True); self.slider.setValue(int(1000 * p['t'] / max(t[-1], 1e-9))); self.slider.blockSignals(False)
            rig, ghost, ctrl = self._feed(rows, new, cfg, st)
            info.append(f'playback {p["t"]:5.2f} / {t[-1]:.2f} s')
        elif self.src_live.isChecked() and self.app.device.ready:
            d = self.app.device; rows = d.snapshot(64)
            if len(rows):
                us = rows['device_us'].astype(np.int64)
                new = range(len(rows) - 1, len(rows)) if self.last_us is None else range(int(np.searchsorted(us, self.last_us, side='right')), len(rows))
                self.last_us = int(us[-1])
                rig, ghost, ctrl = self._feed(rows, new, d.config, d.status)
                age_ms = self._age_ms(d, int(us[-1]), now)
                if not d.status.get('centered'):
                    info.append('arm not centered: arm drawn at 0')
        # UDP poses
        mode = self.ghost_mode.currentIndex()
        if mode == 1:
            got = self.link.poll()
            if 1 in got:
                ghost = got[1]
            if rig is None and 0 in got:
                rig = got[0]
        if rig is not None:
            for v in views:
                v.set_pose('rig', *rig)
            self.link.send(0, *rig)
        if ghost is not None and mode != 2:
            for v in views:
                v.show_ghost(True); v.set_pose('ghost', *ghost)
            self.link.send(1, *ghost)
        if now - self.hud_at > .1:                                # text at 10 Hz, poses every frame
            self.hud_at = now
            hud = [f'rig    arm {self._deg(rig, 0)}   pend {self._deg(rig, 1, True)}']
            if ghost is not None and mode != 2:
                hud.append(f'model  arm {self._deg(ghost, 0)}   pend {self._deg(ghost, 1, True)}')
            mode_name = {1: 'SAFE STOP', 2: 'SWING-UP', 3: 'STABILIZE'}.get(ctrl)
            if mode_name:
                hud.append(f'controller {mode_name}')
            if age_ms is not None:
                hud.append(f'pose age +{age_ms:4.1f} ms  ·  {len(self.fps_t)} fps')
            hud += info
            for v in views:
                v.set_hud('\n'.join(hud))
            d = self.app.device; hz = d.config.get('streamHz') if d.ready else None
            self.stream_label.setText(f'stream now {hz} Hz' if hz else '')
            self.latency.setText(f'Display {len(self.fps_t)} frames/s' + (f' · stream {hz} Hz' if hz else '')
                                 + (f' · pose age {age_ms:.1f} ms above the fastest packet of the last 2 s' if age_ms is not None else ''))

    def _feed(self, rows, idx, cfg, st):
        """Pose of the last record in idx; feeds every record of idx to the model ghost."""
        rig = ghost = None; ctrl = 0
        for k in idx:
            r = rows[k]; th0, th1 = angles(r, cfg, st)
            th0 = math.radians(th0) if th0 is not None else (0. if th1 is not None else None)
            th1 = math.radians(th1) if th1 is not None else None
            if th1 is None:
                continue
            rig = (th0, (th1 % (2 * math.pi)) - math.pi)            # q_pend in [-pi, pi), 0 hanging
            ctrl = int(r['ctrl']) if 'ctrl' in r.dtype.names else 0
            if self.ghost is not None:
                x = self.ghost.feed(int(r['device_us']) * 1e-6, float(r['applied']), th0, th1)
                if x is not None:
                    ghost = (x[0], (x[1] % (2 * math.pi)) - math.pi)
        return rig, ghost, ctrl

    @staticmethod
    def _age_ms(d, dev_us, now):
        log = list(d.rx_log)[-400:]
        if len(log) < 5:
            return None
        offset = min(h - u * 1e-6 for h, u in log)                # fastest packet over the last ~2 s
        return max(0., (now - (dev_us * 1e-6 + offset)) * 1e3)

    @staticmethod
    def _deg(pose, i, pend=False):
        if pose is None or pose[i] is None:
            return '   —  '
        v = math.degrees(pose[i] + (math.pi if pend else 0.))
        v = (v + 180) % 360 - 180 if pend else v
        return f'{v:+7.1f}°'

    def close(self):
        self.timer.stop(); self.link.close()
        if self.popout:
            self.popout.close()
