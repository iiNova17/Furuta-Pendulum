"""Digital twin: geometry, kinematics, the model ghost and the RIP1 UDP packet (no Qt here, so it is testable).

Geometry comes from the Unreal twin (main: twin/RIPTwin/Content/RIP, written by twin/matlab/export_twin_meshes.m from the
CAD): three meshes in the zero pose (arm at 0, pendulum hanging) in Unreal's frame, 1 unit = 1 mm. Here they are turned
back into the CAD / Simscape world frame (right-handed, Z up): world = (x, -y, z), triangle winding restored.

Conventions (twin/README.md): q_arm = theta0 (right-handed about +Z), q_pend = theta1 - pi (0 hanging, pi upright).
The pendulum turns about pend_axis through pivot, both in the arm's frame at q_arm = 0.
"""
import json
import math
import socket
import struct
from pathlib import Path
import numpy as np
from . import estimation as E

ASSETS = Path(__file__).resolve().parent / 'twin_assets'
BODIES = ('housing', 'arm', 'pendulum')
UDP_PORT = 7400
_PACKET = struct.Struct('<4sB3xff')            # 'RIP1', source (0 rig, 1 model), q_arm, q_pend
# Drawing signs (q_arm, q_pend): on the rig, positive theta0 / theta1 turn the opposite way to the CAD frame's
# right-hand rule about +Z / the pendulum axis (both seen reversed in the twin, checked at the rig 2026-11-20).
# Flipping both is a mirror symmetry of the dynamics, so only the drawing changes; packets keep the convention.
DISPLAY_SIGN = (-1., -1.)


# ------------------------------------------------------------------ geometry
def load_ripmesh(path):
    """(positions, normals): float32 arrays (3 n, 3) in the world frame [mm], unshared vertices, flat normals."""
    b = Path(path).read_bytes()
    if b[:4] != b'RIPM':
        raise ValueError(f'{path}: not a .ripmesh file')
    n = struct.unpack_from('<I', b, 4)[0]
    v = np.frombuffer(b, '<f4', count=9 * n, offset=8).reshape(n, 3, 3).astype(np.float32)
    v = v * np.array([1, -1, 1], np.float32)            # Unreal (left-handed) -> world
    v = v[:, [0, 2, 1], :]                              # undo the winding flip of the export
    nrm = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    return v.reshape(-1, 3), np.repeat(nrm, 3, axis=0).astype(np.float32)


def load_geometry():
    """pivot [mm] and unit pendulum axis in the world frame (arm at 0)."""
    g = json.loads((ASSETS / 'twin_geometry.json').read_text(encoding='utf-8'))
    pivot = np.array(g['pivot'], float) * np.array([1, -1, 1])
    axis = np.array(g['pend_axis'], float) * np.array([1, -1, 1])
    return pivot, axis / np.linalg.norm(axis)


def rotation(axis, angle):
    """Right-handed rotation matrix about a unit axis."""
    x, y, z = axis; c, s = math.cos(angle), math.sin(angle); C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def pose_pendulum(points, q_arm, q_pend, pivot, axis):
    """World positions of pendulum points (given in the zero pose) for the joint angles."""
    Rp = rotation(axis, q_pend); Ra = rotation((0., 0., 1.), q_arm)
    return (Ra @ (pivot[:, None] + Rp @ (np.asarray(points, float).T - pivot[:, None]))).T


# ------------------------------------------------------------------ model ghost
def load_model():
    m = json.loads((ASSETS / 'model_params.json').read_text(encoding='utf-8'))
    return E.full(m['params']), float(m.get('Vs', 12.))


class ModelGhost:
    """The identified model (with breakaway) run in real time next to the rig, driven by the duty the firmware applied.

    feed(t, duty, theta0, theta1) with one new record at a time (device time t [s]). The model integrates from the
    previous record to t with that record's duty (RK4, steps <= max_dt). Every `resync_s` seconds (0 = never) it is
    put back on the measured state, so the ghost shows how far the model drifts from the rig over that horizon.
    """
    def __init__(self, params=None, Vs=None, resync_s=2., max_dt=.001):
        if params is None:
            params, vs = load_model(); Vs = Vs or vs
        self.p, self.Vs, self.resync_s, self.max_dt = params, Vs or 12., resync_s, max_dt
        self.reset()

    def reset(self):
        self.x = None; self.t = None; self.d = 0.; self.t_sync = None; self.hist = []

    def _measured_state(self):
        """theta0, theta1 and their speeds from the last few records (least-squares slope over ~20 ms)."""
        h = self.hist[-6:]
        t = np.array([r[0] for r in h]); a0 = np.array([r[1] for r in h]); a1 = np.unwrap(np.array([r[2] for r in h]))
        if len(h) < 3 or t[-1] - t[0] <= 0:
            return [a0[-1], a1[-1], 0., 0.]
        w0 = np.polyfit(t - t[-1], a0, 1)[0]; w1 = np.polyfit(t - t[-1], a1, 1)[0]
        return [a0[-1], a1[-1], float(w0), float(w1)]

    def feed(self, t, duty, theta0, theta1):
        if theta0 is None or theta1 is None or not (math.isfinite(theta0) and math.isfinite(theta1)):
            return self.x
        self.hist.append((t, theta0, theta1)); del self.hist[:-12]
        if self.x is None or self.t is None or t <= self.t or t - self.t > .2:
            self.x = self._measured_state(); self.t = t; self.d = duty; self.t_sync = t
            return self.x
        span = t - self.t; n = max(1, int(math.ceil(span / self.max_dt))); h = span / n; x = list(self.x); d = self.d
        for _ in range(n):
            k1, _ = E.rhs(x, d, self.p, self.Vs); k2, _ = E.rhs([x[q] + .5 * h * k1[q] for q in range(4)], d, self.p, self.Vs)
            k3, _ = E.rhs([x[q] + .5 * h * k2[q] for q in range(4)], d, self.p, self.Vs)
            k4, _ = E.rhs([x[q] + h * k3[q] for q in range(4)], d, self.p, self.Vs)
            x = [x[q] + h / 6 * (k1[q] + 2 * k2[q] + 2 * k3[q] + k4[q]) for q in range(4)]
        if not all(math.isfinite(v) for v in x):
            x = self._measured_state()
        self.x = x; self.t = t; self.d = duty
        if self.resync_s and t - self.t_sync >= self.resync_s:
            self.x = self._measured_state(); self.t_sync = t
        return self.x


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rk4(x, d, p, Vs, h):
    k1, _ = E.rhs(x, d, p, Vs); k2, _ = E.rhs([x[q] + .5 * h * k1[q] for q in range(4)], d, p, Vs)
    k3, _ = E.rhs([x[q] + .5 * h * k2[q] for q in range(4)], d, p, Vs)
    k4, _ = E.rhs([x[q] + h * k3[q] for q in range(4)], d, p, Vs)
    return [x[q] + h / 6 * (k1[q] + 2 * k2[q] + 2 * k3[q] + k4[q]) for q in range(4)]


class FastResyncGhost(ModelGhost):
    """ModelGhost re-synced often (default every 0.2 s): the model's short-horizon prediction error, which stays
    visible while the rig balances (the open-loop model falls off the unstable upright within ~0.5 s)."""
    def __init__(self, params=None, Vs=None, resync_s=.2, max_dt=.001):
        super().__init__(params, Vs, resync_s, max_dt)


class ObserverGhost:
    """The controller's view: firmware 1.12/1.13's model observer (est_observer_ref.m) run on the stream records.

    feed(t, duty, theta0, theta1) as ModelGhost (duty = the applied duty of that record, theta1 0 = upright).
    The observer runs on a fixed grid of Ts (4 ms): xd = estimate at the time of the (late) pendulum reading,
    stepped one tick with the model and the duty that acted then, corrected by alpha-beta gains (f_ab), rolled forward
    nd = round(lag / Ts) ticks through the stored duties to "now". Start-up from the first reading; the arm miss is
    ignored for the first nd ticks. Returns the estimate [theta0, theta1, w0, w1] (None before the first reading).
    """
    def __init__(self, params=None, Vs=None, lag=.012, Ts=.004, f_ab=10., sub=1):
        if params is None:
            params, vs = load_model(); Vs = Vs or vs
        self.p, self.Vs, self.Ts, self.sub = params, Vs or 12., Ts, int(sub)
        self.nd = max(1, int(round(lag / Ts)))
        lam = math.exp(-2 * math.pi * f_ab * Ts); self.al = 1 - lam * lam; self.be = (1 - lam) ** 2
        self.reset()

    def reset(self):
        self.x = None; self.t_next = None; self.started = False; self.n = 0
        self.u_prev = 0.; self.uh = [0.] * (self.nd + 1); self.th0b = [0.] * self.nd; self.xd = [0., math.pi, 0., 0.]
        self.last = None

    def _tick(self, duty, th0, th1):
        Ts, nd = self.Ts, self.nd
        th1 = _wrap(th1)
        u_now = min(max(self.u_prev, -.5), .5)                       # duty unit delay + sat
        self.uh = self.uh[1:] + [u_now]; self.u_prev = duty
        th0_then = self.th0b[0]; self.th0b = self.th0b[1:] + [th0]
        if not self.started:
            self.started = True; xp = [th0, th1, 0., 0.]
        else:
            xp = list(self.xd); xp[1] = _wrap(xp[1])
            for _ in range(self.sub):
                xp = _rk4(xp, self.uh[0], self.p, self.Vs, Ts / self.sub)
        self.n += 1
        r0 = th0_then - xp[0]; r1 = _wrap(th1 - xp[1])
        if self.n <= nd:
            r0 = 0.
        self.xd = [xp[0] + self.al * r0, xp[1] + self.al * r1, xp[2] + self.be / Ts * r0, xp[3] + self.be / Ts * r1]
        x = list(self.xd)
        for j in range(1, nd + 1):
            for _ in range(self.sub):
                x = _rk4(x, self.uh[j], self.p, self.Vs, Ts / self.sub)
        x[1] = _wrap(x[1])
        if not all(math.isfinite(v) for v in x):
            x = [th0, th1, 0., 0.]
        self.x = x

    def feed(self, t, duty, theta0, theta1):
        if theta0 is None or theta1 is None or not (math.isfinite(theta0) and math.isfinite(theta1)):
            return self.x
        if self.t_next is None or t < self.t_next - 4 * self.Ts or t - self.t_next > .2:
            self.reset(); self.t_next = t                            # first record, or the stream jumped: restart
        self.last = (duty, theta0, theta1)
        while t >= self.t_next - 1e-9:                               # one observer tick per Ts of device time
            self._tick(duty, theta0, theta1); self.t_next += self.Ts
        return self.x


def load_design(path=None):
    """Numbers of firmware/include/onboard_design.h (generated by export_onboard.m) as a dict; K as a list."""
    path = Path(path) if path else Path(__file__).resolve().parents[2] / 'firmware' / 'include' / 'onboard_design.h'
    import re
    txt = path.read_text(encoding='utf-8'); d = {}
    for name, val in re.findall(r'constexpr\s+(?:double|int)\s+(\w+)\s*=\s*([-+0-9.eE]+)\s*;', txt):
        d[name] = float(val)
    m = re.search(r'K\[4\]\s*=\s*\{([^}]*)\}', txt)
    d['K'] = [float(v) for v in m.group(1).split(',')]
    return d


class ClosedLoopGhost:
    """The model with its own copy of the onboard controller (firmware/include/onboard.h, numbers from
    onboard_design.h): what the model + controller predict, re-synced to the measured state every `resync_s` s.

    The controller sees the ghost's own state (no estimator), runs every Ts (4 ms): supervisor (SwingUp / Stabilize /
    SafeStop), energy swing-up with breakaway feedforward, stabilizer stab 3 (LQR) or 1 (PID with the arm integral),
    friction compensation (fricGain). Duty through clip `max_duty` and slew `slew` per second at 1 ms, then the model
    (1 ms RK4). feed(t, duty, theta0, theta1) as ModelGhost (the rig's duty is not used); returns the ghost state
    [theta0, theta1, w0, w1]. self.mode: 0 SafeStop, 1 SwingUp, 2 Stabilize (as onboard.h).
    """
    SAFE, SWING, STAB = 0, 1, 2

    def __init__(self, params=None, Vs=None, stab=3, resync_s=2., design=None, max_duty=.5, slew=100., dt=.001):
        if params is None:
            params, vs = load_model(); Vs = Vs or vs
        self.p, self.Vs, self.stab, self.resync_s = params, Vs or 12., int(stab), resync_s
        self.c = design or load_design(); self.max_duty, self.slew, self.dt = max_duty, slew, dt
        self.Ts = self.c['Ts']; self.hist = []
        self.reset()

    def reset(self):
        self.x = None; self.t = None; self.t_sync = None; self.t_ctrl = None
        self.mode = self.SWING; self.first = True; self.pid_sum = 0.; self.u = 0.; self.d = 0.; self.hist = []

    _measured_state = ModelGhost._measured_state

    # ---- onboard.h, line by line (x = [th0, th1 (wrapped), w0, w1]) ----
    def _stabilizer(self, x):
        c = self.c; r = 0.
        if self.stab == 1:
            u0 = c['pid_Kp1'] * x[1] + c['pid_Kd1'] * x[3] + c['pid_Kp0'] * (x[0] - r) + c['pid_Ki0'] * self.pid_sum + c['pid_Kd0'] * x[2]
        else:
            K = c['K']; u0 = -(K[0] * (x[0] - r) + K[1] * x[1] + K[2] * x[2] + K[3] * x[3])
        s = math.tanh(u0 / c['fricBand'])
        coul = c['fricDutyPos'] * max(s, 0.) + c['fricDutyNeg'] * min(s, 0.)
        stat = (c['pl_Rm'] * c['pl_tsp'] * max(s, 0.) + c['pl_Rm'] * c['pl_tsn'] * min(s, 0.)) / c['pl_Vs']
        g = math.exp(-(x[2] / c['sw_w_stick']) ** 2)
        return u0 + c['fricGain'] * (coul + c['fricStick'] * g * (stat - coul))

    def _energy_law(self, x):
        c = self.c; r = 0.; th0, th1, w0, w1 = x
        E_ = .5 * c['pl_Jp'] * w1 * w1 + c['pl_G'] * (math.cos(th1) - 1)
        e = E_ / (2 * c['pl_G'])
        a0 = -c['sw_a_push'] * min(1., max(-1., c['sw_k_E'] * (c['sw_E0'] - e))) * math.tanh(w1 * math.cos(th1) / c['sw_w_sign'])
        if e < -1 + c['sw_kick_E']:
            a0 = c['sw_a_kick']
        a0 = a0 - c['sw_wc'] ** 2 * (th0 - r) - 2 * c['sw_zc'] * c['sw_wc'] * w0
        sf = math.tanh(w0 / c['sw_w_fric'])
        coul = c['pl_tcp'] * max(sf, 0.) + c['pl_tcn'] * min(sf, 0.)
        sa = math.tanh(a0 / c['sw_a_stick']); stat = c['pl_tsp'] * max(sa, 0.) + c['pl_tsn'] * min(sa, 0.)
        g = math.exp(-(w0 / c['sw_w_stick']) ** 2)
        i = c['pl_Ja'] * a0 + c['pl_Beq'] * w0 + coul + c['sw_stick_gain'] * g * (stat - coul)
        i = min(c['sw_i_max'], max(-c['sw_i_max'], i))
        u = (c['pl_Rm'] * i + c['pl_Kb'] * w0) / c['pl_Vs']
        return min(c['sw_u_max'], max(-c['sw_u_max'], u))

    def _supervise(self, x):
        c = self.c; th0, th1, w1 = x[0], x[1], x[3]
        catch = abs(th1) < c['catchTh1'] and abs(w1) < c['catchW1']
        if self.first:
            self.mode = self.STAB if catch else self.SWING; self.first = False; return
        if self.mode == self.SWING:
            if abs(th0) > c['safeTh0']: self.mode = self.SAFE
            elif catch: self.mode = self.STAB
        elif self.mode == self.STAB:
            if abs(th0) > c['safeTh0']: self.mode = self.SAFE
            elif abs(th1) > c['exitTh1']: self.mode = self.SWING

    def _control(self):
        x = [self.x[0], _wrap(self.x[1]), self.x[2], self.x[3]]
        self._supervise(x)
        if self.mode == self.STAB: u = self._stabilizer(x)
        elif self.mode == self.SWING: u = self._energy_law(x)
        else: u = 0.
        if self.stab == 1:
            self.pid_sum += self.Ts * x[0]
        self.u = u

    def _sync(self, t):
        self.x = self._measured_state(); self.t = t; self.t_sync = t
        if self.t_ctrl is None:
            self.t_ctrl = t

    def feed(self, t, duty, theta0, theta1):
        if theta0 is None or theta1 is None or not (math.isfinite(theta0) and math.isfinite(theta1)):
            return self.x
        self.hist.append((t, theta0, theta1)); del self.hist[:-12]
        if self.x is None or self.t is None or t <= self.t or t - self.t > .2:
            self.reset(); self.hist.append((t, theta0, theta1)); self._sync(t); return self.x
        while self.t < t - 1e-12:
            if self.t >= self.t_ctrl - 1e-12:
                self._control(); self.t_ctrl += self.Ts
            h = min(self.dt, t - self.t)
            tgt = min(max(self.u, -self.max_duty), self.max_duty)
            self.d += min(max(tgt - self.d, -self.slew * h), self.slew * h)
            self.x = _rk4(self.x, self.d, self.p, self.Vs, h); self.t += h
            if not all(math.isfinite(v) for v in self.x):
                self._sync(t); break
        if self.resync_s and t - self.t_sync >= self.resync_s:
            self._sync(t)
        return self.x


# ------------------------------------------------------------------ UDP (RIP1, compatible with the Unreal twin and twin/matlab)
def pack(source, q_arm, q_pend):
    return _PACKET.pack(b'RIP1', int(source), float(q_arm), float(q_pend))


def unpack(data):
    """(source, q_arm, q_pend) or None."""
    if len(data) != _PACKET.size:
        return None
    tag, src, qa, qp = _PACKET.unpack(data)
    return (src, qa, qp) if tag == b'RIP1' and math.isfinite(qa) and math.isfinite(qp) else None


class UdpLink:
    """Optional: receive RIP1 packets (e.g. MATLAB twin_send / twin_play) and/or forward poses (e.g. to the Unreal twin)."""
    def __init__(self):
        self.rx = None; self.tx = None; self.tx_addr = None; self.latest = {}

    def listen(self, port=UDP_PORT):
        self.close_rx()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setblocking(False)
        s.bind(('0.0.0.0', int(port))); self.rx = s

    def forward(self, host='127.0.0.1', port=UDP_PORT):
        if self.tx is None:
            self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_addr = (host, int(port))

    def stop_forward(self):
        if self.tx:
            self.tx.close()
        self.tx = None; self.tx_addr = None

    def poll(self):
        """Read all pending packets; returns {source: (q_arm, q_pend)} of the newest per source."""
        if self.rx is None:
            return self.latest
        while True:
            try:
                data, _ = self.rx.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            r = unpack(data)
            if r:
                self.latest[r[0]] = (r[1], r[2])
        return self.latest

    def send(self, source, q_arm, q_pend):
        if self.tx and self.tx_addr:
            try:
                self.tx.sendto(pack(source, q_arm, q_pend), self.tx_addr)
            except OSError:
                pass

    def close_rx(self):
        if self.rx:
            self.rx.close()
        self.rx = None; self.latest = {}

    def close(self):
        self.close_rx(); self.stop_forward()
