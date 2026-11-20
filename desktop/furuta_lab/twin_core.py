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
