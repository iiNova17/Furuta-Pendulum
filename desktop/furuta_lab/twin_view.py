"""Qt Quick 3D view of the digital twin, embeddable in the lab app (QQuickWidget).

Why Qt Quick 3D: it renders in the app's own process through Qt's RHI (Direct3D 11 on Windows, no OpenGL), so the pose
on screen is taken from the serial thread's buffer on every frame without a network hop or a second process; a browser
(three.js in QWebEngine) or a separate engine (Unreal over UDP) add an IPC hop and their own frame queue.
Needs the PySide6-Addons wheel (QtQuick3D).
"""
import math
from PySide6 import QtCore
from PySide6.QtCore import Property, QByteArray, QUrl, Signal
from PySide6.QtGui import QQuaternion, QVector3D
from PySide6.QtQml import QmlElement
from PySide6.QtQuick3D import QQuick3DGeometry
from PySide6.QtQuickWidgets import QQuickWidget
from . import twin_core as T

QML_IMPORT_NAME = 'FurutaTwin'
QML_IMPORT_MAJOR_VERSION = 1
_MESHES = {}


def mesh(name):
    if name not in _MESHES:
        _MESHES[name] = T.load_ripmesh(T.ASSETS / f'{name}.ripmesh')
    return _MESHES[name]


@QmlElement
class RipGeometry(QQuick3DGeometry):
    """A twin body (housing / arm / pendulum) from twin_assets/*.ripmesh: flat-shaded triangles, positions + normals."""
    sourceChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent); self._source = ''

    def _get(self):
        return self._source

    def _set(self, name):
        if name == self._source:
            return
        self._source = name
        pos, nrm = mesh(name)
        import numpy as np
        data = np.hstack([pos, nrm]).astype('<f4')
        self.clear()
        self.setVertexData(QByteArray(data.tobytes()))
        self.setStride(24)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        A = QQuick3DGeometry.Attribute
        self.addAttribute(A.Semantic.PositionSemantic, 0, A.ComponentType.F32Type)
        self.addAttribute(A.Semantic.NormalSemantic, 12, A.ComponentType.F32Type)
        lo, hi = pos.min(0), pos.max(0)
        self.setBounds(QVector3D(*map(float, lo)), QVector3D(*map(float, hi)))
        self.update(); self.sourceChanged.emit()

    source = Property(str, _get, _set, notify=sourceChanged)


EXTRA_GHOSTS = ('fast', 'observer', 'closed')     # extra ghost bodies in twin.qml (orange, green, violet)


class TwinView(QQuickWidget):
    """set_pose('rig' | 'ghost' | 'fast' | 'observer' | 'closed', q_arm, q_pend) [rad, twin convention],
    show_ghost(bool, which='ghost'), set_hud(text)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.setMinimumSize(320, 240)
        self.setSource(QUrl.fromLocalFile(str(T.ASSETS / 'twin.qml')))
        self.error = '' if self.status() == QQuickWidget.Ready else '; '.join(e.toString() for e in self.errors())
        self.ok = not self.error
        self.nodes = {}
        if self.ok:
            root = self.rootObject()
            names = ['armJoint', 'pendJoint', 'pendModel', 'ghostArm', 'ghostPend', 'ghostPendModel', 'hud', 'camOrigin', 'camera']
            for g in EXTRA_GHOSTS:
                names += [f'ghostArm_{g}', f'ghostPend_{g}', f'ghostPendModel_{g}']
            for n in names:
                self.nodes[n] = root.findChild(QtCore.QObject, n)
            pivot, axis = T.load_geometry()
            self.axis = QVector3D(*map(float, axis))
            pairs = [('pendJoint', 'pendModel'), ('ghostPend', 'ghostPendModel')]
            pairs += [(f'ghostPend_{g}', f'ghostPendModel_{g}') for g in EXTRA_GHOSTS if self.nodes.get(f'ghostPend_{g}') is not None]
            for joint, model in pairs:
                self.nodes[joint].setProperty('position', QVector3D(*map(float, pivot)))
                self.nodes[model].setProperty('position', QVector3D(*map(float, -pivot)))
            self.home = (self.nodes['camOrigin'].property('eulerRotation'), self.nodes['camera'].property('position'))
        self.pose = {'rig': (0., 0.), 'ghost': (0., 0.)}

    def set_pose(self, which, q_arm, q_pend):
        if not self.ok or q_arm is None or q_pend is None:
            return
        if self.pose.get(which) == (q_arm, q_pend):
            return
        self.pose[which] = (q_arm, q_pend)
        arm, pend = self._joints(which)
        if self.nodes.get(arm) is None:
            return
        sa, sp = T.DISPLAY_SIGN
        self.nodes[arm].setProperty('rotation', QQuaternion.fromAxisAndAngle(QVector3D(0, 0, 1), math.degrees(sa * q_arm)))
        self.nodes[pend].setProperty('rotation', QQuaternion.fromAxisAndAngle(self.axis, math.degrees(sp * q_pend)))

    @staticmethod
    def _joints(which):
        if which == 'rig':
            return 'armJoint', 'pendJoint'
        if which == 'ghost':
            return 'ghostArm', 'ghostPend'
        return f'ghostArm_{which}', f'ghostPend_{which}'

    def show_ghost(self, on, which='ghost'):
        node = self.nodes.get(self._joints(which)[0]) if self.ok else None
        if node is not None and bool(node.property('visible')) != bool(on):
            node.setProperty('visible', bool(on))

    def set_hud(self, text):
        if self.ok:
            self.nodes['hud'].setProperty('text', text)

    def reset_camera(self):
        if self.ok:
            self.nodes['camOrigin'].setProperty('eulerRotation', self.home[0]); self.nodes['camera'].setProperty('position', self.home[1])
