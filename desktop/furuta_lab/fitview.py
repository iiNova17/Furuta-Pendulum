"""Stacked, x-linked panels for estimation steps: measured vs model, contributions, residuals.

panels: list of dicts {title, unit, curves: [(label, x, y, color, width_or_style)], shade: (x, mask) or None}.
A pole-zero view is available through show_pz().
"""
import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets as W

PALETTE = {'measured': '#263649', 'model': '#d1495b', 'raw': '#b8c2cc', 'filtered': '#1f6fb4',
           'p1': '#2a8f62', 'p2': '#e08a4f', 'p3': '#7b3fa0', 'p4': '#b8860b', 'p5': '#4a9bd8', 'p6': '#8e44ad',
           'residual': '#5d6d7e', 'duty': '#b52931'}
MAX_POINTS = 20000


class FitView(W.QStackedWidget):
    def __init__(self):
        super().__init__()
        self.empty = W.QLabel('Estimation plots appear here.'); self.empty.setAlignment(QtCore.Qt.AlignCenter)
        self.empty.setWordWrap(True)
        self.empty.setStyleSheet('background:white;border:1px dashed #c7d2de;border-radius:7px;color:#5d6d7e;font-size:15px;padding:30px;')
        self.addWidget(self.empty)
        self.layout_widget = pg.GraphicsLayoutWidget(); self.addWidget(self.layout_widget)
        self.plots = []; self.title = ''

    def set_empty(self, text):
        self.empty.setText(text); self.setCurrentIndex(0)

    def show_panels(self, panels, title=''):
        self.layout_widget.clear(); self.plots = []; self.title = title
        first = None
        for i, panel in enumerate(panels):
            p = self.layout_widget.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True, alpha=.18); p.setClipToView(True); p.setDownsampling(auto=True, mode='peak')
            p.getAxis('left').setWidth(62); p.getAxis('left').enableAutoSIPrefix(False); p.getAxis('bottom').enableAutoSIPrefix(False)
            if i == 0 and title:
                p.setTitle(title)
            if first is None:
                first = p
            elif panel.get('link', True):
                p.setXLink(first)
            leg = p.addLegend(offset=(4, 1), labelTextSize='8pt', verSpacing=-6)
            leg.setBrush(pg.mkBrush(255, 255, 255, 215))
            shade = panel.get('shade')
            if shade is not None:
                self._shade(p, *shade)
            for label, x, y, color, style in panel.get('curves', []):
                x = np.asarray(x, float); y = np.asarray(y, float)
                stride = max(1, len(x) // MAX_POINTS)
                kw = {}
                if style == 'dots':
                    kw = dict(pen=None, symbol='o', symbolSize=3, symbolPen=None, symbolBrush=pg.mkBrush(PALETTE.get(color, color)))
                else:
                    width = style if isinstance(style, (int, float)) else 1.3
                    pen = pg.mkPen(PALETTE.get(color, color), width=width)
                    if style == 'dash':
                        pen = pg.mkPen(PALETTE.get(color, color), width=1.3, style=QtCore.Qt.DashLine)
                    kw = dict(pen=pen, connect='finite')
                p.plot(x[::stride], y[::stride], name=label, **kw)
            p.setLabel('left', panel.get('unit', ''))
            if panel.get('xlabel'):
                p.setLabel('bottom', panel['xlabel'])
            elif i == len(panels) - 1:
                p.setLabel('bottom', 'Time', 's')
            self.plots.append(p)
        self.setCurrentIndex(1)

    def _shade(self, p, x, mask):
        """Grey bands where mask is False (samples not used by the fit)."""
        x = np.asarray(x, float); m = np.asarray(mask, bool)
        if not len(x) or m.all():
            return
        d = np.diff(np.r_[0, (~m).astype(int), 0])
        for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)):
            r = pg.LinearRegionItem(values=(x[a], x[min(b, len(x) - 1)]), movable=False,
                                    brush=pg.mkBrush(150, 160, 170, 40), pen=pg.mkPen(None))
            r.setZValue(-10); p.addItem(r)

    def show_pz(self, systems, title='Poles and zeros'):
        """systems: list of (label, poles, zeros, color)."""
        self.layout_widget.clear(); self.plots = []
        p = self.layout_widget.addPlot(row=0, col=0); p.setTitle(title); p.showGrid(x=True, y=True, alpha=.25)
        p.setLabel('bottom', 'Real', 'rad/s'); p.setLabel('left', 'Imaginary', 'rad/s'); p.addLegend(offset=(8, 8))
        p.addLine(x=0, pen=pg.mkPen('#97a3af', width=1)); p.addLine(y=0, pen=pg.mkPen('#97a3af', width=1))
        for label, poles, zeros, color in systems:
            c = PALETTE.get(color, color)
            poles = np.atleast_1d(poles); zeros = np.atleast_1d(zeros)
            p.plot(np.real(poles), np.imag(poles), pen=None, symbol='x', symbolSize=13, symbolPen=pg.mkPen(c, width=2), name=f'{label} poles')
            if len(zeros):
                p.plot(np.real(zeros), np.imag(zeros), pen=None, symbol='o', symbolSize=10, symbolBrush=None,
                       symbolPen=pg.mkPen(c, width=2), name=f'{label} zeros')
        self.plots.append(p); self.setCurrentIndex(1)
