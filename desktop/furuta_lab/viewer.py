"""Multi-signal plot: Separate (stacked, x-linked) or Combined (normalized overlay)."""
import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets as W
from . import signals as S

MAX_POINTS = 4000


class SignalView(W.QStackedWidget):
    rangeChanged = QtCore.Signal()

    def __init__(self):
        super().__init__()
        self.empty = W.QLabel()
        self.empty.setAlignment(QtCore.Qt.AlignCenter)
        self.empty.setWordWrap(True)
        self.empty.setStyleSheet('background:white;border:1px dashed #c7d2de;border-radius:7px;color:#5d6d7e;font-size:15px;padding:30px;')
        self.addWidget(self.empty)
        self.layout_widget = pg.GraphicsLayoutWidget()
        self.addWidget(self.layout_widget)
        self.plots = {}; self.curves = {}; self.signature = None; self.legend = None; self.legends = {}
        self.set_empty('Not connected.\n\nChoose the USB port and press Connect USB,\nor use Open recording… to view a saved run or still capture.')

    def set_empty(self, text):
        self.empty.setText(text); self.setCurrentIndex(0)

    def _rebuild(self, keys, mode):
        self.layout_widget.clear(); self.plots = {}; self.curves = {}; self.legend = None; self.legends = {}
        if mode == 'separate':
            first = None
            for i, k in enumerate(keys):
                p = self.layout_widget.addPlot(row=i, col=0)
                p.showGrid(x=True, y=True, alpha=.18); p.setClipToView(True)
                p.setDownsampling(auto=True, mode='peak'); p.getAxis('left').setWidth(62)
                if first is None:
                    first = p; p.sigXRangeChanged.connect(lambda *_: self.rangeChanged.emit())
                else:
                    p.setXLink(first)
                if i < len(keys) - 1:
                    p.getAxis('bottom').setStyle(showValues=False)
                self.plots[k] = p
                # A one-entry legend names the trace without a rotated axis label.
                leg = p.addLegend(offset=(4, 1), labelTextSize='8pt', verSpacing=-6)
                leg.setBrush(pg.mkBrush(255, 255, 255, 210))
                self.curves[k] = p.plot(pen=pg.mkPen(S.BY_KEY[k].color, width=1.3), name=S.BY_KEY[k].label)
                self.legends[k] = leg
            if first is not None:
                list(self.plots.values())[-1].setLabel('bottom', 'Time', 's')
        else:
            p = self.layout_widget.addPlot(row=0, col=0)
            p.showGrid(x=True, y=True, alpha=.18); p.setClipToView(True); p.setDownsampling(auto=True, mode='peak')
            p.setLabel('bottom', 'Time', 's'); p.setLabel('left', 'Normalized (each signal to ±1)')
            p.sigXRangeChanged.connect(lambda *_: self.rangeChanged.emit())
            self.legend = p.addLegend(offset=(8, 8))
            self.plots['combined'] = p
            for k in keys:
                self.curves[k] = p.plot(pen=pg.mkPen(S.BY_KEY[k].color, width=1.3), name=S.BY_KEY[k].label)
        self.signature = (tuple(keys), mode)

    def show_series(self, t, series, keys, mode, title='', follow=None):
        """Draw keys from series. follow=seconds keeps the last N seconds in view (live)."""
        keys = [k for k in keys if k in series]
        if not keys:
            self.set_empty('Select at least one signal in the checklist.'); return
        self.setCurrentIndex(1)
        if self.signature != (tuple(keys), mode):
            self._rebuild(keys, mode)
        stride = max(1, len(t) // MAX_POINTS) if follow else 1
        tt = t[::stride]
        for k in keys:
            s = series[k]; y = s.values[::stride]
            if mode == 'separate':
                if k == 'pend_deg' and s.unit == 'deg':
                    y = S.continuous_angle(tt, y)
                self.curves[k].setData(tt, y, connect=S.connect_mask(y, k == 'pend_deg' and s.unit == 'deg'))
                item = self.legends[k].getLabel(self.curves[k])
                if item is not None:
                    item.setText(f'{s.label} [{s.unit}]' + (f' — {s.note}' if s.note else ''))
            else:
                if k == 'pend_deg' and s.unit == 'deg':
                    # Deviation from the circular mean, so the +-180 wrap does not dominate.
                    m, _ = S.angle_stats(tt, y)
                    y = S.wrap_deg(y - m) if m is not None else y
                yn, half = S.normalize(y)
                self.curves[k].setData(tt, yn, connect=S.connect_mask(yn))
                if self.legend is not None:
                    item = self.legend.getLabel(self.curves[k])
                    if item is not None:
                        item.setText(f'{s.label} (±{half:.3g} {s.unit})')
        first = self.first_plot()
        if title and first is not None:
            first.setTitle(title)
        if follow and len(t):
            first.setXRange(max(0, t[-1] - follow), t[-1], padding=0)
            for p in self.plots.values():
                p.enableAutoRange(y=True)

    def first_plot(self):
        return next(iter(self.plots.values()), None)

    def x_range(self):
        p = self.first_plot()
        return tuple(p.viewRange()[0]) if p is not None else (None, None)

    def reset(self):
        self.signature = None
        for p in self.plots.values():
            p.enableAutoRange(x=True, y=True)
