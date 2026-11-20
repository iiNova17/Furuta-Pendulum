"""Focused raster GUI. No MATLAB, web view, or OpenGL is needed for acquisition."""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy import signal
from PySide6 import QtCore, QtGui, QtWidgets as W
import pyqtgraph as pg
from serial.tools import list_ports
from .device import Device
from . import analysis as A
from . import signals as S
from .viewer import SignalView
from .fitview import FitView
from .estimate_page import EstimatePage
from .onboard_page import OnboardPage
from .twin_page import TwinPage
from .backend import ROOT,MatlabBackend,jsonable,save_capture,load_capture,load_run

pg.setConfigOptions(useOpenGL=False,antialias=False,background='w',foreground='#263649')

STATE_STYLE={'DISCONNECTED':('DISCONNECTED','#6b7785'),'CONNECTING':('CONNECTING','#6b7785'),'BOOT':('BOOT','#6b7785'),
             'DISARMED':('DISARMED','#1f7a4d'),'ARMED_IDLE':('ARMED','#c77700'),'RUNNING':('RUNNING','#b52931'),
             'STOPPING':('RUNNING · stopping','#b52931'),'FAULT':('FAULT','#7a1020'),'CONNECTION_ERROR':('FAULT · link lost','#7a1020')}
POWER_OFF_STYLE=('QCheckBox{background:#fff6e0;border:2px solid #d99a00;border-radius:6px;padding:7px 12px;font-weight:600;color:#6b4a00;}'
                 'QCheckBox::indicator{width:18px;height:18px;}')
POWER_ON_STYLE=('QCheckBox{background:#b52931;border:2px solid #7a1020;border-radius:6px;padding:7px 12px;font-weight:700;color:white;}'
                'QCheckBox::indicator{width:18px;height:18px;}')
READOUT_WINDOW_S=.5

class Signals(QtCore.QObject):
    done=QtCore.Signal(object,object)
    failed=QtCore.Signal(str)
    progress=QtCore.Signal(str)

class LabApp(W.QMainWindow):
    def __init__(self,estimation_root=None):
        self.estimation_root=estimation_root
        super().__init__();self.setWindowTitle('Furuta Lab — sensors and characterization');self.resize(1320,880)
        self.device=Device();self.backend=MatlabBackend();self.executor=ThreadPoolExecutor(1,thread_name_prefix='Lab analysis')
        self.signals=Signals();self.signals.done.connect(self.finished);self.signals.failed.connect(self.failed)
        self.signals.progress.connect(self.progress);self.cancel=threading.Event();self.busy=False;self.closing=False
        self.capture_data=None;self.analysis_result=None;self.filter_rows=[];self.run_data=None;self.run_manifest={}
        self.run_segments=[];self.dead_windows=None;self.run_start=None;self.running=False;self.last_state=''
        self.left_count=None;self.right_count=None;self.last_refresh=0.;self.max_refresh=0.;self.refresh_delays=[]
        self.actions=[];self.live=True;self.run_metadata={}
        self.view_source=None;self.view_cache=None;self.view_dirty=True;self.seen_events=0;self.analysis_drawn=False
        self.fit_drawn=False;self.run_path=None;self.pending_load=None;self.auto_deadzone_path=None
        self.build();self.refresh_ports()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(200)
        self.report('Start here','1. Choose the USB port and press **Connect USB** (motor stays disabled).\n\n'
                    '2. Watch the signals on **Live & recordings**; tick the ones you want.\n\n'
                    '3. Set the three references (arm center, pendulum down, current zero).\n\n'
                    '4. For noise, use **Still noise & filters**.\n\n'
                    'Motor tests stay disabled until you tick the amber 12 V box and commission finite limits. '
                    'Gear ratio 34:1 is provisional until the scale is verified.')

    # ------------------------------------------------------------------ layout
    def build(self):
        self.setStyleSheet('''QMainWindow,QWidget{background:#f4f6f8;color:#263649;font-size:13px;}
        QGroupBox{background:white;border:1px solid #d8e0e8;border-radius:7px;margin-top:12px;padding:12px;font-weight:600;}
        QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 5px;}
        QGroupBox QLabel,QGroupBox QCheckBox,QGroupBox QRadioButton{background:white;font-weight:400;}
        QPushButton{background:#fff;border:1px solid #c7d2de;border-radius:5px;padding:7px 12px;}
        QPushButton:hover{background:#eaf2ff;} QPushButton:disabled{color:#97a3af;background:#edf0f3;}
        QComboBox,QDoubleSpinBox,QSpinBox,QLineEdit{background:white;border:1px solid #c7d2de;border-radius:4px;padding:5px;}
        QTabBar::tab{padding:10px 17px;background:#e6ebf1;}QTabBar::tab:selected{background:white;color:#075fa8;border-bottom:3px solid #1673b9;}
        QTextBrowser{background:white;border:1px solid #d8e0e8;border-radius:5px;padding:10px;}
        QCheckBox::indicator,QRadioButton::indicator{width:14px;height:14px;border:1px solid #8a97a5;background:white;}
        QCheckBox::indicator{border-radius:3px;} QRadioButton::indicator{border-radius:8px;}
        QCheckBox::indicator:checked,QRadioButton::indicator:checked{background:#1673b9;border:3px solid white;outline:1px solid #1673b9;}
        QCheckBox::indicator:checked{border:1px solid #1673b9;image:none;background:#1673b9;}
        QRadioButton::indicator:checked{border:4px solid #d5e6f5;background:#1673b9;}''')
        root=W.QWidget();self.setCentralWidget(root);v=W.QVBoxLayout(root);v.setContentsMargins(16,12,16,12);v.setSpacing(8)
        top=W.QHBoxLayout();v.addLayout(top)
        title=W.QLabel('FURUTA LAB');title.setStyleSheet('font-size:21px;font-weight:700;color:#125b96');top.addWidget(title)
        top.addSpacing(12);self.port=W.QComboBox();self.port.setMinimumWidth(130);top.addWidget(self.port)
        self.button(top,'Refresh ports',self.refresh_ports,action=False)
        self.connect_button=self.button(top,'Connect USB',self.connect_device,action=False)
        self.button(top,'Open recording…',self.open_recording)
        top.addSpacing(10);self.pill=W.QLabel();self.pill.setMinimumWidth(130);self.pill.setAlignment(QtCore.Qt.AlignCenter);top.addWidget(self.pill)
        top.addStretch()
        self.power=W.QCheckBox('⚠  12 V connected — allow motion');self.power.setChecked(False);self.power.setStyleSheet(POWER_OFF_STYLE)
        self.power.setToolTip('Safety permission. Tick only with the 12 V motor supply connected and the travel limits commissioned.\nUnticking sends STOP.')
        self.power.toggled.connect(self.power_changed);top.addWidget(self.power);top.addSpacing(8)
        self.stop_button=self.button(top,'STOP / DISABLE',self.stop,action=False)
        self.stop_button.setStyleSheet('background:#b52931;color:white;font-weight:700;padding:11px 22px;border:0;')
        chips=W.QHBoxLayout();v.addLayout(chips);self.chips={}
        for key,text in [('centered','Arm centered'),('down','Pendulum down set'),('zero','Current zero set'),('scale','Scale verified')]:
            c=W.QLabel(text);c.setAlignment(QtCore.Qt.AlignCenter);chips.addWidget(c);self.chips[key]=(c,text)
        chips.addSpacing(10)
        self.sensors=W.QLabel('Disconnected · USB-only mode · motor disabled');self.sensors.setStyleSheet('background:#e4edf5;padding:7px 10px;border-radius:5px;')
        chips.addWidget(self.sensors,1)
        self.progress_row=W.QWidget();pr=W.QHBoxLayout(self.progress_row);pr.setContentsMargins(0,0,0,0)
        self.progress_label=W.QLabel('');self.progress_bar=W.QProgressBar();self.progress_bar.setRange(0,0);self.progress_bar.setMaximumWidth(220)
        self.progress_bar.setMaximumHeight(14);self.cancel_button=W.QPushButton('Cancel');self.cancel_button.clicked.connect(lambda:self.safe(self.cancel_job))
        pr.addWidget(self.progress_label,1);pr.addWidget(self.progress_bar);pr.addWidget(self.cancel_button)
        self.progress_row.setStyleSheet('background:#eaf2ff;border-radius:5px;');self.progress_row.hide();v.addWidget(self.progress_row)
        self.nav=W.QTabBar();self.nav.setExpanding(False)
        for title in ['Live && recordings','Still noise && filters','Motor tests','Parameter estimation','Onboard control','Digital twin','Setup && guide']:self.nav.addTab(title)
        self.nav.currentChanged.connect(self.page_changed);v.addWidget(self.nav)
        split=W.QSplitter();v.addWidget(split,1);self.pages=W.QStackedWidget();self.pages.setMinimumWidth(450);split.addWidget(self.pages)
        for build in [self.live_page,self.noise_page,self.motor_page,self.model_page,self.onboard_page,self.twin_page,self.setup_page]:
            area=W.QScrollArea();area.setWidgetResizable(True);area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);page=W.QWidget();layout=W.QVBoxLayout(page);layout.setSpacing(9)
            build(layout);layout.addStretch();area.setWidget(page);self.pages.addWidget(area)
        right=W.QSplitter(QtCore.Qt.Vertical)
        self.right_stack=W.QStackedWidget();self.viewer=SignalView();self.viewer.rangeChanged.connect(self.range_changed)
        self.right_stack.addWidget(self.viewer)
        self.plot=pg.PlotWidget();self.plot.showGrid(x=True,y=True,alpha=.18)
        self.line=self.plot.plot(pen=pg.mkPen('#126cb1',width=1));self.second=self.plot.plot(pen=pg.mkPen('#d16d21',width=1))
        self.right_stack.addWidget(self.plot)
        self.fitview=FitView();self.right_stack.addWidget(self.fitview)
        if self.twin.view is None:
            slot=W.QLabel('3D view not available:\n'+self.twin.error);slot.setAlignment(QtCore.Qt.AlignCenter);slot.setWordWrap(True)
        self.right_stack.addWidget(self.twin.view or slot)   # index 3: digital twin
        right.addWidget(self.right_stack)
        self.bottom=W.QTabWidget();self.output=W.QTextBrowser();self.output.setOpenExternalLinks(False);self.bottom.addTab(self.output,'Messages')
        raw=W.QWidget();rl=W.QVBoxLayout(raw);rl.setContentsMargins(4,4,4,4);row=W.QHBoxLayout();rl.addLayout(row)
        self.raw_info=W.QLabel('No samples.');row.addWidget(self.raw_info,1)
        self.raw_pause=W.QCheckBox('Pause');row.addWidget(self.raw_pause)
        self.button(row,'Export CSV…',self.export_csv,action=False)
        self.raw_table=W.QTableWidget(0,len(S.RAW_FIELDS)+1);self.raw_table.setHorizontalHeaderLabels(['t (s)']+S.RAW_FIELDS)
        self.raw_table.verticalHeader().setVisible(False);self.raw_table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.raw_table.setStyleSheet('QTableWidget{background:white;font-family:Consolas;font-size:12px;}');rl.addWidget(self.raw_table)
        self.bottom.addTab(raw,'Raw data');self.bottom.currentChanged.connect(lambda *_:self.update_raw_table(force=True))
        right.addWidget(self.bottom)
        right.setSizes([560,320]);split.addWidget(right);split.setSizes([470,850])
        self.statusBar().showMessage('Ready. Acquisition runs independently of the UI and MATLAB.')
        self.update_header()

    def button(self,parent,text,fn,action=True):
        b=W.QPushButton(text);b.clicked.connect(lambda checked=False:self.safe(fn))
        parent.addWidget(b)
        if action:self.actions.append(b)
        return b
    def group(self,parent,title):
        box=W.QGroupBox(title);layout=W.QVBoxLayout(box);parent.addWidget(box);return layout
    def text(self,parent,text,small=False):
        label=W.QLabel(text);label.setWordWrap(True)
        if small:label.setStyleSheet('color:#5d6d7e;font-size:12px;')
        parent.addWidget(label);return label
    def number(self,parent,title,value,low,high,decimals=3):
        row=W.QHBoxLayout();row.addWidget(W.QLabel(title),1);n=W.QDoubleSpinBox();n.setRange(low,high);n.setDecimals(decimals);n.setValue(value);n.setMaximumWidth(110);row.addWidget(n);parent.addLayout(row);return n
    def combo(self,parent,title,items):
        parent.addWidget(W.QLabel(title));c=W.QComboBox();c.addItems(items);parent.addWidget(c);return c

    def live_page(self,v):
        g=self.group(v,'Signals')
        row=W.QHBoxLayout();g.addLayout(row)
        self.mode_separate=W.QRadioButton('Separate');self.mode_combined=W.QRadioButton('Combined (normalized ±1)')
        self.mode_separate.setChecked(True);row.addWidget(self.mode_separate);row.addWidget(self.mode_combined);row.addStretch()
        self.mode_separate.toggled.connect(self.view_options_changed)
        row=W.QHBoxLayout();g.addLayout(row)
        self.raw_units=W.QCheckBox('Raw units');self.raw_units.setToolTip('counts, AS5600 raw 0–4095, mV and duty fraction')
        self.raw_units.toggled.connect(self.view_options_changed);row.addWidget(self.raw_units);row.addStretch()
        row.addWidget(W.QLabel('Window (s)'));self.window_s=W.QDoubleSpinBox();self.window_s.setRange(1,60);self.window_s.setValue(10);self.window_s.setDecimals(0)
        self.window_s.setMaximumWidth(80);row.addWidget(self.window_s)
        row=W.QHBoxLayout();g.addLayout(row)
        row.addWidget(W.QLabel('Software smoothing (Hz)'));self.smooth_hz=W.QDoubleSpinBox();self.smooth_hz.setRange(.5,200);self.smooth_hz.setValue(20)
        self.smooth_hz.setDecimals(1);self.smooth_hz.setMaximumWidth(95);row.addWidget(self.smooth_hz)
        row.addStretch();self.smooth_label=W.QLabel();self.smooth_label.setStyleSheet('color:#5d6d7e;font-size:12px;');g.addWidget(self.smooth_label)
        self.smooth_hz.valueChanged.connect(self.view_options_changed)
        grid=W.QGridLayout();grid.setHorizontalSpacing(8);grid.setVerticalSpacing(3);g.addLayout(grid)
        self.signal_checks={};self.readouts={}
        for i,spec in enumerate(S.SPECS):
            c=W.QCheckBox(spec.label);c.setChecked(spec.key in S.DEFAULT_KEYS);c.toggled.connect(self.view_options_changed)
            c.setStyleSheet(f'QCheckBox{{color:{spec.color};font-weight:600;background:white;}}')
            r=W.QLabel('—');r.setTextFormat(QtCore.Qt.RichText);r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter);r.setMinimumWidth(190);r.setWordWrap(True)
            grid.addWidget(c,i,0);grid.addWidget(r,i,1);self.signal_checks[spec.key]=c;self.readouts[spec.key]=r
        grid.setColumnStretch(1,1)
        row=W.QHBoxLayout();g.addLayout(row)
        self.button(row,'All',lambda:self.check_signals(True),action=False);self.button(row,'Default',lambda:self.check_signals(None),action=False)
        row.addStretch()
        self.source_label=self.text(g,'Source: live stream · readouts are mean ± σ over the last 0.5 s',small=True)
        self.back_live=self.button(g,'Back to live view',self.set_live,action=False);self.back_live.hide()
        g=self.group(v,'References — motor disabled, no motion')
        self.text(g,'1. Hold the arm at its mechanical center.')
        self.button(g,'Arm is centered here',lambda:self.calibrate('arm'))
        self.text(g,'2. Let the pendulum hang still.')
        self.button(g,'Pendulum is hanging down here',lambda:self.calibrate('down'))
        self.text(g,'3. Keep the motor unpowered (1 s measurement).')
        self.button(g,'Zero current',lambda:self.calibrate('current'))
        g=self.group(v,'Live recording')
        self.record_button=self.button(g,'Start recording',self.toggle_record)
        self.text(g,'Saved recordings open with Open recording… (top bar) in this same view.',small=True)
        self.update_smooth_label()

    def noise_page(self,v):
        g=self.group(v,'1 · Record the stationary system')
        self.text(g,'1. Both joints still, motor driver disabled.')
        self.still=W.QCheckBox('Both joints are still; driver disabled');g.addWidget(self.still)
        self.text(g,'2. Choose length and ADC rate.')
        self.capture_seconds=self.number(g,'Record seconds',5,.2,30,1)
        self.capture_rate=self.combo(g,'ADC capture rate (Hz)',['500','1000','2000'])
        self.button(g,'Record all three sensors',self.capture_still)
        self.text(g,'ADC/filter settings are restored afterwards. Mechanical sensors stay at 500 Hz. Saved captures: Open recording….',small=True)
        g=self.group(v,'2 · Inspect noise')
        self.noise_sensor=self.combo(g,'Sensor',['Current sensor (mV and A)','Motor encoder (counts)','Pendulum angle (degrees)'])
        self.frequency_view=self.number(g,'Plot frequency up to (Hz)',100,1,1000,0)
        self.button(g,'Plot noise spectrum + statistics',self.analyze_noise)
        self.button(g,'Plot captured signal over time',self.plot_noise_time)
        g=self.group(v,'3 · Compare low-pass cutoffs')
        self.text(g,'1. Enter the motion bandwidth you must keep (still noise cannot choose it).')
        self.bandwidth=self.number(g,'Keep motion up to (Hz)',0,0,500,1)
        self.text(g,'2. Enter the delay you can tolerate.')
        self.delay=self.number(g,'Allowed filter delay (ms)',20,.1,1000,1)
        self.button(g,'Compare low-pass cutoffs',self.compare_filters)
        self.filter_choice=W.QComboBox();g.addWidget(self.filter_choice)
        self.button(g,'Preview selected filter',self.preview_filter)
        self.button(g,'Use selected cutoff for live smoothing',self.use_live_filter)
        self.button(g,'Deploy selected CURRENT filter to MCU',self.deploy_filter)
        self.text(g,'Candidates lose ≤ 1 dB in your band and meet the delay budget. Mechanical filters stay analysis-only. '
                  'The MCU overcurrent trip uses the filtered current (firmware 1.2).',small=True)
    def motor_page(self,v):
        g=self.group(v,'1 · Mark safe travel while unpowered')
        self.text(g,'1. Center the arm: References on the Live tab (chip “Arm centered” turns green).')
        self.text(g,'2. Move by hand (only if it turns freely) to each safe end and mark it.')
        self.button(g,'Mark left safe endpoint',lambda:self.mark_endpoint('left'))
        self.button(g,'Mark right safe endpoint',lambda:self.mark_endpoint('right'))
        self.limit_label=self.text(g,'No endpoints marked. Never seek a hard stop using drive.',small=True)
        g=self.group(v,'2 · Commission a finite motion envelope')
        self.max_duty=self.number(g,'Maximum duty (%)',0,0,100,1)
        self.travel=self.number(g,'Travel each side (counts)',0,0,1e9,0)
        self.max_duration=self.number(g,'Maximum run seconds',10,.1,3600,1)
        self.current_trip=self.number(g,'Current trip A (0 = off)',4,0,50,3)
        self.speed_trip=self.number(g,'Speed trip counts/s (0 = off)',0,0,1e7,0)
        self.slew=self.number(g,'Slew rate (%/s) — how fast duty may change',1000,.1,10000,1)
        self.button(g,'Apply these limits to RAM',self.apply_limits)
        g=self.group(v,'3 · Actuate and record')
        self.wave=self.combo(g,'Waveform',['hold','sine','chirp','prbs'])
        self.duty=self.number(g,'Signed amplitude (%)',2,-100,100,1)
        self.duration=self.number(g,'Command seconds',2,.01,3600,2)
        self.frequency=self.number(g,'Frequency / chirp start (Hz)',1,.01,499,2)
        self.run_button=self.button(g,'Run & record — MOVES MOTOR',self.run_wave)
        g=self.group(v,'4 · Measure breakaway dead zone')
        self.dead_start=self.number(g,'Start duty (%)',1,.1,100,1)
        self.dead_step=self.number(g,'Duty increment (%)',1,.1,100,1)
        self.dead_max=self.number(g,'Maximum test duty (%)',3,.1,100,1)
        self.move_threshold=self.number(g,'Movement threshold (counts)',3,1,1e9,0)
        self.dead_on=self.number(g,'Pulse length (s)',.25,.01,60,2)
        self.dead_rest=self.number(g,'Rest between pulses (s)',.5,.01,60,2)
        self.dead_button=self.button(g,'Run alternating pulses — MOVES MOTOR',self.run_deadzone)
        self.button(g,'Analyze recorded breakaway pulses',self.analyze_deadzone)
        self.button(g,'Save last dead-zone result to device RAM',self.save_deadzone)
        self.deadzone_label=self.text(g,'Device dead zone: unknown',small=True)
        self.deadzone_result=None
        self.text(g,'Each short pulse starts from rest. Result is load-dependent breakaway, not a motor rating. A limit trip or moving baseline invalidates that pulse.',small=True)
    def model_page(self,v):
        self.estimate=EstimatePage(self,self.estimation_root);self.estimate.build(v)
        g=self.group(v,'Extras')
        self.model_source=self.text(g,'Open recording: none.',small=True)
        self.matlab_method=self.combo(g,'MATLAB speed model',['PT1','tfest'])
        self.button(g,'Fit / validate speed model in MATLAB (open recording)',self.fit_matlab)
        self.button(g,'Export latest analysis JSON',self.export_result)
        self.text(g,'MATLAB starts in a separate process on demand and uses the output-shaft arm speed; live sensors and STOP stay independent.',small=True)
    def onboard_page(self,v):
        self.onboard=OnboardPage(self);self.onboard.build(v)
    def twin_page(self,v):
        self.twin=TwinPage(self);self.twin.build(v)
    def setup_page(self,v):
        g=self.group(v,'Encoder scale')
        self.gear=self.number(g,'Reduction (provisional)',34,.1,10000,4)
        self.motor_ppr=self.number(g,'Motor encoder cycles / revolution',11,1,10000,0)
        self.quadrature=self.number(g,'Quadrature multiplier',4,1,4,0)
        self.scale_verified=W.QCheckBox('Scale verified against a measured output angle');g.addWidget(self.scale_verified)
        self.scale_verified.toggled.connect(self.view_options_changed)
        self.button(g,'Apply proposed encoder scale to RAM',self.apply_scale)
        self.text(g,'34 × 11 × 4 = 1496 counts/output revolution is a hypothesis until checked.',small=True)
        g=self.group(v,'Known-displacement scale check')
        self.text(g,'1. Turn the arm by a measured angle by hand.  2. Enter the count change shown in raw units.')
        self.delta_counts=self.number(g,'Measured count change',0,-1e9,1e9,0)
        self.delta_angle=self.number(g,'Measured output angle (deg)',0,-360,360,2)
        self.button(g,'Use measured scale',self.measured_scale)
        g=self.group(v,'Device profile')
        self.bus_voltage=self.number(g,'Assumed motor supply (V)',12,.1,30,2)
        self.button(g,'Clear a resolved fault',lambda:self.submit(lambda:self.device.rpc('CLEAR_FAULT')))
        self.button(g,'Cancel device capture (stuck “Capture/calibration in progress”)',self.cancel_capture)
        self.button(g,'Save current profile to device NVS',lambda:self.submit(lambda:self.device.rpc('SAVE_CONFIG')))
        self.button(g,'Show device / link details (raw units)',self.details)
        g=self.group(v,'Help')
        self.button(g,'Open the focused operating guide',self.guide)

    # ----------------------------------------------------------- task plumbing
    def safe(self,fn):
        try:fn()
        except Exception as e:self.failed(str(e),release=False)
    def submit(self,fn,done=None,label='Working…'):
        if self.busy:raise RuntimeError('Wait for the current task or cancel it')
        self.busy=True;self.cancel.clear()
        for b in self.actions:b.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.progress_label.setText(label);self.progress_row.show()
        future=self.executor.submit(fn)
        def completed(f):
            try:self.signals.done.emit(f.result(),done)
            except Exception as e:self.signals.failed.emit(str(e))
        future.add_done_callback(completed)
    @QtCore.Slot(object,object)
    def finished(self,result,done):
        if self.closing:return
        self.release();self.statusBar().showMessage('Task complete. Raw data retained; no automatic motion started.')
        try:
            if done:done(result)
            else:self.report('Completed',str(jsonable(result)))
        except Exception as e:self.failed(str(e))
    def release(self):
        self.busy=False;self.progress_row.hide()
        for b in self.actions:b.setEnabled(True)
        self.connect_button.setEnabled(True)
    @QtCore.Slot(str)
    def failed(self,message,release=True):
        if release:self.release()
        self.report('Action needs attention',message);self.statusBar().showMessage(message)
    @QtCore.Slot(str)
    def progress(self,message):
        self.statusBar().showMessage(message);self.progress_label.setText(message)
    def report(self,title,text):
        self.output.setMarkdown(f'## {title}\n\n{text}');self.bottom.setCurrentIndex(0)
    def raw_source(self):
        """(rows, t, series, label) of the data the dashboard currently shows."""
        if self.view_source is not None:
            if self.view_cache is None:return None
            t,series=self.view_cache;return self.view_source['rows'],t,series,f'{self.view_source["kind"]} {self.view_source["name"]}'
        t,series,a=self.live_series() if self.device.ready else (None,None,[])
        return (a,t,series,'live buffer') if series else None
    def update_raw_table(self,force=False):
        if self.bottom.currentIndex()!=1 or (self.raw_pause.isChecked() and not force):return
        src=self.raw_source()
        if src is None:self.raw_table.setRowCount(0);self.raw_info.setText('No samples.');return
        rows,t,_,label=src
        if self.view_source is None:
            idx=np.arange(len(rows))[::-1][:50];info=f'Live: latest {len(idx)} samples, newest first (raw units, 100 Hz stream)'
        else:
            t0,t1=self.viewer.x_range();lo=-np.inf if t0 is None else t0;hi=np.inf if t1 is None else t1
            inside=np.flatnonzero((t>=lo)&(t<=hi));idx=inside[:300]
            info=f'{label}: {len(idx)} of {len(inside)} samples in the visible range (raw units)'
        self.raw_info.setText(info);self.raw_table.setRowCount(len(idx))
        for r,i in enumerate(idx):
            self.raw_table.setItem(r,0,W.QTableWidgetItem(f'{t[i]:.4f}'))
            for c,n in enumerate(S.RAW_FIELDS):self.raw_table.setItem(r,c+1,W.QTableWidgetItem(S.raw_text(n,S.field(rows[i],n))))
        if force:self.raw_table.resizeColumnsToContents()
    def export_csv(self):
        src=self.raw_source()
        if src is None:raise ValueError('Connect or open a recording first')
        if self.view_source is None:
            d=self.device;rows=d.snapshot(60000);st=d.status
            t,series=S.compute(rows,d.config,center=st.get('centerCount'),centered=bool(st.get('centered')),raw=False,
                               smooth_hz=self.smooth_hz.value(),scale_verified=self.scale_verified.isChecked())
        else:rows,t,series,_=src
        path,_=W.QFileDialog.getSaveFileName(self,'Export samples',str(ROOT/'data'/'samples.csv'),'CSV (*.csv)')
        if path:S.export_csv(path,rows,series,t);self.statusBar().showMessage(f'Exported {len(rows)} samples to {path}')

    # ------------------------------------------------------------------ views
    def page_changed(self,index):
        self.pages.setCurrentIndex(index)
        if index==3:self.right_stack.setCurrentIndex(2 if self.fit_drawn else 0)
        elif index==4:self.right_stack.setCurrentIndex(3 if self.twin.on_onboard.isChecked() else 0)   # onboard control: live signals or the twin
        elif index==5:self.right_stack.setCurrentIndex(3)      # digital twin
        else:self.right_stack.setCurrentIndex(1 if index and self.analysis_drawn else 0)
        self.live=self.right_stack.currentIndex()==0;self.twin.set_active(self.right_stack.currentIndex()==3)
    def show_fit_view(self):
        self.fit_drawn=True;self.live=False;self.right_stack.setCurrentIndex(2)
    def show_analysis(self):
        self.analysis_drawn=True;self.live=False;self.right_stack.setCurrentIndex(1)
    def prepare_plot(self,log=False):
        self.line.setData([],[]);self.second.setData([],[])
        # Reset old linear millivolt ranges before enabling a logarithmic
        # axis, and restore autoscale when returning from a frequency view.
        self.plot.setYRange(-2,2,padding=0);self.plot.setLogMode(x=False,y=log)
        self.plot.enableAutoRange(x=True,y=True);self.plot.setTitle('')
    def set_live(self):
        self.nav.setCurrentIndex(0);self.live=True;self.view_source=None;self.view_cache=None;self.view_dirty=True
        self.back_live.hide();self.right_stack.setCurrentIndex(0);self.viewer.reset();self.prepare_plot(False)
        self.source_label.setText('Source: live stream · readouts are mean ± σ over the last 0.5 s')
        self.plot.setLabel('bottom','Time','s')
    def check_signals(self,on):
        for k,c in self.signal_checks.items():
            c.blockSignals(True);c.setChecked(k in S.DEFAULT_KEYS if on is None else on);c.blockSignals(False)
        self.view_options_changed()
    def selected_keys(self):return [k for k,c in self.signal_checks.items() if c.isChecked()]
    def view_mode(self):return 'separate' if self.mode_separate.isChecked() else 'combined'
    def update_smooth_label(self):
        fc=self.smooth_hz.value();self.smooth_label.setText(f'2nd-order Butterworth · delay ≈ {S.butter_delay_ms(fc):.1f} ms')
    def view_options_changed(self,*_):
        self.update_smooth_label();self.view_dirty=True;self.view_cache=None
        if self.view_source is not None:self.draw_recording()
    def range_changed(self):
        if self.view_source is not None:self.update_readouts_recording()

    def live_series(self):
        d=self.device;hz=d.config.get('streamHz',100) or 100
        a=d.snapshot(int(self.window_s.value()*hz*1.1)+50)
        if not len(a):return None,None,a
        st=d.status
        t,series=S.compute(a,d.config,center=st.get('centerCount'),centered=bool(st.get('centered')),
                           raw=self.raw_units.isChecked(),smooth_hz=self.smooth_hz.value(),scale_verified=self.scale_verified.isChecked())
        return t,series,a

    def set_readouts(self,series,t,t0,t1):
        visible=set(self.selected_keys())
        for k,label in self.readouts.items():
            if k not in visible or k not in series:label.setText('<span style="color:#97a3af">—</span>');continue
            s=series[k];m,sd=S.stats(t,s.values,t0,t1)
            if k=='mag_status' and m is not None:
                last=s.values[np.isfinite(s.values)];text=f'{S.magnet_text(last[-1]) if len(last) else "—"} (0x{int(last[-1]) if len(last) else 0:02x})'
            else:
                if k=='pend_deg' and s.unit=='deg':m,sd=S.angle_stats(t,s.values,t0,t1)
                text=S.fmt(m,sd,s.unit)
            note=f'<br><span style="color:#b26a00;font-size:11px">{s.note}</span>' if s.note else ''
            label.setText(f'<b>{text}</b>{note}');label.setToolTip(s.note or '')

    def open_recording(self):
        path=W.QFileDialog.getExistingDirectory(self,'Select a run (manifest.json) or still capture (capture.npz) folder',str(ROOT/'data'))
        if not path:return
        p=Path(path)
        if (p/'manifest.json').exists():
            self.submit(lambda:('run',p,load_run(p)),self.recording_loaded,'Loading recording…')
        elif (p/'capture.json').exists():
            self.submit(lambda:('capture',p,load_capture(p)),self.recording_loaded,'Loading still capture…')
        else:raise ValueError('Folder has neither manifest.json (run) nor capture.json (still capture)')
    def recording_loaded(self,r):
        kind,path,data=r
        if kind=='run':
            rows,manifest=data;self.set_run(path,rows,manifest)
            source=dict(name=path.name,kind='Run',rows=rows,config=manifest.get('config',{}),status=manifest.get('status',{}),meta=manifest.get('metadata',{}))
        else:
            data['directory']=str(path);self.captured(data)
            source=dict(name=path.name,kind='Still capture',rows=data['core'],config=data.get('config',{}),status=data.get('status',{}),meta=data.get('metadata',{}))
        self.show_recording(source)
    def show_recording(self,source):
        self.view_source=source;self.view_cache=None;self.nav.setCurrentIndex(0);self.right_stack.setCurrentIndex(0);self.live=False
        self.viewer.reset();self.back_live.show()
        self.source_label.setText(f'Source: {source["kind"].lower()} {source["name"]} · readouts are mean ± σ over the visible time range')
        self.draw_recording()
    def draw_recording(self):
        src=self.view_source
        if src is None:return
        if not len(src['rows']):self.viewer.set_empty(f'{src["kind"]} {src["name"]} has no mechanical/core samples.');return
        if self.view_cache is None:
            st=src.get('status') or {}
            self.view_cache=S.compute(src['rows'],src['config'],center=st.get('centerCount'),centered=bool(st.get('centered')),
                                      raw=self.raw_units.isChecked(),smooth_hz=self.smooth_hz.value(),
                                      scale_verified=bool((src.get('meta') or {}).get('scale_verified')))
        t,series=self.view_cache
        self.viewer.show_series(t,series,self.selected_keys(),self.view_mode(),f'{src["kind"]} {src["name"]}')
        self.update_readouts_recording()
    def update_readouts_recording(self):
        if self.view_cache is None:return
        t,series=self.view_cache;t0,t1=self.viewer.x_range();self.set_readouts(series,t,t0,t1);self.update_raw_table()

    # ------------------------------------------------------------- header bar
    def update_header(self):
        d=self.device;s=d.status;state=s.get('state','DISCONNECTED')
        if d.ready:self.update_deadzone_label()
        text,color=STATE_STYLE.get(state,(state,'#6b7785'))
        if state in ('FAULT',) and s.get('fault'):text=f'FAULT · {s["fault"]}'
        self.pill.setText(text);self.pill.setStyleSheet(f'background:{color};color:white;font-weight:700;border-radius:11px;padding:6px 14px;')
        c=d.config if d.ready else {}
        flags={'centered':bool(s.get('centered')) if d.ready else False,
               'down':S.finite(c.get('pendDownRaw')) and c.get('pendDownRaw',-1)>=0,
               'zero':S.finite(c.get('currentZeroMv')),'scale':self.scale_verified.isChecked()}
        for key,(label,text) in self.chips.items():
            on=flags[key]
            label.setText(('✓ ' if on else '✗ ')+text)
            label.setStyleSheet(f'background:{"#d8f0e3" if on else "#eceff2"};color:{"#1f7a4d" if on else "#6b7785"};'
                                'border-radius:10px;padding:5px 10px;font-weight:600;')

    # ---------------------------------------------------------------- device
    def refresh_ports(self):
        current=self.port.currentText();ports=[p.device for p in list_ports.comports()]
        ports.sort(key=lambda p:p!='COM11');self.port.clear();self.port.addItems(ports)
        if current in ports:self.port.setCurrentText(current)
    def connect_device(self):
        if self.device.connected:self.device.close();self.connect_button.setText('Connect USB');return
        port=self.port.currentText()
        if not port:raise RuntimeError('Choose an available USB port')
        def connect():
            self.device.connect(port);c=copy.deepcopy(self.device.config);c['streamHz']=100;self.device.apply(c)
            return self.device.status
        def done(r):
            self.connect_button.setText('Disconnect');self.seen_events=self.device.event_count
            i=self.device.identity
            self.report('Connected',f'{i.get("firmware","?")} · USB-only mode, motor drive disabled.\n\n'
                        f'ADC oversampling {i.get("adcOversample","n/a")}× ({i.get("adcReadUs",0):.1f} µs per read). '
                        'Set the references, then watch the signals.')
        self.submit(connect,done,'Connecting…')
    def power_changed(self,on):
        self.power.setStyleSheet(POWER_ON_STYLE if on else POWER_OFF_STYLE)
        self.power.setText('⚠  MOTION ALLOWED — 12 V connected' if on else '⚠  12 V connected — allow motion')
        if not on:self.stop()
    def stop(self):
        self.device.stop();self.running=False;self.auto_deadzone_path=None
        path=self.device.finish_record()
        if path:self.load_recorded(path)
        self.record_button.setText('Start recording')
        self.statusBar().showMessage('STOP sent. Motion permission canceled; drive disabled.')
    def require_disarmed(self):
        if not self.device.ready or self.device.status.get('state')!='DISARMED':raise RuntimeError('Connect and disarm first')
    def metadata(self):
        return {'gear':self.gear.value(),'motor_cpr':self.motor_ppr.value()*self.quadrature.value(),
                'scale_verified':self.scale_verified.isChecked(),'12v_declared':self.power.isChecked(),
                'left_count':self.left_count,'right_count':self.right_count}
    def calibrate(self,target):
        self.require_disarmed()
        if self.device.record:raise ValueError('Finish recording before changing a reference')
        if target=='current':self.seen_events=self.device.event_count
        hint={'arm':'Arm center stored.','down':'Pendulum hanging-down reference stored.',
              'current':'Measuring current zero for 1 s — keep the motor unpowered. The result appears here.'}[target]
        self.submit(lambda:self.device.calibrate(target),lambda r:self.report('Reference requested',hint),'Setting reference…')
    def zero_report(self,ev):
        mean=ev.get('calMeanMv');sd=ev.get('calSdMv') or 0;n=ev.get('calSamples');sens=ev.get('sensitivityMvA') or self.device.config.get('sensitivityMvA')
        ok=ev.get('event')=='CURRENT_ZERO_UPDATED'
        amps=f' = {sd/sens:.3f} A' if sens else ''
        over=f' · ADC oversampling {ev["adcOversample"]}×' if ev.get('adcOversample') else ''
        self.report('Current zero '+('stored' if ok else 'REJECTED'),
                    f'**Measured zero:** {mean:.2f} mV · **σ per sample:** {sd:.2f} mV{amps} · {n} samples in 1 s{over}.\n\n'+
                    ('Amperes now use this zero.' if ok else 'Rejected: the mean must lie in 2200–2800 mV with no clipped samples and ≥ 100 samples. Keep the motor unpowered and retry.'))
    def apply_scale(self):
        self.require_disarmed();c=copy.deepcopy(self.device.config)
        c['countsPerRev']=self.gear.value()*self.motor_ppr.value()*self.quadrature.value()
        self.submit(lambda:self.device.apply(c),lambda r:self.report('Scale applied to RAM',f'{c["countsPerRev"]:g} counts/output revolution. '+('Marked verified.' if self.scale_verified.isChecked() else '**Provisional**: verify with a measured angle.')))
    def measured_scale(self):
        self.require_disarmed();dc=self.delta_counts.value();angle=self.delta_angle.value()
        if dc==0 or angle==0:raise ValueError('Enter a nonzero measured count change and angle')
        c=copy.deepcopy(self.device.config);c['countsPerRev']=abs(dc*360/angle);c['armSign']=1 if dc/angle>0 else -1
        self.gear.setValue(c['countsPerRev']/(self.motor_ppr.value()*self.quadrature.value()));self.scale_verified.setChecked(True)
        self.submit(lambda:self.device.apply(c),lambda r:self.report('Measured scale applied',f'{c["countsPerRev"]:.3f} counts/output revolution. Repeat in both directions to assess backlash.'))
    def mark_endpoint(self,side):
        self.require_disarmed()
        if self.power.isChecked():raise ValueError('Mark endpoints with motor supply disconnected; untick the 12 V box')
        a=self.device.snapshot(1)
        if not len(a):raise ValueError('Wait for encoder samples')
        count=int(a['count'][-1])
        if side=='left':self.left_count=count
        else:self.right_count=count
        center=int(self.device.status.get('centerCount',0))
        self.limit_label.setText(f'Left {self.left_count} · center {center} · right {self.right_count} counts')
        if self.left_count is not None and self.right_count is not None:
            lo,hi=sorted([self.left_count,self.right_count])
            if not lo<center<hi:raise ValueError('Center must lie strictly between marked endpoints')
            safe=int(.9*min(center-lo,hi-center));self.travel.setValue(safe)
            self.report('Safe travel proposal',f'{safe} counts each side of center (10% margin inside the closer endpoint). Click **Apply these limits** to commission it.')
    def apply_limits(self):
        self.require_disarmed();c=copy.deepcopy(self.device.config)
        c.update(maxDuty=self.max_duty.value()/100,travelCounts=int(self.travel.value()),maxDuration=self.max_duration.value(),slewPerSecond=self.slew.value()/100,
                 currentTripA=self.current_trip.value(),velocityCountsPerSecond=self.speed_trip.value(),assumedBusV=self.bus_voltage.value())
        self.submit(lambda:self.device.apply(c),lambda r:self.report('Limits applied to RAM','Readback confirmed. Setting limits does not start motion. Trip values of 0 mean OFF. '
                    'The current trip acts on the MCU-filtered current and needs a credible zero/gain.'))
    def toggle_record(self):
        if self.device.record:
            path=self.device.finish_record();self.record_button.setText('Start recording');self.load_recorded(path);return
        self.require_disarmed();path=self.new_path('runs')
        self.device.start_record(path,self.metadata());self.record_button.setText('Finish recording')
        self.report('Recording',f'{path}\n\nClick **Finish recording** to stop. No motor command was started.')
    def new_path(self,kind):return ROOT/'data'/kind/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000:06d}')

    # ------------------------------------------------------------ noise tools
    def capture_still(self):
        self.require_disarmed()
        if self.device.record:raise ValueError('Finish live recording before a raw still capture')
        if not self.still.isChecked():raise ValueError('Confirm both joints are still before measuring noise')
        seconds=self.capture_seconds.value();rate=int(self.capture_rate.currentText());meta=self.metadata();path=self.new_path('captures')
        def capture():
            r=self.device.capture(seconds,self.signals.progress.emit,self.cancel,rate);save_capture(path,r,meta);r['metadata']=meta;r['directory']=str(path);return r
        self.submit(capture,self.captured,'Recording still data…')
    def captured(self,capture):
        self.capture_data=capture;self.analysis_result=None;self.filter_rows=[];self.filter_choice.clear()
        self.report('Still recording ready',f'ADC: {len(capture["adc"])} samples. Mechanical: {len(capture["core"])} samples.\n\n'
                    f'Choose a sensor, then **Plot noise spectrum + statistics**.\n\n{capture.get("directory","")}')
    def noise_name(self):return ['current_mv','arm_count','pend_deg'][self.noise_sensor.currentIndex()]
    def capture_sensitivity(self):
        c=(self.capture_data or {}).get('config',{}) or self.device.config;s=c.get('sensitivityMvA')
        return s if S.finite(s) and s>0 else None
    def analyze_noise(self):
        if self.capture_data is None:raise ValueError('Record or open a still capture first')
        capture=self.capture_data;name=self.noise_name()
        self.submit(lambda:A.noise(capture,name),self.noise_done,'Analyzing noise…')
    def noise_done(self,r):
        r['capture_config']=self.capture_data.get('config',{});r['capture_metadata']=self.capture_data.get('metadata',{})
        self.analysis_result=r;self.show_analysis();self.prepare_plot(True)
        visible=r['frequency']<=min(self.frequency_view.value(),r['fs']/2)
        self.line.setData(r['frequency'][visible],np.maximum(r['psd'][visible],1e-20));self.second.setData([],[])
        self.plot.setLabel('bottom','Frequency','Hz');self.plot.setLabel('left',f'Noise PSD ({r["unit"]}²/Hz)');self.plot.setTitle(r['sensor'])
        self.plot.setXRange(0,min(self.frequency_view.value(),r['fs']/2),padding=0)
        u=r['unit'];sens=self.capture_sensitivity() if r['sensor']=='current_mv' else None
        amps=lambda x:f' = **{x/sens:.4g} A**' if sens else ''
        if sens:r.update(sd_a=r['sd']/sens,peak_to_peak_a=r['peak_to_peak']/sens,sensitivity_mv_a=sens)
        self.report('Stationary noise',f'**SD:** {r["sd"]:.5g} {u}{amps(r["sd"])} · **Peak-to-peak:** {r["peak_to_peak"]:.5g} {u}{amps(r["peak_to_peak"])}\n\n'
                    +(f'Amperes use {sens:g} mV/A from the capture profile.\n\n' if sens else '')+
                    f'Mean {r["mean"]:.5g} {u}; drift {r["drift"]:.5g} {u}. Sample rate {r["fs"]:.2f} Hz; PSD bin spacing {r["resolution"]:.3g} Hz.\n\n'
                    f'{r["windows"]} Welch windows · {r["gaps"]} timestamp gaps · {r["invalid"]} invalid/held samples · jitter up to {r["jitter_percent"]:.2f}%.\n\n'
                    f'{r["note"]}\n\nStill noise does **not** reveal which frequencies carry motion. Enter the required motion bandwidth before comparing filters.')
    def plot_noise_time(self):
        if self.capture_data is None:raise ValueError('Record a still capture first')
        t,x,u=A.channel(self.capture_data,self.noise_name());self.show_analysis();self.prepare_plot(False)
        sens=self.capture_sensitivity();zero=(self.capture_data.get('config',{}) or {}).get('currentZeroMv')
        if self.noise_name()=='current_mv' and sens and S.finite(zero):x=(x-zero)/sens;u='A'
        self.line.setData(t,x);self.second.setData([],[]);self.plot.setLabel('bottom','Capture time','s');self.plot.setLabel('left',u)
    def compare_filters(self):
        if self.analysis_result is None or self.analysis_result.get('sensor')!=self.noise_name():raise ValueError('Analyze the selected sensor noise first')
        self.filter_rows=A.filters(self.analysis_result,self.bandwidth.value(),self.delay.value());self.filter_choice.clear()
        u=self.analysis_result['unit'];sens=self.capture_sensitivity() if self.noise_name()=='current_mv' else None
        for r in self.filter_rows:
            noise=f'{r["predicted_noise_rms"]/sens:.3g} A ({r["predicted_noise_rms"]:.3g} {u})' if sens else f'{r["predicted_noise_rms"]:.3g} {u}'
            self.filter_choice.addItem(f'{r["cutoff_hz"]:g} Hz · noise {noise} · delay {r["delay_ms"]:.1f} ms · '+('meets limits' if r['acceptable'] else 'outside limits'))
        if not self.filter_rows:raise ValueError('No candidate cutoff below this sensor Nyquist frequency')
        acceptable=[r for r in self.filter_rows if r['acceptable']]
        self.report('Filter comparison',f'{len(acceptable)} candidate(s) meet ≤ 1 dB loss at {self.bandwidth.value():g} Hz and ≤ {self.delay.value():g} ms delay in that band.\n\n'
                    '1. The first acceptable row leaves the least noise.\n\n2. Choose a row and **Preview** (orange = filtered, blue = raw).\n\n'
                    '3. Check an actual moving response before feedback use.')
    def selected_filter(self):
        k=self.filter_choice.currentIndex()
        if k<0 or k>=len(self.filter_rows):raise ValueError('Compare filter cutoffs first')
        if self.analysis_result.get('sensor')!=self.noise_name():raise ValueError('Sensor changed; analyze and compare again')
        r=self.filter_rows[k]
        if r['bandwidth_hz']!=self.bandwidth.value() or r['delay_budget_ms']!=self.delay.value():raise ValueError('Bandwidth/delay requirements changed; compare filters again')
        return r
    def preview_filter(self):
        r=self.selected_filter();s=self.analysis_result;self.show_analysis();self.prepare_plot(True)
        visible=s['frequency']<=min(self.frequency_view.value(),s['fs']/2)
        self.line.setData(s['frequency'][visible],np.maximum(s['psd'][visible],1e-20));self.second.setData(s['frequency'][visible],np.maximum(r['filtered_psd'][visible],1e-20))
        self.plot.setLabel('bottom','Frequency','Hz');self.plot.setLabel('left',f'PSD ({s["unit"]}²/Hz)')
        self.plot.setXRange(0,min(self.frequency_view.value(),s['fs']/2),padding=0)
        sens=self.capture_sensitivity() if s['sensor']=='current_mv' else None
        amps=f' = {r["predicted_noise_rms"]/sens:.4g} A' if sens else ''
        self.report('Selected causal low-pass',f'Cutoff {r["cutoff_hz"]:g} Hz · delay {r["delay_ms"]:.3g} ms (2nd-order Butterworth ≈ {S.butter_delay_ms(r["cutoff_hz"]):.3g} ms) · '
                    f'predicted noise RMS {r["predicted_noise_rms"]:.5g} {s["unit"]}{amps} · passband loss {r["passband_loss_db"]:.3g} dB.\n\nNo filter has been deployed. Raw recordings remain unchanged.')
    def deploy_filter(self):
        self.require_disarmed();r=self.selected_filter()
        if self.noise_name()!='current_mv':raise ValueError('Firmware deploys only the current ADC filter; mechanical filters stay offline')
        if not r['acceptable']:raise ValueError('Select a candidate that meets your declared bandwidth/delay limits')
        fs=self.device.config['adcHz'];sos=signal.butter(2,r['cutoff_hz'],fs=fs,output='sos')[0]
        c=copy.deepcopy(self.device.config);c['filter'].update(type='biquad',sos=sos.tolist())
        self.submit(lambda:self.device.apply(c),lambda _:self.report('Current filter deployed to RAM',
                    f'{r["cutoff_hz"]:g} Hz (delay ≈ {S.butter_delay_ms(r["cutoff_hz"]):.1f} ms) at {fs} samples/s. “Current (MCU filter)” and the overcurrent trip now use it; '
                    'raw current remains available. NVS persistence is a separate action.'))
    def use_live_filter(self):
        r=self.selected_filter();self.smooth_hz.setValue(r['cutoff_hz']);self.set_live()
        self.report('Live smoothing',f'Software smoothing set to {r["cutoff_hz"]:g} Hz (delay ≈ {S.butter_delay_ms(r["cutoff_hz"]):.1f} ms). '
                    'It applies to “Current (smoothed)” and both speeds; raw data is unchanged. The live stream is 100 Hz, so cutoffs ≥ 45 Hz are shown unsmoothed.')

    # ------------------------------------------------------------ motor tests
    def authorize_run(self):
        self.require_disarmed()
        if not self.power.isChecked():raise ValueError('12 V is declared disconnected. Powered tests are disabled.')
        c=self.device.config;missing=[]
        if not self.device.status.get('centered'):missing.append('the arm is not centered (Live tab → References → “Arm is centered here”)')
        if not c.get('maxDuty',0)>0:missing.append('maximum duty on the device is 0 % (Motor tests step 2 → “Maximum duty (%)”, then “Apply these limits to RAM”)')
        if not c.get('travelCounts',0)>0:missing.append('travel limit on the device is 0 counts (step 1 endpoints or “Travel each side”, then Apply)')
        if missing:raise ValueError('Motor test blocked: '+'; '.join(missing)+'.')
        self.device.inhibit=False
    def run_wave(self):
        self.authorize_run();s=A.segment(self.wave.currentText(),self.duration.value(),self.duty.value()/100,self.frequency.value())
        if s['kind']=='prbs':s['bitPeriod']=1/self.frequency.value();s['seed']=7
        self.dead_windows=None;self.start_sequence([A.segment(duration=.5),s,A.segment(duration=.5)],'waveform')
    def run_deadzone(self):
        self.authorize_run()
        try:
            on=self.dead_on.value();top=self.dead_max.value()
            seq,windows=A.deadzone_sequence(top,self.dead_step.value(),on,self.dead_rest.value(),
                                            int(self.device.identity.get('maxSegments',16)),self.dead_start.value())
            # Each pulse must reach its level within the first half of the pulse.
            need=top/100/(.5*on);slew=self.device.config.get('slewPerSecond',0)
            if slew<need:raise ValueError(f'Slew rate {slew*100:g} %/s cannot reach {top:g} % within half of a {on:g} s pulse; '
                                          f'set “Slew rate” to at least {need*100:.0f} %/s and Apply, or lengthen the pulse.')
        except Exception:self.device.inhibit=True;raise
        self.dead_windows=windows;self.auto_deadzone_path=self.start_sequence(seq,'breakaway')
    def start_sequence(self,segments,purpose,extra=None):
        c=self.device.config;total=sum(s['duration'] for s in segments);peak=max(abs(s['amplitude']) for s in segments)
        problem=None
        if c['maxDuration']>0 and total>c['maxDuration']:problem=f'Sequence lasts {total:.2f} s but the device maximum run is {c["maxDuration"]:g} s (step 2 “Maximum run seconds”, then Apply).'
        elif peak>c['maxDuty']+1e-12:problem=(f'Sequence needs {peak*100:g} % duty but the device maximum duty is {c["maxDuty"]*100:g} %. '
                                              'Raise “Maximum duty (%)” and Apply, or lower the amplitude / “Maximum test duty”.')
        if problem:
            self.device.inhibit=True;raise ValueError(problem)
        if self.busy:
            self.device.inhibit=True;raise RuntimeError('Wait for the current task or cancel it')
        meta=self.metadata()|{'purpose':purpose,'segments':segments,'dead_windows':self.dead_windows}|(extra or {})
        path=self.new_path('runs');self.run_metadata=meta;self.run_start=None;self.run_segments=segments
        self.device.start_record(path,meta)
        def run():
            try:self.device.run(segments,meta)
            except Exception:self.device.stop();self.device.finish_record();raise
            return str(path)
        try:self.submit(run,lambda p:(setattr(self,'running',True),self.report('Running finite experiment',f'Recording to {p}. STOP remains active.')),'Uploading and starting…')
        except Exception:
            self.device.inhibit=True;self.device.finish_record();raise
        return path
    def load_recorded(self,path):
        rows,manifest=load_run(path);self.set_run(path,rows,manifest)
        self.report('Run saved',f'{len(rows)} samples.\n\n{path}\n\nView it with Open recording….')
        self.estimate.recording_finished(path,manifest);self.onboard.recording_finished(path,rows,manifest)
    def set_run(self,path,rows,manifest):
        self.run_data,self.run_manifest,self.run_path=rows,manifest,Path(path)
        meta=manifest.get('metadata',{});self.dead_windows=meta.get('dead_windows');self.run_start=None
        self.set_model_source(Path(path).name);self.twin.run_changed()
    def set_model_source(self,name):
        meta=self.run_manifest.get('metadata',{})
        self.model_source.setText(f'Open recording: {name} · {len(self.run_data)} samples · {meta.get("purpose","recording")}')
    def run_loaded(self,r):
        path,(rows,manifest)=r;self.set_run(path,rows,manifest);meta=manifest.get('metadata',{})
        self.report('Run loaded',f'{len(rows)} samples · {meta.get("purpose","run")}.\n\n{path}')
        if self.auto_deadzone_path is not None and Path(path)==Path(self.auto_deadzone_path):
            self.auto_deadzone_path=None
            if meta.get('purpose')=='breakaway':self.analyze_deadzone(save=True)
        self.estimate.recording_finished(path,manifest);self.onboard.recording_finished(path,rows,manifest)
    def require_run(self):
        if self.run_data is None or len(self.run_data)<150:raise ValueError('Finish or open a sufficiently long recorded run first')
        return self.run_data,self.run_manifest['config'],self.run_manifest.get('metadata',{})
    def analyze_deadzone(self,save=False):
        d,c,m=self.require_run();windows=m.get('dead_windows')
        if not windows:raise ValueError('Open a breakaway-pulse run')
        active=np.flatnonzero(d['state']==3)
        if not len(active):raise ValueError('No running-state samples in recording')
        r=A.deadzone(d,int(d['device_us'][active[0]]),windows,self.move_threshold.value());self.analysis_result=r;self.deadzone_result=r
        p=r['pulses'];found=r['breakaway_pct'];top=max((abs(q['command_pct']) for q in p),default=0)
        fmt=lambda v:'not found' if v is None else f'{v:.2f} %'
        unreached=sum(1 for q in p if not q['plateau_reached'])
        rows='\n'.join(f'| {q["command_pct"]:+.1f} | {q["applied_pct"]:+.1f} | {q["delta_counts"]:+.0f} | {"yes" if q["moved"] else "no"} | {"yes" if q["valid"] else "no"} |' for q in p)
        text=(f'**Dead zone (breakaway duty):** + {fmt(found["positive"])} · − {fmt(found["negative"])} '
              f'(movement ≥ {r["threshold_counts"]:g} counts, tested up to {top:g} %).\n\n'
              +(f'⚠ {unreached} pulse(s) never reached their commanded duty (slew limit or pulse too short); they are ignored.\n\n' if unreached else '')+
              '| command % | applied % | moved counts | moved | valid |\n|---|---|---|---|---|\n'+rows+f'\n\n{r["note"]}')
        self.report('Breakaway result',text)
        if save:self.save_deadzone()
    def save_deadzone(self):
        r=self.deadzone_result
        if r is None:raise ValueError('Run or analyze a breakaway test first')
        found=r['breakaway_pct']
        if found['positive'] is None and found['negative'] is None:
            self.output.append('\nNothing saved to the device: no breakaway was found in the tested range.');return
        self.require_disarmed();c=copy.deepcopy(self.device.config)
        for key,name in (('deadzonePos','positive'),('deadzoneNeg','negative')):
            if found[name] is not None:c[key]=round(abs(found[name])/100,6)
        def done(_):
            self.update_deadzone_label()
            self.output.append(f'\n**Saved to device RAM:** dead zone + {fmt(c.get("deadzonePos"))} / − {fmt(c.get("deadzoneNeg"))}. '
                               'Use “Save current profile to device NVS” (Setup) to keep it after a restart.')
        fmt=lambda v:'unknown' if v is None or not S.finite(v) else f'{v*100:.2f} %'
        self.submit(lambda:self.device.apply(c),done,'Saving dead zone…')
    def update_deadzone_label(self):
        c=self.device.config;f=lambda v:'unknown' if v is None or not S.finite(v) else f'{v*100:.2f} %'
        if 'deadzonePos' not in c:self.deadzone_label.setText('Device dead zone: firmware too old to store it');return
        self.deadzone_label.setText(f'Device dead zone: + {f(c.get("deadzonePos"))} · − {f(c.get("deadzoneNeg"))}')
    def fit_matlab(self):
        d,c,m=self.require_run();a,t,g=A.prepared_core(d);cpr=c.get('countsPerRev')
        if not S.finite(cpr) or cpr<=0:raise ValueError('The recording has no encoder scale (countsPerRev)')
        angle=np.interp(g,t,(c.get('armSign',1) or 1)*(a['count']-a['count'][0]).astype(float)*2*np.pi/cpr)
        _,speed,_,edge=A.smooth_derivatives(g,angle,.08);u=np.interp(g,t,a['applied']);method=self.matlab_method.currentText()
        self.submit(lambda:self.backend.speed(g[edge:-edge],u[edge:-edge],speed[edge:-edge],method),self.matlab_done,'MATLAB starting in background (tens of seconds)…')
    def matlab_done(self,r):
        r['source_manifest']=copy.deepcopy(self.run_manifest)
        self.analysis_result=r;self.show_analysis();self.prepare_plot(False)
        self.line.setData(np.asarray(r['time']).ravel(),np.asarray(r['measured']).ravel());self.second.setData(np.asarray(r['time']).ravel(),np.asarray(r['simulated']).ravel())
        self.plot.setLabel('bottom','Time','s');self.plot.setLabel('left','Arm (output shaft) speed','rad/s')
        self.report('MATLAB model validation',f'{r["method"]}: holdout fit {r["fit_percent"]:.3g}%, RMSE {r["rmse"]:.5g}.\n\n{r["note"]}\n\n{r["job_directory"]}')
    def export_result(self):
        if self.analysis_result is None:raise ValueError('Run an analysis first')
        path,_=W.QFileDialog.getSaveFileName(self,'Export analysis',str(ROOT/'data'/'analysis.json'),'JSON (*.json)')
        if path:Path(path).write_text(json.dumps(jsonable(self.analysis_result),indent=2),encoding='utf-8')
    def details(self):
        a=self.device.snapshot(1);last={n:a[n][-1].item() for n in a.dtype.names if not n.startswith('_')} if len(a) else {}
        self.report('Device and link',json.dumps(jsonable({'identity':{k:v for k,v in self.device.identity.items() if k not in ('status','challenge')},
                    'status':self.device.status,'config':self.device.config,'last_sample_raw':last,
                    'link_errors':self.device.parser.errors,'bad_frames':self.device.parser.bad_frames[-3:],
                    'max_serial_loop_gap_s':self.device.max_io_gap,'max_gui_refresh_s':self.max_refresh}),indent=2))
    def cancel_capture(self):
        if not self.device.ready:raise RuntimeError('Connect first')
        def done(r):
            if r is None:self.report('Cancel capture','This firmware has no BURST_CANCEL (flash 1.10). A capture ends by itself after its length.')
            else:self.report('Cancel capture',f'Device capture cancelled (was {"recording" if r.get("wasActive") else "armed" if r.get("wasPending") else "idle"}).')
        self.submit(self.device.cancel_burst,done,'Cancelling the device capture…')
    def guide(self):
        self.output.setMarkdown((ROOT/'docs'/'FOCUSED_GUIDE.md').read_text(encoding='utf-8'))
    def cancel_job(self):self.cancel.set();self.stop();threading.Thread(target=self.backend.cancel,daemon=True).start()

    # ---------------------------------------------------------------- refresh
    def refresh(self):
        begin=time.perf_counter();self.refresh_delays.append(begin-self.last_refresh if self.last_refresh else 0);self.last_refresh=begin
        d=self.device;s=d.status;self.update_header()
        if self.view_source is None:
            t,series,a=self.live_series() if d.ready else (None,None,[])
            if series:
                last=a[-1];key=S.best_current_key(d.config);cur=series[key];m,sd=S.stats(t,cur.values,t[-1]-READOUT_WINDOW_S)
                self.sensors.setText(f'Current {S.fmt(m,sd,cur.unit)}  ·  magnet {S.magnet_text(last["mag_status"])}  ·  '
                                     f'{len(a)} samples  ·  link errors {d.parser.errors}  ·  dropped {d.meta.get("dropped",0)}'
                                     +('  ·  capture recording' if s.get('burstActive') else '  ·  capture armed' if s.get('burstPending') else ''))
                if self.right_stack.currentIndex()==0:
                    self.viewer.show_series(t,series,self.selected_keys(),self.view_mode(),'Live — latest %g s'%self.window_s.value(),follow=self.window_s.value())
                self.set_readouts(series,t,t[-1]-READOUT_WINDOW_S,None);self.update_raw_table()
            else:
                state=s.get('state','DISCONNECTED')
                self.sensors.setText(f'{state} · USB-only defaults · no samples yet')
                self.viewer.set_empty('Connected — waiting for samples…' if d.connected else
                                      'Not connected.\n\nChoose the USB port and press Connect USB,\nor use Open recording… to view a saved run or still capture.')
                for r in self.readouts.values():r.setText('<span style="color:#97a3af">—</span>')
        if d.event_count!=self.seen_events:
            self.seen_events=d.event_count;ev=d.last_event or {}
            if ev.get('event') in ('CURRENT_ZERO_UPDATED','CURRENT_ZERO_REJECTED'):self.zero_report(ev)
        state=s.get('state')
        if self.running:
            if state=='RUNNING':self.run_start=self.run_start or time.monotonic()
            elif self.run_start or state=='FAULT':
                self.running=False;d.inhibit=True;path=d.finish_record()
                if path:self.pending_load=path
        if self.pending_load and not self.busy:
            path=self.pending_load;self.pending_load=None
            self.submit(lambda:(path,load_run(path)),self.run_loaded,'Loading run…')
        if not self.busy:
            allow=d.ready and self.power.isChecked() and state=='DISARMED'
            self.run_button.setEnabled(allow);self.dead_button.setEnabled(allow)
            for w in self.estimate.preset_widgets.values():w['run'].setEnabled(allow)
        self.estimate.update_status();self.onboard.update_status()
        self.max_refresh=max(self.max_refresh,time.perf_counter()-begin)
    def closeEvent(self,event):
        self.closing=True;self.timer.stop();self.twin.close();self.cancel.set();self.device.close();self.backend.cancel();self.executor.shutdown(wait=False,cancel_futures=True);event.accept()

def main():
    app=W.QApplication(sys.argv);app.setFont(QtGui.QFont('Segoe UI',10))
    window=LabApp();window.show();sys.exit(app.exec())

if __name__=='__main__':main()
