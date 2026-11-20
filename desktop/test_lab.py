import json
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from furuta_lab import wire,analysis as A
from furuta_lab.backend import ROOT

class LabTests(unittest.TestCase):
    def test_cpp_golden_wire(self):
        g=json.loads((ROOT/'protocol/golden_vectors.json').read_text())
        for v in g['vectors']:
            b=wire.encode(v['type'],0x123456789abcdef0,v['sequence'],bytes.fromhex(v['payloadHex']))
            self.assertEqual(b.hex(),v['frameHex'])
            parser=wire.Parser(synced=True);frames=[]
            for value in b:frames+=parser.feed(bytes([value]))
            self.assertEqual(len(frames),1);self.assertEqual(frames[0][1],0x123456789abcdef0)
        a,meta=wire.core_batch(bytes.fromhex(g['corePayloadHex']))
        self.assertEqual(int(a['device_us'][0]),int(g['coreTime']))
        self.assertEqual(int(a['count'][0]),-(2**53)-1)
    def test_core_record_ctrl_byte(self):
        # firmware 1.6 puts the onboard controller state in a former padding byte; size unchanged
        self.assertEqual(wire.CORE.itemsize,88)
        self.assertEqual(wire.CORE.fields['ctrl'][1],wire.CORE.fields['state'][1]+1)
        self.assertEqual(wire.CTRL_NAMES[3],'STABILIZE')
    def test_parser_corruption_and_resync(self):
        p=wire.Parser(synced=True);good=wire.encode(1,2,3,b'hello');bad=bytearray(good);bad[4]^=4
        frames=p.feed(bytes(bad)+b'1'*2100+b'\0'+good)
        self.assertEqual(len(frames),1);self.assertGreater(p.errors,0)
    def test_parser_drops_partial_first_frame(self):
        good=wire.encode(4,2,3,b'x'*40);p=wire.Parser()
        frames=p.feed(bytes([0xf6,0])*5+good[17:]+bytes([0])+good+good)
        self.assertEqual(len(frames),2);self.assertEqual(p.errors,0);self.assertGreater(p.discarded,0)
        p.feed(good[5:]);self.assertEqual(p.errors,1)
    def capture(self):
        n=8192;adc=np.zeros(n,wire.ADC);t=np.arange(n)/2000
        adc['device_us']=np.round(t*1e6);adc['adc_raw']=3000
        adc['current_mv']=2500+10*np.sin(2*np.pi*60*t)+np.random.default_rng(2).normal(0,2,n)
        adc['device_us'][4000:]+=1000
        return {'adc':adc,'core':np.empty(0,wire.CORE)}
    def test_noise_gap_and_filter_constraints(self):
        c=self.capture();before=c['adc'].copy();r=A.noise(c,'current_mv')
        self.assertEqual(r['gaps'],1);self.assertAlmostEqual(r['fs'],2000)
        peak=r['frequency'][np.argmax(r['psd'])];self.assertLess(abs(peak-60),3)
        rows=A.filters(r,5,30);accepted=[q for q in rows if q['acceptable']]
        self.assertTrue(accepted);self.assertLess(accepted[0]['predicted_noise_rms'],r['sd'])
        self.assertTrue(all(q['passband_loss_db']<=1 and q['delay_ms']<=30 for q in accepted))
        self.assertTrue(np.array_equal(c['adc'],before))
        with self.assertRaises(ValueError):A.filters(r,0,20)
    def test_rank_deficiency_refused(self):
        from furuta_lab import estimation as E
        with self.assertRaises(ValueError):E._ols(np.column_stack([np.ones(200),np.zeros(200)]),np.ones(200))
    def test_psd_includes_the_end_of_record(self):
        c=self.capture();c['adc']=c['adc'][:1000].copy();c['adc']['current_mv']=2500
        c['adc']['current_mv'][-80:]+=10
        r=A.noise(c,'current_mv');self.assertGreater(r['rms_psd'],0)
    def test_deadzone_not_found_and_invalid_pulses(self):
        seq,windows=A.deadzone_sequence(3,1);self.assertLessEqual(len(seq),256)
        t=np.arange(0,6,.01);d=np.zeros(len(t),wire.CORE);d['device_us']=t*1e6;d['validity']=7
        r=A.deadzone(d,0,windows,3);self.assertIsNone(r['breakaway_pct']['positive'])
        for begin,end,command in windows:
            ix=(t>=begin)&(t<end);d['applied'][ix]=command/100;d['limits'][ix]=2
        begin,end,command=windows[4];d['count'][t>=begin+.05]=5
        r=A.deadzone(d,0,windows,3);self.assertAlmostEqual(r['breakaway_pct']['positive'],3,places=4)
    def test_physical_units(self):
        from furuta_lab import signals as S
        n=500;a=np.zeros(n,wire.CORE);a['device_us']=np.arange(n)*10000;a['validity']=7
        a['count']=1000+np.arange(n);a['pend_raw']=(2985+np.arange(n)*8)%4096
        a['current_mv']=2505.95+185*.5;a['filtered_mv']=2505.95+185*.25;a['applied']=.1
        c={'countsPerRev':1496,'armSign':-1,'pendSign':1,'pendDownRaw':2985,'currentZeroMv':2505.95,'sensitivityMvA':185,'filter':{'type':'bypass'}}
        t,s=S.compute(a,c,center=1000,centered=True,smooth_hz=5)
        self.assertAlmostEqual(s['arm_deg'].values[1],-360/1496);self.assertIn('scale unverified',s['arm_deg'].note)
        self.assertAlmostEqual(s['pend_deg'].values[0],180);self.assertAlmostEqual(s['pend_deg'].values[256],0,places=6)
        self.assertTrue(np.all((s['pend_deg'].values>-180)&(s['pend_deg'].values<=180)))
        self.assertAlmostEqual(s['current'].values[0],.5,places=6);self.assertAlmostEqual(s['current_mcu'].values[0],.25,places=5)
        self.assertAlmostEqual(s['current_smooth'].values[-1],.5,places=4);self.assertAlmostEqual(s['applied'].values[0],10,places=5)
        self.assertAlmostEqual(np.nanmedian(s['arm_speed'].values[50:]),-100*360/1496,places=3)
        self.assertEqual(S.best_current_key(c),'current_smooth');self.assertEqual(S.best_current_key({'filter':{'type':'biquad'}}),'current_mcu')
        _,r=S.compute(a,c,raw=True);self.assertEqual(r['current'].unit,'mV');self.assertEqual(r['arm_deg'].unit,'counts')
        _,z=S.compute(a,{**c,'currentZeroMv':None});self.assertEqual(z['current'].unit,'mV');self.assertIn('zero not set',z['current'].note)
        self.assertAlmostEqual(S.butter_delay_ms(20),11.25,places=2)
        self.assertEqual(S.fmt(.04,.06,'A'),'0.04 ± 0.06 A')
        self.assertEqual(S.wrap_deg(np.array([-180.,540.]))[0],180)
        y=S.continuous_angle(np.arange(4.),np.array([179.9,-179.9,179.8,-179.95]));self.assertLess(np.ptp(y),.5)
        m,sd=S.angle_stats(np.arange(4.),np.array([179.,-179.,179.,-179.]));self.assertAlmostEqual(abs(m),180,places=6);self.assertLess(sd,1.2)
        self.assertEqual(list(S.connect_mask(np.array([179.,-179.,-178.]),True)),[False,True,False])
    def test_dashboard_views_with_recording(self):
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        from furuta_lab import signals as S_
        app=QApplication.instance() or QApplication([]);w=LabApp()
        n=2000;a=np.zeros(n,wire.CORE);a['device_us']=np.arange(n)*2000;a['validity']=7;a['count']=np.arange(n)//10
        a['current_mv']=2500+np.random.default_rng(1).normal(0,10,n);a['filtered_mv']=a['current_mv']
        cfg={'countsPerRev':1496,'pendDownRaw':2048,'currentZeroMv':2500,'sensitivityMvA':185,'filter':{'type':'bypass'}}
        w.show_recording(dict(name='synthetic',kind='Run',rows=a,config=cfg,status={'centered':True,'centerCount':'0'},meta={}))
        w.check_signals(True);self.assertEqual(len(w.viewer.plots),len(w.signal_checks))
        self.assertIn(' A',w.readouts['current'].text())
        w.mode_combined.setChecked(True);self.assertEqual(list(w.viewer.plots),['combined'])
        w.raw_units.setChecked(True);self.assertIn('mV',w.readouts['current'].text())
        w.raw_units.setChecked(False);w.bottom.setCurrentIndex(1);self.assertGreater(w.raw_table.rowCount(),0)
        self.assertEqual(w.raw_table.horizontalHeaderItem(4).text(),'pend_raw')
        import tempfile,os;f=os.path.join(tempfile.mkdtemp(),'x.csv');t,series=w.view_cache;S_.export_csv(f,a,series,t)
        lines=open(f,encoding='utf-8').read().splitlines();self.assertEqual(len(lines),n+1);self.assertIn('current [A]',lines[0])
        w.set_live();self.assertIsNone(w.view_source);w.refresh();self.assertEqual(w.viewer.currentIndex(),0);w.close()
    def test_usb_default_gui_and_setup_gating(self):
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        app=QApplication.instance() or QApplication([]);w=LabApp()
        self.assertFalse(w.power.isChecked());self.assertFalse(w.device.connected)
        with self.assertRaises(RuntimeError):w.authorize_run()
        self.assertEqual(w.gear.value(),34);self.assertFalse(w.scale_verified.isChecked())
        w.prepare_plot(True);w.set_live();self.assertTrue(w.plot.getViewBox().state['autoRange'][0]);w.guide();w.close()
    def test_stop_cancels_a_late_start(self):
        from furuta_lab.device import Device
        class Port:
            def __init__(self):self.frames=[]
            def write(self,b):self.frames.append(b)
            def close(self):pass
        d=Device();d.port=Port();d.connected=True;d.inhibit=False
        d._send('ARM');d.stop()
        with self.assertRaises(RuntimeError):d._send('START')
        self.assertEqual(json.loads(wire.decode(d.port.frames[-1][:-1])[3])['op'],'STOP');d.close()

    def test_reference_refreshes_profile(self):
        from furuta_lab.device import Device
        d=Device();calls=[]
        def rpc(op,**a):
            calls.append(op)
            return {'ok':True,'config':{'pendDownRaw':2985}} if op=='GET_CONFIG' else {'ok':True}
        d.rpc=rpc;d.config={'pendDownRaw':None}
        d.calibrate('down');self.assertEqual(calls,['CALIBRATE','GET_CONFIG']);self.assertEqual(d.config['pendDownRaw'],2985)

    def test_deadzone_start_duty(self):
        seq,w=A.deadzone_sequence(20,5,start_pct=10)
        self.assertEqual([x[2] for x in w],[10,-10,15,-15,20,-20])
        with self.assertRaises(ValueError):A.deadzone_sequence(5,1,start_pct=10)

    # ------------------------------------------------------------ onboard control page (firmware 1.6)
    def test_run_onboard_sends_start_arguments(self):
        from furuta_lab.device import Device
        d=Device();calls=[];d.identity={'onboardControl':True};d.inhibit=False
        d.rpc=lambda op,**a:(calls.append((op,a)),{'ok':True,'mode':'x','seconds':20})[1]
        d.run_onboard(swing_up=False);d.run_onboard(swingUp=True);d.run_onboard()
        self.assertEqual(calls,[('ARM',{}),('START_ONBOARD',{'swingUp':False}),('ARM',{}),('START_ONBOARD',{'swingUp':True}),
                                ('ARM',{}),('START_ONBOARD',{'swingUp':True})])
        d.stop()
        with self.assertRaises(RuntimeError):d.run_onboard()
    def test_onboard_page_knobs_checks_and_summary(self):
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        from furuta_lab import onboard_page as O
        app=QApplication.instance() or QApplication([]);w=LabApp();p=w.onboard
        self.assertEqual(w.nav.tabText(4),'Onboard control');self.assertFalse(p.start_button.isEnabled())
        w.refresh();self.assertFalse(p.start_button.isEnabled())
        with self.assertRaises(RuntimeError):p.start()                     # not connected
        params={'K0':-.18,'K1':-2.155,'aPush':80.,'swingUp':True,'Ts':.004,'modelArmSign':1,'modelPendSign':-1}
        p.params_loaded(params);self.assertEqual(p.edited(),{})
        row=[r for r in range(p.table.rowCount()) if p.table.item(r,0).text()=='aPush'][0]
        p.table.item(row,1).setText('84');self.assertEqual(p.edited(),{'aPush':84.})
        ts=[r for r in range(p.table.rowCount()) if p.table.item(r,0).text()=='Ts'][0]
        self.assertFalse(p.table.item(ts,1).flags()&__import__('PySide6').QtCore.Qt.ItemIsEditable)
        p.table.item(row,1).setText('abc')
        with self.assertRaises(ValueError):p.edited()
        # readiness with a fake connected device whose pendSign differs from the model
        d=w.device;d.ready=True;d.identity={'onboardControl':True,'firmware':'FurutaWorkbench/1.6'}
        d.status={'state':'DISARMED','centered':True,'centerCount':0}
        d.config={'countsPerRev':1496,'armSign':1,'pendSign':1,'pendDownRaw':3008,'maxDuty':.5,'travelCounts':498,
                  'slewPerSecond':10,'currentTripA':0,'maxDuration':20,'heartbeatMs':300}
        items=dict((t.split(' (')[0],ok) for ok,t in p.checks())
        self.assertFalse(items['Profile signs armSign 1, pendSign 1']);self.assertIsNone(items['Slew 10 /s — below the 100 /s it was designed with'])
        self.assertFalse(items['12 V box ticked']);d.ready=False
        # angles as the firmware computes them: raw = down -> hanging (180), raw = down + 2048 -> upright (0)
        a=np.zeros(1,wire.CORE);a['validity']=7;a['count']=187;a['pend_raw']=3008
        th0,th1=O.angles(a[0],{'countsPerRev':1496,'armSign':1,'pendSign':-1,'pendDownRaw':3008},{'centered':True,'centerCount':0})
        self.assertAlmostEqual(th0,187*360/1496);self.assertAlmostEqual(abs(th1),180)
        a['pend_raw']=3008-2048;self.assertAlmostEqual(O.angles(a[0],{'pendSign':-1,'pendDownRaw':3008},{})[1],0)
        # summary of a synthetic run: 1 s swing-up, then stabilize
        n=1000;r=np.zeros(n,wire.CORE);r['device_us']=np.arange(n)*4000;r['validity']=7;r['state']=1
        r['state'][100:900]=3;r['ctrl'][100:350]=2;r['ctrl'][350:900]=3;r['current_mv']=2500;r['filtered_mv']=2500
        txt=O.summary(r,{'countsPerRev':1496,'currentZeroMv':2500,'sensitivityMvA':185,'pendDownRaw':3008,'pendSign':-1},
                      {'status':{'centered':True,'centerCount':0},'final_status':{'state':'DISARMED'}})
        self.assertIn('First STABILIZE (caught) at **1.00 s**',txt);self.assertIn('ran for **3.20 s**',txt)
        # stop reason from a firmware notice
        d.event_count=5;p.event_mark=4;d.last_event={'event':'ONBOARD_SAFE_STOP'};d.status={'state':'DISARMED'}
        p.finished_run();self.assertIn('safeTh0',p.last_end)
        w.close()
    def test_onboard_start_flow_and_burst_download(self):
        """Start = stream 250 Hz, period, capture armed, then record, ARM, START_ONBOARD; afterwards the capture is saved."""
        import tempfile,time as _t
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        app=QApplication.instance() or QApplication([]);w=LabApp();p=w.onboard;d=w.device;ops=[]
        tmp=Path(tempfile.mkdtemp());w.new_path=lambda kind:tmp/kind/'run1'
        knobs={'K0':-.18,'Ts':.004,'modelArmSign':1,'modelPendSign':-1,'swingUp':True}
        def rpc(op,**a):
            ops.append(op)
            if op=='SET_CONFIG':return {'ok':True,'config':a['config']}
            if op=='ONBOARD_GET':return {'ok':True,'params':dict(knobs)}
            if op=='ONBOARD_CONFIG':knobs.update(a['params']);return {'ok':True,'params':dict(knobs)}
            if op=='START_ONBOARD':return {'ok':True,'mode':'LQR only','seconds':20,'Ts':knobs['Ts']}
            if op=='BURST_INFO':return {'ok':True,'active':False,'pending':False,'adcCount':0,'coreCount':0,'adcHz':2000}
            return {'ok':True}
        d.rpc=rpc;d.connected=d.ready=True;d.port=type('P',(),{'write':lambda s,b:None,'close':lambda s:None})()
        d.identity={'onboardControl':True,'onboardPeriods':[.001,.002,.004],'pendHz':1000}
        d.status={'state':'DISARMED','centered':True,'centerCount':0}
        d.config={'countsPerRev':1496,'armSign':1,'pendSign':-1,'pendDownRaw':3008,'maxDuty':.5,'travelCounts':498,
                  'slewPerSecond':100,'currentTripA':1.5,'maxDuration':20,'heartbeatMs':300,'streamHz':100,'adcHz':2000}
        w.power.setChecked(True);p.controller.setCurrentIndex(1);p.period.setCurrentIndex(p.period.findData(.001))   # LQR only, 1 ms
        p.start()
        end=_t.monotonic()+5
        while w.busy and _t.monotonic()<end:app.processEvents();_t.sleep(.01)
        self.assertEqual(ops,['BURST_INFO','SET_CONFIG','ONBOARD_GET','ONBOARD_CONFIG','CAPTURE_BURST','ARM','START_ONBOARD'])
        self.assertTrue((tmp/'runs'/'run1'/'manifest.json').exists())
        meta=json.loads((tmp/'runs'/'run1'/'manifest.json').read_text())['metadata']
        self.assertEqual(meta['controller_period_s'],.001);self.assertEqual(meta['onboard_params']['Ts'],.001)
        self.assertEqual(d.config['streamHz'],250);self.assertTrue(p.active);self.assertGreater(p.burst_armed,20)
        # run ends -> capture downloaded into the run folder
        d.finish_record();p.active=False;p.finished_run();self.assertIsNotNone(p.pending_burst)
        seconds,path=p.pending_burst;p.pending_burst=None;p.download(seconds,path)
        end=_t.monotonic()+5
        while w.busy and _t.monotonic()<end:app.processEvents();_t.sleep(.01)
        self.assertTrue((tmp/'runs'/'run1'/'burst'/'capture.npz').exists())
        d.ready=d.connected=False;d.port=None;w.close()
    def test_capture_busy_handling(self):
        """'Capture/calibration in progress': a stale pending capture is cancelled and the profile change retried;
        a running capture gives a clear message; a failed start cancels the capture it armed."""
        from furuta_lab.device import Device
        d=Device();d.identity={'burstCancel':True,'onboardControl':True};d.status={'state':'DISARMED'};ops=[];state={'pending':True,'active':False,'busy':True}
        def rpc(op,**a):
            ops.append(op)
            if op=='SET_CONFIG':
                if state['busy']:raise RuntimeError('Capture/calibration in progress')
                return {'ok':True,'config':a['config']}
            if op=='BURST_INFO':return {'ok':True,'pending':state['pending'],'active':state['active']}
            if op=='BURST_CANCEL':state.update(busy=False,pending=False);return {'ok':True,'wasPending':True}
            return {'ok':True}
        d.rpc=rpc;d.apply({'x':1})
        self.assertEqual(ops,['SET_CONFIG','BURST_INFO','BURST_CANCEL','SET_CONFIG']);self.assertEqual(d.config,{'x':1})
        state.update(busy=True,pending=False,active=True);ops.clear()
        with self.assertRaises(RuntimeError) as cm:d.apply({'x':2})
        self.assertIn('still recording',str(cm.exception));self.assertNotIn('BURST_CANCEL',ops)
        # failed start: ARM refused after the capture was armed -> capture cancelled
        import tempfile,time as _t
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        QApplication.instance() or QApplication([]);w=LabApp();p=w.onboard;dv=w.device;seen=[]
        tmp=Path(tempfile.mkdtemp());w.new_path=lambda kind:tmp/kind/'run1'
        def rpc2(op,**a):
            seen.append(op)
            if op=='ONBOARD_GET':return {'ok':True,'params':{'Ts':.004}}
            if op=='BURST_INFO':return {'ok':True,'active':False,'pending':False}
            if op=='ARM':raise RuntimeError('Arm center required')
            return {'ok':True,'config':a.get('config',{})}
        dv.rpc=rpc2;dv.connected=dv.ready=True;dv.port=type('P',(),{'write':lambda s,b:None,'close':lambda s:None})()
        dv.identity={'onboardControl':True,'onboardPeriods':[.004],'burstCancel':True}
        dv.status={'state':'DISARMED','centered':True};dv.config={'maxDuty':.5,'travelCounts':498,'maxDuration':20,'streamHz':250,'adcHz':2000}
        w.power.setChecked(True);p.start();end=_t.monotonic()+5
        while w.busy and _t.monotonic()<end:QApplication.processEvents();_t.sleep(.01)
        self.assertIn('CAPTURE_BURST',seen);self.assertEqual(seen[-1],'BURST_CANCEL')
        self.assertTrue(dv.inhibit)                                           # motion disabled again
        # a live recording blocks the start without touching it
        dv.record=object();seen.clear()
        with self.assertRaises(RuntimeError):p.start()
        self.assertEqual(seen,[]);dv.record=None
        dv.ready=dv.connected=False;dv.port=None;w.close()
    def test_breakaway_model_and_unlimited_run(self):
        """Stick-slip arm friction: 11 % duty from rest holds the arm (as on the rig), 20 % breaks it free;
        without tsp/tsn the model is unchanged. Run length 0 = no limit in the onboard page."""
        from furuta_lab import estimation as E
        P={'Rm':3.4254,'Kb':.48137,'Ja':5.6343e-3,'Jp':1.8531e-4,'h':2.0123e-4,'G':2.8987e-2,'Beq':1.6122e-2,'b1':1.2082e-4,
           'tcp':.25814,'tcn':.25689,'tp':6.379e-4}
        S=P|E.breakaway_from_deadzone(.13,.14,P['Rm']);x=[0.,np.pi,0.,0.]
        self.assertAlmostEqual(S['tsp'],.13*12/P['Rm'])
        self.assertGreater(E.rhs(x,.111,P,12.)[0][2],10)          # smoothed Coulomb only: the arm accelerates
        self.assertLess(abs(E.rhs(x,.111,S,12.)[0][2]),1e-9)       # with breakaway: it stays put
        self.assertGreater(E.rhs(x,.20,S,12.)[0][2],10)            # above the breakaway it slips
        self.assertLess(abs(E.rhs(x,-.13,S,12.)[0][2]),1e-9)       # -13 % is inside the 14 % negative dead zone
        th=np.vstack([E._vec(P),E._vec(S)]);X,_=E.sim_batch(th,np.full((1,200),.111),np.array([x]),.001,12.)
        self.assertGreater(abs(X[0,0,-1,0]),1e-3);self.assertLess(abs(X[1,0,-1,0]),1e-9)   # batch = single-state physics
        self.assertEqual(len(E._vec(P)),13);self.assertEqual(E.full(P)['tsp'],0.)
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        from furuta_lab import onboard_page as O
        app=QApplication.instance() or QApplication([]);w=LabApp();p=w.onboard
        self.assertEqual(p.period.currentData(),.004)                # 4 ms is the default
        self.assertEqual(O.run_length({'maxDuration':0}),'no limit');self.assertEqual(O.run_length({'maxDuration':20}),'20 s')
        d=w.device;sent=[];d.ready=d.connected=True;d.identity={'onboardControl':True,'runLengthUnlimited':True}
        d.status={'state':'DISARMED'};d.config={'maxDuration':20}
        d.apply=lambda c:sent.append(dict(c));w.submit=lambda fn,done=None,label='':(fn(),done and done(None))
        p.no_limit.setChecked(True);self.assertFalse(p.rec_seconds.isEnabled());p.apply_limits()
        self.assertEqual(sent[-1]['maxDuration'],0.)
        d.identity={'onboardControl':True}
        with self.assertRaises(RuntimeError):p.apply_limits()        # older firmware refuses 0
        d.ready=d.connected=False;w.close()

class TwinTests(unittest.TestCase):
    def test_meshes_and_kinematics(self):
        from furuta_lab import twin_core as T
        for b in T.BODIES:
            pos,nrm=T.load_ripmesh(T.ASSETS/f'{b}.ripmesh');v=pos.reshape(-1,3,3).astype(float)
            self.assertEqual(pos.shape,nrm.shape);self.assertGreater(len(pos),300)
            self.assertGreater(np.einsum('ij,ij',v[:,0],np.cross(v[:,1],v[:,2]))/6,0)   # positive enclosed volume: outward winding/normals
        pivot,axis=T.load_geometry();pend,_=T.load_ripmesh(T.ASSETS/'pendulum.ripmesh')
        tip=pend[np.argmin(pend[:,2])]                                      # lowest point when hanging
        self.assertLess(tip[2],pivot[2])
        up=T.pose_pendulum([tip],0.,np.pi,pivot,axis)[0];self.assertGreater(up[2],pivot[2])   # q_pend = pi: upright
        self.assertAlmostEqual(np.linalg.norm(up-pivot),np.linalg.norm(tip-pivot),6)
        r=T.pose_pendulum([tip],np.pi/2,0.,pivot,axis)[0]                   # arm +90 deg about +Z
        self.assertAlmostEqual(r[0],-tip[1],6);self.assertAlmostEqual(r[1],tip[0],6)
    def test_model_ghost_and_udp(self):
        from furuta_lab import twin_core as T
        g=T.ModelGhost(resync_s=0)
        for k in range(300):x=g.feed(k*.004,0.,0.,0.)                       # hanging, no drive: stays at rest
        self.assertLess(max(abs(v) for v in x),1e-6)
        g=T.ModelGhost(resync_s=.5)
        for k in range(50):x=g.feed(k*.004,0.,0.,np.pi-.05)                 # held near upright: the model falls
        self.assertGreater(abs(x[1]-(np.pi-.05)),1e-3)
        x=g.feed(.6,0.,0.,np.pi-.05);self.assertAlmostEqual(x[1],np.pi-.05,6)   # re-sync puts it back
        self.assertEqual(T.unpack(T.pack(1,.5,-2.)),(1,.5,-2.));self.assertIsNone(T.unpack(b'XXXX'+bytes(12)))
        rx,tx=T.UdpLink(),T.UdpLink();rx.listen(0);port=rx.rx.getsockname()[1];tx.forward('127.0.0.1',port)
        tx.send(0,.25,3.)
        import time;end=time.monotonic()+1
        while 0 not in rx.poll() and time.monotonic()<end:time.sleep(.01)
        self.assertEqual(rx.poll()[0],(.25,3.));rx.close();tx.close()
    def test_twin_page_live_and_playback(self):
        from PySide6.QtWidgets import QApplication
        from furuta_lab.app import LabApp
        from furuta_lab import wire
        app=QApplication.instance() or QApplication([]);w=LabApp();w.show();p=w.twin
        w.nav.setCurrentIndex(5);self.assertEqual(w.right_stack.currentIndex(),3)
        if p.view is None:w.close();self.skipTest(f'3D view unavailable: {p.error}')
        self.assertTrue(p.timer.isActive())
        w.nav.setCurrentIndex(0);self.assertFalse(p.timer.isActive())           # no frames while hidden
        p.on_onboard.setChecked(True);w.nav.setCurrentIndex(4);self.assertEqual(w.right_stack.currentIndex(),3)
        rows=np.zeros(200,dtype=wire.CORE);rows['device_us']=np.arange(200)*4000
        rows['count']=1000+np.arange(200);rows['pend_raw']=2048+1024;rows['validity']=3     # pendulum at +90 deg from down
        cfg={'countsPerRev':1496,'pendDownRaw':2048,'armSign':1,'pendSign':1};st={'centered':True,'centerCount':1000}
        rig,ghost,_=p._feed(rows,range(0,50),cfg,st)
        self.assertAlmostEqual(rig[0],np.radians(49*360/1496),6);self.assertAlmostEqual(rig[1],np.pi/2,3)        # 90 deg from hanging
        self.assertIsNotNone(ghost)
        w.run_data,w.run_manifest,w.run_path=rows,{'config':cfg,'status':st},Path('fake_run')
        p.src_rec.setChecked(True);self.assertEqual(len(p.frames[0]),200)
        p.play['t']=.5;p.play['wall']=__import__('time').perf_counter();p.tick()
        self.assertAlmostEqual(p.view.pose['rig'][0],np.radians(125*360/1496),2)  # record at 0.5 s
        q=p.view.nodes['armJoint'].property('rotation')                     # drawn reversed (DISPLAY_SIGN, seen at the rig)
        self.assertLess(q.z()*q.scalar(),0)
        p.on_onboard.setChecked(False);w.close()

from test_estimation import *  # noqa: E402,F401,F403

if __name__=='__main__':unittest.main(verbosity=2)
