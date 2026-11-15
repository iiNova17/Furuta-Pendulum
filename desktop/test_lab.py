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

from test_estimation import *  # noqa: E402,F401,F403

if __name__=='__main__':unittest.main(verbosity=2)
