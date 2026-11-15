"""Explicit USB-only real-GUI test. Never calibrates, arms or starts motion."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy import signal
from PySide6 import QtCore,QtWidgets
from furuta_lab.app import LabApp
from furuta_lab.backend import jsonable,ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',default='COM11');p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--backend',action='store_true');args=p.parse_args()
    out=ROOT/'test-results'/'python-hardware';out.mkdir(parents=True,exist_ok=True)
    app=QtWidgets.QApplication([]);w=LabApp();w.show();app.processEvents()
    errors=[];w.signals.failed.connect(errors.append);w.port.setCurrentText(args.port)
    result={};state={'stage':'connect','started':time.monotonic(),'last':time.monotonic(),'phase_at':time.monotonic()}
    delays=[];noise_results={};backend_result={};exit_code=[0];view_results={}
    w.connect_device()
    def finish(error=None):
        if error:errors.append(str(error));exit_code[0]=1
        if w.device.connected:
            a=w.device.snapshot(60000);result.update(samples=len(a),max_applied=float(np.max(np.abs(a['applied']))) if len(a) else None,
                firmware=w.device.identity.get('firmware'),serial_max_loop_gap_s=w.device.max_io_gap,
                parser_errors_since_ready=w.device.parser.errors-state.get('parser_start',0),state=w.device.status,
                max_gui_refresh_s=w.max_refresh,gui_probe_p99_s=float(np.percentile(delays,99)) if delays else None,
                gui_probe_max_s=max(delays,default=0),elapsed_s=time.monotonic()-state['started'],
                stream_rate_hz=float((len(a)-1)/((int(a['device_us'][-1])-int(a['device_us'][0]))*1e-6)) if len(a)>1 else None,
                configured_stream_hz=w.device.config.get('streamHz'),dropped=w.device.meta.get('dropped'),
                adc_overruns_delta=w.device.status.get('adcOverruns',0)-state.get('adc0',0),
                control_overruns_delta=w.device.status.get('controlOverruns',0)-state.get('ctl0',0),
                adc_oversample=w.device.status.get('adcOversample'),bad_frames=w.device.parser.bad_frames[:5],
                discarded_on_connect=w.device.parser.discarded,views=view_results)
        result.update(errors=errors,noise=noise_results,synthetic_matlab_backend=backend_result)
        result['adc_rate_restored']=w.device.config.get('adcHz')==state.get('original_adc_rate')
        (out/'result.json').write_text(json.dumps(jsonable(result),indent=2),encoding='utf-8')
        compact={k:v for k,v in result.items() if k!='synthetic_matlab_backend'}
        compact['synthetic_matlab_backend']={k:v for k,v in backend_result.items() if k not in ('time','measured','simulated')}
        print(json.dumps(jsonable(compact),indent=2),flush=True)
        timer.stop();w.close();app.quit()
    def check_views():
        """Live dashboard: every signal visible in both views, readouts populated, raw-units toggle."""
        w.set_live();w.check_signals(True)
        for mode,button in (('separate',w.mode_separate),('combined',w.mode_combined)):
            button.setChecked(True);w.refresh();app.processEvents()
            empty=[k for k,r in w.readouts.items() if '—' in r.text()]
            view_results[mode]={'plots':len(w.viewer.plots),'curves':len(w.viewer.curves),'empty_readouts':empty,
                                'readouts':{k:r.text().split('</b>')[0].replace('<b>','') for k,r in w.readouts.items()}}
            if empty or len(w.viewer.curves)!=len(w.signal_checks):raise AssertionError(f'{mode} view incomplete: {empty}')
            w.grab().save(str(out/f'live_{mode}.png'))
        w.raw_units.setChecked(True);w.refresh();view_results['raw_current']=w.readouts['current'].text().split('</b>')[0]
        if 'mV' not in view_results['raw_current']:raise AssertionError('Raw units toggle did not switch current to mV')
        w.raw_units.setChecked(False);w.mode_separate.setChecked(True);w.check_signals(None);w.refresh()
        view_results['header']=w.sensors.text();view_results['pill']=w.pill.text()
    def tick():
        try:
            now=time.monotonic();delays.append(now-state['last']);state['last']=now
            if errors:raise RuntimeError(errors[-1])
            if now-state['started']>210:raise TimeoutError('Smoke test timed out')
            if state['stage']=='connect':
                if not w.busy and w.device.ready:
                    state.update(stage='settle',phase_at=now,original_adc_rate=w.device.config['adcHz'],
                                 adc0=w.device.status.get('adcOverruns',0),ctl0=w.device.status.get('controlOverruns',0))
            elif state['stage']=='settle' and now-state['phase_at']>2:
                state['parser_start']=w.device.parser.errors
                w.nav.setCurrentIndex(1);w.still.setChecked(True);w.capture_seconds.setValue(2);w.capture_still();state['stage']='capture'
            elif state['stage']=='capture' and not w.busy and w.capture_data is not None:
                result['capture_adc']=len(w.capture_data['adc']);result['capture_core']=len(w.capture_data['core'])
                w.noise_sensor.setCurrentIndex(0);w.analyze_noise();state.update(stage='noise',sensor=0)
            elif state['stage']=='noise' and not w.busy:
                r=w.analysis_result;k=state['sensor'];noise_results[r['sensor']]={n:v for n,v in r.items() if n not in ('frequency','psd','time','values')}
                if k<2:
                    state['sensor']=k+1;w.noise_sensor.setCurrentIndex(k+1);w.analyze_noise()
                else:
                    w.noise_sensor.setCurrentIndex(0);w.analyze_noise();state['stage']='filters'
            elif state['stage']=='filters' and not w.busy:
                w.bandwidth.setValue(5);w.delay.setValue(50);w.compare_filters();w.preview_filter()
                result['filter_candidates']=len(w.filter_rows);result['first_filter_acceptable']=w.filter_rows[0]['acceptable']
                w.grab().save(str(out/'noise.png'));w.set_live();state['stage']='backend'
                if args.backend:
                    t=np.arange(3000)*.01;u=np.random.default_rng(4).choice([-.1,.1],3000);y=signal.lfilter([0,.1],[1,-.9],u)
                    w.submit(lambda:w.backend.speed(t,u,y,'PT1'),lambda r:backend_result.update(r))
            elif state['stage']=='backend' and not w.busy and now-state['started']>=args.seconds:
                if not w.device.connected:raise RuntimeError('Connection lost')
                if args.backend and not backend_result.get('ok'):raise RuntimeError('MATLAB backend did not complete')
                a=w.device.snapshot(60000)
                if not len(a) or np.any(a['applied']!=0):raise AssertionError('No samples or nonzero output')
                check_views();w.grab().save(str(out/'live.png'));finish()
        except Exception as e:finish(e)
    timer=QtCore.QTimer();timer.timeout.connect(tick);timer.start(20)
    app.exec();return exit_code[0]

if __name__=='__main__':sys.exit(main())
