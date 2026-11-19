"""UART and MCU lease run independently of Qt and MATLAB computations."""
import collections
import copy
import json
import secrets
import threading
import time
from pathlib import Path
import numpy as np
import serial
from . import wire

class Device:
    def __init__(self):
        self.lock=threading.RLock();self.write_lock=threading.Lock();self.port=None
        self.parser=wire.Parser();self.session=secrets.randbits(63)+1;self.sequence=0
        self.pending={};self.blocks=collections.deque();self.rows=0;self.max_rows=60000
        self.status={'state':'DISCONNECTED'};self.config={};self.identity={}
        self.events=collections.deque(maxlen=250);self.connected=False;self.ready=False
        self.closed=threading.Event();self.inhibit=True;self.generation=0
        self.last_rx=0.;self.max_io_gap=0.;self.record=None;self.record_count=0
        self.progress='';self.meta={};self.boot=None;self.thread=None
        self.last_event=None;self.event_count=0

    def log(self,message):
        with self.lock:self.events.append((time.time(),str(message)))

    def connect(self,port):
        self.close();self.closed.clear();self.parser=wire.Parser();self.sequence=0
        self.session=secrets.randbits(63)+1;self.boot=None;self.blocks.clear();self.rows=0
        self.config={};self.identity={};self.inhibit=True;self.max_io_gap=0
        p=serial.Serial(baudrate=460800,timeout=.01,write_timeout=.3)
        p.dtr=False;p.rts=False;p.port=port;p.open();p.reset_input_buffer();self.port=p
        self.connected=True;self.status={'state':'CONNECTING'}
        self.thread=threading.Thread(target=self._loop,name='Furuta UART',daemon=True);self.thread.start()
        try:
            h=self.rpc('HELLO');self.identity=h
            if h.get('protocol')!=1 or h.get('coreRecordBytes')!=88:raise RuntimeError('Incompatible firmware')
            self.rpc('SESSION_OPEN',bootId=h['status']['bootId'],challenge=h['challenge'])
            self.boot=int(h['status']['bootId']);self.config=self.rpc('GET_CONFIG')['config']
            self.ready=True;self.rpc('START_STREAM');self.log('Connected; driver disabled')
        except Exception:
            self.close();raise

    def _send(self,op,args=None,waiter=False):
        with self.write_lock:
            if not self.connected:raise RuntimeError('Device disconnected')
            if op in ('ARM','START','START_ONBOARD') and self.inhibit:raise RuntimeError('Run canceled / motion disabled')
            self.sequence+=1
            if self.sequence>0xffffffff:raise RuntimeError('Reconnect to renew command sequence')
            seq=self.sequence;b=wire.command(self.session,seq,op,**(args or {}))
            event=threading.Event();item={'op':op,'event':event,'reply':None,'at':time.monotonic(),'waiter':waiter}
            with self.lock:self.pending[seq]=item
            try:self.port.write(b)
            except Exception:
                with self.lock:self.pending.pop(seq,None)
                raise
            return seq,b,item

    def rpc(self,op,**args):
        seq,b,item=self._send(op,args,True)
        try:
            # One retry of the same ID is idempotent. Keep the interval below
            # the firmware's recent-command cache eviction horizon.
            for attempt in range(2):
                if item['event'].wait(.6):break
                if attempt==0:
                    with self.write_lock:self.port.write(b)
            else:raise TimeoutError(f'{op}: no response')
            r=item['reply']
            if not r or not r.get('ok',False):raise RuntimeError((r or {}).get('error',f'{op} failed'))
            return r
        finally:
            with self.lock:self.pending.pop(seq,None)

    def _loop(self):
        beat=status_at=last=time.monotonic()
        try:
            while not self.closed.is_set():
                now=time.monotonic();self.max_io_gap=max(self.max_io_gap,now-last);last=now
                if self.ready and self.status.get('state') in ('ARMED_IDLE','RUNNING','STOPPING') and not self.inhibit and now-beat>=.08:
                    self._send('HEARTBEAT');beat=now
                if self.ready and now-status_at>=.5:self._send('STATUS');status_at=now
                b=self.port.read(max(1,min(self.port.in_waiting,32768)))
                if b:
                    self.last_rx=now
                    for kind,session,seq,payload in self.parser.feed(b):
                        if kind==4 and self.ready and session==self.session:
                            rows,meta=wire.core_batch(payload)
                            if meta['boot']!=self.boot:raise RuntimeError('MCU restarted; reconnect and re-center')
                            # The MCU bumps configId whenever its profile changes
                            # (SET_CONFIG, CALIBRATE down/upright/arm, current zero),
                            # including changes made by another client. Re-read it.
                            if meta['config']!=self.meta.get('config') and self.meta:self._send('GET_CONFIG')
                            self.meta=meta
                            with self.lock:
                                self.blocks.append(rows);self.rows+=len(rows)
                                while self.rows>self.max_rows and self.blocks:self.rows-=len(self.blocks.popleft())
                                if self.record:
                                    self.record.write(len(payload).to_bytes(4,'little')+payload);self.record_count+=len(rows)
                        elif kind in (2,3,5):
                            refresh_config=False
                            if kind==5:seq,r=wire.burst_chunk(payload)
                            else:r=json.loads(payload);seq=r.get('id',seq)
                            with self.lock:
                                item=self.pending.get(seq) if kind!=3 else None
                                if session!=self.session and (not item or item['op'] not in ('HELLO','SESSION_OPEN')):continue
                                if 'status' in r:self.status=r['status']
                                if kind==3:
                                    self.log(r.get('event',r));self.last_event=r;self.event_count+=1
                                    refresh_config=r.get('event')=='CURRENT_ZERO_UPDATED'
                                elif item:
                                    if 'config' in r:self.config=r['config']
                                    item['reply']=r;item['event'].set()
                                    if not item['waiter']:self.pending.pop(seq,None)
                            if refresh_config:self._send('GET_CONFIG')
                with self.lock:
                    for key,item in list(self.pending.items()):
                        if not item['waiter'] and now-item['at']>2:self.pending.pop(key,None)
        except Exception as e:
            if not self.closed.is_set():
                self.log(f'Connection error: {e}');self.stop();self.status={'state':'CONNECTION_ERROR','fault':str(e)}
            self.connected=False;self.ready=False
            with self.lock:
                for item in self.pending.values():item['reply']={'ok':False,'error':'Connection lost'};item['event'].set()

    def snapshot(self,n=3000):
        with self.lock:
            parts=[];count=0
            for b in reversed(self.blocks):
                parts.append(b);count+=len(b)
                if count>=n:break
            return np.concatenate(parts[::-1])[-n:] if parts else np.empty(0,wire.CORE)

    def stop(self):
        self.inhibit=True;self.generation+=1
        if self.connected:
            try:self._send('STOP')
            except Exception:pass

    def calibrate(self,target):
        """Store a reference on the MCU and refresh the local profile copy."""
        r=self.rpc('CALIBRATE',target=target)
        if target!='current':self.config=self.rpc('GET_CONFIG')['config']
        return r

    def apply(self,config):
        if self.status.get('state')!='DISARMED':raise RuntimeError('Disarm before changing settings')
        if self.record:raise RuntimeError('Finish recording before changing its calibration/profile')
        self.config=self.rpc('SET_CONFIG',config=config)['config']

    def run(self,segments,metadata):
        if self.inhibit:raise RuntimeError('Powered motion is not enabled in the interface')
        generation=self.generation
        def check():
            if self.inhibit or generation!=self.generation:raise RuntimeError('Run canceled by STOP')
        check();self.rpc('UPLOAD_BEGIN')
        for i,s in enumerate(segments):check();self.rpc('SET_SEGMENT',index=i,segment=s)
        check();self.rpc('VALIDATE');check();self.rpc('ARM');check();self.rpc('START')
        self.log('Finite experiment started')

    def onboard_params(self,params=None,defaults=False):
        """Read or change the onboard controller's knobs (firmware 1.6, product mode).

        params: dict of knob -> value (see ONBOARD_CONFIG in firmware/src/main.cpp), merged over the
        current values on the MCU; defaults=True restores the values generated from ctrl_hybrid_design.m.
        """
        if not self.identity.get('onboardControl'):raise RuntimeError('Firmware without onboard control (flash 1.6)')
        if defaults:return self.rpc('ONBOARD_DEFAULTS')['params']
        if params:
            if self.status.get('state')!='DISARMED':raise RuntimeError('Disarm before changing controller knobs')
            return self.rpc('ONBOARD_CONFIG',params=params)['params']
        return self.rpc('ONBOARD_GET')['params']

    def run_onboard(self,swing_up=True):
        """Product mode: the ESP32 runs swing-up + LQR (or LQR only) until maxDuration, STOP or a trip.

        The laptop only keeps the heartbeat going (_loop) and receives telemetry; the 'ctrl' field of
        every core row shows the controller state (wire.CTRL_NAMES).
        """
        if self.inhibit:raise RuntimeError('Powered motion is not enabled in the interface')
        if not self.identity.get('onboardControl'):raise RuntimeError('Firmware without onboard control (flash 1.6)')
        generation=self.generation
        def check():
            if self.inhibit or generation!=self.generation:raise RuntimeError('Run canceled by STOP')
        check();self.rpc('ARM');check();r=self.rpc('START_ONBOARD',swingUp=bool(swing_up))
        self.log(f"Onboard controller started: {r.get('mode')}, up to {r.get('seconds')} s"+(f" ({r['warning']})" if r.get('warning') else ''))
        return r

    def capture(self,seconds,progress=None,cancel=None,adc_rate=None):
        original=copy.deepcopy(self.config);changed=False
        if adc_rate is not None and adc_rate!=original['adcHz']:
            c=copy.deepcopy(original);c['adcHz']=int(adc_rate);c['filter']['type']='bypass'
            self.apply(c);changed=True
        try:return self._capture(seconds,progress,cancel)
        finally:
            if changed and self.connected:
                # STOP disables drive but a finite passive capture may finish
                # naturally. Restore only after the MCU releases its buffer.
                until=time.monotonic()+seconds+2
                while self.connected and time.monotonic()<until:
                    info=self.rpc('BURST_INFO')
                    if not info['active'] and not info.get('pending'):break
                    time.sleep(.05)
                self.apply(original)

    def _capture(self,seconds,progress=None,cancel=None):
        if self.status.get('state')!='DISARMED':raise RuntimeError('Still capture requires DISARMED')
        self.rpc('CAPTURE_BURST',seconds=seconds,withRun=False)
        deadline=time.monotonic()+seconds+5
        while True:
            if cancel and cancel.is_set():raise RuntimeError('Capture download canceled')
            info=self.rpc('BURST_INFO')
            if not info['active']:break
            if progress:progress(f'Recording still data: {max(0,deadline-time.monotonic()-5):.1f} s remaining')
            if time.monotonic()>deadline:raise TimeoutError('Capture did not finish')
            time.sleep(.1)
        if info.get('overflow'):raise RuntimeError('Capture overflow; reduce duration')
        result={};total=info['adcCount']+info['coreCount'];done=0
        # Stop the display stream for bulk download, not acquisition.
        self.rpc('STOP_STREAM')
        try:
            for channel,dtype,count in [('adc',wire.ADC,info['adcCount']),('core',wire.CORE,info['coreCount'])]:
                parts=[];offset=0
                chunk=int(self.identity.get('adcChunkMax',16)) if channel=='adc' else 16
                while offset<count:
                    if cancel and cancel.is_set():raise RuntimeError('Download canceled')
                    r=self.rpc('READ_CHUNK',channel=channel,offset=offset,count=chunk)
                    if r['offset']!=offset or not len(r['data']):raise RuntimeError('Bad capture chunk order')
                    parts.append(r['data']);offset+=len(r['data']);done+=len(r['data'])
                    if progress:progress(f'Downloading {100*done/max(total,1):.0f}%')
                result[channel]=np.concatenate(parts) if parts else np.empty(0,dtype)
        finally:
            if self.connected:self.rpc('START_STREAM')
        result['config']=copy.deepcopy(self.config);result['status']=copy.deepcopy(self.status);result['info']=info
        if len(result['core']) and np.any(result['core']['applied']!=0):raise RuntimeError('Still recording contained motor drive')
        return result

    def start_record(self,path,metadata):
        path=Path(path);path.mkdir(parents=True,exist_ok=False)
        with self.lock:
            if self.record:raise RuntimeError('Already recording')
            manifest={'schema':2,'source':'hardware','created':time.time(),'config':copy.deepcopy(self.config),
                      'status':copy.deepcopy(self.status),'metadata':metadata,'complete':False}
            (path/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
            self.record=(path/'samples.bin').open('wb');self.record_path=path;self.record_count=0

    def finish_record(self):
        with self.lock:
            if not self.record:return None
            self.record.close();self.record=None;p=self.record_path
            m=json.loads((p/'manifest.json').read_text());m.update(complete=True,samples=self.record_count)
            m.update(final_status=copy.deepcopy(self.status),link_errors=self.parser.errors)
            (p/'manifest.json').write_text(json.dumps(m,indent=2),encoding='utf-8');return p

    def close(self):
        # The stream is deliberately left running: with the FT232R bridge an
        # idle line after close made the next open return replayed stale bytes
        # (measured 2026-11-07). The parser resynchronizes on the next open.
        self.stop();self.closed.set()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=1.5)
        if self.port:
            try:self.port.close()
            except Exception:pass
        self.finish_record();self.connected=False;self.ready=False;self.port=None
        self.status={'state':'DISCONNECTED'}
