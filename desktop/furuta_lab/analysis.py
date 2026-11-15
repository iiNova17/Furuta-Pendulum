"""Offline characterization, with explicit bandwidth and identifiability limits."""
import numpy as np
from scipy import signal

def channel(capture,name):
    a=capture['adc'] if name=='current_mv' else capture['core']
    if len(a)<8:raise ValueError('Not enough samples')
    t=a['device_us'].astype(float)*1e-6;good=np.ones(len(a),bool)
    if name=='current_mv':
        x=a['current_mv'].astype(float);good&=(a['adc_raw']>16)&(a['adc_raw']<4079);unit='mV'
    elif name=='arm_count':
        x=(a['count']-a['count'][0]).astype(float);good&=(a['validity']&1)!=0;unit='counts'
    elif name=='pend_deg':
        x=np.unwrap(a['pend_raw'].astype(float)*2*np.pi/4096)*180/np.pi
        t=a['pend_us'].astype(float)*1e-6
        good&=(a['validity']&2)!=0;good&=np.r_[True,np.diff(t)>0];unit='degrees'
    else:raise ValueError('Unknown sensor')
    good&=np.isfinite(x)&np.isfinite(t);x[~good]=np.nan
    return t-t[0],x,unit

def spans(t,x):
    t=np.asarray(t);x=np.asarray(x);dt=np.diff(t)
    positive=dt[np.isfinite(dt)&(dt>0)]
    if not len(positive):raise ValueError('No increasing timestamps')
    step=float(np.median(positive));good=np.flatnonzero(np.isfinite(t)&np.isfinite(x))
    if not len(good):raise ValueError('No valid samples')
    cuts=np.flatnonzero((np.diff(good)>1)|(np.diff(t[good])>1.5*step)|(np.diff(t[good])<=0))+1
    return np.split(good,cuts),step

def noise(capture,name):
    t,x,unit=channel(capture,name);parts,dt=spans(t,x)
    usable=[p for p in parts if len(p)>=32]
    if not usable:raise ValueError('No valid contiguous span of 32 samples. Inspect wiring/timing.')
    nperseg=min(1024,max(map(len,usable)));nperseg=2**int(np.floor(np.log2(nperseg)))
    total=0;psd=None;used=0;resampled=0;max_jitter=0
    for p in usable:
        if len(p)<nperseg:continue
        tp=t[p];xp=x[p];d=np.diff(tp);max_jitter=max(max_jitter,float(np.std(d)/dt*100))
        # Each block is sampled onto the common measured-rate grid. No grid
        # crosses a missing sample, invalid flag or repeated timestamp.
        grid=np.arange(tp[0],tp[-1]+dt*.01,dt);xp=np.interp(grid,tp,xp)
        if len(xp)<nperseg:continue
        resampled+=1
        starts=np.arange(0,len(xp)-nperseg+1,max(1,nperseg//2))
        if starts[-1]!=len(xp)-nperseg:starts=np.r_[starts,len(xp)-nperseg]
        windows=xp[starts[:,None]+np.arange(nperseg)]
        f,periodograms=signal.periodogram(windows,fs=1/dt,window='hann',detrend='constant',axis=-1)
        power=np.mean(periodograms,axis=0);weight=len(starts)
        psd=power*weight if psd is None else psd+power*weight
        total+=weight;used+=len(xp)
    if not total:raise ValueError('No complete Welch windows')
    psd/=total;finite=x[np.isfinite(x)];duration=float(t[-1]-t[0])
    return {'sensor':name,'unit':unit,'time':t,'values':x,'frequency':f,'psd':psd,'fs':1/dt,
            'sd':float(np.std(finite,ddof=1)),'mean':float(np.mean(finite)),
            'peak_to_peak':float(np.ptp(finite)),'duration':duration,'samples':len(x),
            'used_samples':used,'blocks':len(usable),'gaps':int(np.sum(np.diff(t)>1.5*dt)),
            'invalid':int(np.sum(~np.isfinite(x))),'windows':total,'resolution':float(f[1]-f[0]),
            'jitter_percent':max_jitter,'resampled_blocks':resampled,
            'drift':float(np.mean(finite[-max(1,len(finite)//5):])-np.mean(finite[:max(1,len(finite)//5)])),
            'rms_psd':float(np.sqrt(np.trapezoid(psd,f))),
            'note':'Welch averages valid contiguous blocks; interpolation is within each block only. No missing samples are bridged.'}

def filters(summary,bandwidth,max_delay_ms,cutoffs=None):
    if not 0<bandwidth<summary['fs']/2:raise ValueError('Enter required motion bandwidth below Nyquist; still noise cannot determine it')
    if max_delay_ms<=0:raise ValueError('Delay budget must be positive')
    if summary['sd']<=1e-12:raise ValueError('No measurable stationary variation. This capture cannot rank noise filters; inspect quantization during a moving test.')
    if bandwidth<2*summary['resolution']:
        raise ValueError(f'PSD spacing {summary["resolution"]:.2g} Hz is too coarse for a {bandwidth:g} Hz preservation band. Record longer or choose 500 Hz ADC capture to reduce timing gaps; do not infer a low cutoff from unresolved bins.')
    if cutoffs is None:cutoffs=[1,2,3,5,8,10,15,20,30,50,80,100,150,200,300]
    rows=[];f=summary['frequency'];fs=summary['fs']
    for fc in cutoffs:
        if not bandwidth<fc<fs*.45:continue
        sos=signal.butter(2,fc,fs=fs,output='sos');_,h=signal.sosfreqz(sos,worN=f,fs=fs)
        test=np.linspace(0,bandwidth,256);_,hb=signal.sosfreqz(sos,worN=test,fs=fs)
        gain_db=20*np.log10(max(abs(hb[-1]),1e-12))
        delay=float(np.max(-np.gradient(np.unwrap(np.angle(hb)),2*np.pi*test))*1000)
        rms=float(np.sqrt(np.trapezoid(summary['psd']*abs(h)**2,f)))
        rows.append({'cutoff_hz':float(fc),'predicted_noise_rms':rms,'passband_loss_db':-float(gain_db),
                     'delay_ms':delay,'acceptable':bool(gain_db>=-1 and delay<=max_delay_ms),
                     'bandwidth_hz':bandwidth,'delay_budget_ms':max_delay_ms,
                     'sos':sos,'filtered_psd':summary['psd']*abs(h)**2})
    return sorted(rows,key=lambda r:(not r['acceptable'],r['predicted_noise_rms']))

def smooth_derivatives(t,x,seconds=.05):
    dt=float(np.median(np.diff(t)));n=max(5,int(seconds/dt)//2*2+1)
    n=min(n,len(x)//2*2-1)
    if n<5:raise ValueError('Recording too short')
    values=signal.savgol_filter(np.asarray(x,float),n,3,axis=0)
    speed=signal.savgol_filter(np.asarray(x,float),n,3,deriv=1,delta=dt,axis=0)
    accel=signal.savgol_filter(np.asarray(x,float),n,3,deriv=2,delta=dt,axis=0)
    return values,speed,accel,n//2

def prepared_core(data):
    t=(data['device_us']-data['device_us'][0]).astype(float)*1e-6
    good=(data['validity']&5)==5
    parts,_=spans(t,np.where(good,0.,np.nan));ix=max(parts,key=len)
    if len(ix)<150:raise ValueError('Need a longer contiguous valid recording')
    # Fit at measured timestamps, after interpolation within this single span.
    a=data[ix];t=t[ix];dt=float(np.median(np.diff(t)));grid=np.arange(t[0],t[-1],dt)
    return a,t,grid

def segment(kind='hold',duration=.3,amplitude=0,f0=1,phase=None,duty=None,f1=None):
    s={'kind':kind,'duration':float(duration),'amplitude':float(amplitude),'offset':0.,'f0':float(f0),'f1':float(f0*3 if f1 is None else f1)}
    if phase is not None:s['phase']=float(phase)
    if duty is not None:s['duty']=float(duty)
    return s

def deadzone_sequence(maximum_pct,step_pct,on=.25,rest=.5,max_segments=256,start_pct=None):
    """Alternating +/- pulses from start_pct to maximum_pct in step_pct increments.

    max_segments comes from the connected firmware (HELLO maxSegments)."""
    start_pct=step_pct if start_pct is None else start_pct
    if not (0<step_pct and 0<start_pct<=maximum_pct<=100) or on<=0 or rest<=0:
        raise ValueError('Need 0 < start duty <= maximum duty <= 100 % and a positive increment')
    levels=np.arange(start_pct,maximum_pct+1e-8,step_pct)
    seq=[segment(duration=.5)];windows=[];at=.5
    for value in levels:
        # Opposite signs alternate at each level; each starts after a rest.
        for sign in [1,-1]:
            seq.append(segment(duration=on,amplitude=sign*value/100))
            windows.append((at,at+on,sign*value));at+=on
            seq.append(segment(duration=rest));at+=rest
    if len(seq)>max_segments:raise ValueError(f'{len(levels)} levels need {len(seq)} segments; the connected firmware accepts {max_segments}. Update the firmware or use a larger increment.')
    return seq,windows

def deadzone(data,start_us,windows,threshold):
    if threshold<1:raise ValueError('Movement threshold must be at least one count')
    t=(data['device_us'].astype(float)-start_us)*1e-6;rows=[]
    for begin,end,command in windows:
        ix=(t>=begin)&(t<end);before=(t>=begin-.12)&(t<begin)
        if np.sum(ix)<3 or np.sum(before)<2:continue
        count=data['count'][ix].astype(float);base=float(np.median(data['count'][before]))
        delta=float(np.median(count[-max(2,len(count)//4):])-base)
        # Expected slew and reversal blanking at pulse edges do not invalidate
        # a settled plateau. Amplitude clipping and a protection fault do.
        limits=bool(np.any(data['limits'][ix]&1)) or bool(np.any(data['state'][ix]==5))
        valid=bool(np.all((data['validity'][ix]&1)!=0))
        resting=np.ptp(data['count'][before])<=max(1,threshold/2)
        applied=float(np.median(data['applied'][ix][-max(2,len(count)//3):])*100)
        plateau=abs(applied-command)<=max(.1,.05*abs(command))
        rows.append({'command_pct':command,'applied_pct':applied,'delta_counts':delta,
                     'valid':valid and resting and not limits and plateau,'plateau_reached':plateau,'moved':abs(delta)>=threshold})
    found={}
    for name,sign in [('positive',1),('negative',-1)]:
        matches=[r for r in rows if r['valid'] and r['moved'] and np.sign(r['command_pct'])==sign]
        found[name]=min((abs(r['applied_pct']) for r in matches),default=None)
    return {'breakaway_pct':found,'pulses':rows,'threshold_counts':threshold,
            'note':'Finite pulse breakaway under this load/supply. No detection means not found in tested range. This does not measure sustaining friction or hard-stop position.'}
