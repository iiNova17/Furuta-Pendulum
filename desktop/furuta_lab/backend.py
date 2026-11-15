"""MATLAB jobs are isolated processes; they never own serial or the GUI thread."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from scipy.io import savemat

ROOT=Path(__file__).resolve().parents[2]

def jsonable(value):
    import numpy as np
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    if isinstance(value,np.ndarray):return jsonable(value.tolist())
    if isinstance(value,np.generic):return jsonable(value.item())
    if isinstance(value,float) and not np.isfinite(value):return None
    return value

class MatlabBackend:
    def __init__(self):self.lock=threading.Lock();self.process=None
    def speed(self,time_values,applied,speed,method='PT1',timeout=180):
        if not self.lock.acquire(blocking=False):raise RuntimeError('A MATLAB job is already running')
        try:
            exe=os.environ.get('FURUTA_MATLAB') or shutil.which('matlab')
            if not exe and Path('D:/MATLAB/R2023b/bin/matlab.exe').exists():exe='D:/MATLAB/R2023b/bin/matlab.exe'
            if not exe:raise RuntimeError('Set FURUTA_MATLAB to matlab.exe; sensor/noise tools work without MATLAB')
            folder=ROOT/'data'/'matlab-jobs'/time.strftime('%Y%m%d_%H%M%S')
            folder.mkdir(parents=True,exist_ok=False)
            savemat(folder/'input.mat',{'t':time_values,'u':applied,'y':speed},oned_as='column')
            request={'input':str(folder/'input.mat'),'output':str(folder/'result.json'),'method':method}
            (folder/'request.json').write_text(json.dumps(request),encoding='utf-8')
            quote=lambda p:str(p).replace("'","''")
            expr=f"addpath('{quote(ROOT/'matlab')}'); furuta_backend('{quote(folder/'request.json')}');"
            flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
            self.process=subprocess.Popen([exe,'-wait','-nodesktop','-nosplash','-softwareopengl','-batch',expr,
                                           '-logfile',str(folder/'matlab.log')],creationflags=flags)
            try:code=self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.cancel();raise TimeoutError(f'MATLAB timed out. Logs retained in {folder}')
            if code or not (folder/'result.json').exists():raise RuntimeError(f'MATLAB job failed. See {folder / "matlab.log"}')
            result=json.loads((folder/'result.json').read_text(encoding='utf-8-sig'))
            if not result.get('ok'):raise RuntimeError(result.get('error','MATLAB fit failed'))
            result['job_directory']=str(folder);return result
        finally:self.process=None;self.lock.release()
    def cancel(self):
        p=self.process
        if p and p.poll() is None:
            if os.name=='nt':subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
            else:p.terminate()

def save_capture(path,capture,metadata):
    import numpy as np
    path=Path(path);path.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(path/'capture.npz',adc=capture['adc'],core=capture['core'])
    (path/'capture.json').write_text(json.dumps(jsonable({k:v for k,v in capture.items() if k not in ('adc','core')}|{'metadata':metadata}),indent=2),encoding='utf-8')
    # A plain MAT file keeps the focused frontend interoperable with MATLAB.
    a=capture['adc'];c=capture['core']
    savemat(path/'capture.mat',{'adc':{n:a[n] for n in a.dtype.names if not n.startswith('_')},'core':{n:c[n] for n in c.dtype.names if not n.startswith('_')},
                               'metadata_json':json.dumps(jsonable(metadata))},oned_as='column')
    return path

def load_capture(path):
    import numpy as np
    path=Path(path);m=json.loads((path/'capture.json').read_text(encoding='utf-8'))
    with np.load(path/'capture.npz',allow_pickle=False) as a:m.update(adc=a['adc'].copy(),core=a['core'].copy())
    return m

def load_run(path):
    import numpy as np
    from .wire import core_batch,CORE
    path=Path(path);manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    if 'config' not in manifest and (path/'configuration.json').exists():
        manifest['config']=json.loads((path/'configuration.json').read_text(encoding='utf-8'))
    parts=[]
    with (path/'samples.bin').open('rb') as f:
        while True:
            length=f.read(4)
            if not length:break
            if len(length)!=4:raise ValueError('Incomplete batch length')
            n=int.from_bytes(length,'little')
            if not 22<=n<=2026:raise ValueError('Corrupt batch length')
            b=f.read(n)
            if len(b)!=n:raise ValueError('Incomplete batch; retain the original recording')
            a,_=core_batch(b);parts.append(a)
    return (np.concatenate(parts) if parts else np.empty(0,CORE)),manifest
