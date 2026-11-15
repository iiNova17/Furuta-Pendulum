"""Protocol v1: byte-exact compatibility with firmware/protocol.h."""
import json
import struct
import zlib
import numpy as np

HEADER = struct.Struct('<BBHQIH')
CORE = np.dtype([
    ('device_us','<u8'),('seq','<u4'),('count','<i8'),('pend_raw','<u2'),('adc_raw','<u2'),
    ('current_mv','<f4'),('filtered_mv','<f4'),('requested','<f4'),('applied','<f4'),
    ('pend_age_us','<u4'),('adc_age_us','<u4'),('mag_status','u1'),('agc','u1'),
    ('magnitude','<u2'),('validity','<u4'),('limits','<u4'),('apply_us','<u8'),
    ('adc_us','<u8'),('pend_us','<u8'),('state','u1'),('_pad','u1',3)])
ADC = np.dtype([('device_us','<u8'),('adc_raw','<u2'),('current_mv','<f4'),
                ('requested','<f4'),('applied','<f4')])
assert CORE.itemsize == 88 and ADC.itemsize == 22

def cobs(data):
    out=bytearray(b'\0');at=0;code=1
    for b in data:
        if b==0:
            out[at]=code;at=len(out);out.append(0);code=1
        else:
            out.append(b);code+=1
            if code==255:out[at]=code;at=len(out);out.append(0);code=1
    out[at]=code;out.append(0)
    return bytes(out)

def uncobs(data):
    out=bytearray();at=0
    while at<len(data):
        code=data[at];at+=1
        if not code or at+code-1>len(data):raise ValueError('Invalid COBS')
        part=data[at:at+code-1]
        if 0 in part:raise ValueError('Zero in COBS')
        out.extend(part);at+=code-1
        if code<255 and at<len(data):out.append(0)
        if len(out)>2048:raise ValueError('Oversized packet')
    return bytes(out)

def encode(kind,session,seq,payload):
    if len(payload)>2026:raise ValueError('Payload too large')
    raw=HEADER.pack(1,kind,0,session,seq,len(payload))+payload
    return cobs(raw+struct.pack('<I',zlib.crc32(raw)))

def command(session,seq,op,**args):
    args.update(id=seq,op=op)
    return encode(1,session,seq,json.dumps(args,separators=(',',':'),allow_nan=False).encode())

def decode(data):
    b=uncobs(data)
    if len(b)<22:raise ValueError('Short frame')
    version,kind,flags,session,seq,n=HEADER.unpack_from(b)
    if version!=1 or len(b)!=22+n:raise ValueError('Version/length mismatch')
    if zlib.crc32(b[:-4])!=struct.unpack_from('<I',b,len(b)-4)[0]:raise ValueError('CRC mismatch')
    return kind,session,seq,b[18:-4]

class Parser:
    """COBS stream parser.

    A port opened while the MCU is already streaming starts in the middle of a
    frame, and the USB bridge may still deliver stale fragments from before the
    open. With synced=False (the default for a fresh connection) every frame
    that fails to decode before the first valid frame is discarded silently
    (counted in `discarded`) instead of being counted as a link error. After
    that, bad frames count as errors and the first 20 are kept for diagnosis."""
    def __init__(self,synced=False):
        self.buffer=bytearray();self.errors=0;self.last_error='';self.synced=synced
        self.discarded=0;self.bad_frames=[]
    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while True:
            at=self.buffer.find(0)
            if at<0:break
            b=bytes(self.buffer[:at]);del self.buffer[:at+1]
            if not b:continue
            try:frames.append(decode(b));self.synced=True
            except ValueError as e:
                if not self.synced:self.discarded+=len(b);continue
                self.errors+=1;self.last_error=str(e)
                if len(self.bad_frames)<20:self.bad_frames.append({'error':str(e),'length':len(b),'head':b[:64].hex()})
        if len(self.buffer)>2064:
            # Retain an oversized marker until its delimiter to avoid decoding
            # the tail of a damaged packet as a new frame.
            self.buffer[:]=b'\xff'*2065
        return frames

def core_batch(payload):
    if len(payload)<22:raise ValueError('Missing batch header')
    boot,cfg,run,dropped,n=struct.unpack_from('<QIIIH',payload)
    if len(payload)!=22+n*88:raise ValueError('Bad core batch')
    return np.frombuffer(payload,CORE,offset=22,count=n).copy(),dict(boot=boot,config=cfg,run=run,dropped=dropped)

def burst_chunk(payload):
    if len(payload)<11:raise ValueError('Missing burst header')
    seq,offset,n,core=struct.unpack_from('<IIHB',payload)
    dtype=CORE if core else ADC
    if len(payload)!=11+n*dtype.itemsize:raise ValueError('Bad burst size')
    return seq,dict(ok=True,offset=offset,core=bool(core),data=np.frombuffer(payload,dtype,offset=11,count=n).copy())
