classdef DeviceSession < handle
    properties
        Transport
        Mode = 'Disconnected'
        Connected = false
        Ready = false
        Config = furuta.Defaults.profile()
        Status = struct('state','DISCONNECTED','centered',false,'centerCount','0')
        Identity = struct
        Data = furuta.Protocol.emptySamples()
        Recorder = []
        OnData = []
        OnStatus = []
        OnEvent = []
        Timer = []
        Pending
        Callbacks = {}
        CommandId = uint32(0)
        Nonce = uint64(0)
        Clock
        LastTick = 0
        LastHeartbeat = 0
        LastStatus = 0
        Busy = false
        Closing = false
        Events = {}
        Download = []
        CalibrationQuality = struct
        LeaseEnabled = true
        SamplePackets = {}
    end
    methods
        function obj = DeviceSession(),obj.Pending=containers.Map('KeyType','char','ValueType','any');obj.Clock=tic;end
        function connect(obj,mode,port,baud)
            if nargin<3,port='';end
            if nargin<4,baud=460800;end
            obj.disconnect();obj.Closing=false;obj.Mode=char(mode);obj.Nonce=furuta.Defaults.nonce();obj.LeaseEnabled=true;
            if strcmp(mode,'Simulator'),obj.Transport=furuta.SimulatorTransport();
            else,obj.Transport=furuta.SerialTransport(port,baud);end
            obj.Transport.OnPacket=@(p)obj.receive(p);obj.Transport.OnRaw=@(b)obj.raw(b);obj.Transport.OnError=@(e)obj.transportError(e);
            obj.Connected=true;obj.Clock=tic;obj.LastTick=0;obj.LastHeartbeat=0;obj.LastStatus=0;
            obj.request('HELLO',struct,@(r)obj.hello(r));
            obj.startTimer();
        end
        function connectReplay(obj,run)
            obj.disconnect();obj.Closing=false;obj.Mode='Replay';obj.Config=run.Config;
            obj.Status=struct('state','REPLAY','centered',false,'centerCount','0');obj.Ready=true;
            obj.Transport=furuta.ReplayTransport(run);obj.Transport.OnPacket=@(p)obj.receive(p);
            obj.Connected=true;obj.Clock=tic;obj.LastTick=0;obj.startTimer();
        end
        function startTimer(obj)
            obj.Timer=timer('ExecutionMode','fixedSpacing','Period',.05,'BusyMode','drop',...
                'TimerFcn',@(~,~)obj.poll(),'ErrorFcn',@(~,e)obj.transportError(e.Data.Message));
            start(obj.Timer);
        end
        function hello(obj,r)
            if ~r.ok,obj.transportError(r.error);return;end
            if ~isfield(r,'protocol')||r.protocol~=1||r.coreRecordBytes~=88,obj.transportError('Incompatible workbench protocol');return;end
            obj.Identity=r;obj.request('SESSION_OPEN',struct('bootId',r.status.bootId,'challenge',r.challenge),@(a)obj.opened(a));
        end
        function opened(obj,r)
            if ~r.ok,obj.transportError(r.error);return;end
            obj.request('GET_CONFIG',struct,@(q)obj.gotConfig(q));obj.request('START_STREAM',struct,[]);
        end
        function gotConfig(obj,r)
            if r.ok
                c=r.config;for f={'currentZeroMv','countsPerRev','pendDownRaw','deadzonePos','deadzoneNeg'},if isfield(c,f{1})&&isempty(c.(f{1})),c.(f{1})=NaN;end,end
                obj.Config=c;obj.Ready=true;obj.event(struct('configSnapshot',c,'status',obj.Status));
            end
            obj.changed();
        end
        function id = request(obj,op,args,callback)
            if nargin<3,args=struct;end
            if nargin<4,callback=[];end
            assert(obj.Connected&&~strcmp(obj.Mode,'Replay'),'Furuta:Connection','Connect hardware/simulator first');
            assert(obj.CommandId<intmax('uint32'),'Furuta:Sequence','Reconnect to renew exhausted command IDs');
            if ismember(op,{'STOP','DISARM','ABORT'}),obj.LeaseEnabled=false;end
            if strcmp(op,'ARM'),obj.LeaseEnabled=true;end
            obj.CommandId=obj.CommandId+1;
            id=double(obj.CommandId);bytes=furuta.Protocol.command(obj.Nonce,obj.CommandId,op,args);
            key=sprintf('%u',id);
            obj.Pending(key)=struct('Bytes',bytes,'Sent',toc(obj.Clock),'Tries',1,'Callback',callback,'Op',op);
            obj.Transport.write(bytes);
            if ~ismember(op,{'HEARTBEAT','STATUS'}),obj.event(struct('command',op,'id',id,'arguments',args));end
        end
        function receive(obj,p)
            if obj.Closing,return;end
            try
                if p.Type==4
                    if ~obj.Ready||(~strcmp(obj.Mode,'Replay')&&p.Session~=obj.Nonce),return;end
                    assert(numel(obj.SamplePackets)<2048,'Furuta:Backlog','Sample processing fell behind');
                    obj.SamplePackets{end+1}=p;
                    return
                elseif p.Type==5
                    r=furuta.Protocol.burst(p);r.ok=true;obj.resolve(r.Id,r);return
                elseif p.Type~=2&&p.Type~=3,return
                end
                r=jsondecode(native2unicode(p.Payload,'UTF-8'));
                if isfield(r,'status'),obj.Status=r.status;obj.changed();end
                if p.Type==2,obj.resolve(r.id,r);
                else
                    obj.event(r);
                    if isfield(r,'event')&&strcmp(r.event,'CURRENT_ZERO_UPDATED')
                        obj.CalibrationQuality=r;obj.request('GET_CONFIG',struct,@(q)obj.gotConfig(q));
                    end
                end
            catch ex,obj.event(struct('decodeError',ex.message));end
        end
        function resolve(obj,id,r)
            key=sprintf('%u',id);
            if ~isKey(obj.Pending,key),return;end
            pending=obj.Pending(key);remove(obj.Pending,key);
            if ~r.ok,obj.event(struct('error',r.error,'command',pending.Op));end
            if ~isempty(pending.Callback),obj.Callbacks{end+1}=@()pending.Callback(r);end
        end
        function poll(obj)
            if obj.Busy||~obj.Connected||obj.Closing,return;end
            obj.Busy=true;guard=onCleanup(@()obj.release()); %#ok<NASGU>
            try
                now=toc(obj.Clock);dt=now-obj.LastTick;obj.LastTick=now;
                % Interactive simulator uses a bounded synthetic clock. Hardware
                % watchdogs still use the real device clock, independently.
                if strcmp(obj.Mode,'Simulator'),dt=min(dt,.05);end
                obj.Transport.step(dt);
                if ~obj.Connected,return;end
                obj.flushSamples();
                for k=1:32
                    if isempty(obj.Callbacks),break;end
                    callback=obj.Callbacks{1};obj.Callbacks(1)=[];callback();
                end
                if strcmp(obj.Mode,'Replay'),return;end
                now=toc(obj.Clock);
                if obj.Ready&&obj.LeaseEnabled&&ismember(obj.Status.state,{'ARMED_IDLE','RUNNING','STOPPING'})&&now-obj.LastHeartbeat>=.1
                    obj.LastHeartbeat=now;obj.request('HEARTBEAT',struct,[]);
                end
                if obj.Ready&&now-obj.LastStatus>=.5
                    obj.LastStatus=now;obj.request('STATUS',struct,[]);
                end
                keys=obj.Pending.keys;
                for k=1:numel(keys)
                    if ~isKey(obj.Pending,keys{k}),continue;end
                    v=obj.Pending(keys{k});
                    if now-v.Sent>1
                        if v.Tries>=3
                            remove(obj.Pending,keys{k});obj.event(struct('error',['Timeout: ',v.Op]));
                            if ~isempty(v.Callback),obj.Callbacks{end+1}=@()v.Callback(struct('ok',false,'error',['Timeout: ',v.Op]));end
                            if ismember(v.Op,{'HELLO','SESSION_OPEN','HEARTBEAT'}),obj.transportError(['Connection timeout: ',v.Op]);return;end
                        else
                            v.Tries=v.Tries+1;v.Sent=now;obj.Pending(keys{k})=v;obj.Transport.write(v.Bytes);
                        end
                    end
                end
            catch ex,obj.transportError(ex.message);end
        end
        function release(obj),obj.Busy=false;end
        function flushSamples(obj)
            if isempty(obj.SamplePackets),return;end
            packets=obj.SamplePackets;obj.SamplePackets={};T=furuta.Protocol.sampleBatches(packets);
            if isempty(T),return;end
            if any(T.BootId~=T.BootId(1))||(~isempty(obj.Data)&&T.BootId(1)~=obj.Data.BootId(end))
                obj.transportError('Device restarted. Reconnect and re-center arm.');return
            end
            obj.Data=[obj.Data;T];if height(obj.Data)>60000,obj.Data=obj.Data(end-59999:end,:);end
            if ~isempty(obj.Recorder)&&obj.Recorder.Active
                for k=1:numel(packets),obj.Recorder.appendBatch(packets{k}.Payload);end
            end
            if ~isempty(obj.OnData),obj.OnData(T);end
        end
        function changed(obj),if ~isempty(obj.OnStatus),obj.OnStatus(obj.Status);end,end
        function event(obj,e)
            obj.Events{end+1}=e;if numel(obj.Events)>2000,obj.Events(1)=[];end
            if ~isempty(obj.Recorder)&&obj.Recorder.Active,obj.Recorder.event(e);end
            if ~isempty(obj.OnEvent),obj.OnEvent(e);end
        end
        function raw(obj,b)
            if ~isempty(obj.Recorder)&&obj.Recorder.Active,obj.Recorder.raw(b);end
        end
        function transportError(obj,message)
            if ~obj.Connected,return;end
            obj.Ready=false;
            try
                if obj.Connected&&~strcmp(obj.Mode,'Replay')
                    bytes=furuta.Protocol.command(obj.Nonce,obj.CommandId+1,'STOP',struct);obj.Transport.write(bytes);
                end
            catch,end
            obj.Status=struct('state','CONNECTION_ERROR','fault',message,'centered',false,'centerCount','0');
            try,obj.event(struct('error',message));catch,end
            obj.Connected=false;obj.LeaseEnabled=false;
            if ~isempty(obj.Timer)&&isvalid(obj.Timer),stop(obj.Timer);end
            if ~isempty(obj.Transport)
                obj.Transport.OnError=[];obj.Transport.OnPacket=[];obj.Transport.OnRaw=[];
                delete(obj.Transport);obj.Transport=[];
            end
            if ~isempty(obj.Recorder),try,obj.Recorder.finish(false);catch,end,end
            obj.Pending=containers.Map('KeyType','char','ValueType','any');obj.Callbacks={};
            obj.changed();
        end
        function applyConfig(obj,c,callback)
            if nargin<3,callback=[];end
            furuta.Defaults.validate(c);
            obj.request('SET_CONFIG',struct('config',furuta.Defaults.deviceProfile(c)),@(r)obj.applied(r,callback));
        end
        function applied(obj,r,callback)
            if r.ok,obj.gotConfig(r);end
            if ~isempty(callback),callback(r);end
        end
        function upload(obj,e,callback)
            furuta.ExperimentBuilder.validate(e,obj.Config);actions={{'UPLOAD_BEGIN',struct}};
            for k=1:numel(e.segments)
                s=e.segments(k);
                if strcmp(s.kind,'arbitrary')
                    actions{end+1}={'TABLE_CLEAR',struct};
                    for j=1:96:numel(s.table),actions{end+1}={'TABLE_CHUNK',struct('offset',j-1,'values',s.table(j:min(j+95,end)))};end
                    s.table=[];
                end
                actions{end+1}={'SET_SEGMENT',struct('index',k-1,'segment',s)};
            end
            actions{end+1}={'VALIDATE',struct};obj.uploadNext(actions,1,callback);
        end
        function uploadNext(obj,actions,k,callback)
            if k>numel(actions),if ~isempty(callback),callback(struct('ok',true));end;return;end
            a=actions{k};obj.request(a{1},a{2},@(r)obj.uploadAck(r,actions,k,callback));
        end
        function uploadAck(obj,r,actions,k,callback)
            if ~r.ok,if ~isempty(callback),callback(r);end;return;end
            obj.uploadNext(actions,k+1,callback);
        end
        function beginRecord(obj,root,e)
            assert(isempty(obj.Recorder)||~obj.Recorder.Active,'Furuta:Recording','Recording already active');
            obj.Data=furuta.Protocol.emptySamples();
            identity=obj.Identity;identity.status=obj.Status;identity.calibrationQuality=obj.CalibrationQuality;
            obj.Recorder=furuta.RunRecorder(root,obj.Config,e,obj.Mode,identity);
        end
        function finishRecord(obj)
            if ~isempty(obj.Recorder),obj.Recorder.finish(true);end
        end
        function downloadBurst(obj,callback)
            obj.request('BURST_INFO',struct,@(r)obj.downloadInfo(r,callback));
        end
        function downloadInfo(obj,r,callback)
            if ~r.ok,callback(r);return;end
            if r.active||r.pending,callback(struct('ok',false,'error','Wait for burst completion'));return;end
            obj.Download=struct('Info',r,'ADC',table,'Core',table,'Channel','adc','Offset',0,'Callback',callback);
            obj.downloadNext();
        end
        function downloadNext(obj)
            total=obj.Download.Info.adcCount;if strcmp(obj.Download.Channel,'core'),total=obj.Download.Info.coreCount;end
            if obj.Download.Offset>=total
                if strcmp(obj.Download.Channel,'adc'),obj.Download.Channel='core';obj.Download.Offset=0;obj.downloadNext();return;end
                r=obj.Download;r.ok=true;cb=r.Callback;obj.Download=[];cb(r);return;
            end
            obj.request('READ_CHUNK',struct('channel',obj.Download.Channel,'offset',obj.Download.Offset,'count',16),@(r)obj.downloadAck(r));
        end
        function downloadAck(obj,r)
            if ~r.ok,cb=obj.Download.Callback;obj.Download=[];cb(r);return;end
            if r.Offset~=obj.Download.Offset,cb=obj.Download.Callback;obj.Download=[];cb(struct('ok',false,'error','Out-of-order burst chunk'));return;end
            if strcmp(obj.Download.Channel,'adc'),obj.Download.ADC=[obj.Download.ADC;r.Data];else,obj.Download.Core=[obj.Download.Core;r.Data];end
            obj.Download.Offset=obj.Download.Offset+height(r.Data);obj.downloadNext();
        end
        function disconnect(obj)
            if obj.Closing,return;end
            obj.Closing=true;
            if obj.Connected&&~strcmp(obj.Mode,'Replay')
                try,obj.Transport.write(furuta.Protocol.command(obj.Nonce,obj.CommandId+1,'STOP',struct));catch,end
            end
            if ~isempty(obj.Timer)&&isvalid(obj.Timer),stop(obj.Timer);delete(obj.Timer);end
            obj.Timer=[];if ~isempty(obj.Transport),delete(obj.Transport);end;obj.Transport=[];
            if ~isempty(obj.Recorder),try,obj.Recorder.finish(true);catch,end,end
            obj.Connected=false;obj.Ready=false;obj.Data=furuta.Protocol.emptySamples();obj.Pending=containers.Map('KeyType','char','ValueType','any');
            obj.Callbacks={};obj.SamplePackets={};obj.Status=struct('state','DISCONNECTED','centered',false,'centerCount','0');obj.Mode='Disconnected';obj.Closing=false;
        end
        function delete(obj),obj.disconnect();end
    end
end
