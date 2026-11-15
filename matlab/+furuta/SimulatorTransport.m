classdef SimulatorTransport < handle
    % Synthetic plant only. Wire encoding/decoding is shared with hardware.
    properties
        OnPacket = []
        OnRaw = []
        OnError = []
        Parser = furuta.Protocol()
        Config = furuta.Defaults.profile()
        State = 'DISARMED'
        Fault = ''
        BootId = uint64(0)
        Session = uint64(0)
        Sequence = uint32(0)
        ConfigId = uint32(1)
        RunId = uint32(0)
        Now = 0
        Heartbeat = 0
        Center = 0
        Centered = false
        Count = 0
        Velocity = 0
        Phi = 0
        PhiVelocity = 0
        Requested = 0
        Applied = 0
        Started = 0
        Experiment = struct('segments',[])
        Staged = struct([])
        ExperimentValid = false
        Streaming = false
        WeakField = true
        SensorLost = false
        DropPackets = false
        NoiseMv = 5
        BurstActive = false
        BurstPending = false
        BurstUntil = 0
        BurstDuration = 0
        BurstADC = zeros(0,5)
        BurstCore = zeros(0,20)
        BurstAdcN = 0
        BurstCoreN = 0
        CalUntil = 0
        FilterState = []
        FilterRing = []
        SavedConfig
        TableUpload = []
        Cached
        Random
        Tick = 0
        Challenge = uint64(0)
        LastHelloId = 0
        LastHelloSession = uint64(0)
        LastOpenId = 0
        HighestId = 0
        TripSince = 0
        ReverseUntil = 0
    end
    methods
        function obj = SimulatorTransport()
            obj.BootId=furuta.Defaults.nonce();obj.Cached=containers.Map('KeyType','char','ValueType','any');
            obj.Random=RandStream('mt19937ar','Seed',5741);
        end
        function write(obj,bytes)
            ps=obj.Parser.feed(bytes);
            for k=1:numel(ps)
                p=ps{k};if p.Type~=1,continue;end
                try
                    q=jsondecode(native2unicode(p.Payload,'UTF-8'));obj.dispatch(p,q);
                catch ex
                    if ~isempty(obj.OnError),obj.OnError(ex.message);end
                end
            end
        end
        function emit(obj,type,payload)
            obj.Sequence=obj.Sequence+1;
            if obj.DropPackets && mod(double(obj.Sequence),7)==0,return;end
            bytes=furuta.Protocol.encode(type,obj.Session,obj.Sequence,payload);
            if ~isempty(obj.OnRaw),obj.OnRaw(bytes);end
            p=furuta.Protocol.decode(bytes(1:end-1));if ~isempty(obj.OnPacket),obj.OnPacket(p);end
        end
        function event(obj,name)
            d=struct('event',name,'status',obj.status());
            obj.emit(3,unicode2native(jsonencode(d),'UTF-8'));
        end
        function s = status(obj)
            s=struct('state',obj.State,'fault',obj.Fault,'centered',obj.Centered,'configId',double(obj.ConfigId),...
                'runId',double(obj.RunId),'bootId',char(string(obj.BootId)),'deviceUs',char(string(uint64(round(obj.Now*1e6)))),...
                'dropped',0,'adcOverruns',0,'controlOverruns',0,'encoderOk',true,'pendStatus',32+16*obj.WeakField,...
                'pendValid',~obj.SensorLost,'currentMv',2500,'centerCount',char(string(int64(round(obj.Center)))),...
                'burstActive',obj.BurstActive,'burstPending',obj.BurstPending,'burstAdcCount',obj.BurstAdcN,...
                'burstCoreCount',obj.BurstCoreN,'burstOverflow',false,'adcCalibrationSource',-1);
        end
        function stop(obj)
            if ~strcmp(obj.State,'FAULT'),obj.State='DISARMED';end
            obj.Requested=0;obj.Applied=0;obj.ExperimentValid=false;obj.BurstPending=false;
        end
        function fail(obj,why)
            obj.State='FAULT';obj.Fault=why;obj.Requested=0;obj.Applied=0;obj.event(why);
        end
        function dispatch(obj,p,q)
            r=struct('id',q.id,'op',q.op,'ok',true);key=sprintf('%u',q.id);op=char(q.op);
            emergency=ismember(op,{'STOP','DISARM','ABORT'});
            try
                if emergency,obj.stop();
                elseif ~ismember(op,{'HELLO','SESSION_OPEN'})
                    assert(obj.Session~=0&&p.Session==obj.Session,'Furuta:Session','Session mismatch');
                    if isKey(obj.Cached,key)
                        old=obj.Cached(key);assert(isequal(old.Request,q),'Furuta:ID','ID reused with different payload');
                        if isfield(old,'Payload'),obj.emit(5,old.Payload);else,obj.emit(2,unicode2native(jsonencode(old.Response),'UTF-8'));end
                        return
                    end
                    assert(q.id>obj.HighestId,'Furuta:ID','Stale command outside retry cache');obj.HighestId=q.id;
                end
                switch op
                    case 'HELLO'
                        if q.id~=obj.LastHelloId||p.Session~=obj.LastHelloSession
                            obj.Challenge=furuta.Defaults.nonce();obj.LastHelloId=q.id;obj.LastHelloSession=p.Session;
                        end
                        r.challenge=char(string(obj.Challenge));
                        r.firmware='FurutaWorkbench/1.0 SIMULATOR';r.protocol=1;r.coreRecordBytes=88;r.baud=460800;
                        r.flashBytes=16777216;r.psramBytes=8388608;r.maxBurstBytes=4194304;r.maxSegments=256;r.maxTable=4096;
                    case 'SESSION_OPEN'
                        assert(p.Session~=0&&isfield(q,'bootId')&&isfield(q,'challenge')&&strcmp(q.bootId,char(string(obj.BootId)))&&strcmp(q.challenge,char(string(obj.Challenge))),...
                            'Furuta:Session','Fresh HELLO challenge and boot ID required');
                        if p.Session~=obj.Session||q.id~=obj.LastOpenId
                            obj.stop();obj.Session=p.Session;obj.LastOpenId=q.id;obj.HighestId=q.id;
                            obj.Cached=containers.Map('KeyType','char','ValueType','any');obj.Streaming=false;
                        end
                    case {'STOP','DISARM','ABORT'}
                    case 'HEARTBEAT',obj.Heartbeat=obj.Now;
                    case {'STATUS','CAPABILITIES','EXPERIMENT_STATUS'}
                    case 'GET_CONFIG',r.config=obj.Config;
                    case 'START_STREAM',obj.Streaming=true;
                    case 'STOP_STREAM',obj.Streaming=false;
                    case 'ARM'
                        assert(strcmp(obj.State,'DISARMED')&&obj.Centered&&obj.Config.maxDuty>0&&obj.Config.travelCounts>0,...
                            'Furuta:Arm','Center arm and commission positive duty/travel limits');
                        assert(obj.Config.currentTripA==0||isfinite(obj.Config.currentZeroMv),'Furuta:Arm','Current zero required');
                        obj.State='ARMED_IDLE';obj.Heartbeat=obj.Now;
                    case 'START'
                        assert(strcmp(obj.State,'ARMED_IDLE')&&obj.ExperimentValid,'Furuta:Start','Arm and validate before starting');
                        obj.State='RUNNING';obj.Started=obj.Now;obj.RunId=obj.RunId+1;obj.TripSince=0;
                        if obj.BurstPending,obj.startBurst();end
                    case 'CLEAR_FAULT'
                        if strcmp(obj.State,'FAULT'),obj.Fault='';obj.State='DISARMED';obj.stop();end
                    otherwise
                        assert(~ismember(obj.State,{'ARMED_IDLE','RUNNING'}),'Furuta:State','Disarm before changing configuration/capture');
                        switch op
                            case 'SET_CONFIG'
                                c=obj.Config;
                                for f=fieldnames(q.config).',c.(f{1})=q.config.(f{1});end
                                for f={'countsPerRev','currentZeroMv','pendDownRaw'}
                                    if isempty(c.(f{1})),c.(f{1})=NaN;end
                                end
                                assert(c.maxDuty>=0&&c.maxDuty<=1&&c.travelCounts>=0&&c.slewPerSecond>0&&c.maxDuration>0,...
                                    'Furuta:Config','Invalid profile');
                                furuta.Defaults.validate(c);
                                obj.Config=c;obj.ConfigId=obj.ConfigId+1;obj.FilterState=[];obj.FilterRing=[];obj.ExperimentValid=false;r.config=c;
                            case 'SAVE_CONFIG',obj.SavedConfig=obj.Config;
                            case 'CALIBRATE'
                                switch q.target
                                    case 'arm',obj.Center=round(obj.Count);obj.Centered=true;obj.ConfigId=obj.ConfigId+1;
                                    case 'down',obj.Config.pendDownRaw=obj.pendRaw();obj.ConfigId=obj.ConfigId+1;
                                    case 'upright',obj.Config.pendDownRaw=mod(obj.pendRaw()+2048,4096);obj.ConfigId=obj.ConfigId+1;
                                    case 'current',obj.Config.currentZeroMv=NaN;obj.ConfigId=obj.ConfigId+1;obj.CalUntil=obj.Now+1;r.pending=true;
                                    otherwise,error('Furuta:Calibration','Unknown target');
                                end
                            case 'UPLOAD_BEGIN',obj.Staged=struct([]);obj.ExperimentValid=false;
                            case 'TABLE_CLEAR',obj.TableUpload=[];
                            case 'TABLE_CHUNK'
                                assert(q.offset==numel(obj.TableUpload)&&numel(q.values)<=128&&numel(obj.TableUpload)+numel(q.values)<=4096,'Furuta:Table','Invalid chunk');
                                obj.TableUpload=[obj.TableUpload, q.values(:).'];
                            case 'SET_SEGMENT'
                                assert(q.index==numel(obj.Staged)&&q.index<256,'Furuta:Segment','Upload segments in order');
                                s=q.segment;if strcmp(s.kind,'arbitrary')&&isempty(s.table),s.table=obj.TableUpload;end
                                obj.Staged=[obj.Staged,s];
                            case {'VALIDATE','UPLOAD_EXPERIMENT'}
                                obj.Experiment=struct('segments',obj.Staged);furuta.ExperimentBuilder.validate(obj.Experiment,obj.Config);
                                obj.ExperimentValid=true;r.duration=sum([obj.Staged.duration]);
                            case 'CAPTURE_BURST'
                                assert(q.seconds>0&&q.seconds<=120&&(q.seconds*(obj.Config.adcHz*32+500*104))<4194304,'Furuta:Capture','Burst memory budget exceeded');
                                assert(~obj.BurstActive,'Furuta:Capture','Capture already active');
                                obj.BurstDuration=q.seconds;obj.BurstADC=zeros(ceil(q.seconds*obj.Config.adcHz)+10,5);
                                obj.BurstCore=zeros(ceil(q.seconds*500)+10,20);obj.BurstAdcN=0;obj.BurstCoreN=0;
                                obj.BurstPending=isfield(q,'withRun')&&q.withRun;
                                if ~obj.BurstPending,obj.startBurst();end
                            case 'BURST_INFO'
                                r.adcCount=obj.BurstAdcN;r.coreCount=obj.BurstCoreN;r.active=obj.BurstActive;r.pending=obj.BurstPending;
                                r.overflow=false;r.adcHz=obj.Config.adcHz;r.startUs=char(string(uint64(round((obj.BurstUntil-obj.BurstDuration)*1e6))));
                            case 'READ_CHUNK'
                                assert(~obj.BurstActive&&~obj.BurstPending,'Furuta:Burst','Wait for capture completion');
                                core=strcmp(q.channel,'core');total=obj.BurstAdcN;if core,total=obj.BurstCoreN;end
                                n=min([16,q.count,total-q.offset]);assert(n>=0,'Furuta:Burst','Offset exceeds capture');
                                prefix=[furuta.Protocol.le(uint32(q.id)),furuta.Protocol.le(uint32(q.offset)),furuta.Protocol.le(uint16(n)),uint8(core)];
                                if core
                                    T=obj.rowsToTable(obj.BurstCore(q.offset+(1:n),:));b=furuta.Protocol.packCore(T);
                                else
                                    a=obj.BurstADC(q.offset+(1:n),:);b=zeros(22,n,'uint8');
                                    b(1:8,:)=reshape(furuta.Protocol.le(uint64(a(:,1))),8,[]);
                                    b(9:10,:)=reshape(furuta.Protocol.le(uint16(a(:,2))),2,[]);
                                    for j=3:5,b(11+4*(j-3):14+4*(j-3),:)=reshape(furuta.Protocol.le(single(a(:,j))),4,[]);end
                                    b=b(:).';
                                end
                                obj.Cached(key)=struct('Request',q,'Payload',[prefix,b]);obj.emit(5,[prefix,b]);return
                            otherwise,error('Furuta:Command','Unknown operation %s',op);
                        end
                end
            catch ex,r.ok=false;r.error=ex.message;
            end
            r.status=obj.status();
            if ~ismember(op,{'HELLO','SESSION_OPEN','STOP','DISARM','ABORT'})
                obj.Cached(key)=struct('Request',q,'Response',r);
                if obj.Cached.Count>64,keys=obj.Cached.keys;remove(obj.Cached,keys{1});end
            end
            obj.emit(2,unicode2native(jsonencode(r),'UTF-8'));
        end
        function startBurst(obj)
            obj.BurstPending=false;obj.BurstActive=true;obj.BurstUntil=obj.Now+obj.BurstDuration;
        end
        function raw = pendRaw(obj),raw=mod(round((obj.Phi+pi)*4096/(2*pi)),4096);end
        function y = filtered(obj,x)
            f=obj.Config.filter;if isempty(obj.FilterState),obj.FilterState=[x 0 0];end
            switch f.type
                case 'ema',obj.FilterState(1)=obj.FilterState(1)+f.alpha*(x-obj.FilterState(1));y=obj.FilterState(1);
                case 'moving'
                    obj.FilterRing=[obj.FilterRing,x];if numel(obj.FilterRing)>f.window,obj.FilterRing(1)=[];end
                    y=mean(obj.FilterRing);
                case 'biquad'
                    a=f.sos;y=a(1)*x+obj.FilterState(2);z=a(2)*x-a(5)*y+obj.FilterState(3);
                    obj.FilterState(3)=a(3)*x-a(6)*y;obj.FilterState(2)=z;
                otherwise,y=x;
            end
        end
        function step(obj,dt)
            rows=zeros(ceil(dt*500)+4,20);nr=0;steps=max(1,round(dt/.001));h=dt/steps;mv=2500;filt=mv;
            for k=1:steps
                obj.Now=obj.Now+h;obj.Tick=obj.Tick+1;
                isArmed=ismember(obj.State,{'ARMED_IDLE','RUNNING'});
                if isArmed
                    if obj.Now-obj.Heartbeat>obj.Config.heartbeatMs/1000,obj.fail('HEARTBEAT_TIMEOUT');
                    elseif abs(obj.Count-obj.Center)>obj.Config.travelCounts,obj.fail('ARM_TRAVEL');
                    elseif obj.Config.velocityCountsPerSecond>0&&abs(obj.Velocity)>obj.Config.velocityCountsPerSecond,obj.fail('ARM_VELOCITY');
                    end
                end
                if strcmp(obj.State,'RUNNING')
                    t=obj.Now-obj.Started;at=0;obj.Requested=0;
                    for s=obj.Experiment.segments(:).'
                        if t>=at&&t<at+s.duration,obj.Requested=furuta.ExperimentBuilder.value(s,t-at);break;end
                        at=at+s.duration;
                    end
                    if t>=sum([obj.Experiment.segments.duration]),obj.stop();obj.event('DISARMED');
                    else
                        target=max(-obj.Config.maxDuty,min(obj.Config.maxDuty,obj.Requested));
                        if obj.Applied*target<0,target=0;end
                        next=obj.Applied+max(-obj.Config.slewPerSecond*h,min(obj.Config.slewPerSecond*h,target-obj.Applied));
                        if abs(next)<1e-9&&abs(obj.Applied)>1e-9,next=0;obj.ReverseUntil=obj.Now+.001;end
                        if obj.Now<obj.ReverseUntil,next=0;end
                        obj.Applied=next;
                    end
                end
                accel=(9000*obj.Applied-obj.Velocity)/.15;obj.Velocity=obj.Velocity+h*accel;obj.Count=obj.Count+h*obj.Velocity;
                phiA=-30*sin(obj.Phi)-.5*obj.PhiVelocity+accel*.0001;
                obj.PhiVelocity=obj.PhiVelocity+h*phiA;obj.Phi=obj.Phi+h*obj.PhiVelocity;
                adcN=max(1,round(h*obj.Config.adcHz));
                for j=1:adcN
                    at=obj.Now-h+j*h/adcN;mv=2500+185*(obj.Applied*.7+accel*.00001)+obj.NoiseMv*randn(obj.Random);
                    raw=max(0,min(4095,round(mv/3100*4095)));filt=obj.filtered(mv);
                    if obj.BurstActive&&at<obj.BurstUntil
                        obj.BurstAdcN=obj.BurstAdcN+1;obj.BurstADC(obj.BurstAdcN,:)=[round(at*1e6),raw,mv,obj.Requested,obj.Applied];
                    end
                end
                if obj.CalUntil>0&&obj.Now>=obj.CalUntil
                    obj.CalUntil=0;obj.Config.currentZeroMv=2500;obj.ConfigId=obj.ConfigId+1;obj.event('CURRENT_ZERO_UPDATED');
                end
                if obj.BurstActive&&obj.Now>=obj.BurstUntil,obj.BurstActive=false;obj.event('BURST_COMPLETE');end
                if mod(obj.Tick,2)==0
                    valid=7;status=32+16*obj.WeakField;if obj.SensorLost,valid=5;status=0;end
                    row=[round(obj.Now*1e6),obj.Tick/2,round(obj.Count),obj.pendRaw(),raw,mv,filt,obj.Requested,obj.Applied,...
                        0,0,status,128*obj.WeakField+50,1700,valid,0,round(obj.Now*1e6),round(obj.Now*1e6),round(obj.Now*1e6),...
                        find(strcmp(obj.State,{'BOOT','DISARMED','ARMED_IDLE','RUNNING','STOPPING','FAULT'}))-1];
                    if obj.BurstActive,obj.BurstCoreN=obj.BurstCoreN+1;obj.BurstCore(obj.BurstCoreN,:)=row;end
                    if obj.Streaming&&mod(obj.Tick/2,500/obj.Config.streamHz)==0,nr=nr+1;rows(nr,:)=row;end
                end
            end
            if nr>0
                T=obj.rowsToTable(rows(1:nr,:));
                for k=1:16:height(T),block=T(k:min(k+15,end),:);obj.emit(4,furuta.Protocol.packSamples(block));end
            end
        end
        function T = rowsToTable(obj,rows)
            [names,types]=furuta.Protocol.layout();T=table;
            for k=1:numel(names),T.(names{k})=cast(rows(:,k),types{k});end
            n=size(rows,1);T.BootId=repmat(obj.BootId,n,1);T.ConfigId=repmat(obj.ConfigId,n,1);
            T.RunId=repmat(obj.RunId,n,1);T.Dropped=zeros(n,1,'uint32');
        end
        function delete(obj),obj.stop();obj.OnPacket=[];obj.OnRaw=[];obj.OnError=[];end
    end
end
