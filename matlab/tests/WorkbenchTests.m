classdef WorkbenchTests < matlab.unittest.TestCase
    properties
        Packets = {}
    end
    methods
        function collect(obj,p),obj.Packets{end+1}=p;end
        function r = command(obj,s,id,op,args)
            if nargin<5,args=struct;end
            if strcmp(op,'SESSION_OPEN'),args.bootId=char(string(s.BootId));args.challenge=char(string(s.Challenge));end
            obj.Packets={};s.write(furuta.Protocol.command(uint64(99),id,op,args));
            assert(~isempty(obj.Packets));p=obj.Packets{end};
            if p.Type==5,r=furuta.Protocol.burst(p);r.ok=true;else,r=jsondecode(native2unicode(p.Payload,'UTF-8'));end
        end
    end
    methods (TestClassSetup)
        function paths(~),addpath(fileparts(fileparts(mfilename('fullpath'))));end
    end
    methods (Test)
        function wireGoldenVectors(t)
            root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
            g=jsondecode(fileread(fullfile(root,'protocol','golden_vectors.json')));
            t.verifyEqual(furuta.Protocol.crc(uint8('123456789')),uint32(hex2dec(g.crc123456789)));
            nonce=bitor(bitshift(uint64(hex2dec('12345678')),32),uint64(hex2dec('9abcdef0')));
            for v=g.vectors(:).'
                payload=reshape(uint8(sscanf(v.payloadHex,'%2x')),1,[]);frame=furuta.Protocol.encode(v.type,nonce,v.sequence,payload);
                expected=uint8(sscanf(v.frameHex,'%2x').');t.verifyEqual(frame,expected);
                parser=furuta.Protocol();all={};for k=1:numel(frame),all=[all,parser.feed(frame(k))];end %#ok<AGROW>
                t.verifyNumElements(all,1);t.verifyEqual(all{1}.Session,nonce);t.verifyEqual(all{1}.Payload,payload);
                bad=frame;bad(3)=bitxor(bad(3),uint8(16));t.verifyEmpty(parser.feed(bad));t.verifyGreaterThan(parser.Errors,0);
            end
            packet=struct('Payload',uint8(sscanf(g.corePayloadHex,'%2x').'));
            samples=furuta.Protocol.samples(packet);
            t.verifyEqual(samples.Count,-bitshift(int64(1),53)-int64(1));
            t.verifyEqual(samples.DeviceUs,furuta.Protocol.decimal(g.coreTime));
            t.verifyEqual(furuta.Protocol.packSamples(samples),packet.Payload);
        end
        function waveformGolden(t)
            root=fileparts(fileparts(fileparts(mfilename('fullpath'))));g=jsondecode(fileread(fullfile(root,'protocol','golden_vectors.json')));
            for v=g.waveforms(:).'
                s=furuta.ExperimentBuilder.segment(v.kind,.2,2,1,3,.01);s.phase=-.2;s.end=-.1;
                t.verifyEqual(furuta.ExperimentBuilder.value(s,(0:19)*.073),v.values(:).','AbsTol',1e-12);
            end
        end
        function calibrationAndFilters(t)
            c=furuta.Defaults.profile();furuta.Defaults.validate(c);c.slewPerSecond=0;t.verifyError(@()furuta.Defaults.validate(c),'Furuta:Profile');
            r=furuta.CalibrationService.circularZero([4095 0 1]);t.verifyEqual(r.Raw,0);
            q=furuta.CalibrationService.encoderScale([100;-100],[pi/2;-pi/2]);t.verifyEqual(q.CountsPerRev,400,'AbsTol',1e-10);
            f=furuta.FilterService.design('moving',100,10,3);t.verifyEqual(furuta.FilterService.apply([1;3;5;7],f),[1;2;3;5]);
            f=furuta.FilterService.design('ema',100,10,3);f.alpha=.5;t.verifyEqual(furuta.FilterService.apply([2;4],f),[2;3]);
            f.zeroPhase=true;t.verifyError(@()furuta.FilterService.deployable(f),'Furuta:Filter');
            r=furuta.CalibrationService.zeroCurrent(2500+sin((1:1000)'),(1:1000)'*.001,ones(1000,1)*3200);
            t.verifyTrue(r.Valid);t.verifyLessThan(abs(r.MeanMv-2500),.01);
        end
        function spectraAndSampling(t)
            fs=1000;time=(0:4095)'/fs;x=2*sin(2*pi*(32*fs/4096)*time);
            r=furuta.AnalysisService.spectrum(time,x,struct('Window','rectangular','Segment',1024));
            t.verifyEqual(max(r.Amplitude),2,'AbsTol',1e-10);t.verifyEqual(r.IntegratedVariance,2,'AbsTol',.01);
            r=furuta.AnalysisService.frf(time,x,3*x,512);good=r.Excited;
            t.verifyLessThan(max(abs(abs(r.H(good))-3)),1e-8);t.verifyGreaterThan(min(r.Coherence(good)),.9999);
            time(100:end)=time(100:end)+.01;t.verifyError(@()furuta.AnalysisService.spectrum(time,x),'Furuta:Sampling');
        end
        function arxHeldOutRecovery(t)
            rng(7);u=randn(5000,1);y=filter([0 .1],[1 -.9],u);time=(0:4999)'*.01;
            r=furuta.IdentificationService.arx(time,u,y,1,1,1,.65);
            t.verifyEqual(r.A(2),-.9,'AbsTol',.001);t.verifyEqual(r.B(end),.1,'AbsTol',.001);t.verifyGreaterThan(r.FitPercent,99);
        end
        function physicalPendulumDecay(t)
            time=(0:.01:12)';wn=5;z=.03;x=.12*exp(-z*wn*time).*cos(wn*sqrt(1-z*z)*time);
            r=furuta.IdentificationService.decay(time,x);t.verifyEqual(r.NaturalFrequencyRadS,wn,'RelTol',.01);t.verifyEqual(r.DampingRatio,z,'RelTol',.03);
        end
        function physicalDynamicsEnergy(t)
            p=struct('armInertia',.002,'mass',.03,'armLength',.1,'com',.08,'pendInertiaCom',.000064,...
                'armFriction',0,'pendFriction',0,'R',2,'L',.1,'Ke',0,'Kt',0,'gear',1,'efficiency',1,'busV',12);
            dx=furuta.FurutaModel.derivative([0;pi;0;0;0],0,p);t.verifyLessThan(norm(dx),1e-12);
            time=(0:.002:.5)';X=furuta.FurutaModel.simulate(time,zeros(size(time)),p,[0;pi-.2;0;0;0]);
            J=p.pendInertiaCom+p.mass*p.com^2;A=p.armInertia+p.mass*p.armLength^2;c=p.mass*p.armLength*p.com;
            energy=.5*(A+J*sin(X(:,2)).^2).*X(:,3).^2+c*cos(X(:,2)).*X(:,3).*X(:,4)+.5*J*X(:,4).^2+p.mass*9.80665*p.com*cos(X(:,2));
            t.verifyLessThan(range(energy),1e-8);
        end
        function simulatorWorkflowAndBurst(t)
            s=furuta.SimulatorTransport();guard=onCleanup(@()delete(s)); %#ok<NASGU>
            s.OnPacket=@(p)t.collect(p);
            r=t.command(s,1,'HELLO');t.verifyTrue(r.ok);r=t.command(s,2,'SESSION_OPEN');t.verifyTrue(r.ok);
            r=t.command(s,3,'ARM');t.verifyFalse(r.ok);
            c=furuta.Defaults.profile();c.maxDuty=.2;c.travelCounts=20000;c.currentZeroMv=2500;c.countsPerRev=4400;c.pendDownRaw=2048;
            t.verifyTrue(t.command(s,4,'SET_CONFIG',struct('config',furuta.Defaults.deviceProfile(c))).ok);
            t.verifyTrue(t.command(s,5,'CALIBRATE',struct('target','arm')).ok);
            t.verifyTrue(t.command(s,6,'UPLOAD_BEGIN').ok);
            seg=furuta.ExperimentBuilder.segment('sine',.1,.2,3,3);
            t.verifyTrue(t.command(s,7,'SET_SEGMENT',struct('index',0,'segment',seg)).ok);
            t.verifyTrue(t.command(s,8,'VALIDATE').ok);
            t.verifyTrue(t.command(s,9,'CAPTURE_BURST',struct('seconds',.3,'withRun',true)).ok);
            t.verifyTrue(t.command(s,10,'START_STREAM').ok);t.verifyTrue(t.command(s,11,'ARM').ok);t.verifyTrue(t.command(s,12,'START').ok);
            run=s.RunId;t.command(s,12,'START');t.verifyEqual(s.RunId,run); % Duplicate cannot restart.
            for k=1:6,s.step(.05);t.command(s,12+k,'HEARTBEAT');end
            t.verifyEqual(s.State,'DISARMED');t.verifyEqual(s.Applied,0);t.verifyGreaterThan(s.BurstAdcN,500);
            r=t.command(s,30,'READ_CHUNK',struct('channel','adc','offset',0,'count',16));t.verifyEqual(height(r.Data),16);
            t.verifyGreaterThan(std(double(r.Data.CurrentMv)),0);
            again=t.command(s,30,'READ_CHUNK',struct('channel','adc','offset',0,'count',16));t.verifyEqual(again.Data,r.Data);
            t.command(s,31,'ARM');s.step(.4);t.verifyEqual(s.Fault,'HEARTBEAT_TIMEOUT');
            t.command(s,32,'STOP');t.verifyEqual(s.State,'FAULT');
            t.command(s,33,'CLEAR_FAULT');t.verifyEqual(s.State,'DISARMED');
        end
        function recordingRoundTrip(t)
            s=furuta.SimulatorTransport();clean=onCleanup(@()delete(s)); %#ok<NASGU>
            T=s.rowsToTable([1000 1 -123 4095 3200 2500 2499 0 0 0 0 48 128 1700 15 0 1000 1000 1000 1]);
            base=tempname;mkdir(base);r=furuta.RunRecorder(base,furuta.Defaults.profile(),struct,'Simulator',struct);
            r.append(T);r.finish(true);loaded=furuta.RunRecorder.load(r.Directory);t.verifyEqual(loaded.Samples,T);t.verifyTrue(loaded.Manifest.synthetic);delete(r);
        end
        function guiConstructsDisconnected(t)
            a=FurutaWorkbench('Visible','off');guard=onCleanup(@()delete(a)); %#ok<NASGU>
            t.verifyFalse(a.Session.Connected);t.verifyEqual(a.UI.Mode.Value,'Hardware');
            t.verifyNumElements(a.UI.Tabs.Children,8);a.preview();
        end
        function batchedReceptionAndFraming(t)
            g=jsondecode(fileread(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))),'protocol','golden_vectors.json')));
            p=struct('Payload',uint8(sscanf(g.corePayloadHex,'%2x').'));
            one=furuta.Protocol.samples(p);many=furuta.Protocol.sampleBatches({p,p,p});
            t.verifyEqual(many,[one;one;one]);
            frame=furuta.Protocol.encode(4,uint64(11),uint32(3),p.Payload);
            parser=furuta.Protocol();t.verifyEmpty(parser.feed(frame(1:13)));
            packets=parser.feed([frame(14:end),frame,frame(1:9)]);
            t.verifyNumElements(packets,2);t.verifyEqual(parser.feed(frame(10:end)),packets(1));
            parser.feed([ones(1,2100,'uint8'),uint8(0)]);
            t.verifyNumElements(parser.feed(frame),1);t.verifyGreaterThan(parser.Errors,0);
        end
        function twinPreviewAndMeasuredAngles(t)
            c=furuta.Defaults.profile();s=struct('centered',false,'centerCount','0');
            sample=struct('Validity',uint32(7),'Count',int64(0),'PendRaw',uint16(1000));
            [p,note]=furuta.TwinService.pose(sample,c,s,1000);
            t.verifyEqual(p(3,:),[1 0 -.8],'AbsTol',1e-12);t.verifySubstring(note,'relative preview');
            sample.PendRaw=uint16(2024);moved=furuta.TwinService.pose(sample,c,s,1000);
            t.verifyGreaterThan(norm(moved(3,:)-p(3,:)),.5);
            c.countsPerRev=400;c.pendDownRaw=1000;s.centered=true;sample.Count=int64(100);
            measured=furuta.TwinService.pose(sample,c,s,1000);
            t.verifyEqual(measured(2,:),[0 1 0],'AbsTol',1e-12);
            sample.Validity=uint32(1);invalid=furuta.TwinService.pose(sample,c,s,1000);
            t.verifyTrue(all(isnan(invalid(3,:))));
        end
        function contiguousSelectionPreservesLoss(t)
            time=[(0:9),12:31]'*.001;x=ones(size(time));
            indices=furuta.AnalysisService.longestSpan(time,x);
            t.verifyEqual(indices,(11:30)');t.verifyTrue(furuta.AnalysisService.quality(time(indices),x(indices)).Uniform);
            x(19)=NaN;indices=furuta.AnalysisService.longestSpan(time,x);
            t.verifyEqual(indices,(20:30)');
            t.verifyError(@()furuta.AnalysisService.longestSpan([0;0],[1;1]),'Furuta:Sampling');
        end
    end
end
