classdef WorkbenchApp < handle
    properties
        Figure
        UI = struct
        Session
        RefreshTimer
        Experiment = struct('segments',struct([]))
        LoadedRun = []
        Burst = []
        LastAnalysis = []
        LastModel = []
        LastFilter = []
        Root
        LogLines = {}
        LastConfig = ''
        Closing = false
        Running = false
        FinishAt = []
        DisplayPaused = false
        PendingBurstMeta = []
        Registry = cell(0,5)
        StatusDirty = true
        LogDirty = false
        RefreshBusy = false
        LastSampleKey = ''
        TwinPreviewRaw = NaN
        LastRefreshSeconds = 0
        MaxRefreshSeconds = 0
    end
    methods
        function obj = WorkbenchApp(varargin)
            parser=inputParser;addParameter(parser,'Visible','on');parse(parser,varargin{:});
            obj.Root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
            obj.Session=furuta.DeviceSession();
            obj.Session.OnStatus=@(~)obj.markStatus();obj.Session.OnEvent=@(e)obj.log(e);
            obj.build(parser.Results.Visible);
            obj.Experiment.segments=furuta.ExperimentBuilder.segment('sine',.05,2,1,3,0);
            obj.sequenceTable();c=furuta.Defaults.profile();obj.fillProfile(c);obj.LastConfig=jsonencode(c);
            % Finish expensive initial web-figure layout before opening a port.
            drawnow;
            obj.RefreshTimer=timer('ExecutionMode','fixedSpacing','Period',.2,'BusyMode','drop','TimerFcn',@(~,~)obj.refresh());
            start(obj.RefreshTimer);
            obj.log(struct('info','Disconnected. Choose Simulator to explore; hardware only connects when Connect is clicked.'));
        end
        function safe(obj,fn)
            try,fn();catch ex
                obj.log(struct('error',ex.message));
                if isvalid(obj.Figure)&&strcmp(obj.Figure.Visible,'on'),uialert(obj.Figure,ex.message,'Workbench');end
            end
        end
        function b = button(obj,parent,text,fn)
            b=uibutton(parent,'Text',text,'ButtonPushedFcn',@(~,~)obj.safe(fn));
        end
        function e = number(~,parent,value,limits)
            e=uieditfield(parent,'numeric','Value',value,'Limits',limits);
        end
        function build(obj,visible)
            obj.Figure=uifigure('Name','Furuta Experiment Workbench','Position',[50 40 1460 900],'Visible',visible,...
                'Color',[.96 .97 .98],'CloseRequestFcn',@(~,~)delete(obj));
            root=uigridlayout(obj.Figure,[3 1]);root.RowHeight={94,'1x',42};root.Padding=[12 10 12 10];
            top=uigridlayout(root,[2 10]);top.RowHeight={34,30};top.ColumnWidth={115,100,110,95,110,95,95,110,'1x',165};
            obj.UI.Mode=uidropdown(top,'Items',{'Hardware','Simulator','Replay'},'Value','Hardware',...
                'ValueChangedFcn',@(~,~)obj.safe(@()obj.modeChanged()));
            obj.UI.Port=uidropdown(top,'Items',{'Select port'},'Value','Select port');
            obj.button(top,'Refresh ports',@()obj.refreshPorts());
            obj.UI.Baud=uidropdown(top,'Items',{'460800'},'Value','460800');
            obj.UI.Connect=obj.button(top,'Connect',@()obj.connect());
            obj.UI.Arm=obj.button(top,'Arm',@()obj.send('ARM'));
            obj.UI.Run=obj.button(top,'Run sequence',@()obj.runSequence());
            obj.button(top,'Upload sequence',@()obj.uploadSequence());
            obj.UI.State=uilabel(top,'Text','DISCONNECTED','FontWeight','bold','FontSize',15);
            obj.UI.Stop=obj.button(top,'STOP / DISABLE',@()obj.stop());obj.UI.Stop.BackgroundColor=[.75 .12 .13];obj.UI.Stop.FontColor=[1 1 1];
            obj.UI.Arm.Enable='off';obj.UI.Run.Enable='off';
            obj.UI.Source=uidropdown(top,'Items',{'Live buffer','Loaded run','ADC burst','Mechanical burst'},'Value','Live buffer');
            obj.UI.Channel=uidropdown(top,'Items',{'CurrentMv','CurrentA','FilteredMv','ArmCounts','CountVelocity','ArmRad','PendRaw','PendRad','DownRad','ArmVelocity','PendVelocity','Requested','Applied'},'Value','CurrentMv');
            uilabel(top,'Text','From / to (s)');
            obj.UI.Start=obj.number(top,0,[0 Inf]);obj.UI.End=obj.number(top,1e9,[0 Inf]);
            obj.UI.Pause=uicheckbox(top,'Text','Pause view','ValueChangedFcn',@(s,~)obj.pauseView(s.Value));
            obj.button(top,'Record',@()obj.record());
            obj.button(top,'Finish recording',@()obj.finishRecord());
            obj.UI.Health=uilabel(top,'Text','Raw data + warnings preserved','FontColor',[.45 .25 .05]);
            obj.UI.Health.Layout.Column=[9 10];
            obj.UI.Tabs=uitabgroup(root);
            obj.liveTab();obj.experimentTab();obj.calibrationTab();obj.timeTab();obj.spectrumTab();obj.filterTab();obj.identificationTab();obj.settingsTab();
            axes=findall(obj.Figure,'Type','axes');
            for ax=axes(:).',ax.NextPlot='replacechildren';end
            for ax=[obj.UI.LiveAxes,obj.UI.CommandAxes,obj.UI.TwinAxes]
                disableDefaultInteractivity(ax);ax.Toolbar.Visible='off';
            end
            obj.UI.Bottom=uilabel(root,'Text','Outputs require explicit commissioned limits, an arm reference, Arm and Run.','FontSize',12,'WordWrap','on');
        end
        function liveTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Live & Manual');g=uigridlayout(tab,[2 3]);g.ColumnWidth={310,'1x','1x'};g.RowHeight={'1x','1x'};
            p=uipanel(g,'Title','Manual finite command');p.Layout.Row=[1 2];f=uigridlayout(p,[13 2]);
            f.RowHeight={24,30,30,30,32,32,32,32,32,32,'1x',24,24};f.ColumnWidth={'1x','1x'};
            uilabel(f,'Text','Signed duty (%)');obj.UI.ManualDuty=obj.number(f,0,[-100 100]);
            uilabel(f,'Text','Duration (s)');obj.UI.ManualDuration=obj.number(f,.25,[.01 10]);
            obj.button(f,'Apply timed command',@()obj.manual());obj.button(f,'Disarm',@()obj.send('DISARM'));
            obj.button(f,'Center arm here',@()obj.calibrate('arm'));obj.button(f,'Current zero (1 s)',@()obj.calibrate('current'));
            obj.button(f,'Pendulum down here',@()obj.calibrate('down'));obj.button(f,'Clear fault',@()obj.send('CLEAR_FAULT'));
            obj.button(f,'Simulator setup',@()obj.setupSimulator());obj.button(f,'Export data',@()obj.exportCSV());
            obj.UI.Values=uitable(f,'ColumnName',{'Signal','Value'},'Data',cell(0,2));obj.UI.Values.Layout.Row=[7 11];obj.UI.Values.Layout.Column=[1 2];
            note=uilabel(f,'Text','Duty is not torque. Stop disables drive; the mechanism may coast.','WordWrap','on');note.Layout.Row=[12 13];note.Layout.Column=[1 2];
            obj.UI.LiveAxes=uiaxes(g);obj.UI.LiveAxes.Layout.Row=1;obj.UI.LiveAxes.Layout.Column=[2 3];
            title(obj.UI.LiveAxes,'Selected sensor');xlabel(obj.UI.LiveAxes,'Device-relative time (s)');grid(obj.UI.LiveAxes,'on');
            obj.UI.LiveLine=plot(obj.UI.LiveAxes,nan,nan,'Color',[.05 .35 .65]);
            obj.UI.FilterLine=line(obj.UI.LiveAxes,nan,nan,'Color',[.85 .4 .08]);
            obj.UI.CommandAxes=uiaxes(g);obj.UI.CommandAxes.Layout.Row=2;obj.UI.CommandAxes.Layout.Column=2;
            obj.UI.RequestLine=plot(obj.UI.CommandAxes,nan,nan);obj.UI.AppliedLine=line(obj.UI.CommandAxes,nan,nan,'Color',[.85 .35 .1]);
            title(obj.UI.CommandAxes,'Requested / applied duty');xlabel(obj.UI.CommandAxes,'Time (s)');ylabel(obj.UI.CommandAxes,'Normalized duty');grid(obj.UI.CommandAxes,'on');
            twin=uigridlayout(g,[2 1]);twin.Layout.Row=2;twin.Layout.Column=3;twin.RowHeight={'1x',65};
            obj.UI.TwinAxes=obj.analysisAxes(twin,1,1);
            obj.UI.TwinLine=plot3(obj.UI.TwinAxes,[0 1 1],[0 0 0],[0 0 -.8],'-o','LineWidth',3);
            view(obj.UI.TwinAxes,35,25);axis(obj.UI.TwinAxes,[-2 2 -2 2 -1.5 1.5]);grid(obj.UI.TwinAxes,'on');title(obj.UI.TwinAxes,'Measured-angle schematic (unit lengths)');
            obj.UI.TwinStatus=uilabel(twin,'Text','Connect to see sensors. Arm animation needs counts per output revolution and a center reference.','WordWrap','on','FontColor',[.5 .28 .04]);
        end
        function experimentTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Experiments');g=uigridlayout(tab,[2 2]);g.ColumnWidth={390,'1x'};g.RowHeight={'1x','1x'};
            f=uigridlayout(g,[13 2]);f.Layout.Row=[1 2];f.RowHeight=[repmat({30},1,11),{'1x',34}];
            uilabel(f,'Text','Waveform');obj.UI.Kind=uidropdown(f,'Items',{'hold','step','pulse','doublet','ramp','triangle','sine','square','chirp','multisine','prbs','arbitrary'},'Value','sine');
            uilabel(f,'Text','Amplitude / offset (%)');a=uigridlayout(f,[1 2]);a.Padding=0;obj.UI.Amplitude=obj.number(a,5,[-100 100]);obj.UI.Offset=obj.number(a,0,[-100 100]);
            uilabel(f,'Text','Duration (s)');obj.UI.Duration=obj.number(f,2,[.001 120]);
            uilabel(f,'Text','Frequency start / end Hz');a=uigridlayout(f,[1 2]);a.Padding=0;obj.UI.F0=obj.number(a,1,[.001 99]);obj.UI.F1=obj.number(a,3,[.001 99]);
            uilabel(f,'Text','Repeat count / rest (s)');a=uigridlayout(f,[1 2]);a.Padding=0;obj.UI.Repeats=obj.number(a,1,[1 8]);obj.UI.Rest=obj.number(a,0,[0 60]);
            uilabel(f,'Text','Pre / post baseline (s)');a=uigridlayout(f,[1 2]);a.Padding=0;obj.UI.Pre=obj.number(a,.5,[0 30]);obj.UI.Post=obj.number(a,.5,[0 30]);
            obj.button(f,'Replace sequence',@()obj.editSequence('replace'));obj.button(f,'Append segment',@()obj.editSequence('append'));
            obj.button(f,'Remove last',@()obj.editSequence('remove'));obj.button(f,'Preview',@()obj.preview());
            obj.button(f,'Import time/duty CSV',@()obj.importWave());obj.button(f,'Export sequence JSON',@()obj.exportExperiment());
            obj.button(f,'Import sequence JSON',@()obj.importExperiment());obj.button(f,'Load selected segment JSON',@()obj.segmentJson());
            uilabel(f,'Text','Advanced segment JSON (phase, duty, tones, PRBS, sweep, interpolation)');
            obj.button(f,'Apply JSON as segment',@()obj.applySegmentJson());
            obj.UI.SegmentJSON=uitextarea(f,'Value',{'{}'});obj.UI.SegmentJSON.Layout.Row=12;obj.UI.SegmentJSON.Layout.Column=[1 2];
            obj.button(f,'Upload and validate',@()obj.uploadSequence());obj.button(f,'Record + run',@()obj.recordRun());
            obj.UI.PreviewAxes=uiaxes(g);obj.UI.PreviewAxes.Layout.Column=2;obj.UI.PreviewAxes.Layout.Row=1;
            obj.UI.SequenceTable=uitable(g,'ColumnName',{'Kind','Duration s','Amplitude','Offset','f0 Hz','f1 Hz'},'Data',cell(0,6));
            obj.UI.SequenceTable.Layout.Column=2;obj.UI.SequenceTable.Layout.Row=2;
        end
        function calibrationTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Calibration');g=uigridlayout(tab,[3 2]);g.RowHeight={90,'1x','1x'};
            f=uigridlayout(g,[2 4]);f.Layout.Column=[1 2];
            obj.button(f,'Set arm center here',@()obj.calibrate('arm'));obj.button(f,'Set pendulum hanging down',@()obj.calibrate('down'));
            obj.button(f,'Set pendulum upright',@()obj.calibrate('upright'));obj.button(f,'Zero current (1 s)',@()obj.calibrate('current'));
            obj.button(f,'Read device profile',@()obj.readProfile());obj.button(f,'Save profile to device NVS',@()obj.send('SAVE_CONFIG'));
            obj.button(f,'Analyze selected zero window',@()obj.zeroWindow());obj.button(f,'Export profile JSON',@()obj.exportProfile());
            p=uipanel(g,'Title','Encoder scale: delta counts versus known delta angle (degrees)');a=uigridlayout(p,[2 1]);a.RowHeight={'1x',32};
            obj.UI.ScalePoints=uitable(a,'ColumnName',{'Delta counts','Known degrees'},'Data',zeros(4,2),'ColumnEditable',[true true]);
            obj.button(a,'Fit counts per output revolution',@()obj.scaleCalibration());
            p=uipanel(g,'Title','Current gain: externally known signed current / measured mean mV');a=uigridlayout(p,[2 1]);a.RowHeight={'1x',32};
            obj.UI.GainPoints=uitable(a,'ColumnName',{'Known A','Mean mV'},'Data',[0 2500;0 2500;0 2500;0 2500],'ColumnEditable',[true true]);
            obj.button(a,'Fit gain and offset',@()obj.gainCalibration());
            obj.UI.CalibrationReport=uitextarea(g,'Editable','off','Value',{'No calibrated accuracy is assumed. Weak-field data remains qualified.'});obj.UI.CalibrationReport.Layout.Row=3;obj.UI.CalibrationReport.Layout.Column=[1 2];
        end
        function timeTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Time Analysis');g=uigridlayout(tab,[3 1]);g.RowHeight={40,'1x',150};
            f=uigridlayout(g,[1 8]);f.ColumnWidth={120,110,110,120,100,100,100,'1x'};
            obj.button(f,'Plot + statistics',@()obj.timeAnalysis());obj.button(f,'Overlay saved run',@()obj.overlayRun());
            obj.button(f,'Autocorrelation',@()obj.autocorrelation());obj.button(f,'Step metrics',@()obj.stepMetrics());
            obj.UI.Baseline=obj.number(f,0,[-Inf Inf]);obj.UI.Target=obj.number(f,1,[-Inf Inf]);
            obj.UI.Detrend=uicheckbox(f,'Text','Remove mean');obj.button(f,'Export last analysis',@()obj.exportAnalysis());
            obj.UI.TimeAxes=obj.analysisAxes(g,2,1);
            obj.UI.TimeReport=uitextarea(g,'Editable','off');obj.UI.TimeReport.Layout.Row=3;
        end
        function spectrumTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Spectrum & Noise');g=uigridlayout(tab,[3 1]);g.RowHeight={70,'1x',140};
            f=uigridlayout(g,[2 7]);f.RowHeight={28,28};
            obj.UI.SpectrumMode=uidropdown(f,'Items',{'FFT amplitude','Welch PSD','ASD','Spectrogram','FRF + coherence'},'Value','Welch PSD');
            obj.UI.Window=uidropdown(f,'Items',{'hann','rectangular'});
            obj.UI.SegmentLength=obj.number(f,1024,[8 65536]);obj.UI.Nfft=obj.number(f,0,[0 1048576]);
            obj.UI.SegmentLength.Tooltip='Welch segment length in samples';obj.UI.Nfft.Tooltip='FFT length (0 selects record/segment length)';
            obj.UI.Resample=uicheckbox(f,'Text','Resample small jitter');obj.button(f,'Analyze',@()obj.spectrum());
            obj.UI.RemoveMean=uicheckbox(f,'Text','Remove mean','Value',true);
            obj.UI.BandLow=obj.number(f,0,[0 Inf]);obj.UI.BandHigh=obj.number(f,50,[0 Inf]);
            obj.UI.BandLow.Tooltip='Band lower edge in Hz';obj.UI.BandHigh.Tooltip='Band upper edge in Hz';
            obj.button(f,'Band RMS',@()obj.bandPower());obj.button(f,'Export plot',@()obj.exportPlot(obj.UI.SpectrumAxes));
            uilabel(f,'Text','Carrier > Nyquist is not observable.');
            obj.button(f,'Inspect timing',@()obj.inspectTiming());obj.button(f,'Use contiguous span',@()obj.selectContiguous());
            obj.UI.SpectrumAxes=obj.analysisAxes(g,2,1);
            obj.UI.SpectrumReport=uitextarea(g,'Editable','off');obj.UI.SpectrumReport.Layout.Row=3;
        end
        function filterTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Filters');g=uigridlayout(tab,[3 2]);g.RowHeight={110,'1x',120};
            f=uigridlayout(g,[3 5]);f.Layout.Column=[1 2];f.RowHeight={26,26,26};
            obj.UI.FilterType=uidropdown(f,'Items',{'bypass','ema','moving','butter','notch'});
            obj.UI.Cutoff=obj.number(f,10,[.001 Inf]);obj.UI.FilterOrder=obj.number(f,8,[1 256]);
            obj.UI.Cutoff.Tooltip='Low-pass cutoff or notch center frequency (Hz)';obj.UI.FilterOrder.Tooltip='Moving-average window or notch Q (Butterworth order is 2)';
            obj.UI.ZeroPhase=uicheckbox(f,'Text','Offline zero phase');obj.button(f,'Design + compare',@()obj.filterPreview());
            uilabel(f,'Text','Cutoff/center Hz; window or notch Q');
            obj.UI.LiveFilter=uicheckbox(f,'Text','Display filter in Live');
            obj.button(f,'Deploy causal filter to MCU',@()obj.deployFilter());obj.button(f,'Export filter',@()obj.exportFilter());
            obj.UI.FilterView=uidropdown(f,'Items',{'Gain / delay','Gain / phase','Raw / filtered PSD'});
            uilabel(f,'Text','Design rate Hz (0 = measured)');obj.UI.FilterSampleRate=obj.number(f,0,[0 100000]);
            obj.UI.FilterResample=uicheckbox(f,'Text','Resample small jitter');
            obj.UI.FilterSampleRate.Tooltip='For MCU deployment use adcHz (normally 2000) and explicitly enable resampling. Gaps must first be excluded.';
            obj.UI.FilterAxes=obj.analysisAxes(g,2,1);
            obj.UI.FilterResponseAxes=obj.analysisAxes(g,2,2);
            obj.UI.FilterReport=uitextarea(g,'Editable','off');obj.UI.FilterReport.Layout.Row=3;obj.UI.FilterReport.Layout.Column=[1 2];
        end
        function identificationTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Identification');g=uigridlayout(tab,[3 2]);g.RowHeight={95,'1x',200};g.ColumnWidth={'1x',430};
            f=uigridlayout(g,[2 8]);f.Layout.Column=[1 2];
            obj.UI.Method=uidropdown(f,'Items',{'ARX','Motor speed PT1','tfest','n4sid','ssest','Pendulum decay','Physical nonlinear'},'Value','ARX');
            obj.UI.Na=obj.number(f,1,[1 20]);obj.UI.Nb=obj.number(f,1,[1 20]);obj.UI.Nk=obj.number(f,1,[0 50]);
            obj.UI.Split=obj.number(f,.7,[.3 .9]);obj.UI.ModelOrder=obj.number(f,2,[1 12]);
            obj.button(f,'Estimate + validate',@()obj.identify());obj.button(f,'Export model MAT',@()obj.exportModel());
            uilabel(f,'Text','na / nb / delay; training fraction; model order');
            obj.UI.ModelView=uidropdown(f,'Items',{'Measured / model','Bode','Step','Residual'});
            obj.button(f,'View model',@()obj.viewModel());obj.button(f,'Export parameter registry',@()obj.exportRegistry());
            obj.UI.FreeParams=uieditfield(f,'text','Value','armFriction,pendFriction');obj.UI.FreeParams.Layout.Column=[5 8];
            obj.UI.ModelAxes=obj.analysisAxes(g,2,1);
            p=uipanel(g,'Title','Physical model: measured/fixed parameters JSON');p.Layout.Row=2;p.Layout.Column=2;
            a=uigridlayout(p,[2 1]);a.RowHeight={'1x',90};
            obj.UI.Physics=uitextarea(a,'Value',{jsonencode(struct('armInertia',[],'mass',[],'armLength',[],'com',[],'pendInertiaCom',[],...
                'armFriction',[],'pendFriction',[],'R',[],'L',[],'Ke',[],'Kt',[],'gear',[],'efficiency',[],'busV',12))});
            obj.UI.Bounds=uitable(a,'ColumnName',{'Parameter','Lower','Upper'},'Data',{'armFriction',.000001,1;'pendFriction',.000001,1},'ColumnEditable',[true true true]);
            obj.UI.ModelReport=uitextarea(g,'Editable','off');obj.UI.ModelReport.Layout.Row=3;obj.UI.ModelReport.Layout.Column=1;
            obj.UI.Registry=uitable(g,'ColumnName',{'Parameter','Value','Units','Source','Quality'},'Data',obj.Registry);obj.UI.Registry.Layout.Row=3;obj.UI.Registry.Layout.Column=2;
        end
        function settingsTab(obj)
            tab=uitab(obj.UI.Tabs,'Title','Runs & Settings');g=uigridlayout(tab,[3 2]);g.RowHeight={75,'1x',180};
            f=uigridlayout(g,[2 5]);f.Layout.Column=[1 2];
            obj.button(f,'Apply profile to RAM',@()obj.applyProfile());obj.button(f,'Read profile',@()obj.readProfile());
            obj.button(f,'Import profile JSON',@()obj.importProfile());obj.button(f,'Export profile JSON',@()obj.exportProfile());obj.button(f,'Save to device NVS',@()obj.send('SAVE_CONFIG'));
            obj.button(f,'Open saved run',@()obj.openRun(false));obj.button(f,'Replay saved run',@()obj.openRun(true));obj.button(f,'Save selected CSV',@()obj.exportCSV());
            obj.button(f,'Save burst MAT',@()obj.saveBurst());obj.button(f,'Guide / help',@()obj.showGuide());
            obj.UI.Profile=uitable(g,'ColumnName',{'Field','Value','Units / meaning'},'ColumnEditable',[false true false],'ColumnFormat',{'char','numeric','char'});
            obj.UI.Profile.Layout.Row=2;obj.UI.Profile.Layout.Column=1;
            p=uipanel(g,'Title','Capture and simulated fault controls');p.Layout.Row=2;p.Layout.Column=2;
            a=uigridlayout(p,[8 2]);a.RowHeight={30,32,32,32,32,32,'1x',24};
            uilabel(a,'Text','Raw current capture (s)');obj.UI.BurstDuration=obj.number(a,2,[.05 120]);
            obj.button(a,'Capture passive burst',@()obj.captureBurst(false));obj.button(a,'Prepare burst with next Run',@()obj.captureBurst(true));
            obj.button(a,'Download completed burst',@()obj.downloadBurst());obj.button(a,'Select ADC data',@()obj.selectBurst());
            obj.UI.SimFault=uidropdown(a,'Items',{'weak field','healthy field','lost pendulum','drop packets'});obj.button(a,'Apply simulator condition',@()obj.simFault());
            uilabel(a,'Text','Simulator ADC noise SD (mV)');obj.UI.SimNoise=obj.number(a,5,[0 100]);
            obj.button(a,'Apply simulator noise',@()obj.simNoise());obj.button(a,'Stop replay / resume',@()obj.replayPause());
            obj.UI.CaptureStatus=uitextarea(a,'Editable','off');obj.UI.CaptureStatus.Layout.Row=[7 8];obj.UI.CaptureStatus.Layout.Column=[1 2];
            obj.UI.Log=uitextarea(g,'Editable','off');obj.UI.Log.Layout.Row=3;obj.UI.Log.Layout.Column=[1 2];
        end
        function refreshPorts(obj)
            ports=cellstr(serialportlist('available'));if isempty(ports),ports={'No available ports'};end;obj.UI.Port.Items=ports;
        end
        function ax = analysisAxes(~,parent,row,column)
            panel=uipanel(parent,'BorderType','none');panel.Layout.Row=row;panel.Layout.Column=column;
            ax=uiaxes(panel,'Units','normalized','OuterPosition',[0 0 1 1]);
        end
        function modeChanged(obj)
            if obj.Session.Connected,obj.Session.disconnect();end
            obj.TwinPreviewRaw=NaN;obj.LastSampleKey='';
            obj.LastConfig='';obj.UI.Connect.Text='Connect';obj.statusChanged();
        end
        function connect(obj)
            obj.TwinPreviewRaw=NaN;obj.LastSampleKey='';
            if obj.Session.Connected,obj.Session.disconnect();obj.UI.Connect.Text='Connect';return;end
            mode=obj.UI.Mode.Value;
            if strcmp(mode,'Replay'),obj.openRun(true);return;end
            obj.LastConfig='';obj.Session.connect(mode,obj.UI.Port.Value,str2double(obj.UI.Baud.Value));obj.UI.Connect.Text='Disconnect';
        end
        function send(obj,op)
            if strcmp(obj.Session.Mode,'Replay'),error('Furuta:Replay','Replay cannot arm or command a motor');end
            obj.Session.request(op,struct,[]);
        end
        function stop(obj)
            if ~obj.Session.Connected,return;end
            if strcmp(obj.Session.Mode,'Replay'),obj.Session.Transport.Playing=false;return;end
            obj.Session.request('STOP',struct,[]);obj.Running=false;
        end
        function requireDisarmed(obj)
            assert(obj.Session.Ready&&strcmp(obj.Session.Status.state,'DISARMED'),'Furuta:State','Connect and disarm first');
        end
        function setupSimulator(obj)
            assert(strcmp(obj.Session.Mode,'Simulator'),'Furuta:Simulator','This example profile is only for the simulator');
            obj.requireDisarmed();c=obj.Session.Config;c.maxDuty=.2;c.travelCounts=20000;c.countsPerRev=4400;
            c.currentZeroMv=2500;c.pendDownRaw=2048;c.maxDuration=30;
            obj.Session.applyConfig(c,@(r)obj.afterSimSetup(r));
        end
        function afterSimSetup(obj,r)
            if r.ok,obj.Session.request('CALIBRATE',struct('target','arm'),[]);obj.log(struct('info','Synthetic example profile applied. No motion started.'));end
        end
        function e = executionExperiment(obj)
            base=obj.Experiment.segments;segments=struct([]);
            if obj.UI.Pre.Value>0,segments=furuta.ExperimentBuilder.segment('hold',0,obj.UI.Pre.Value);end
            for k=1:round(obj.UI.Repeats.Value)
                segments=[segments,base]; %#ok<AGROW>
                if k<round(obj.UI.Repeats.Value)&&obj.UI.Rest.Value>0,segments=[segments,furuta.ExperimentBuilder.segment('hold',0,obj.UI.Rest.Value)];end %#ok<AGROW>
            end
            if obj.UI.Post.Value>0,segments=[segments,furuta.ExperimentBuilder.segment('hold',0,obj.UI.Post.Value)];end
            e=struct('segments',segments);
        end
        function editSequence(obj,action)
            s=furuta.ExperimentBuilder.segment(obj.UI.Kind.Value,obj.UI.Amplitude.Value/100,obj.UI.Duration.Value,obj.UI.F0.Value,obj.UI.F1.Value,obj.UI.Offset.Value/100);
            switch action
                case 'replace',obj.Experiment.segments=s;
                case 'append',obj.Experiment.segments=[obj.Experiment.segments,s];
                case 'remove',if ~isempty(obj.Experiment.segments),obj.Experiment.segments(end)=[];end
            end
            obj.sequenceTable();obj.preview();
        end
        function sequenceTable(obj)
            rows=cell(numel(obj.Experiment.segments),6);
            for k=1:numel(obj.Experiment.segments)
                s=obj.Experiment.segments(k);rows(k,:)={s.kind,s.duration,s.amplitude,s.offset,s.f0,s.f1};
            end
            obj.UI.SequenceTable.Data=rows;
        end
        function preview(obj)
            e=obj.executionExperiment();assert(~isempty(e.segments),'Furuta:Experiment','Sequence is empty');
            [t,y,z]=furuta.ExperimentBuilder.preview(e,obj.Session.Config);
            plot(obj.UI.PreviewAxes,t,y,t,z,'LineWidth',1.2);legend(obj.UI.PreviewAxes,{'Requested','Limited preview'});grid(obj.UI.PreviewAxes,'on');
            xlabel(obj.UI.PreviewAxes,'Device-relative time (s)');ylabel(obj.UI.PreviewAxes,'Normalized duty');title(obj.UI.PreviewAxes,'Device scheduling; limits and reversal blanking applied');
        end
        function segmentJson(obj)
            assert(~isempty(obj.Experiment.segments),'Furuta:Experiment','No segment');obj.UI.SegmentJSON.Value={jsonencode(obj.Experiment.segments(end),'PrettyPrint',true)};
        end
        function applySegmentJson(obj)
            s=jsondecode(strjoin(obj.UI.SegmentJSON.Value,newline));def=furuta.ExperimentBuilder.segment('hold',0,1);
            assert(isstruct(s)&&isscalar(s),'Furuta:Experiment','One segment object required');
            for f=fieldnames(s).',assert(isfield(def,f{1}),'Furuta:Experiment','Unknown field');def.(f{1})=s.(f{1});end
            c=obj.Session.Config;c.maxDuty=1;c.maxDuration=120;furuta.ExperimentBuilder.validate(struct('segments',def),c);
            obj.Experiment.segments=[obj.Experiment.segments,def];obj.sequenceTable();obj.preview();
        end
        function uploadSequence(obj)
            obj.requireDisarmed();obj.Session.upload(obj.executionExperiment(),@(r)obj.uploaded(r));
        end
        function uploaded(obj,r)
            if r.ok,obj.log(struct('info','Experiment uploaded and validated. Arm, then Run.'));else,error('Furuta:Upload','%s',r.error);end
        end
        function runSequence(obj)
            assert(obj.Session.Ready&&strcmp(obj.Session.Status.state,'ARMED_IDLE'),'Furuta:State','Upload sequence, then Arm, then Run');
            obj.Session.request('START',struct,@(r)obj.started(r));
        end
        function started(obj,r)
            if r.ok,obj.Running=true;obj.FinishAt=[];else,obj.log(r);end
        end
        function recordRun(obj)
            obj.requireDisarmed();e=obj.executionExperiment();furuta.ExperimentBuilder.validate(e,obj.Session.Config);
            obj.Session.beginRecord(fullfile(obj.Root,'data'),e);
            obj.Session.upload(e,@(r)obj.runUploaded(r));
        end
        function runUploaded(obj,r)
            if ~r.ok,obj.Session.finishRecord();obj.log(r);return;end
            obj.Session.request('ARM',struct,@(a)obj.runArmed(a));
        end
        function runArmed(obj,r)
            if ~r.ok,obj.Session.finishRecord();obj.log(r);return;end
            obj.Session.request('START',struct,@(s)obj.started(s));
        end
        function manual(obj)
            obj.requireDisarmed();s=furuta.ExperimentBuilder.segment('hold',obj.UI.ManualDuty.Value/100,obj.UI.ManualDuration.Value);
            obj.Session.upload(struct('segments',s),@(r)obj.runUploaded(r));
        end
        function record(obj)
            assert(obj.Session.Ready&&~strcmp(obj.Session.Mode,'Replay'),'Furuta:Recording','Connect a device/simulator first');
            obj.Session.beginRecord(fullfile(obj.Root,'data'),obj.executionExperiment());
        end
        function finishRecord(obj),obj.Session.finishRecord();obj.log(struct('info','Recording finalized.'));end
        function calibrate(obj,target)
            obj.requireDisarmed();
            if ismember(target,{'down','upright'})
                T=obj.Session.Data;assert(height(T)>20,'Furuta:Calibration','Wait for fresh sensor samples');
                T=T(max(1,end-249):end,:);T=T(bitand(T.Validity,2)~=0,:);
                r=furuta.CalibrationService.circularZero(T.PendRaw);
                assert(r.SdRad<.03,'Furuta:Calibration','Hold pendulum still before setting zero');
                c=obj.Session.Config;c.pendDownRaw=r.Raw;if strcmp(target,'upright'),c.pendDownRaw=mod(c.pendDownRaw+2048,4096);end
                obj.Session.applyConfig(c,[]);obj.report(obj.UI.CalibrationReport,r);
            else,obj.Session.request('CALIBRATE',struct('target',target),@(r)obj.afterCalibration(r));end
        end
        function afterCalibration(obj,r)
            if r.ok,obj.Session.request('GET_CONFIG',struct,@(a)obj.Session.gotConfig(a));end
        end
        function zeroWindow(obj)
            [t,x,~,T]=obj.dataset();assert(ismember('CurrentMv',T.Properties.VariableNames),'Furuta:Data','No ADC voltage');
            r=furuta.CalibrationService.zeroCurrent(T.CurrentMv,t,T.AdcRaw);obj.report(obj.UI.CalibrationReport,r);
            obj.LastAnalysis=r;
        end
        function scaleCalibration(obj)
            obj.requireDisarmed();a=obj.UI.ScalePoints.Data;r=furuta.CalibrationService.encoderScale(a(:,1),a(:,2)*pi/180);
            c=obj.Session.Config;c.countsPerRev=r.CountsPerRev;c.armSign=r.Sign;obj.Session.applyConfig(c,[]);
            obj.report(obj.UI.CalibrationReport,r);obj.register('countsPerRev',r.CountsPerRev,'counts/rev','known displacement','fit');
        end
        function gainCalibration(obj)
            obj.requireDisarmed();a=obj.UI.GainPoints.Data;r=furuta.CalibrationService.gain(a(:,1),a(:,2));
            assert(r.SensitivityMvA>0,'Furuta:Calibration','Check signed-current convention');
            c=obj.Session.Config;c.currentZeroMv=r.ZeroMv;c.sensitivityMvA=r.SensitivityMvA;obj.Session.applyConfig(c,[]);
            obj.report(obj.UI.CalibrationReport,r);obj.register('sensitivityMvA',r.SensitivityMvA,'mV/A','external current reference','fit');
        end
        function fillProfile(obj,c)
            names={'maxDuty','slewPerSecond','maxDuration','travelCounts','velocityCountsPerSecond','currentTripA','currentTripMs','currentZeroMv','sensitivityMvA','countsPerRev','armSign','pendSign','pendDownRaw','assumedBusV','heartbeatMs','streamHz','adcHz','pwmHz'};
            units={'fraction (0 disables arm)','duty/s','s','counts from center','counts/s (0 off)','A (0 off)','ms','mV','mV/A','counts/output rev','+1 or -1','+1 or -1','raw 0..4095','V assumed, not measured','ms','Hz (50/100/250)','Hz, max 2000','Hz'};
            rows=cell(numel(names),3);
            for k=1:numel(names),v=c.(names{k});if isempty(v),v=NaN;end;rows(k,:)={names{k},v,units{k}};end
            obj.UI.Profile.Data=rows;
        end
        function c = profileFromUI(obj)
            c=obj.Session.Config;a=obj.UI.Profile.Data;
            for k=1:size(a,1),v=a{k,2};if ischar(v)||isstring(v),v=str2double(v);end;c.(a{k,1})=v;end
        end
        function applyProfile(obj),obj.requireDisarmed();obj.Session.applyConfig(obj.profileFromUI(),[]);end
        function readProfile(obj),obj.Session.request('GET_CONFIG',struct,@(r)obj.Session.gotConfig(r));end
        function exportProfile(obj)
            [f,p]=uiputfile('*.json');if isequal(f,0),return;end;obj.writeJSON(fullfile(p,f),obj.profileFromUI());
        end
        function importProfile(obj)
            [f,p]=uigetfile('*.json');if isequal(f,0),return;end;c=jsondecode(fileread(fullfile(p,f)));
            defaults=furuta.Defaults.profile();for k=fieldnames(c).',if isfield(defaults,k{1}),defaults.(k{1})=c.(k{1});end,end
            obj.fillProfile(defaults);obj.log(struct('info','Profile loaded into editor only. Apply deliberately while disarmed; re-center after boot.'));
        end
        function importWave(obj)
            [f,p]=uigetfile('*.csv');if isequal(f,0),return;end;a=readmatrix(fullfile(p,f));a=a(all(isfinite(a(:,1:2)),2),1:2);
            assert(size(a,1)>=2&&all(diff(a(:,1))>0)&&max(abs(a(:,2)))<=1,'Furuta:Waveform','CSV must contain ordered time_s and normalized duty');
            dt=median(diff(a(:,1)));assert(max(abs(diff(a(:,1))-dt))<.01*dt,'Furuta:Waveform','Uniform sample times required; resample explicitly first');
            s=furuta.ExperimentBuilder.segment('arbitrary',0,size(a,1)*dt);s.table=a(:,2).';s.tablePeriod=dt;
            obj.Experiment.segments=[obj.Experiment.segments,s];obj.sequenceTable();obj.preview();
        end
        function exportExperiment(obj)
            [f,p]=uiputfile('*.json');if isequal(f,0),return;end;obj.writeJSON(fullfile(p,f),obj.Experiment);
        end
        function importExperiment(obj)
            [f,p]=uigetfile('*.json');if isequal(f,0),return;end;e=jsondecode(fileread(fullfile(p,f)));
            c=obj.Session.Config;c.maxDuty=1;c.maxDuration=120;furuta.ExperimentBuilder.validate(e,c);
            obj.Experiment=e;obj.sequenceTable();obj.preview();
        end
        function [t,x,units,T,c] = dataset(obj)
            source=obj.UI.Source.Value;center=int64(0);c=obj.Session.Config;
            switch source
                case 'Loaded run'
                    assert(~isempty(obj.LoadedRun),'Furuta:Data','Open a saved run first');T=obj.LoadedRun.Samples;c=obj.LoadedRun.Config;
                    if isfield(obj.LoadedRun.Manifest.device,'status'),center=obj.centerCount(obj.LoadedRun.Manifest.device.status);end
                    if ~isempty(T)&&isfield(obj.LoadedRun,'Configurations')
                        tt=double(T.DeviceUs-T.DeviceUs(1))*1e-6;ids=unique(T.ConfigId(tt>=obj.UI.Start.Value&tt<=obj.UI.End.Value));
                        if numel(ids)==1
                            key=sprintf('v%u',ids);
                            if isfield(obj.LoadedRun.Configurations,key)
                                snapshot=obj.LoadedRun.Configurations.(key);c=snapshot.Config;center=obj.centerCount(snapshot.Status);
                            end
                        end
                    end
                case 'ADC burst'
                    assert(~isempty(obj.Burst)&&~isempty(obj.Burst.ADC),'Furuta:Data','Download an ADC burst first');T=obj.Burst.ADC;
                    if isfield(obj.Burst,'Config'),c=obj.Burst.Config;end
                    T.Time=double(T.DeviceUs-T.DeviceUs(1))*1e-6;
                    T.CurrentA=nan(height(T),1);if isfinite(c.currentZeroMv),T.CurrentA=(double(T.CurrentMv)-c.currentZeroMv)/c.sensitivityMvA;end
                case 'Mechanical burst'
                    assert(~isempty(obj.Burst)&&~isempty(obj.Burst.Core),'Furuta:Data','Download a mechanical burst first');T=obj.Burst.Core;
                    if isfield(obj.Burst,'Config'),c=obj.Burst.Config;center=obj.Burst.Center;end
                otherwise
                    T=obj.Session.Data;center=obj.centerCount(obj.Session.Status);
            end
            assert(~isempty(T),'Furuta:Data','No samples yet');
            if ~strcmp(source,'ADC burst'),T=furuta.CalibrationService.derive(T,c,center);end
            t=T.Time;sel=t>=obj.UI.Start.Value&t<=obj.UI.End.Value;T=T(sel,:);t=T.Time;
            name=obj.UI.Channel.Value;assert(ismember(name,T.Properties.VariableNames),'Furuta:Channel','Channel unavailable in selected source');
            x=double(T.(name));
            switch name
                case {'CurrentMv','FilteredMv'},units='mV';
                case {'CurrentA'},units='A (estimated)';
                case {'ArmRad','PendRad','DownRad'},units='rad';
                case {'ArmVelocity','PendVelocity'},units='rad/s';
                case 'CountVelocity',units='counts/s';
                case {'Requested','Applied'},units='normalized duty';
                case 'PendRaw',units='raw 12-bit count';
                otherwise,units='counts';
            end
            assert(numel(t)>=2,'Furuta:Data','Select a nonempty interval');
        end
        function n = centerCount(~,s)
            n=int64(0);if isfield(s,'centerCount'),n=furuta.Protocol.decimal(s.centerCount,true);end
        end
        function timeAnalysis(obj)
            [t,x,u,T]=obj.dataset();if obj.UI.Detrend.Value,x=x-mean(x,'omitnan');end
            plot(obj.UI.TimeAxes,t,x);grid(obj.UI.TimeAxes,'on');xlabel(obj.UI.TimeAxes,'Device-relative time (s)');ylabel(obj.UI.TimeAxes,u);
            r=furuta.AnalysisService.stats(x);r.Sampling=furuta.AnalysisService.quality(t,x);
            if ismember('LimitFlags',T.Properties.VariableNames),r.LimitedSamples=sum(T.LimitFlags~=0);end
            if ismember('AdcRaw',T.Properties.VariableNames),r.ClippedADCSamples=sum(T.AdcRaw<=16|T.AdcRaw>=4079);end
            obj.LastAnalysis=r;obj.report(obj.UI.TimeReport,r);
        end
        function autocorrelation(obj)
            [~,x]=obj.dataset();assert(all(isfinite(x)),'Furuta:Data','Select finite data');r=furuta.AnalysisService.correlation(x,min(500,numel(x)-2));
            plot(obj.UI.TimeAxes,r.Lag,r.Value);xlabel(obj.UI.TimeAxes,'Lag (samples)');obj.LastAnalysis=r;
        end
        function stepMetrics(obj)
            [t,x]=obj.dataset();r=furuta.AnalysisService.stepMetrics(t,x,obj.UI.Baseline.Value,obj.UI.Target.Value);
            obj.LastAnalysis=r;obj.report(obj.UI.TimeReport,r);
        end
        function spectrum(obj)
            obj.analysisAllowed();[t,x,u,T]=obj.dataset();o=struct('Window',obj.UI.Window.Value,'Segment',round(obj.UI.SegmentLength.Value),...
                'Nfft',round(obj.UI.Nfft.Value),'RemoveMean',obj.UI.RemoveMean.Value,'Resample',obj.UI.Resample.Value);
            obj.validateSensorData(T);
            mode=obj.UI.SpectrumMode.Value;ax=obj.UI.SpectrumAxes;obj.clearAxes(ax);
            if strcmp(mode,'FRF + coherence')
                assert(ismember('Applied',T.Properties.VariableNames),'Furuta:Data','Applied input is unavailable');
                r=furuta.AnalysisService.frf(t,double(T.Applied),x,o.Segment);
                yyaxis(ax,'left');plot(ax,r.Frequency,r.GainDb);ylabel(ax,'Gain dB (output per duty)');
                yyaxis(ax,'right');ax.YAxis(2).Visible='on';plot(ax,r.Frequency,r.Coherence);ylabel(ax,'Coherence');ylim(ax,[0 1]);yyaxis(ax,'left');
            else
                r=furuta.AnalysisService.spectrum(t,x,o);
                switch mode
                    case 'FFT amplitude',plot(ax,r.Frequency,r.Amplitude);ylabel(ax,u);
                    case 'Welch PSD',semilogy(ax,r.PsdFrequency,max(r.PSD,realmin));ylabel(ax,[u,'^2/Hz']);
                    case 'ASD',semilogy(ax,r.PsdFrequency,max(r.ASD,realmin));ylabel(ax,[u,'/sqrt(Hz)']);
                    case 'Spectrogram'
                        imagesc(ax,r.SpectrogramTime,r.PsdFrequency,10*log10(max(r.Spectrogram,realmin)));axis(ax,'xy');colorbar(ax);xlabel(ax,'Time (s)');ylabel(ax,'Hz');
                end
            end
            title(ax,mode);if ~strcmp(mode,'Spectrogram'),xlabel(ax,'Frequency (Hz)');end;grid(ax,'on');
            obj.LastAnalysis=r;obj.report(obj.UI.SpectrumReport,r);
        end
        function inspectTiming(obj)
            [t,x]=obj.dataset();r=furuta.AnalysisService.quality(t,x);obj.report(obj.UI.SpectrumReport,r);
        end
        function selectContiguous(obj)
            obj.analysisAllowed();[t,x,~,T]=obj.dataset();
            name=obj.UI.Channel.Value;mask=0;
            if ismember(name,{'PendRaw','PendRad','DownRad','PendVelocity'}),mask=2;
            elseif ismember(name,{'CurrentMv','FilteredMv','CurrentA'}),mask=4;
            elseif ismember(name,{'ArmCounts','CountVelocity','ArmRad','ArmVelocity'}),mask=1;end
            if ismember('Validity',T.Properties.VariableNames)&&mask
                x(bitand(T.Validity,uint32(mask))==0)=NaN;
                if mask==2,x([false;diff(T.PendUs)<=0])=NaN;end
            end
            if mask==4&&ismember('AdcRaw',T.Properties.VariableNames),x(T.AdcRaw<=16|T.AdcRaw>=4079)=NaN;end
            if ismember('ConfigId',T.Properties.VariableNames)
                changes=[false;diff(double(T.ConfigId))~=0];x(changes)=NaN;
            end
            indices=furuta.AnalysisService.longestSpan(t,x);
            obj.UI.Start.Value=t(indices(1));obj.UI.End.Value=t(indices(end));
            r=furuta.AnalysisService.quality(t(indices),x(indices));
            r.SelectedFrom=t(indices(1));r.SelectedTo=t(indices(end));r.Excluded=numel(t)-numel(indices);
            r.Note='Selected longest finite contiguous span inside previous interval. Raw data unchanged.';
            obj.report(obj.UI.SpectrumReport,r);obj.log(r);
        end
        function bandPower(obj)
            assert(isstruct(obj.LastAnalysis)&&isfield(obj.LastAnalysis,'PSD'),'Furuta:Analysis','Compute a PSD first');
            r=furuta.AnalysisService.bandPower(obj.LastAnalysis,obj.UI.BandLow.Value,obj.UI.BandHigh.Value);obj.report(obj.UI.SpectrumReport,r);
        end
        function filterPreview(obj)
            obj.analysisAllowed();[t,x,u,T]=obj.dataset();obj.validateSensorData(T);
            [t,x,q]=furuta.AnalysisService.prepare(t,x,obj.UI.FilterResample.Value);
            fs=1/median(diff(t));designFs=obj.UI.FilterSampleRate.Value;
            if designFs>0&&abs(designFs-fs)>.01
                assert(obj.UI.FilterResample.Value,'Furuta:Sampling','Enable Resample small jitter to compare at the explicit design rate. Exclude gaps first.');
                assert(abs(designFs-fs)<=.05*fs,'Furuta:Sampling','Explicit design rate must be within 5%% of measured rate for this comparison');
                target=(t(1):1/designFs:t(end)).';x=interp1(t,x,target,'linear');t=target;fs=designFs;q.Resampled=true;
            elseif designFs>0,fs=designFs;end
            f=furuta.FilterService.design(obj.UI.FilterType.Value,fs,obj.UI.Cutoff.Value,round(obj.UI.FilterOrder.Value));f.zeroPhase=obj.UI.ZeroPhase.Value;
            y=furuta.FilterService.apply(x,f);r=furuta.FilterService.response(f);
            plot(obj.UI.FilterAxes,t,x,t,y);legend(obj.UI.FilterAxes,{'Raw','Filtered'});ylabel(obj.UI.FilterAxes,u);xlabel(obj.UI.FilterAxes,'Time (s)');grid(obj.UI.FilterAxes,'on');
            ax=obj.UI.FilterResponseAxes;obj.clearAxes(ax);
            if strcmp(obj.UI.FilterView.Value,'Raw / filtered PSD')
                a=furuta.AnalysisService.spectrum(t,x);b=furuta.AnalysisService.spectrum(t,y);
                semilogy(ax,a.PsdFrequency,max(a.PSD,realmin),b.PsdFrequency,max(b.PSD,realmin));legend(ax,{'Raw','Filtered'});ylabel(ax,[u,'^2/Hz']);
            else
                yyaxis(ax,'left');plot(ax,r.Frequency,r.GainDb);ylabel(ax,'Gain dB');
                yyaxis(ax,'right');
                ax.YAxis(2).Visible='on';
                if strcmp(obj.UI.FilterView.Value,'Gain / phase'),plot(ax,r.Frequency,r.Phase*180/pi);ylabel(ax,'Phase deg');
                else,plot(ax,r.Frequency,r.GroupDelaySeconds*1000);ylabel(ax,'Group delay ms');end
                yyaxis(ax,'left');
            end
            xlabel(obj.UI.FilterResponseAxes,'Frequency (Hz)');obj.LastFilter=f;
            obj.LastAnalysis=struct('Filter',f,'Sampling',q,'Raw',furuta.AnalysisService.stats(x),'Filtered',furuta.AnalysisService.stats(y));
            obj.report(obj.UI.FilterReport,obj.LastAnalysis);
        end
        function deployFilter(obj)
            obj.requireDisarmed();assert(~isempty(obj.LastFilter),'Furuta:Filter','Design a filter first');
            assert(abs(obj.LastFilter.fs-obj.Session.Config.adcHz)<.01,'Furuta:Filter','Deployment requires design at ADC sample rate; select an ADC burst recorded at that rate');
            c=obj.Session.Config;c.filter=furuta.FilterService.deployable(obj.LastFilter);obj.Session.applyConfig(c,[]);
        end
        function exportFilter(obj)
            assert(~isempty(obj.LastFilter),'Furuta:Filter','No filter designed');[f,p]=uiputfile('*.json');if ~isequal(f,0),obj.writeJSON(fullfile(p,f),obj.LastFilter);end
        end
        function identify(obj)
            obj.analysisAllowed();[t,y,units,T,c]=obj.dataset();assert(all(isfinite(y)),'Furuta:Data','Calibrate/select finite output data first');
            obj.validateSensorData(T);
            input=double(T.Applied);method=obj.UI.Method.Value;
            if strcmp(method,'Motor speed PT1')
                assert(ismember(obj.UI.Channel.Value,{'ArmVelocity','CountVelocity'}),'Furuta:Units','Choose ArmVelocity or CountVelocity for a speed model');
            elseif strcmp(method,'Pendulum decay')
                assert(strcmp(obj.UI.Channel.Value,'DownRad'),'Furuta:Units','Choose DownRad for the small-angle decay model');
            end
            switch method
                case 'ARX',r=furuta.IdentificationService.arx(t,input,y,round(obj.UI.Na.Value),round(obj.UI.Nb.Value),round(obj.UI.Nk.Value),obj.UI.Split.Value);
                case 'Motor speed PT1',r=furuta.IdentificationService.motor(t,input,y,obj.UI.Split.Value);
                case {'tfest','n4sid','ssest'},r=furuta.IdentificationService.toolbox(method,t,input,y,round(obj.UI.ModelOrder.Value),obj.UI.Split.Value);
                case 'Pendulum decay'
                    known=struct;if isfield(c,'geometry'),known=c.geometry;end
                    p=jsondecode(strjoin(obj.UI.Physics.Value,newline));
                    if isfield(p,'mass')&&isscalar(p.mass)&&isfield(p,'com')&&isscalar(p.com)
                        known.massKg=p.mass;known.comM=p.com;
                    end
                    r=furuta.IdentificationService.decay(t,y,known);r.KnownGeometry=known;
                case 'Physical nonlinear'
                    assert(all(ismember({'ArmRad','PendRad','CurrentA','ArmVelocity','PendVelocity'},T.Properties.VariableNames)),'Furuta:Model','Need calibrated mechanical/current core data');
                    p=jsondecode(strjoin(obj.UI.Physics.Value,newline));free=strtrim(strsplit(obj.UI.FreeParams.Value,','));
                    a=obj.UI.Bounds.Data;bounds=zeros(numel(free),2);
                    for k=1:numel(free),j=find(strcmp(a(:,1),free{k}),1);assert(~isempty(j),'Furuta:Bounds','Missing bound');bounds(k,:)=[a{j,2},a{j,3}];end
                    x0=[T.ArmRad(2);T.PendRad(2);T.ArmVelocity(2);T.PendVelocity(2);T.CurrentA(2)];
                    r=furuta.IdentificationService.fitPhysical(t(2:end),input(2:end),[T.ArmRad(2:end),T.PendRad(2:end),T.CurrentA(2:end)],p,free,bounds,x0,obj.UI.Split.Value);
            end
            r.InputUnits='normalized duty';r.OutputUnits=units;r.Source=obj.UI.Source.Value;
            if strcmp(method,'Physical nonlinear'),r.OutputUnits='rad / rad / A';units=r.OutputUnits;end
            obj.LastModel=r;plot(obj.UI.ModelAxes,r.Time,r.Measured,r.Time,r.Simulated,'--');xlabel(obj.UI.ModelAxes,'Time (s)');ylabel(obj.UI.ModelAxes,units);grid(obj.UI.ModelAxes,'on');
            title(obj.UI.ModelAxes,[method,' / measured versus model']);obj.report(obj.UI.ModelReport,r);
            if isfield(r,'TimeConstant'),obj.register('speed time constant',r.TimeConstant,'s','fit','local held-out fit');obj.register('speed gain',r.Gain,[units,' per duty'],'fit','local');end
            if isfield(r,'NaturalFrequencyRadS'),obj.register('pendulum natural frequency',r.NaturalFrequencyRadS,'rad/s','free decay','arm fixed required');obj.register('damping ratio',r.DampingRatio,'1','free decay','small-angle');end
            if isfield(r,'InertiaKgM2'),obj.register('pendulum pivot inertia',r.InertiaKgM2,'kg m^2','decay + supplied mass/COM','geometry dependent');end
            if isfield(r,'Parameters'),for k=free,obj.register(k{1},r.Parameters.(k{1}),'SI (see model)','nonlinear fit',sprintf('Identifiable=%d',r.Identifiable));end,end
        end
        function analysisAllowed(obj)
            assert(~ismember(obj.Session.Status.state,{'ARMED_IDLE','RUNNING'}),'Furuta:State','Disarm before heavy offline analysis; host stalls otherwise expire the device lease');
        end
        function validateSensorData(obj,T)
            if ~ismember('Validity',T.Properties.VariableNames)
                if ismember('AdcRaw',T.Properties.VariableNames)&&ismember(obj.UI.Channel.Value,{'CurrentMv','CurrentA'})
                    assert(all(T.AdcRaw>16&T.AdcRaw<4079),'Furuta:Quality','ADC burst contains clipping; inspect it in time view and choose an unclipped interval.');
                end
                return;
            end
            if ismember('ConfigId',T.Properties.VariableNames)
                assert(numel(unique(T.ConfigId))==1,'Furuta:Quality','Multiple profile versions in interval; choose one stable configuration for analysis.');
            end
            name=obj.UI.Channel.Value;mask=0;
            if ismember(name,{'PendRaw','PendRad','DownRad','PendVelocity'}),mask=2;
            elseif ismember(name,{'CurrentMv','FilteredMv','CurrentA'}),mask=4;
            elseif ismember(name,{'ArmCounts','CountVelocity','ArmRad','ArmVelocity'}),mask=1;end
            if mask
                assert(all(bitand(T.Validity,uint32(mask))~=0),'Furuta:Quality','Selected interval includes invalid/stale sensor data. Choose a valid contiguous interval.');
            end
            if mask==2
                assert(all(diff(T.PendUs)>0),'Furuta:Quality','Held pendulum samples present; select fresh data before frequency/identification claims.');
            end
        end
        function residualPlot(obj)
            assert(isstruct(obj.LastModel)&&isfield(obj.LastModel,'ResidualCorrelation'),'Furuta:Model','Fit a model with residual diagnostics first');
            r=obj.LastModel.ResidualCorrelation;plot(obj.UI.ModelAxes,r.Lag,r.Value);xlabel(obj.UI.ModelAxes,'Lag (samples)');ylabel(obj.UI.ModelAxes,'Residual autocorrelation');
            hold(obj.UI.ModelAxes,'on');r=obj.LastModel.InputResidualCorrelation;plot(obj.UI.ModelAxes,r.Lag,r.Value);hold(obj.UI.ModelAxes,'off');legend(obj.UI.ModelAxes,{'Residual','Input/residual'});
        end
        function viewModel(obj)
            obj.analysisAllowed();r=obj.LastModel;assert(isstruct(r),'Furuta:Model','Fit a model first');
            ax=obj.UI.ModelAxes;obj.clearAxes(ax);
            switch obj.UI.ModelView.Value
                case 'Measured / model',plot(ax,r.Time,r.Measured,r.Time,r.Simulated,'--');xlabel(ax,'Time (s)');ylabel(ax,r.OutputUnits);
                case 'Residual',obj.residualPlot();
                case 'Step'
                    if isfield(r,'A')
                        t=(0:1000)'*r.SampleTime;y=filter(r.B,r.A,ones(size(t)));plot(ax,t,y);xlabel(ax,'Time (s)');ylabel(ax,'Output per unit input');
                    elseif isfield(r,'Model'),[y,t]=step(r.Model);plot(ax,t,squeeze(y));xlabel(ax,'Time (s)');
                    else,error('Furuta:Model','Step analysis requires a linear input/output model');end
                case 'Bode'
                    if isfield(r,'A')
                        f=logspace(-2,log10(.5/r.SampleTime),500)';z=exp(-2i*pi*f*r.SampleTime);h=zeros(size(z));d=h;
                        for k=1:numel(r.B),h=h+r.B(k)*z.^(k-1);end
                        for k=1:numel(r.A),d=d+r.A(k)*z.^(k-1);end
                        h=h./d;
                    elseif isfield(r,'Model')
                        [mag,phase,w]=bode(r.Model);f=w/(2*pi);h=squeeze(mag).*exp(1i*squeeze(phase)*pi/180);
                    else,error('Furuta:Model','Bode analysis requires a local linear model');end
                    yyaxis(ax,'left');semilogx(ax,f,20*log10(max(abs(h),eps)));ylabel(ax,'Gain dB');
                    yyaxis(ax,'right');ax.YAxis(2).Visible='on';semilogx(ax,f,unwrap(angle(h))*180/pi);ylabel(ax,'Phase deg');yyaxis(ax,'left');xlabel(ax,'Hz');
            end
            grid(ax,'on');
        end
        function register(obj,name,value,unit,source,quality)
            obj.Registry(end+1,:)={name,value,unit,source,quality};obj.UI.Registry.Data=obj.Registry;
        end
        function captureBurst(obj,withRun)
            obj.requireDisarmed();obj.PendingBurstMeta=struct('Config',obj.Session.Config,'Center',obj.centerCount(obj.Session.Status));
            obj.Session.request('CAPTURE_BURST',struct('seconds',obj.UI.BurstDuration.Value,'withRun',withRun),[]);
        end
        function downloadBurst(obj),obj.requireDisarmed();obj.Session.downloadBurst(@(r)obj.downloaded(r));end
        function downloaded(obj,r)
            if ~r.ok,obj.log(r);return;end
            if ~isempty(obj.PendingBurstMeta),r.Config=obj.PendingBurstMeta.Config;r.Center=obj.PendingBurstMeta.Center;end
            obj.Burst=r;obj.UI.CaptureStatus.Value={sprintf('Downloaded ADC: %d samples; mechanical: %d',height(r.ADC),height(r.Core))};
            obj.UI.Source.Value='ADC burst';obj.UI.Channel.Value='CurrentMv';
        end
        function selectBurst(obj),assert(~isempty(obj.Burst),'Furuta:Data','Download a burst first');obj.UI.Source.Value='ADC burst';obj.UI.Channel.Value='CurrentMv';end
        function saveBurst(obj)
            assert(~isempty(obj.Burst),'Furuta:Burst','No downloaded burst');[f,p]=uiputfile('*.mat');if isequal(f,0),return;end
            burst=obj.Burst;burst.Callback=[];config=obj.Session.Config;save(fullfile(p,f),'burst','config','-v7.3');
        end
        function simFault(obj)
            assert(strcmp(obj.Session.Mode,'Simulator'),'Furuta:Simulator','Connect simulator first');tr=obj.Session.Transport;
            tr.WeakField=strcmp(obj.UI.SimFault.Value,'weak field');tr.SensorLost=strcmp(obj.UI.SimFault.Value,'lost pendulum');tr.DropPackets=strcmp(obj.UI.SimFault.Value,'drop packets');
        end
        function simNoise(obj),assert(strcmp(obj.Session.Mode,'Simulator'),'Furuta:Simulator','Connect simulator first');obj.Session.Transport.NoiseMv=obj.UI.SimNoise.Value;end
        function replayPause(obj),assert(strcmp(obj.Session.Mode,'Replay'),'Furuta:Replay','Open replay first');obj.Session.Transport.Playing=~obj.Session.Transport.Playing;end
        function capabilities(obj),obj.log(furuta.Defaults.capabilities());end
        function showGuide(obj)
            file=fullfile(obj.Root,'docs','QUICK_START.md');
            fig=uifigure('Name','Furuta Workbench Guide','Position',[100 80 1000 780]);
            g=uigridlayout(fig,[2 1]);g.RowHeight={30,'1x'};
            uilabel(g,'Text','Quick start and operating reference (also saved in docs/QUICK_START.md)','FontWeight','bold');
            uitextarea(g,'Value',splitlines(string(fileread(file))),'Editable','off','FontName','Consolas');
        end
        function openRun(obj,replay)
            obj.analysisAllowed();
            directory=uigetdir(fullfile(obj.Root,'data'),'Select a saved run');if isequal(directory,0),return;end
            run=furuta.RunRecorder.load(directory);obj.LoadedRun=run;
            if replay,obj.Session.connectReplay(run);obj.UI.Mode.Value='Replay';obj.UI.Connect.Text='Disconnect';obj.UI.Source.Value='Live buffer';
            else,obj.UI.Source.Value='Loaded run';end
        end
        function overlayRun(obj)
            directory=uigetdir(fullfile(obj.Root,'data'),'Overlay saved run');if isequal(directory,0),return;end
            run=furuta.RunRecorder.load(directory);T=furuta.CalibrationService.derive(run.Samples,run.Config,obj.centerCount(run.Manifest.device.status));
            name=obj.UI.Channel.Value;assert(ismember(name,T.Properties.VariableNames),'Furuta:Channel','Channel absent');
            hold(obj.UI.TimeAxes,'on');plot(obj.UI.TimeAxes,T.Time,double(T.(name)),'DisplayName',run.Manifest.id);hold(obj.UI.TimeAxes,'off');legend(obj.UI.TimeAxes,'show');
        end
        function exportCSV(obj)
            obj.analysisAllowed();
            [~,~,units,T,c]=obj.dataset();[f,p]=uiputfile({'*.csv','CSV + metadata';'*.mat','MATLAB samples + metadata'});if isequal(f,0),return;end
            metadata=struct('config',c,'units',units,'source',obj.UI.Source.Value,'channel',obj.UI.Channel.Value);
            [~,~,extension]=fileparts(f);
            if strcmpi(extension,'.mat'),save(fullfile(p,f),'T','metadata','-v7.3');
            else,furuta.RunRecorder.exportCSV(fullfile(p,f),T,metadata);end
        end
        function exportAnalysis(obj)
            assert(~isempty(obj.LastAnalysis),'Furuta:Analysis','No analysis');[f,p]=uiputfile('*.mat');if isequal(f,0),return;end
            analysis=obj.LastAnalysis;save(fullfile(p,f),'analysis');
        end
        function exportModel(obj)
            assert(~isempty(obj.LastModel),'Furuta:Model','No model fitted');[f,p]=uiputfile('*.mat');if isequal(f,0),return;end
            model=obj.LastModel;save(fullfile(p,f),'model');
        end
        function exportRegistry(obj)
            [f,p]=uiputfile('*.json');if isequal(f,0),return;end;obj.writeJSON(fullfile(p,f),struct('entries',{obj.Registry},'profile',obj.Session.Config));
        end
        function exportPlot(~,ax),[f,p]=uiputfile('*.png');if ~isequal(f,0),exportgraphics(ax,fullfile(p,f));end,end
        function writeJSON(~,file,value)
            fid=fopen(file,'w');assert(fid>=0,'Furuta:File','Cannot write file');g=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(value,'PrettyPrint',true),'char'); %#ok<NASGU>
        end
        function report(~,target,r)
            fields=fieldnames(r);lines={};
            for k=1:numel(fields)
                x=r.(fields{k});
                if isnumeric(x)&&numel(x)<=8,txt=mat2str(x,6);
                elseif ischar(x)||isstring(x),txt=char(x);
                elseif islogical(x)&&isscalar(x),txt=mat2str(x);
                else,txt=sprintf('%s %s',class(x),mat2str(size(x)));end
                lines{end+1}=[fields{k},': ',txt]; %#ok<AGROW>
            end
            target.Value=lines;
        end
        function log(obj,e)
            try,s=jsonencode(e);catch,s=char(string(e));end
            obj.LogLines{end+1}=[char(datetime('now','TimeZone','Africa/Cairo','Format','HH:mm:ss')),' ',s];
            if numel(obj.LogLines)>250,obj.LogLines(1:end-250)=[];end
            obj.LogDirty=true;
        end
        function markStatus(obj),obj.StatusDirty=true;end
        function statusChanged(obj)
            obj.StatusDirty=false;
            if obj.Closing||~isfield(obj.UI,'State'),return;end
            if ~obj.Session.Connected,obj.UI.Connect.Text='Connect';end
            obj.UI.Port.Enable=obj.choose(obj.Session.Connected,'off','on');
            obj.UI.Baud.Enable=obj.choose(obj.Session.Connected,'off','on');
            obj.UI.Arm.Enable=obj.choose(obj.Session.Ready&&strcmp(obj.Session.Status.state,'DISARMED')&&~strcmp(obj.Session.Mode,'Replay'),'on','off');
            obj.UI.Run.Enable=obj.choose(obj.Session.Ready&&strcmp(obj.Session.Status.state,'ARMED_IDLE'),'on','off');
            obj.UI.State.Text=[upper(obj.Session.Mode),' / ',obj.Session.Status.state];
            if obj.Session.Ready
                c=jsonencode(obj.Session.Config);
                if ~strcmp(c,obj.LastConfig),obj.fillProfile(obj.Session.Config);obj.LastConfig=c;end
            end
            if obj.Running&&~strcmp(obj.Session.Status.state,'RUNNING')
                obj.Running=false;obj.FinishAt=tic;
            end
        end
        function pauseView(obj,value),obj.DisplayPaused=value;end
        function refresh(obj)
            if obj.Closing||~isvalid(obj.Figure)||obj.RefreshBusy,return;end
            obj.RefreshBusy=true;clock=tic;guard=onCleanup(@()obj.releaseRefresh(clock)); %#ok<NASGU>
            if obj.StatusDirty,obj.statusChanged();end
            if obj.LogDirty,obj.UI.Log.Value=obj.LogLines;obj.LogDirty=false;end
            if ~isempty(obj.FinishAt)&&toc(obj.FinishAt)>1,obj.Session.finishRecord();obj.FinishAt=[];end
            s=obj.Session.Status;
            if isfield(s,'burstAdcCount')
                obj.UI.CaptureStatus.Value={sprintf('ADC buffered: %d | mechanical: %d',s.burstAdcCount,s.burstCoreCount),...
                    sprintf('Active: %d | waiting for Run: %d',s.burstActive,s.burstPending)};
            end
            if ~isempty(obj.Session.Recorder)&&obj.Session.Recorder.Active
                rec=obj.Session.Recorder.Directory;
            else,rec='Recording off';end
            obj.UI.Bottom.Text=sprintf('%s | Data buffer: %d samples | Current trip: %.3g A (0 = off) | STOP disables drive',...
                rec,height(obj.Session.Data),obj.Session.Config.currentTripA);
            T=obj.Session.Data;if isempty(T)||obj.DisplayPaused,return;end
            try
                last=T(end,:);weak=bitand(last.MagStatus,24)~=0;
                magnetic=obj.choose(weak,'WEAK/STRONG warning','normal');
                if bitand(last.Validity,2)==0,magnetic='INVALID / STALE';end
                errors=0;if isa(obj.Session.Transport,'furuta.SerialTransport'),errors=obj.Session.Transport.Parser.Errors;end
                obj.UI.Health.Text=sprintf('Magnet: %s | ADC %.1f mV | MCU drops %u | link errors %d',magnetic,last.CurrentMv,last.Dropped,errors);
                if ~strcmp(obj.UI.Tabs.SelectedTab.Title,'Live & Manual'),return;end
                key=sprintf('%u/%u/%u/%s/%s',last.BootId,last.DeviceUs,last.ConfigId,obj.UI.Channel.Value,s.state);
                if strcmp(key,obj.LastSampleKey),return;end
                obj.LastSampleKey=key;
                rows={'Motor count',sprintf('%d',last.Count);'Pendulum raw',sprintf('%u',last.PendRaw);...
                    'Current raw ADC',sprintf('%u',last.AdcRaw);'ADC mV',sprintf('%.2f',last.CurrentMv);...
                    'Applied duty',sprintf('%.3f',last.Applied);'Magnet MD/ML/MH',sprintf('%d/%d/%d',bitand(last.MagStatus,32)>0,bitand(last.MagStatus,16)>0,bitand(last.MagStatus,8)>0);...
                    'ADC age us',sprintf('%u',last.AdcAgeUs);'Pendulum age us',sprintf('%u',last.PendAgeUs);'Firmware state',s.state};
                T=T(max(1,end-2499):end,:);D=furuta.CalibrationService.derive(T,obj.Session.Config,obj.centerCount(s));name=obj.UI.Channel.Value;
                D.Time=double(T.DeviceUs-obj.Session.Data.DeviceUs(1))*1e-6;
                obj.UI.Values.Data=[rows;{'Arm rad',sprintf('%.5g',D.ArmRad(end));'Pendulum upright rad',sprintf('%.5g',D.PendRad(end));'Current A estimate',sprintf('%.5g',D.CurrentA(end));'Arm velocity rad/s',sprintf('%.5g',D.ArmVelocity(end));'Pendulum velocity rad/s',sprintf('%.5g',D.PendVelocity(end))}];
                if ismember(name,D.Properties.VariableNames)
                    x=double(D.(name));[tx,xx]=obj.decimate(D.Time,x,600);set(obj.UI.LiveLine,'XData',tx,'YData',xx);
                    if ~strcmp(obj.UI.LiveAxes.YLabel.String,name),obj.UI.LiveAxes.YLabel.String=name;end
                    liveFs=1/median(diff(D.Time));
                    if obj.UI.LiveFilter.Value&&~isempty(obj.LastFilter)&&~obj.LastFilter.zeroPhase&&abs(liveFs-obj.LastFilter.fs)<.05*liveFs
                        y=furuta.FilterService.apply(x,obj.LastFilter);[tt,yy]=obj.decimate(D.Time,y,1200);set(obj.UI.FilterLine,'XData',tt,'YData',yy);
                    else,set(obj.UI.FilterLine,'XData',nan,'YData',nan);end
                end
                set(obj.UI.RequestLine,'XData',D.Time,'YData',double(D.Requested));set(obj.UI.AppliedLine,'XData',D.Time,'YData',double(D.Applied));
                if ~isfinite(obj.TwinPreviewRaw)&&bitand(last.Validity,2)~=0,obj.TwinPreviewRaw=double(last.PendRaw);end
                [points,note]=furuta.TwinService.pose(last,obj.Session.Config,s,obj.TwinPreviewRaw);
                set(obj.UI.TwinLine,'XData',points(:,1),'YData',points(:,2),'ZData',points(:,3));
                if ~strcmp(obj.UI.TwinStatus.Text,note),obj.UI.TwinStatus.Text=note;end
            catch ex,obj.UI.Health.Text=ex.message;end
        end
        function releaseRefresh(obj,clock)
            obj.LastRefreshSeconds=toc(clock);obj.MaxRefreshSeconds=max(obj.MaxRefreshSeconds,obj.LastRefreshSeconds);obj.RefreshBusy=false;
        end
        function delete(obj)
            if obj.Closing,return;end;obj.Closing=true;
            if ~isempty(obj.RefreshTimer)&&isvalid(obj.RefreshTimer),stop(obj.RefreshTimer);delete(obj.RefreshTimer);end
            if ~isempty(obj.Session),delete(obj.Session);end
            if ~isempty(obj.Figure)&&isvalid(obj.Figure),delete(obj.Figure);end
        end
    end
    methods (Static)
        function clearAxes(ax)
            if numel(ax.YAxis)>1
                yyaxis(ax,'right');cla(ax);ax.YAxis(2).Visible='off';yyaxis(ax,'left');
            end
            cla(ax);ax.NextPlot='replacechildren';ax.XScale='linear';ax.YScale='linear';
            ax.XLimMode='auto';ax.YLimMode='auto';
        end
        function s = choose(condition,a,b),if condition,s=a;else,s=b;end,end
        function [t,y] = decimate(t,y,maxPoints)
            if numel(y)<=maxPoints,return;end
            width=ceil(numel(y)/(maxPoints/2));ix=[];
            for k=1:width:numel(y)
                z=k:min(k+width-1,numel(y));[~,lo]=min(y(z));[~,hi]=max(y(z));ix=[ix,sort([z(lo),z(hi)])]; %#ok<AGROW>
            end
            t=t(ix);y=y(ix);
        end
    end
end
