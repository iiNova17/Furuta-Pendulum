function result = hardware_passive_smoke(port,seconds,output)
%HARDWARE_PASSIVE_SMOKE Connect real GUI and record without arming or motion.
if nargin<1,port='COM11';end
if nargin<2,seconds=30;end
if nargin<3,output='test-results/hardware-passive';end
if ~isfolder(output),mkdir(output);end
app=FurutaWorkbench('Visible','on');guard=onCleanup(@()delete(app)); %#ok<NASGU>
app.UI.Port.Items={port};app.UI.Port.Value=port;app.connect();
waitFor(@()app.Session.Ready||~app.Session.Connected,20);
assert(app.Session.Ready,'Furuta:HardwareTest','Hardware handshake failed');
app.Session.request('STOP',struct,[]);
waitFor(@()strcmp(app.Session.Status.state,'DISARMED')||strcmp(app.Session.Status.state,'FAULT'),5);
assert(~ismember(app.Session.Status.state,{'ARMED_IDLE','RUNNING'}));
app.Session.beginRecord(output,struct('purpose','Passive GUI hardware verification; no ARM/START'));
fid=fopen(fullfile(output,'recording-active.txt'),'w');fprintf(fid,'Passive recording active on %s. Drive disabled.\n',port);fclose(fid);
parserStart=app.Session.Transport.Parser.Errors;
loops=[];clock=tic;
while toc(clock)<seconds&&app.Session.Connected
    tick=tic;pause(.05);drawnow;loops(end+1)=toc(tick); %#ok<AGROW>
end
app.Session.finishRecord();
T=app.Session.Data;
result=struct('Connected',app.Session.Connected,'Ready',app.Session.Ready,'Status',app.Session.Status,...
    'Config',app.Session.Config,'Samples',height(T),'Elapsed',toc(clock),...
    'LoopMedian',median(loops),'LoopP95',prctile(loops,95),'LoopMax',max(loops),'Events',{app.Session.Events});
if ~isempty(T)
    result.DeviceSpan=double(T.DeviceUs(end)-T.DeviceUs(1))*1e-6;
    result.SampleRate=(height(T)-1)/result.DeviceSpan;
    result.MaxApplied=max(abs(T.Applied));result.CountRange=double(max(T.Count)-min(T.Count));
    result.PendRawRange=double([min(T.PendRaw),max(T.PendRaw)]);
    result.AdcMv=double([mean(T.CurrentMv),std(T.CurrentMv)]);
    result.Drops=double(T.Dropped(end)-T.Dropped(1));
end
if isa(app.Session.Transport,'furuta.SerialTransport')
    result.ParserErrors=app.Session.Transport.Parser.Errors-parserStart;
    result.ParserLastError=app.Session.Transport.Parser.LastError;
    result.MaxSerialBacklog=app.Session.Transport.MaxBacklog;
end
result.MaxRefreshSeconds=app.MaxRefreshSeconds;
% Exercise acquisition, download and offline analysis through the actual app.
app.UI.BurstDuration.Value=.5;app.captureBurst(false);
waitFor(@()isfield(app.Session.Status,'burstAdcCount')&&app.Session.Status.burstAdcCount>0&&~app.Session.Status.burstActive,10);
app.downloadBurst();waitFor(@()~isempty(app.Burst),30);
burst=app.Burst;burst.Callback=[];save(fullfile(output,'passive-burst.mat'),'burst');
result.BurstADC=height(burst.ADC);result.BurstCore=height(burst.Core);
try
    app.UI.Start.Value=0;app.UI.End.Value=1e9;app.selectContiguous();
    result.AnalysisSpan=[app.UI.Start.Value,app.UI.End.Value];
    app.spectrum();result.SpectrumOK=true;
catch ex,result.SpectrumOK=false;result.SpectrumError=ex.message;end
try
    app.UI.FilterType.Value='butter';app.UI.Cutoff.Value=20;
    app.UI.FilterSampleRate.Value=2000;app.UI.FilterResample.Value=true;
    app.filterPreview();result.FilterPreviewOK=true;
    result.FilterDesignHz=app.LastFilter.fs;
    result.FilterDeployable=~isempty(furuta.FilterService.deployable(app.LastFilter));
catch ex,result.FilterPreviewOK=false;result.FilterPreviewError=ex.message;end
app.UI.Source.Value='Live buffer';
exportapp(app.Figure,fullfile(output,'live.png'));
save(fullfile(output,'result.mat'),'result');
fid=fopen(fullfile(output,'result.json'),'w');fwrite(fid,jsonencode(result,'PrettyPrint',true));fclose(fid);
disp(rmfield(result,'Events'));
assert(result.Connected&&result.Samples>100,'Furuta:HardwareTest','Connection failed or insufficient samples');
assert(result.MaxApplied==0,'Furuta:HardwareTest','Unexpected nonzero output during passive test');
end
function waitFor(fn,timeout)
clock=tic;while ~fn(),assert(toc(clock)<timeout,'Furuta:HardwareTest','Timed out');pause(.05);drawnow;end
end
