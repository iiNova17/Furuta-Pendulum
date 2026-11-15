function result = visual_smoke(output)
%VISUAL_SMOKE Exercises GUI in SIMULATOR mode only and exports its rendering.
if nargin<1,output=fullfile(fileparts(fileparts(mfilename('fullpath'))),'test-results');end
if ~isfolder(output),mkdir(output);end
app=FurutaWorkbench('Visible','off');guard=onCleanup(@()delete(app)); %#ok<NASGU>
app.UI.Mode.Value='Simulator';app.connect();waitFor(@()app.Session.Ready,20);
app.setupSimulator();waitFor(@()app.Session.Status.centered&&app.Session.Config.maxDuty>.1,15);
app.Experiment.segments=furuta.ExperimentBuilder.segment('sine',.08,2,2,2,0);app.sequenceTable();app.preview();
app.Session.beginRecord(output,app.executionExperiment());
app.Session.upload(app.executionExperiment(),@(r)app.runUploaded(r));
waitFor(@()app.Running,15);waitFor(@()~app.Running,20);
waitFor(@()~app.Session.Recorder.Active,15);
assert(strcmp(app.Session.Status.state,'DISARMED'),'Simulator entered a fault during GUI run');
assert(height(app.Session.Data)>200,'No streamed samples in GUI');
run=furuta.RunRecorder.load(app.Session.Recorder.Directory);
assert(height(run.Samples)>200,'Recording did not preserve the experiment');
assert(any(run.Samples.Applied>0)&&any(run.Samples.Applied<0),'Both excitation directions must be recorded');
app.UI.Channel.Value='CurrentMv';app.timeAnalysis();
app.spectrum();app.filterPreview();
app.UI.BurstDuration.Value=.3;app.captureBurst(false);
waitFor(@()~app.Session.Status.burstActive&&app.Session.Status.burstAdcCount>0,15);
app.downloadBurst();waitFor(@()~isempty(app.Burst),20);
app.UI.Source.Value='ADC burst';app.UI.Channel.Value='CurrentMv';app.UI.Cutoff.Value=20;
app.UI.FilterType.Value='butter';app.filterPreview();app.spectrum();
app.UI.Source.Value='Live buffer';app.UI.Channel.Value='CurrentMv';app.refresh();
assert(all(isfinite(app.UI.TwinLine.XData))&&all(isfinite(app.UI.TwinLine.ZData)),'Measured animation geometry did not render');
app.Figure.Position=[30 30 1460 900];app.Figure.Visible='on';drawnow;pause(.8);drawnow;
exportapp(app.Figure,fullfile(output,'workbench-live.png'));
% Parent may be a layout: use the actual tab to keep R2023b hierarchy explicit.
tabs=app.UI.Tabs.Children;
app.UI.Source.Value='ADC burst';
for k=1:numel(tabs),if strcmp(tabs(k).Title,'Spectrum & Noise'),app.UI.Tabs.SelectedTab=tabs(k);end,end
drawnow;pause(.8);drawnow;exportapp(app.Figure,fullfile(output,'workbench-spectrum.png'));
result=struct('State',app.Session.Status.state,'Samples',height(run.Samples),'ADC',height(app.Burst.ADC),...
    'CoreBurst',height(app.Burst.Core),'RunDirectory',run.Directory,'Mode',app.Session.Mode);
disp(result);save(fullfile(output,'visual-smoke-result.mat'),'result');
app.showGuide();figs=findall(groot,'Type','figure','Name','Furuta Workbench Guide');
assert(~isempty(figs),'Guide window did not open');delete(figs);
end
function waitFor(fn,timeout)
start=tic;while ~fn(),assert(toc(start)<timeout,'Furuta:Smoke','Timed out waiting for simulator workflow');pause(.05);drawnow;end
end
