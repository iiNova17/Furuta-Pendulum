function inspect_passive_results()
load('test-results/hardware-after/passive-burst.mat','burst');
t=double(burst.ADC.DeviceUs-burst.ADC.DeviceUs(1))*1e-6;
disp(furuta.AnalysisService.quality(t,double(burst.ADC.CurrentMv)));
disp('ADC intervals us [min 1% 5% median 95% 99% max]');disp(prctile(diff(t)*1e6,[0 1 5 50 95 99 100]));
disp('ADC burst mV statistics');disp(furuta.AnalysisService.stats(burst.ADC.CurrentMv));
disp('Mechanical intervals us');disp(prctile(diff(double(burst.Core.DeviceUs)),[0 1 5 50 95 99 100]));
profile on;
a=FurutaWorkbench('Visible','on');guard=onCleanup(@()delete(a)); %#ok<NASGU>
a.UI.Port.Items={'COM11'};a.UI.Port.Value='COM11';a.connect();
clock=tic;while toc(clock)<20,pause(.05);end
profile off;p=profile('info');save('test-results/hardware-profile.mat','p');
ft=p.FunctionTable;[~,order]=sort([ft.TotalTime],'descend');
for k=order(1:min(20,numel(order))),fprintf('%.3f %d %s\n',ft(k).TotalTime,ft(k).NumCalls,ft(k).FunctionName);end
fprintf('Max refresh %.3f s; serial backlog %d; parser errors %d\n',a.MaxRefreshSeconds,a.Session.Transport.MaxBacklog,a.Session.Transport.Parser.Errors);
end
