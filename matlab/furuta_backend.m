function furuta_backend(requestFile)
%FURUTA_BACKEND File-based, headless analysis worker. No serial connection.
r=jsondecode(fileread(requestFile));output=struct('ok',false);
try
    d=load(r.input);t=d.t(:);u=d.u(:);y=d.y(:);
    if strcmp(r.method,'PT1')
        fit=furuta.IdentificationService.motor(t,u,y,.7);
    elseif strcmp(r.method,'tfest')
        fit=furuta.IdentificationService.toolbox('tfest',t,u,y,2,.7);
    else,error('Furuta:Method','Unsupported backend method');end
    save(fullfile(fileparts(r.output),'model.mat'),'fit');
    output=struct('ok',true,'method',fit.Method,'fit_percent',fit.FitPercent,'rmse',fit.RMSE,...
        'time',fit.Time,'measured',fit.Measured,'simulated',fit.Simulated,...
        'note','MATLAB model saved to model.mat. Validation is chronological holdout; input is applied duty.');
    if isfield(fit,'TimeConstant'),output.time_constant_s=fit.TimeConstant;output.gain=fit.Gain;end
catch ex,output.error=ex.message;
end
f=fopen(r.output,'w');assert(f>=0,'Cannot write backend response');guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,jsonencode(output),'char');
end
