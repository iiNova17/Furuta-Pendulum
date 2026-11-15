classdef CalibrationService
    methods (Static)
        function r = zeroCurrent(mv,t,raw)
            mv=double(mv(:));t=double(t(:)); valid=isfinite(mv)&isfinite(t);
            if nargin>2,valid=valid & double(raw(:))>16 & double(raw(:))<4079;end
            x=mv(valid); tx=t(valid);
            assert(numel(x)>=100,'Furuta:Calibration','At least 100 valid, unclipped samples required');
            r.MeanMv=mean(x);r.SdMv=std(x);r.MedianMv=median(x);r.SpanMv=range(x);
            r.Rejected=numel(mv)-numel(x);r.DriftMv=mean(x(end-floor(numel(x)/4):end))-mean(x(1:ceil(numel(x)/4)));
            groups=min(20,floor(numel(x)/20)); edges=round(linspace(1,numel(x)+1,groups+1)); blocks=zeros(groups,1);
            for k=1:groups,blocks(k)=mean(x(edges(k):edges(k+1)-1));end
            r.BlockMeanSE=std(blocks)/sqrt(groups);r.Duration=tx(end)-tx(1);
            r.Valid=r.MeanMv>=2200 && r.MeanMv<=2800 && r.Rejected==0;
            r.Qualified=r.SdMv>30 || abs(r.DriftMv)>20;
            r.Note='Block-mean uncertainty estimate; assess correlation/drift and current-trip margin.';
        end
        function r = circularZero(raw)
            x=double(raw(:))*2*pi/4096;x=x(isfinite(x)); assert(~isempty(x),'Furuta:Calibration','No valid angles');
            v=mean(exp(1i*x));r.Raw=mod(round(angle(v)*4096/(2*pi)),4096);
            r.Concentration=abs(v);r.SdRad=sqrt(max(0,-2*log(max(abs(v),eps))));
        end
        function r = encoderScale(counts,displacementRad)
            x=double(displacementRad(:));y=double(counts(:));
            assert(numel(x)>=2 && all(isfinite([x;y])) && x'*x>0,'Furuta:Calibration','Need known nonzero angular displacements');
            slope=(x'*y)/(x'*x);r.CountsPerRev=abs(slope)*2*pi;r.Sign=sign(slope);
            r.ResidualCounts=y-slope*x;r.RmsCounts=sqrt(mean(r.ResidualCounts.^2));
        end
        function D = derive(T,c,center)
            D=T; if isempty(T),return;end
            D.Time=double(T.DeviceUs-T.DeviceUs(1))*1e-6;
            D.ArmCounts=double(T.Count-int64(center));
            D.ArmRad=nan(height(T),1);
            if ~isempty(c.countsPerRev)&&isfinite(c.countsPerRev)&&c.countsPerRev>0
                D.ArmRad=c.armSign*2*pi*D.ArmCounts/c.countsPerRev;
            end
            D.ArmRad(bitand(T.Validity,1)==0)=NaN;
            D.PendRad=nan(height(T),1);
            if ~isempty(c.pendDownRaw)&&isfinite(c.pendDownRaw)
                D.PendRad=furuta.CalibrationService.wrap(c.pendSign*(double(T.PendRaw)-c.pendDownRaw)*2*pi/4096+pi);
            end
            D.PendRad(bitand(T.Validity,2)==0)=NaN;
            D.DownRad=furuta.CalibrationService.wrap(D.PendRad-pi);
            D.CurrentA=nan(height(T),1);D.FilteredA=D.CurrentA;
            if ~isempty(c.currentZeroMv)&&isfinite(c.currentZeroMv)
                D.CurrentA=(double(T.CurrentMv)-c.currentZeroMv)/c.sensitivityMvA;
                D.FilteredA=(double(T.FilteredMv)-c.currentZeroMv)/c.sensitivityMvA;
            end
            invalid=bitand(T.Validity,4)==0;D.CurrentA(invalid)=NaN;D.FilteredA(invalid)=NaN;
            D.CountVelocity=furuta.CalibrationService.velocity(D.Time,D.ArmCounts,false);
            D.ArmVelocity=furuta.CalibrationService.velocity(D.Time,D.ArmRad,false);
            D.PendVelocity=furuta.CalibrationService.velocity(D.Time,D.PendRad,true);
        end
        function y = wrap(x),y=mod(x+pi,2*pi)-pi;end
        function v = velocity(t,x,circular)
            v=nan(size(x));if numel(x)<2,return;end
            d=diff(x);if circular,d=furuta.CalibrationService.wrap(d);end
            dt=diff(t);good=dt>0 & dt<.1 & isfinite(d);z=nan(size(d));z(good)=d(good)./dt(good);
            v(2:end)=z; % Backward difference, no invented derivative across a gap.
        end
        function r = gain(knownA,mv)
            X=[ones(numel(knownA),1),double(knownA(:))];y=double(mv(:));
            assert(size(X,1)>=3&&rank(X)==2,'Furuta:Calibration','Need at least three known signed current points');
            b=X\y;r.ZeroMv=b(1);r.SensitivityMvA=b(2);r.ResidualMv=y-X*b;
        end
    end
end
