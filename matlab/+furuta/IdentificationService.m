classdef IdentificationService
    methods (Static)
        function r = arx(t,u,y,na,nb,nk,split)
            if nargin<7,split=.7;end
            [t,x,q]=furuta.AnalysisService.prepare(t,[u(:),y(:)],false);u=x(:,1);y=x(:,2);
            assert(all([na nb nk]==fix([na nb nk]))&&na>=1&&nb>=1&&nk>=0,'Furuta:Model','Invalid orders/delay');
            cut=floor(numel(t)*split);lag=max(na,nk+nb-1);assert(cut>lag+na+nb+10&&cut<numel(t)-lag-10,'Furuta:Model','Not enough estimation/validation samples');
            u0=mean(u(1:cut));y0=mean(y(1:cut));u=u-u0;y=y-y0;
            X=zeros(cut-lag,na+nb);
            for k=lag+1:cut
                X(k-lag,:)=[-y(k-1:-1:k-na).',u(k-nk:-1:k-nk-nb+1).'];
            end
            assert(rank(X)==size(X,2)&&cond(X)<1e10,'Furuta:Identifiability','Insufficient excitation or ill-conditioned regression');
            beta=X\y(lag+1:cut);a=[1;beta(1:na)];b=[zeros(nk,1);beta(na+1:end)];
            predicted=nan(size(y));
            for k=lag+1:numel(y)
                predicted(k)=[-y(k-1:-1:k-na).',u(k-nk:-1:k-nk-nb+1).']*beta;
            end
            % Free validation simulation uses measured initial state only once.
            simulated=y;for k=cut+1:numel(y)
                simulated(k)=-a(2:end)'*simulated(k-1:-1:k-na)+beta(na+1:end)'*u(k-nk:-1:k-nk-nb+1);
            end
            ix=cut+1:numel(y);err=y(ix)-simulated(ix);sigma2=sum((y(lag+1:cut)-X*beta).^2)/max(1,size(X,1)-size(X,2));
            r=struct('Method','ARX','A',a,'B',b,'Na',na,'Nb',nb,'Nk',nk,'SampleTime',median(diff(t)),...
                'InputMean',u0,'OutputMean',y0,'Coefficients',beta,'Covariance',sigma2*pinv(X'*X),...
                'Condition',cond(X),'Poles',roots(a),'Stable',all(abs(roots(a))<1),...
                'Measured',y+y0,'Predicted',predicted+y0,'Simulated',simulated+y0,'Time',t,...
                'ValidationStart',cut+1,'Residual',err,'Quality',q);
            r.RMSE=sqrt(mean(err.^2));den=norm(y(ix)-mean(y(ix)));
            r.FitPercent=NaN;if den>eps,r.FitPercent=100*(1-norm(err)/den);end
            r.ResidualCorrelation=furuta.AnalysisService.correlation(err,min(100,numel(err)-2));
            r.InputResidualCorrelation=furuta.IdentificationService.crossResidual(u(ix),err);
        end
        function r = toolbox(method,t,u,y,order,split)
            if nargin<6,split=.7;end
            assert(exist('iddata','file')>0,'Furuta:Dependency','System Identification Toolbox is required');
            [t,x]=furuta.AnalysisService.prepare(t,[u(:),y(:)],false);cut=floor(numel(t)*split);Ts=median(diff(t));
            u0=mean(x(1:cut,1));y0=mean(x(1:cut,2));x=x-[u0 y0];
            train=iddata(x(1:cut,2),x(1:cut,1),Ts);validation=iddata(x(cut+1:end,2),x(cut+1:end,1),Ts);
            switch method
                case 'tfest',model=tfest(train,order,max(0,order-1));
                case 'n4sid',model=n4sid(train,order);
                case 'ssest',model=ssest(train,order);
                otherwise,error('Furuta:Model','Unsupported toolbox estimator');
            end
            [pred,fit]=compare(validation,model);err=validation.OutputData-pred.OutputData;
            r=struct('Method',method,'Model',model,'FitPercent',fit(1),'RMSE',sqrt(mean(err.^2)),...
                'Time',t(cut+1:end),'Measured',validation.OutputData+y0,'Simulated',pred.OutputData+y0,...
                'Residual',err,'SampleTime',Ts,'InputMean',u0,'OutputMean',y0,'Poles',pole(model),'Stable',isstable(model));
            r.ResidualCorrelation=furuta.AnalysisService.correlation(err,min(100,numel(err)-2));
            r.InputResidualCorrelation=furuta.IdentificationService.crossResidual(validation.InputData,err);
            try,r.Covariance=getcov(model);catch,r.Covariance=[];end
        end
        function r = motor(t,u,speed,split)
            if nargin<4,split=.7;end
            a=furuta.IdentificationService.arx(t,u,speed,1,1,1,split);
            pole=-a.A(2);assert(pole>0 && pole<1,'Furuta:Model','Identified speed pole is not a stable first-order pole');
            r=a;r.Method='Local motor speed PT1';r.TimeConstant=-a.SampleTime/log(pole);r.Gain=a.B(end)/(1-pole);
            r.Note='Gain is speed units per selected input unit; this does not uniquely identify R, Ke, Kt or inertia.';
        end
        function r = decay(t,phi,known)
            if nargin<3,known=struct;end
            [t,phi]=furuta.AnalysisService.prepare(t,phi(:),false);t=t-t(1);
            assert(max(abs(phi))<.5,'Furuta:Model','Small-angle decay requires |phiDown| < 0.5 rad');
            crossings=find(phi(1:end-1).*phi(2:end)<0);assert(numel(crossings)>=3,'Furuta:Model','Need several free-decay oscillations');
            omega=pi/median(diff(t(crossings)));p0=[log(omega),log(.1),phi(1),0];
            options=optimset('Display','off','MaxIter',1000);
            p=fminsearch(@(p)sum((furuta.IdentificationService.decayCurve(p,t)-phi).^2),p0,options);
            wn=exp(p(1));zeta=exp(p(2));fit=furuta.IdentificationService.decayCurve(p,t);
            r=struct('Method','Pendulum free decay','NaturalFrequencyRadS',wn,'DampingRatio',zeta,...
                'Time',t,'Measured',phi,'Simulated',fit,'RMSE',sqrt(mean((phi-fit).^2)),...
                'Note','Arm must be held fixed. Frequency determines m*g*l/J, not independent m, l and J.');
            if isfield(known,'massKg')&&isfield(known,'comM')&&isscalar(known.massKg)&&isscalar(known.comM)&&isfinite(known.massKg)&&isfinite(known.comM)
                r.InertiaKgM2=known.massKg*9.80665*known.comM/wn^2;r.ViscousNmS=2*zeta*wn*r.InertiaKgM2;
            end
        end
        function y = decayCurve(p,t)
            wn=exp(p(1));z=exp(p(2));wd=wn*sqrt(complex(1-z*z));
            if abs(wd)<1e-8,wd=1e-8;end
            y=real(exp(-z*wn*t).*(p(3)*cos(wd*t)+p(4)*sin(wd*t)));
            if any(~isfinite(y)),y=ones(size(t))*1e12;end
        end
        function r = crossResidual(u,e)
            u=u(:)-mean(u);e=e(:)-mean(e);m=min(100,numel(e)-2);r.Lag=(-m:m)';r.Value=zeros(size(r.Lag));
            for j=1:numel(r.Lag)
                k=r.Lag(j);if k>=0,a=u(1:end-k);b=e(1+k:end);else,a=u(1-k:end);b=e(1:end+k);end
                r.Value(j)=a'*b/max(eps,sqrt((u'*u)*(e'*e)));
            end
        end
        function r = fitPhysical(t,u,Y,p,free,bounds,x0,split)
            if nargin<8,split=.7;end
            [t,combined]=furuta.AnalysisService.prepare(t,[u(:),Y],false);u=combined(:,1);Y=combined(:,2:end);
            assert(size(Y,2)==3,'Furuta:Model','Physical model outputs must be arm rad, pendulum upright rad, current A');
            assert(numel(x0)==5&&all(isfinite(x0)),'Furuta:Model','Five finite initial states required');
            furuta.FurutaModel.derivative(x0(:),u(1),p);
            cut=floor(numel(t)*split);names=fieldnames(p);
            for k=1:numel(free),assert(ismember(free{k},names),'Furuta:Model','Unknown free parameter');end
            assert(size(bounds,1)==numel(free)&&size(bounds,2)==2&&all(isfinite(bounds(:)))&&all(bounds(:,2)>bounds(:,1)),...
                'Furuta:Bounds','Finite ordered bounds required for every free parameter');
            lo=bounds(:,1);hi=bounds(:,2);v=cellfun(@(f)p.(f),free(:));
            assert(all(v>lo&v<hi),'Furuta:Model','Initial values must be inside bounds');
            z0=log((v-lo)./(hi-v));scale=max(std(Y(1:cut,:),0,1),[.01 .01 .01]);
            fun=@(z)furuta.IdentificationService.physicalResidual(z,p,free,lo,hi,t(1:cut),u(1:cut),Y(1:cut,:),x0,scale);
            z=fminsearch(@(z)sum(fun(z).^2),z0,optimset('Display','off','MaxIter',300,'MaxFunEvals',1500));
            fitted=p;v=lo+(hi-lo)./(1+exp(-z));for k=1:numel(free),fitted.(free{k})=v(k);end
            predicted=furuta.FurutaModel.simulate(t,u,fitted,x0);
            display=predicted(:,[1 2 5]);display(:,2)=furuta.CalibrationService.wrap(display(:,2));
            validationError=display(cut+1:end,:)-Y(cut+1:end,:);validationError(:,2)=furuta.CalibrationService.wrap(validationError(:,2));
            J=zeros(numel(fun(z)),numel(z));base=fun(z);
            for k=1:numel(z),zz=z;zz(k)=zz(k)+1e-4;J(:,k)=(fun(zz)-base)/1e-4;end
            r=struct('Method','Nonlinear physical fit','Parameters',fitted,'Free',{free},'Bounds',bounds,...
                'SensitivityCondition',cond(J),'Identifiable',rank(J)==numel(z)&&cond(J)<1e8,...
                'Time',t,'Measured',Y,'Simulated',display,'ValidationStart',cut+1,...
                'RMSE',sqrt(mean(validationError.^2,1)));
        end
        function e = physicalResidual(z,p,free,lo,hi,t,u,Y,x0,scale)
            v=lo+(hi-lo)./(1+exp(-z));for k=1:numel(free),p.(free{k})=v(k);end
            try,X=furuta.FurutaModel.simulate(t,u,p,x0);err=X(:,[1 2 5])-Y;err(:,2)=furuta.CalibrationService.wrap(err(:,2));e=reshape(err./scale,[],1);
            catch,e=ones(numel(Y),1)*1e6;end
            e(~isfinite(e))=1e6;
        end
    end
end
