classdef AnalysisService
    methods (Static)
        function indices = longestSpan(t,x)
            % Select measured samples only: never interpolate across a loss.
            t=double(t(:));x=double(x(:));d=diff(t);dt=median(d(d>0&isfinite(d)));
            assert(isfinite(dt)&&dt>0,'Furuta:Sampling','No usable sample interval');
            good=isfinite(t)&isfinite(x);cuts=[true;~isfinite(d)|d<=0|d>1.5*dt];
            available=find(good);assert(~isempty(available),'Furuta:Sampling','No finite samples');
            edges=[1;find(diff(available)>1|cuts(available(2:end)))+1;numel(available)+1];
            [best,segment]=max(diff(edges));indices=available(edges(segment):edges(segment+1)-1);
            assert(best>=8,'Furuta:Sampling','No contiguous span with at least 8 finite samples');
        end
        function q = quality(t,x)
            t=double(t(:));x=double(x);assert(numel(t)==size(x,1),'Furuta:Data','Time/channel lengths differ');
            d=diff(t);q.Samples=numel(t);q.Invalid=sum(any(~isfinite(x),2)|~isfinite(t));
            q.SampleInterval=median(d);q.SampleRate=1/q.SampleInterval;
            q.Gaps=sum(d>1.5*q.SampleInterval);q.Nonmonotonic=sum(d<=0);
            q.JitterPercent=100*std(d)/q.SampleInterval;
            q.MaxInterval=max(d);q.Duration=t(end)-t(1);
            q.Uniform=q.Samples>=8 && q.Invalid==0 && q.Gaps==0 && q.Nonmonotonic==0 && q.JitterPercent<=5;
        end
        function [t,x,q] = prepare(t,x,resample)
            if nargin<3,resample=false;end
            t=double(t(:));x=double(x);q=furuta.AnalysisService.quality(t,x);
            assert(q.Samples>=8 && q.Nonmonotonic==0 && q.Invalid==0,'Furuta:Data','Select at least 8 finite, ordered samples; do not fill gaps with zero');
            if ~q.Uniform
                assert(resample && q.Gaps==0 && max(diff(t))<2*q.SampleInterval,...
                    'Furuta:Sampling','Nonuniform/gapped data: select a contiguous interval or explicitly resample small jitter');
                target=(t(1):q.SampleInterval:t(end)).';x=interp1(t,x,target,'linear');t=target;
                q.Resampled=true;
            else,q.Resampled=false;end
        end
        function r = stats(x)
            x=double(x(:));r.Invalid=sum(~isfinite(x));x=x(isfinite(x));
            assert(~isempty(x),'Furuta:Data','No valid values');
            r.Mean=mean(x);r.Median=median(x);r.SD=std(x);r.RMS=sqrt(mean(x.^2));
            r.Minimum=min(x);r.Maximum=max(x);r.PeakToPeak=range(x);
            r.Outliers=sum(abs(x-median(x))>6*max(1.4826*median(abs(x-median(x))),eps));
        end
        function w = window(n,type)
            if strcmp(type,'rectangular'),w=ones(n,1);
            else,w=.5-.5*cos(2*pi*(0:n-1)'/max(1,n-1));end
        end
        function r = spectrum(t,x,options)
            if nargin<3,options=struct;end
            o=struct('Resample',false,'RemoveMean',true,'Window','hann','Segment',1024,'Overlap',.5,'Nfft',0);
            for f=fieldnames(options).',o.(f{1})=options.(f{1});end
            [t,x,q]=furuta.AnalysisService.prepare(t,x,o.Resample);
            if o.RemoveMean,x=x-mean(x,1);end
            assert(size(x,2)==1,'Furuta:Data','Choose one spectrum channel');
            fs=1/median(diff(t));n=numel(x);w=furuta.AnalysisService.window(n,o.Window);
            nfft=max(n,o.Nfft);a=abs(fft(x.*w,nfft))/sum(w);k=floor(nfft/2)+1;a=a(1:k);
            if rem(nfft,2)==0,a(2:end-1)=2*a(2:end-1);else,a(2:end)=2*a(2:end);end
            r.Frequency=(0:k-1)'*fs/nfft;r.Amplitude=a;r.Fs=fs;r.Nyquist=fs/2;
            r.ResolutionHz=fs/n;r.Quality=q;
            L=min(n,o.Segment);assert(L>=8 && o.Overlap>=0 && o.Overlap<1,'Furuta:Welch','Invalid segment/overlap');
            step=max(1,L-round(L*o.Overlap));nfft=max(L,o.Nfft);w=furuta.AnalysisService.window(L,o.Window);
            ix=1:step:n-L+1;P=zeros(floor(nfft/2)+1,numel(ix));
            for j=1:numel(ix)
                v=abs(fft(x(ix(j):ix(j)+L-1).*w,nfft)).^2/(fs*sum(w.^2));v=v(1:size(P,1));
                if rem(nfft,2)==0,v(2:end-1)=2*v(2:end-1);else,v(2:end)=2*v(2:end);end
                P(:,j)=v;
            end
            r.PsdFrequency=(0:size(P,1)-1)'*fs/nfft;r.PSD=mean(P,2);r.ASD=sqrt(r.PSD);
            r.Segments=numel(ix);r.WelchResolutionHz=fs/L;r.IntegratedVariance=sum(r.PSD)*fs/nfft;
            r.Spectrogram=P;r.SpectrogramTime=t(ix)+L/(2*fs);r.Window=o.Window;
        end
        function r = frf(t,u,y,L)
            if nargin<4,L=512;end
            [t,x,q]=furuta.AnalysisService.prepare(t,[u(:),y(:)],false);
            fs=1/median(diff(t));x=x-mean(x,1);n=size(x,1);L=min(L,floor(n/2));
            w=furuta.AnalysisService.window(L,'hann');ix=1:max(1,floor(L/2)):n-L+1;
            assert(numel(ix)>=2,'Furuta:Coherence','At least two averaged segments required');
            suu=zeros(L,1);syy=suu;syu=suu;
            for at=ix
                U=fft(x(at:at+L-1,1).*w);Y=fft(x(at:at+L-1,2).*w);
                suu=suu+abs(U).^2;syy=syy+abs(Y).^2;syu=syu+Y.*conj(U);
            end
            k=1:floor(L/2)+1;r.Frequency=(k'-1)*fs/L;
            r.H=syu(k)./max(suu(k),eps);r.Coherence=abs(syu(k)).^2./max(suu(k).*syy(k),eps);
            r.Excited=suu(k)>max(suu)*1e-6;r.GainDb=20*log10(max(abs(r.H),eps));
            r.PhaseDeg=unwrap(angle(r.H))*180/pi;r.Segments=numel(ix);r.Quality=q;
        end
        function r = bandPower(s,f1,f2)
            assert(f1>=0&&f2>f1&&f2<=s.Nyquist,'Furuta:Band','Invalid frequency band');
            ix=s.PsdFrequency>=f1&s.PsdFrequency<=f2;
            df=s.PsdFrequency(2)-s.PsdFrequency(1);r.Power=sum(s.PSD(ix))*df;r.RMS=sqrt(r.Power);
        end
        function r = stepMetrics(t,y,baseline,target)
            t=double(t(:));y=double(y(:));delta=target-baseline;
            assert(abs(delta)>eps && numel(y)>10,'Furuta:Response','Need distinct baseline/target and a response interval');
            z=(y-baseline)/delta;i10=find(z>=.1,1);i90=find(z>=.9,1);
            r.RiseTime=NaN;if ~isempty(i10)&&~isempty(i90),r.RiseTime=t(i90)-t(i10);end
            [peak,k]=max(z);r.OvershootPercent=max(0,100*(peak-1));r.PeakTime=t(k)-t(1);
            last=find(abs(z-1)>.02,1,'last');r.SettlingTime=NaN;
            if isempty(last),r.SettlingTime=0;elseif last<numel(t),r.SettlingTime=t(last+1)-t(1);end
            r.SteadyError=target-mean(y(max(1,end-round(.1*numel(y))):end));
            e=target-y;r.IAE=trapz(t,abs(e));r.ISE=trapz(t,e.^2);r.ITAE=trapz(t,(t-t(1)).*abs(e));
        end
        function r = correlation(x,maxLag)
            x=double(x(:));x=x-mean(x);maxLag=min(maxLag,numel(x)-2);r.Lag=(0:maxLag)';
            r.Value=zeros(maxLag+1,1);den=x'*x;
            for k=0:maxLag,r.Value(k+1)=(x(1:end-k)'*x(1+k:end))/max(den,eps);end
        end
    end
end
