classdef FilterService
    methods (Static)
        function f = design(type,fs,frequency,order)
            if nargin<4,order=8;end
            f=struct('type',char(type),'alpha',.1,'window',order,'sos',[1 0 0 1 0 0],'fs',fs,'zeroPhase',false);
            assert(isfinite(fs)&&fs>0,'Furuta:Filter','Invalid sample rate');
            switch type
                case 'bypass'
                case 'ema'
                    assert(frequency>0&&frequency<fs/2,'Furuta:Filter','Cutoff outside Nyquist');f.alpha=1-exp(-2*pi*frequency/fs);
                case 'moving'
                    assert(order>=1&&order<=256&&fix(order)==order,'Furuta:Filter','Window must be 1..256');
                case 'butter'
                    assert(frequency>0&&frequency<fs/2,'Furuta:Filter','Cutoff outside Nyquist');
                    k=tan(pi*frequency/fs);a=1+sqrt(2)*k+k*k;
                    f.sos=[k*k 2*k*k k*k a 2*(k*k-1) 1-sqrt(2)*k+k*k]/a;f.sos(4)=1;f.type='biquad';
                case 'notch'
                    assert(frequency>0&&frequency<fs/2&&order>0,'Furuta:Filter','Invalid notch frequency/Q');
                    w=2*pi*frequency/fs;alpha=sin(w)/(2*order);
                    f.sos=[1 -2*cos(w) 1 1+alpha -2*cos(w) 1-alpha]/(1+alpha);f.sos(4)=1;f.type='biquad';
                otherwise,error('Furuta:Filter','Unsupported filter type');
            end
        end
        function y = apply(x,f)
            x=double(x(:));y=nan(size(x));valid=isfinite(x);
            starts=find(valid & [true;~valid(1:end-1)]);ends=find(valid & [~valid(2:end);true]);
            for j=1:numel(starts)
                ix=starts(j):ends(j);a=x(ix);
                switch f.type
                    case 'bypass',b=a;
                    case 'ema'
                        b=a;for k=2:numel(a),b(k)=b(k-1)+f.alpha*(a(k)-b(k-1));end
                    case 'moving'
                        cs=[0;cumsum(a)];k=(1:numel(a)).';lo=max(0,k-f.window);b=(cs(k+1)-cs(lo+1))./(k-lo);
                    case 'biquad'
                        assert(all(abs(roots(f.sos(4:6)))<1),'Furuta:Filter','Unstable filter');
                        b=filter(f.sos(1:3),f.sos(4:6),a);
                    otherwise,error('Furuta:Filter','Unsupported filter');
                end
                if isfield(f,'zeroPhase')&&f.zeroPhase
                    assert(exist('filtfilt','file')>0,'Furuta:Dependency','Zero-phase filtering requires Signal Processing Toolbox');
                    [num,den]=furuta.FilterService.coefficients(f);b=filtfilt(num,den,a);
                end
                y(ix)=b;
            end
        end
        function [b,a] = coefficients(f)
            switch f.type
                case 'bypass',b=1;a=1;
                case 'ema',b=f.alpha;a=[1 -(1-f.alpha)];
                case 'moving',b=ones(1,f.window)/f.window;a=1;
                case 'biquad',b=f.sos(1:3);a=f.sos(4:6);
            end
        end
        function r = response(f)
            [b,a]=furuta.FilterService.coefficients(f);r.Frequency=linspace(0,f.fs/2,1024).';
            z=exp(-1i*2*pi*r.Frequency/f.fs);
            h=zeros(size(z));g=h;
            for k=1:numel(b),h=h+b(k)*z.^(k-1);end
            for k=1:numel(a),g=g+a(k)*z.^(k-1);end
            h=h./g;r.GainDb=20*log10(max(abs(h),eps));r.Phase=unwrap(angle(h));
            r.GroupDelaySeconds=-gradient(r.Phase)./gradient(2*pi*r.Frequency);
        end
        function f = deployable(f)
            assert(~isfield(f,'zeroPhase')||~f.zeroPhase,'Furuta:Filter','Zero-phase processing is offline only');
            f=rmfield(f,intersect(fieldnames(f),{'fs','zeroPhase'}));
        end
    end
end
