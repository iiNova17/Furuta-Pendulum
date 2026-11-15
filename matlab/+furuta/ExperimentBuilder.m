classdef ExperimentBuilder
    methods (Static)
        function s = segment(kind,amplitude,duration,f0,f1,offset)
            if nargin<6, offset=0; end
            if nargin<5, f1=2; end
            if nargin<4, f0=1; end
            s=struct('kind',char(kind),'duration',duration,'amplitude',amplitude,'offset',offset,...
                'f0',f0,'f1',f1,'phase',0,'duty',.5,'end',0,'bitPeriod',.05,'seed',1,...
                'logarithmic',false,'linear',false,'tablePeriod',.01,...
                'frequencies',[],'amplitudes',[],'phases',[],'table',[]);
            if strcmp(kind,'multisine')
                s.frequencies=f0*(1:4); s.amplitudes=amplitude/4*ones(1,4); s.phases=[0 pi/2 pi 3*pi/2];
            end
        end
        function y = value(s,t)
            t=max(0,t); z=s.f0*t+s.phase/(2*pi); frac=z-floor(z);
            switch char(s.kind)
                case {'hold','step'}, y=ones(size(t))*s.amplitude;
                case 'pulse', y=s.amplitude*(t<s.duty*s.duration);
                case 'doublet', y=s.amplitude*(2*(t<s.duration/2)-1);
                case 'sine', y=s.amplitude*sin(2*pi*z);
                case 'square', y=s.amplitude*(2*(frac<s.duty)-1);
                case 'triangle', y=s.amplitude*(1-4*abs(frac-.5));
                case 'ramp', y=s.amplitude+(s.end-s.amplitude)*min(1,t/s.duration);
                case 'chirp'
                    cycles=s.f0*t+(s.f1-s.f0)*t.^2/(2*s.duration);
                    if s.logarithmic && s.f0~=s.f1
                        cycles=s.f0*s.duration/log(s.f1/s.f0)*((s.f1/s.f0).^(t/s.duration)-1);
                    end
                    y=s.amplitude*sin(2*pi*cycles+s.phase);
                case 'prbs'
                    bits=ones(1,511); x=bitand(uint16(s.seed),511); if x==0,x=uint16(1);end
                    for k=1:511
                        bits(k)=double(bitand(x,1));
                        x=bitor(bitshift(x,-1),bitshift(bitand(bitxor(x,bitshift(x,-4)),1),8));
                    end
                    y=s.amplitude*(2*bits(mod(floor(t/s.bitPeriod),511)+1)-1);
                case 'multisine'
                    y=zeros(size(t));
                    for k=1:numel(s.frequencies), y=y+s.amplitudes(k)*sin(2*pi*s.frequencies(k)*t+s.phases(k)); end
                case 'arbitrary'
                    assert(~isempty(s.table),'Furuta:Table','Import a waveform table');
                    method='previous'; if s.linear,method='linear';end
                    index=min(numel(s.table)-1,t/s.tablePeriod);
                    if numel(s.table)==1,y=ones(size(t))*s.table(1);
                    else,y=interp1(0:numel(s.table)-1,s.table,index,method);end
                otherwise, error('Furuta:Waveform','Unknown waveform %s',s.kind)
            end
            y=y+s.offset;
        end
        function validate(e,c)
            assert(isfield(e,'segments') && ~isempty(e.segments) && numel(e.segments)<=256,'Furuta:Experiment','Need 1..256 segments');
            total=0;
            for s=e.segments(:).'
                for f={'duration','amplitude','offset','f0','f1','phase','duty','end','bitPeriod','tablePeriod'}
                    assert(isscalar(s.(f{1}))&&isfinite(s.(f{1})),'Furuta:Experiment','Nonfinite/scalar waveform field');
                end
                assert(s.duration>0 && s.f0>0 && s.f1>0 && max(s.f0,s.f1)<100 && s.duty>0 && s.duty<=1 && s.bitPeriod>=.002 && s.tablePeriod>=.001,...
                    'Furuta:Experiment','Invalid duration/frequency/duty/timing');
                peak=abs(s.amplitude);
                if strcmp(s.kind,'ramp'),peak=max(peak,abs(s.end));end
                if strcmp(s.kind,'multisine')
                    assert(~isempty(s.frequencies)&&numel(s.frequencies)<=8&&numel(s.phases)==numel(s.frequencies)&&numel(s.amplitudes)==numel(s.frequencies),...
                        'Furuta:Experiment','Multisine vectors must have equal length (1..8)');
                    assert(all(isfinite([s.frequencies(:);s.phases(:);s.amplitudes(:)])) && all(s.frequencies>0 & s.frequencies<100),'Furuta:Experiment','Invalid tones');
                    peak=sum(abs(s.amplitudes));
                elseif strcmp(s.kind,'arbitrary')
                    assert(~isempty(s.table)&&numel(s.table)<=4096&&all(isfinite(s.table))&&s.duration<=s.tablePeriod*numel(s.table),...
                        'Furuta:Experiment','Invalid table size/duration');peak=max(abs(s.table));
                end
                assert(abs(s.offset)+peak<=c.maxDuty+1e-9,'Furuta:Experiment','Amplitude plus offset exceeds commissioned duty limit');
                furuta.ExperimentBuilder.value(s,0); total=total+s.duration;
            end
            assert(total<=c.maxDuration,'Furuta:Experiment','Total duration exceeds profile maximum');
        end
        function [t,y,limited] = preview(e,c,fs)
            if nargin<3,fs=1000;end
            total=sum([e.segments.duration]); t=(0:1/fs:total).';y=zeros(size(t));at=0;
            for s=e.segments(:).'
                ix=t>=at & t<at+s.duration;y(ix)=furuta.ExperimentBuilder.value(s,t(ix)-at);at=at+s.duration;
            end
            limited=zeros(size(y)); blank=0;
            for k=2:numel(y)
                target=max(-c.maxDuty,min(c.maxDuty,y(k)));
                if limited(k-1)*target<0,target=0;end
                v=limited(k-1)+max(-c.slewPerSecond/fs,min(c.slewPerSecond/fs,target-limited(k-1)));
                if abs(v)<1e-9&&abs(limited(k-1))>1e-9,v=0;blank=k+1;end
                if k<blank,v=0;end
                limited(k)=v;
            end
            limited(end)=0;
        end
    end
end
