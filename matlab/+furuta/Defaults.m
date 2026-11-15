classdef Defaults
    methods (Static)
        function c = profile()
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            c = jsondecode(fileread(fullfile(root,'config','default_profile.json')));
            for k = {'currentZeroMv','countsPerRev','pendDownRaw'}
                if isempty(c.(k{1})), c.(k{1})=NaN; end
            end
            c.filter.sos = c.filter.sos(:).';
        end
        function c = deviceProfile(c)
            for k = {'geometry','notes'}
                if isfield(c,k{1}), c=rmfield(c,k{1}); end
            end
        end
        function s = capabilities()
            s.Release=version('-release'); s.Signal=exist('pwelch','file')>0;
            s.Identification=exist('tfest','file')>0; s.Control=exist('tf','file')>0;
            s.Optimization=exist('lsqnonlin','file')>0;
            s.Note='Availability checked; license checkout failures are reported when used.';
        end
        function validate(c)
            names={'maxDuty','slewPerSecond','maxDuration','travelCounts','velocityCountsPerSecond','currentTripA','currentTripMs','sensitivityMvA','assumedBusV','heartbeatMs','streamHz','adcHz','pwmHz'};
            for f=names,assert(isfield(c,f{1})&&isscalar(c.(f{1}))&&isfinite(c.(f{1})),'Furuta:Profile','Expected finite scalar %s',f{1});end
            assert(c.schema==1&&c.maxDuty>=0&&c.maxDuty<=1&&c.slewPerSecond>0&&c.slewPerSecond<=100&&c.maxDuration>0&&c.maxDuration<=3600&&...
                c.travelCounts>=0&&c.travelCounts<=1e9&&c.velocityCountsPerSecond>=0&&c.currentTripA>=0&&c.currentTripA<=50&&...
                c.currentTripMs>=0&&c.currentTripMs<=1000&&c.sensitivityMvA>0&&c.assumedBusV>0&&c.assumedBusV<=13.5,...
                'Furuta:Profile','Profile limits/scale outside supported range');
            assert(ismember(c.armSign,[-1 1])&&ismember(c.pendSign,[-1 1])&&ismember(c.streamHz,[50 100 250]),'Furuta:Profile','Invalid sign/stream rate');
            assert(c.heartbeatMs>=100&&c.heartbeatMs<=2000&&fix(c.heartbeatMs)==c.heartbeatMs&&...
                c.adcHz>=100&&c.adcHz<=2000&&mod(1000000,c.adcHz)==0&&c.pwmHz>=1000&&c.pwmHz<=30000&&fix(c.pwmHz)==c.pwmHz,...
                'Furuta:Profile','Unsupported timing profile');
            for f={'currentZeroMv','countsPerRev','pendDownRaw'}
                assert(isfield(c,f{1})&&isscalar(c.(f{1})),'Furuta:Profile','Optional scales must be scalar or NaN');
            end
            assert(isnan(c.currentZeroMv)||(c.currentZeroMv>=1800&&c.currentZeroMv<=3000),'Furuta:Profile','Invalid current zero');
            assert(isnan(c.countsPerRev)||(isfinite(c.countsPerRev)&&c.countsPerRev>0),'Furuta:Profile','Invalid counts/output rev');
            assert(isnan(c.pendDownRaw)||(c.pendDownRaw>=0&&c.pendDownRaw<=4095&&fix(c.pendDownRaw)==c.pendDownRaw),'Furuta:Profile','Invalid pendulum zero');
            f=c.filter;a=f.sos;
            assert(ismember(f.type,{'bypass','ema','moving','biquad'})&&isfinite(f.alpha)&&f.alpha>0&&f.alpha<=1&&...
                f.window>=1&&f.window<=256&&fix(f.window)==f.window&&numel(a)==6&&all(isfinite(a)),...
                'Furuta:Profile','Invalid causal filter');
            assert(a(4)==1&&abs(a(6))<1&&1+a(5)+a(6)>0&&1-a(5)+a(6)>0,'Furuta:Profile','SOS must be normalized and strictly stable');
        end
        function id = nonce()
            id=bitor(bitshift(uint64(randi([1,2^32-1])),32),uint64(randi([1,2^32-1])));
        end
    end
end
