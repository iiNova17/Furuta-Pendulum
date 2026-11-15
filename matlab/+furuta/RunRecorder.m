classdef RunRecorder < handle
    properties
        Directory
        Manifest
        Buffer = furuta.Protocol.emptySamples()
        Files = {}
        LastFlush
        EventFile = -1
        RawFile = -1
        SampleFile = -1
        Active = true
    end
    methods
        function obj = RunRecorder(root,profile,experiment,mode,device)
            if nargin<5,device=struct;end
            stamp=char(datetime('now','TimeZone','UTC','Format','yyyyMMdd_HHmmss_SSS'));
            obj.Directory=fullfile(root,[stamp,'_',char(mode)]);
            assert(~isfolder(obj.Directory),'Furuta:Recording','Run directory already exists');
            mkdir(obj.Directory);
            obj.Manifest=struct('schema',1,'id',stamp,'mode',char(mode),'synthetic',strcmp(mode,'Simulator'),...
                'createdUtc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSSXXX')),...
                'displayTimezone','Africa/Cairo','device',device,'release',version,'complete',false,...
                'sampleFiles',{{}},'sampleFile','samples.bin','sampleFormat','length-prefixed-core-batches-v1','samples',0,...
                'units',struct('DeviceUs','microseconds','CurrentMv','mV','Requested','normalized duty','Applied','normalized duty'),...
                'note','Raw samples are immutable. Bus voltage is assumed. Weak-field flags are preserved.');
            obj.json('manifest.json',obj.Manifest);obj.json('configuration.json',profile);obj.json('experiment.json',experiment);
            obj.EventFile=fopen(fullfile(obj.Directory,'events.jsonl'),'a');obj.RawFile=fopen(fullfile(obj.Directory,'raw_capture.bin'),'a');
            obj.SampleFile=fopen(fullfile(obj.Directory,'samples.bin'),'a','ieee-le');
            assert(obj.EventFile>=0&&obj.RawFile>=0&&obj.SampleFile>=0,'Furuta:Recording','Cannot open recording files');obj.LastFlush=tic;
        end
        function append(obj,T,payload)
            if ~obj.Active||isempty(T),return;end
            if nargin<3,payload=furuta.Protocol.packSamples(T);end
            obj.appendBatch(payload);
        end
        function appendBatch(obj,payload)
            if ~obj.Active,return;end
            count=double(furuta.Protocol.read(payload(21:22),'uint16'));
            assert(numel(payload)==22+88*count,'Furuta:Recording','Invalid sample batch');
            n=fwrite(obj.SampleFile,uint32(numel(payload)),'uint32');
            assert(n==1&&fwrite(obj.SampleFile,payload,'uint8')==numel(payload),'Furuta:Disk','Sample write failed');
            obj.Manifest.samples=obj.Manifest.samples+count;
            % No HDF5 initialization or table serialization in an armed callback.
            if toc(obj.LastFlush)>=1,obj.flush();end
        end
        function raw(obj,b)
            if obj.Active&&obj.RawFile>=0
                n=fwrite(obj.RawFile,b,'uint8');assert(n==numel(b),'Furuta:Disk','Raw recording write failed');
            end
        end
        function event(obj,e)
            if ~obj.Active,return;end
            r=struct('utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSSXXX')),'event',e);
            assert(fprintf(obj.EventFile,'%s\n',jsonencode(r))>0,'Furuta:Disk','Event write failed');
        end
        function json(obj,name,value)
            path=fullfile(obj.Directory,name);file=fopen([path,'.tmp'],'w');
            assert(file>=0,'Furuta:Disk','Cannot write run metadata');
            guard=onCleanup(@()fclose(file));n=fwrite(file,jsonencode(value),'char');
            assert(n>0,'Furuta:Disk','Metadata write failed');clear guard;movefile([path,'.tmp'],path,'f');
        end
        function flush(obj)
            obj.json('manifest.json',obj.Manifest);obj.LastFlush=tic;
        end
        function finish(obj,complete)
            if nargin<2,complete=true;end
            if ~obj.Active,return;end
            obj.flush();obj.Manifest.complete=complete;obj.json('manifest.json',obj.Manifest);
            obj.Active=false;if obj.EventFile>=0,fclose(obj.EventFile);obj.EventFile=-1;end
            if obj.RawFile>=0,fclose(obj.RawFile);obj.RawFile=-1;end
            if obj.SampleFile>=0,fclose(obj.SampleFile);obj.SampleFile=-1;end
        end
        function delete(obj)
            try,obj.finish(false);catch
                if obj.EventFile>=0,fclose(obj.EventFile);end
                if obj.RawFile>=0,fclose(obj.RawFile);end
                if obj.SampleFile>=0,fclose(obj.SampleFile);end
            end
        end
    end
    methods (Static)
        function run = load(directory)
            run.Manifest=jsondecode(fileread(fullfile(directory,'manifest.json')));
            run.Config=jsondecode(fileread(fullfile(directory,'configuration.json')));
            for k={'currentZeroMv','countsPerRev','pendDownRaw'},if isempty(run.Config.(k{1})),run.Config.(k{1})=NaN;end,end
            run.Experiment=jsondecode(fileread(fullfile(directory,'experiment.json')));
            run.Configurations=struct;
            if isfield(run.Manifest,'device')&&isfield(run.Manifest.device,'status')&&isfield(run.Manifest.device.status,'configId')
                s=run.Manifest.device.status;run.Configurations.(sprintf('v%u',s.configId))=struct('Config',run.Config,'Status',s);
            end
            eventFile=fopen(fullfile(directory,'events.jsonl'),'r');
            if eventFile>=0
                while true
                    line=fgetl(eventFile);if ~ischar(line),break;end
                    try
                        e=jsondecode(line);e=e.event;
                        if isstruct(e)&&isfield(e,'configSnapshot')&&isfield(e.status,'configId')
                            c=e.configSnapshot;for k={'currentZeroMv','countsPerRev','pendDownRaw'},if isempty(c.(k{1})),c.(k{1})=NaN;end,end
                            run.Configurations.(sprintf('v%u',e.status.configId))=struct('Config',c,'Status',e.status);
                        end
                    catch,end
                end
                fclose(eventFile);
            end
            chunks={};
            if isfield(run.Manifest,'sampleFile')
                file=fopen(fullfile(directory,'samples.bin'),'r','ieee-le');assert(file>=0,'Furuta:File','Missing samples.bin');
                guard=onCleanup(@()fclose(file)); %#ok<NASGU>
                packets={};
                while true
                    n=fread(file,1,'uint32=>double');if isempty(n),break;end
                    assert(n>=22&&n<=2026,'Furuta:File','Corrupt sample batch length');
                    b=fread(file,n,'uint8=>uint8').';
                    if numel(b)~=n
                        assert(~run.Manifest.complete,'Furuta:File','Incomplete final sample batch');
                        warning('Furuta:Incomplete','Incomplete final batch skipped; earlier raw data retained');break;
                    end
                    packets{end+1}=struct('Payload',b); %#ok<AGROW>
                    if numel(packets)>=256
                        chunks{end+1}=furuta.Protocol.sampleBatches(packets);packets={}; %#ok<AGROW>
                    end
                end
                if ~isempty(packets),chunks{end+1}=furuta.Protocol.sampleBatches(packets);end
            else
                files=run.Manifest.sampleFiles;if ischar(files),files={files};end
                for k=1:numel(files)
                    [~,name,ext]=fileparts(files{k});assert(strcmp([name,ext],files{k}),'Furuta:File','Invalid chunk path');
                    v=load(fullfile(directory,files{k}),'T');chunks{end+1}=v.T; %#ok<AGROW>
                end
            end
            if isempty(chunks),run.Samples=furuta.Protocol.emptySamples();else,run.Samples=vertcat(chunks{:});end
            run.Directory=directory;
        end
        function exportCSV(path,T,metadata)
            writetable(T,path);file=fopen([path,'.json'],'w');assert(file>=0,'Furuta:File','Cannot write CSV sidecar');
            guard=onCleanup(@()fclose(file));fwrite(file,jsonencode(metadata),'char');
        end
    end
end
