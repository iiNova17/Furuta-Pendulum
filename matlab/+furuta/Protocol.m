classdef Protocol < handle
    properties
        Buffer = uint8([])
        Discard = false
        Errors = 0
        LastError = ''
        % False for a freshly opened port: frames that fail to decode before
        % the first valid frame are stale/partial bytes and dropped silently.
        Synced = true
        Discarded = 0
    end
    methods
        function packets = feed(obj,bytes)
            packets={};
            bytes=uint8(bytes(:).');stops=find(bytes==0);first=1;
            for at=stops
                frame=[obj.Buffer,bytes(first:at-1)];obj.Buffer=uint8([]);
                if ~obj.Discard&&~isempty(frame)
                    try
                        assert(numel(frame)<=2064,'Furuta:Size','Oversized frame');
                        packets{end+1}=furuta.Protocol.decode(frame); %#ok<AGROW>
                        obj.Synced=true;
                    catch ex
                        if ~obj.Synced,obj.Discarded=obj.Discarded+numel(frame);
                        else,obj.Errors=obj.Errors+1;obj.LastError=ex.message;end
                    end
                end
                obj.Discard=false;first=at+1;
            end
            if ~obj.Discard
                obj.Buffer=[obj.Buffer,bytes(first:end)];
                if numel(obj.Buffer)>2064,obj.Buffer=uint8([]);obj.Discard=true;obj.Errors=obj.Errors+1;obj.LastError='Oversized frame';end
            end
        end
    end
    methods (Static)
        function c = crc(bytes)
            persistent tab engine
            if usejava('jvm')
                if isempty(engine),engine=java.util.zip.CRC32;end
                engine.reset();engine.update(typecast(uint8(bytes(:)),'int8'));
                c=uint32(engine.getValue());return
            end
            if isempty(tab)
                tab=zeros(1,256,'uint32');
                for i=0:255
                    v=uint32(i);
                    for j=1:8
                        if bitand(v,1), v=bitxor(bitshift(v,-1),uint32(hex2dec('EDB88320')));
                        else, v=bitshift(v,-1); end
                    end
                    tab(i+1)=v;
                end
            end
            c=uint32(4294967295);
            for b=uint8(bytes(:).')
                c=bitxor(bitshift(c,-8),tab(double(bitand(bitxor(c,uint32(b)),255))+1));
            end
            c=bitxor(c,uint32(4294967295));
        end
        function b = le(x)
            [~,~,endian]=computer;
            if endian=='B', x=swapbytes(x); end
            b=reshape(typecast(x(:),'uint8'),1,[]);
        end
        function x = read(b,type)
            x=typecast(uint8(b(:).'),type);
            [~,~,endian]=computer;
            if endian=='B', x=swapbytes(x); end
        end
        function value = decimal(text,signed)
            if nargin<2,signed=false;end
            text=char(text);negative=~isempty(text)&&text(1)=='-';if negative,text=text(2:end);end
            assert(~isempty(text)&&all(text>='0'&text<='9'),'Furuta:Integer','Invalid decimal integer');
            v=uint64(0);limit=intmax('uint64');if signed,limit=uint64(intmax('int64'))+uint64(negative);end
            for k=1:numel(text)
                d=uint64(double(text(k))-double('0'));
                assert(v<=idivide(limit-d,uint64(10),'floor'),'Furuta:Integer','Integer out of range');v=v*uint64(10)+d;
            end
            if signed
                if negative&&v==bitshift(uint64(1),63),value=intmin('int64');
                elseif negative,value=-int64(v);else,value=int64(v);end
            else,assert(~negative,'Furuta:Integer','Unsigned integer cannot be negative');value=v;end
        end
        function out = cobs(in)
            out=uint8(0); at=1; code=1;
            for b=uint8(in(:).')
                if b==0, out(at)=code; at=numel(out)+1; out(at)=0; code=1;
                else
                    out(end+1)=b; code=code+1; %#ok<AGROW>
                    if code==255, out(at)=code; at=numel(out)+1; out(at)=0; code=1; end
                end
            end
            out(at)=code; out(end+1)=0;
        end
        function out = uncobs(in)
            in=uint8(in(:).');out=zeros(1,numel(in),'uint8');i=1;used=0;
            while i<=numel(in)
                code=double(in(i)); i=i+1;
                assert(code>0 && i+code-2<=numel(in),'Furuta:COBS','Invalid COBS block');
                part=in(i:i+code-2); assert(all(part~=0),'Furuta:COBS','Zero inside block');
                out(used+(1:numel(part)))=part;used=used+numel(part);i=i+code-1;
                if code<255 && i<=numel(in),used=used+1;out(used)=0;end
                assert(used<=2048,'Furuta:Size','Oversized frame');
            end
            out=out(1:used);
        end
        function bytes = encode(type,session,seq,payload)
            payload=uint8(payload(:).');
            assert(numel(payload)<=2026,'Furuta:Size','Payload exceeds protocol limit');
            b=[uint8([1,type]),furuta.Protocol.le(uint16(0)),furuta.Protocol.le(uint64(session)),...
                furuta.Protocol.le(uint32(seq)),furuta.Protocol.le(uint16(numel(payload))),payload];
            bytes=furuta.Protocol.cobs([b,furuta.Protocol.le(furuta.Protocol.crc(b))]);
        end
        function p = decode(encoded)
            b=furuta.Protocol.uncobs(encoded);
            assert(numel(b)>=22 && b(1)==1,'Furuta:Version','Invalid version/length');
            n=double(furuta.Protocol.read(b(17:18),'uint16'));
            assert(numel(b)==22+n,'Furuta:Length','Length mismatch');
            assert(furuta.Protocol.crc(b(1:end-4))==furuta.Protocol.read(b(end-3:end),'uint32'),'Furuta:CRC','CRC mismatch');
            p=struct('Type',b(2),'Session',furuta.Protocol.read(b(5:12),'uint64'),...
                'Sequence',furuta.Protocol.read(b(13:16),'uint32'),'Payload',b(19:end-4));
        end
        function bytes = command(session,id,op,args)
            if nargin<4, args=struct; end
            args.id=double(id); args.op=char(op);
            bytes=furuta.Protocol.encode(1,session,id,unicode2native(jsonencode(args),'UTF-8'));
        end
        function [names,types,widths] = layout()
            names={'DeviceUs','Sequence','Count','PendRaw','AdcRaw','CurrentMv','FilteredMv','Requested','Applied',...
                'PendAgeUs','AdcAgeUs','MagStatus','AGC','Magnitude','Validity','LimitFlags','ApplyUs','AdcUs','PendUs','State'};
            types={'uint64','uint32','int64','uint16','uint16','single','single','single','single',...
                'uint32','uint32','uint8','uint8','uint16','uint32','uint32','uint64','uint64','uint64','uint8'};
            widths=[8 4 8 2 2 4 4 4 4 4 4 1 1 2 4 4 8 8 8 1];
        end
        function T = emptySamples()
            [names,types]=furuta.Protocol.layout();cols=cell(size(names));
            for k=1:numel(names),cols{k}=zeros(0,1,types{k});end
            T=table(cols{:},'VariableNames',names);
            T.BootId=zeros(0,1,'uint64'); T.ConfigId=zeros(0,1,'uint32');
            T.RunId=zeros(0,1,'uint32'); T.Dropped=zeros(0,1,'uint32');
        end
        function T = unpackCore(bytes,n)
            assert(numel(bytes)==88*n,'Furuta:Samples','Core sample length mismatch');
            a=reshape(uint8(bytes),88,n); [names,types,widths]=furuta.Protocol.layout();
            cols=cell(size(names));at=1;
            for k=1:numel(names)
                part=a(at:at+widths(k)-1,:);cols{k}=furuta.Protocol.read(part(:),types{k}).';
                at=at+widths(k);
            end
            T=table(cols{:},'VariableNames',names);
        end
        function T = sampleBatches(packets)
            cores=cell(size(packets));meta=cell(size(packets));
            for k=1:numel(packets)
                b=packets{k}.Payload;assert(numel(b)>=22,'Furuta:Samples','Missing batch header');
                n=double(furuta.Protocol.read(b(21:22),'uint16'));
                assert(numel(b)==22+88*n,'Furuta:Samples','Core sample length mismatch');
                cores{k}=b(23:end);meta{k}=repmat(b(1:20).',1,n);
            end
            bytes=[cores{:}];m=[meta{:}];T=furuta.Protocol.unpackCore(bytes,size(m,2));
            T.BootId=furuta.Protocol.read(reshape(m(1:8,:),1,[]),'uint64').';
            T.ConfigId=furuta.Protocol.read(reshape(m(9:12,:),1,[]),'uint32').';
            T.RunId=furuta.Protocol.read(reshape(m(13:16,:),1,[]),'uint32').';
            T.Dropped=furuta.Protocol.read(reshape(m(17:20,:),1,[]),'uint32').';
        end
        function T = samples(p)
            b=p.Payload; assert(numel(b)>=22,'Furuta:Samples','Missing batch header');
            n=double(furuta.Protocol.read(b(21:22),'uint16')); T=furuta.Protocol.unpackCore(b(23:end),n);
            T.BootId=repmat(furuta.Protocol.read(b(1:8),'uint64'),n,1);
            T.ConfigId=repmat(furuta.Protocol.read(b(9:12),'uint32'),n,1);
            T.RunId=repmat(furuta.Protocol.read(b(13:16),'uint32'),n,1);
            T.Dropped=repmat(furuta.Protocol.read(b(17:20),'uint32'),n,1);
        end
        function b = packCore(T)
            [names,~,widths]=furuta.Protocol.layout(); a=zeros(88,height(T),'uint8'); at=1;
            for k=1:numel(names)
                a(at:at+widths(k)-1,:)=reshape(furuta.Protocol.le(T.(names{k})),widths(k),[]);
                at=at+widths(k);
            end
            b=a(:).';
        end
        function b = packSamples(T)
            b=[furuta.Protocol.le(T.BootId(1)),furuta.Protocol.le(T.ConfigId(1)),...
                furuta.Protocol.le(T.RunId(1)),furuta.Protocol.le(T.Dropped(end)),...
                furuta.Protocol.le(uint16(height(T))),furuta.Protocol.packCore(T)];
        end
        function result = burst(p)
            b=p.Payload; assert(numel(b)>=11,'Furuta:Burst','Missing chunk header');
            result.Id=double(furuta.Protocol.read(b(1:4),'uint32'));
            result.Offset=double(furuta.Protocol.read(b(5:8),'uint32'));
            n=double(furuta.Protocol.read(b(9:10),'uint16')); result.Channel=b(11);
            if result.Channel==1
                result.Data=furuta.Protocol.unpackCore(b(12:end),n);
            else
                assert(numel(b)==11+22*n,'Furuta:Burst','ADC chunk length mismatch');
                a=reshape(b(12:end),22,n);
                result.Data=table(furuta.Protocol.read(reshape(a(1:8,:),1,[]),'uint64').',...
                    furuta.Protocol.read(reshape(a(9:10,:),1,[]),'uint16').',...
                    furuta.Protocol.read(reshape(a(11:14,:),1,[]),'single').',...
                    furuta.Protocol.read(reshape(a(15:18,:),1,[]),'single').',...
                    furuta.Protocol.read(reshape(a(19:22,:),1,[]),'single').',...
                    'VariableNames',{'DeviceUs','AdcRaw','CurrentMv','Requested','Applied'});
            end
        end
    end
end
