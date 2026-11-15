classdef SerialTransport < handle
    properties
        Port
        Parser = furuta.Protocol()
        OnPacket = []
        OnRaw = []
        OnError = []
        Closed = false
        Pumping = false
        MaxBacklog = 0
    end
    methods
        function obj = SerialTransport(port,baud)
            obj.Port=serialport(port,baud,'Timeout',.1);
            setDTR(obj.Port,false);setRTS(obj.Port,false);
            % Each transport owns a fresh parser (a handle default would be
            % shared between instances) and starts unsynchronized.
            flush(obj.Port,'input');obj.Parser=furuta.Protocol();obj.Parser.Synced=false;
            % One owner drains the port from DeviceSession.poll. A byte callback
            % can interrupt graphics/table operations and re-enter the parser.
            configureCallback(obj.Port,'off');
        end
        function write(obj,bytes)
            assert(~obj.Closed,'Furuta:Serial','Transport closed');write(obj.Port,uint8(bytes),'uint8');
        end
        function step(obj,~),obj.pump();end
        function pump(obj)
            if obj.Closed||obj.Pumping,return;end
            obj.Pumping=true;guard=onCleanup(@()obj.release()); %#ok<NASGU>
            try
                n=obj.Port.NumBytesAvailable;
                obj.MaxBacklog=max(obj.MaxBacklog,n);
                assert(n<262144,'Furuta:SerialBacklog','Serial processing fell behind. Disconnect and reconnect before running.');
                if n>0
                    b=read(obj.Port,min(n,16384),'uint8');
                    if ~isempty(obj.OnRaw),obj.OnRaw(uint8(b));end
                    ps=obj.Parser.feed(b);
                    for k=1:numel(ps),if ~isempty(obj.OnPacket),obj.OnPacket(ps{k});end,end
                end
            catch ex
                if ~isempty(obj.OnError),obj.OnError(ex.message);end
            end
        end
        function release(obj),obj.Pumping=false;end
        function delete(obj)
            obj.Closed=true;
            try,configureCallback(obj.Port,'off');catch,end
            obj.Port=[];obj.OnPacket=[];obj.OnRaw=[];obj.OnError=[];
        end
    end
end
