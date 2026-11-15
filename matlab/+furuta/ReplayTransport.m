classdef ReplayTransport < handle
    properties
        OnPacket = []
        OnRaw = []
        OnError = []
        Run
        Index = 1
        Elapsed = 0
        Playing = true
        Speed = 1
        Sequence = uint32(0)
    end
    methods
        function obj = ReplayTransport(run),obj.Run=run;end
        function write(~,~),error('Furuta:Replay','Replay transport cannot send hardware commands');end
        function step(obj,dt)
            T=obj.Run.Samples;if ~obj.Playing||isempty(T)||obj.Index>height(T),return;end
            obj.Elapsed=obj.Elapsed+dt*obj.Speed;t=double(T.DeviceUs-T.DeviceUs(1))*1e-6;
            last=find(t<=obj.Elapsed,1,'last');if isempty(last)||last<obj.Index,return;end
            for k=obj.Index:16:last
                p=struct('Type',uint8(4),'Session',uint64(0),'Sequence',obj.Sequence,'Payload',furuta.Protocol.packSamples(T(k:min(k+15,last),:)));
                obj.Sequence=obj.Sequence+1;if ~isempty(obj.OnPacket),obj.OnPacket(p);end
            end
            obj.Index=last+1;
        end
        function delete(obj),obj.Playing=false;obj.OnPacket=[];end
    end
end
