classdef TwinService
    % Sensor-driven geometry. Unknown arm scale never invents an output angle.
    methods (Static)
        function [points,note] = pose(sample,c,status,previewRaw)
            q=0;a=NaN;notes={};
            centered=isfield(status,'centered')&&status.centered;
            if centered&&isfinite(c.countsPerRev)&&c.countsPerRev>0&&bitand(sample.Validity,1)~=0
                center=furuta.Protocol.decimal(status.centerCount,true);
                q=c.armSign*2*pi*double(sample.Count-center)/c.countsPerRev;
            else
                notes{end+1}='Arm held at center in preview: set arm center AND fit counts/output revolution in Calibration.';
            end
            if bitand(sample.Validity,2)~=0
                if isfinite(c.pendDownRaw)
                    a=c.pendSign*(double(sample.PendRaw)-c.pendDownRaw)*2*pi/4096+pi;
                elseif isfinite(previewRaw)
                    a=c.pendSign*(double(sample.PendRaw)-previewRaw)*2*pi/4096+pi;
                    notes{end+1}='Pendulum relative preview: first valid position shown down. Click Pendulum down here for a physical reference.';
                end
            else
                notes{end+1}='Pendulum data invalid/stale; rod hidden.';
            end
            base=[cos(q),sin(q),0];tip=base+.8*[sin(a)*(-sin(q)),sin(a)*cos(q),cos(a)];
            points=[0 0 0;base;tip];
            if isempty(notes),note='Measured arm and pendulum angles. Schematic uses unit lengths.';
            else,note=strjoin(notes,' ');end
        end
    end
end
