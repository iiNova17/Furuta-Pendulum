classdef OpenLoopDuty < FurutaController
    %OPENLOOPDUTY Plays a duty sequence, ignoring the measurements.
    %   Use it to replay an experiment (e.g. the report's square waves) through the harness.
    %   Knobs (P): t [s] and d (duty) column vectors, held between samples (zero-order hold).
    %   Default: the 25 % / 1.4 Hz square wave of the report's fit experiments.

    properties (Access = private)
        k
    end

    methods (Static)
        function P = design(~, P)
            if nargin < 2, P = struct(); end
            t = (0:1e-3:8)';
            P = FurutaController.defaults(P, struct('t', t, 'd', 0.25*sign(sin(2*pi*1.4*t))));
        end
    end

    methods (Access = protected)
        function resetImpl(obj)
            obj.k = 0;
        end

        function [duty, mode, dbg] = stepImpl(obj, ~, ~)
            t = obj.k*obj.Ts;
            obj.k = obj.k + 1;
            n = find(obj.P.t <= t + 1e-12, 1, 'last');
            if isempty(n)
                duty = 0;
            else
                duty = obj.P.d(n);
            end
            mode = 0;
            dbg = zeros(4, 1);
        end
    end
end
