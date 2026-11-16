classdef (Abstract) FurutaController < matlab.System
    %FURUTACONTROLLER The interface every System-object controller in the harness implements.
    %
    %   Called every Ts (1 ms) with what the ESP32-S3 measures:
    %     y = [theta0; theta1; i]   arm angle [rad] (0 = centre), pendulum angle [rad]
    %                               (wrapped to [-pi, pi), 0 = upright, +-pi = hanging), current [A]
    %     r = arm angle reference [rad]
    %   and returns
    %     duty  in [-1, 1]  (the firmware clips it to cfg.act.max_duty, slew-limits it, ...)
    %     mode  hybrid state for the dashboard: 0 = stopped / safe, 1 = swing-up, 2 = stabilizing
    %     dbg   4x1, anything you want logged (estimates, ...); zeros if unused
    %
    %   Velocities are NOT measured: estimate them yourself (see LqrBalance for an example).
    %
    %   Tuning knobs live in the struct obj.P. The static method design(cfg, P) fills in your
    %   defaults for any field P does not set and computes derived values (gains, ...) from the
    %   identified model in cfg. run_sim calls it, so P passed to run_sim only needs the fields
    %   you want to change:  run_sim('LqrBalance', 'balance', 'Params', struct('q_th1', 500))
    %
    %   To make your own: copy controllers/template/ to controllers/<yourname>/, rename the class.

    properties (Nontunable)
        Ts = 1e-3
        P = struct()
    end

    methods
        function obj = FurutaController(varargin)
            % Name-value construction, e.g. FurutaController('Ts', 1e-3, ...) for use outside Simulink.
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Static)
        function P = design(cfg, P) %#ok<INUSL>
            % Override in your class: fill defaults and derive gains from cfg (harness_config).
        end
    end

    methods (Static, Access = protected)
        function P = defaults(P, D)
            % Copy every field of D that P does not already set.
            f = fieldnames(D);
            for k = 1:numel(f)
                if ~isfield(P, f{k})
                    P.(f{k}) = D.(f{k});
                end
            end
        end
    end

    methods (Access = protected)
        function n = getNumInputsImpl(~), n = 2; end
        function n = getNumOutputsImpl(~), n = 3; end
        function [s1, s2, s3] = getOutputSizeImpl(~), s1 = [1 1]; s2 = [1 1]; s3 = [4 1]; end
        function [t1, t2, t3] = getOutputDataTypeImpl(~), [t1, t2, t3] = deal('double'); end
        function [c1, c2, c3] = isOutputComplexImpl(~), [c1, c2, c3] = deal(false); end
        function [f1, f2, f3] = isOutputFixedSizeImpl(~), [f1, f2, f3] = deal(true); end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end
end
