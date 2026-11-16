classdef FurutaSensors < matlab.System
    %FURUTASENSORS What a controller on the ESP32-S3 measures, sampled every Ts.
    %   y = [theta0; theta1; i]
    %   theta0  arm angle from the motor encoder, quantized to 2*pi/arm_cpr, 0 = centre
    %   theta1  pendulum angle from the AS5600: 12 bit, read every pend_period, lagging by
    %           pend_delay, wrapped to [-pi, pi) with 0 = upright and +-pi = hanging
    %   i       armature current plus noise through the MCU's 40 Hz Butterworth filter
    %   Settings: cfg.sens from harness_config.

    properties (Nontunable)
        Ts = 1e-3
        S = struct('arm_cpr', 1496, 'pend_cpr', 4096, 'pend_period', 2e-3, 'pend_delay', 7e-3, ...
                   'i_noise_sd', 0.05, 'i_fc', 40, 'quantize', true, 'i_filter', true)
    end

    properties (Access = private)
        buf     % pendulum angle history for the delay
        held    % last AS5600 reading
        k       % tick counter
        nhold
        b
        a
        z
    end

    methods
        function obj = FurutaSensors(varargin)
            % Name-value construction, e.g. FurutaSensors('Ts', 1e-3, ...) for use outside Simulink.
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.nhold = max(1, round(obj.S.pend_period/obj.Ts));
            K = tan(pi*obj.S.i_fc*obj.Ts);          % bilinear Butterworth, prewarped
            n = 1/(1 + sqrt(2)*K + K^2);
            obj.b = [K^2, 2*K^2, K^2]*n;
            obj.a = [1, 2*(K^2 - 1)*n, (1 - sqrt(2)*K + K^2)*n];
        end

        function resetImpl(obj)
            obj.buf = [];
            obj.held = 0;
            obj.k = 0;
            obj.z = [0; 0];
        end

        function y = stepImpl(obj, x, i, noise)
            th0 = x(1);
            nd = round(obj.S.pend_delay/obj.Ts);
            if isempty(obj.buf)
                obj.buf = repmat(x(2), nd + 1, 1);
            end
            obj.buf = [obj.buf(2:end); x(2)];
            if mod(obj.k, obj.nhold) == 0
                obj.held = obj.buf(1);
            end
            obj.k = obj.k + 1;
            th1 = wrap_pi(obj.held);
            if obj.S.quantize
                q0 = 2*pi/obj.S.arm_cpr;
                q1 = 2*pi/obj.S.pend_cpr;
                th0 = q0*round(th0/q0);
                th1 = wrap_pi(q1*round(th1/q1));
            end
            im = i + obj.S.i_noise_sd*noise;
            if obj.S.i_filter                          % direct form II transposed
                v = obj.b(1)*im + obj.z(1);
                obj.z(1) = obj.b(2)*im - obj.a(2)*v + obj.z(2);
                obj.z(2) = obj.b(3)*im - obj.a(3)*v;
                im = v;
            end
            y = [th0; th1; im];
        end

        function n = getNumInputsImpl(~), n = 3; end
        function n = getNumOutputsImpl(~), n = 1; end
        function sz = getOutputSizeImpl(~), sz = [3 1]; end
        function t = getOutputDataTypeImpl(~), t = 'double'; end
        function c = isOutputComplexImpl(~), c = false; end
        function f = isOutputFixedSizeImpl(~), f = true; end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end
end
