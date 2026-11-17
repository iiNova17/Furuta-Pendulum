classdef FirmwareActuator < matlab.System
    %FIRMWAREACTUATOR What the firmware does to a duty command before it reaches the motor.
    %   [d, fault] = step(obj, duty_cmd, y)
    %   Mirrors fw::Controller::tick in firmware/include/engine.h: safety trips first (arm travel,
    %   overcurrent on the filtered current, non-finite command), then clip to max_duty, slew
    %   limit, pass through zero (one standby tick) before reversing, then 10-bit PWM.
    %   After a trip the motor stays off (d = 0) for the rest of the run, as on the rig.
    %   fault: 0 ok, 1 ARM_TRAVEL, 2 OVERCURRENT, 3 non-finite duty command.
    %   Settings: cfg.act from harness_config.

    properties (Nontunable)
        Ts = 1e-3
        A = struct('max_duty', 0.5, 'slew', 100, 'standby', 1e-3, 'duty_levels', 1023, ...
                   'travel', 2*pi/3, 'i_trip', 1.5, 'i_trip_time', 10e-3, 'reverse_through_zero', true)
    end

    properties (Access = private)
        applied
        standby_left
        fault
        trip_ticks
    end

    methods
        function obj = FirmwareActuator(varargin)
            % Name-value construction, e.g. FirmwareActuator('Ts', 1e-3, ...) for use outside Simulink.
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function resetImpl(obj)
            obj.applied = 0;
            obj.standby_left = 0;
            obj.fault = 0;
            obj.trip_ticks = 0;
        end

        function [d, fault] = stepImpl(obj, cmd, y)
            A = obj.A;
            if obj.fault == 0
                if ~isfinite(cmd)
                    obj.fault = 3;
                elseif abs(y(1)) > A.travel
                    obj.fault = 1;
                elseif abs(y(3)) > A.i_trip
                    obj.trip_ticks = obj.trip_ticks + 1;
                    if obj.trip_ticks*obj.Ts >= A.i_trip_time
                        obj.fault = 2;
                    end
                else
                    obj.trip_ticks = 0;
                end
            end
            fault = obj.fault;
            if obj.fault ~= 0
                obj.applied = 0;
                d = 0;
                return
            end
            target = min(max(cmd, -A.max_duty), A.max_duty);
            if A.reverse_through_zero && obj.applied*target < 0
                target = 0;                       % go to zero before reversing
            end
            rate = A.slew*obj.Ts;
            nxt = obj.applied + min(max(target - obj.applied, -rate), rate);
            if abs(nxt) < 1e-9 && abs(obj.applied) > 1e-9
                nxt = 0;
                obj.standby_left = round(A.standby/obj.Ts);
            end
            if obj.standby_left > 0
                nxt = 0;
                obj.standby_left = obj.standby_left - 1;
            end
            obj.applied = nxt;
            d = nxt;
            if isfinite(A.duty_levels)
                d = round(d*A.duty_levels)/A.duty_levels;
            end
        end

        function n = getNumInputsImpl(~), n = 2; end
        function n = getNumOutputsImpl(~), n = 2; end
        function [s1, s2] = getOutputSizeImpl(~), s1 = [1 1]; s2 = [1 1]; end
        function [t1, t2] = getOutputDataTypeImpl(~), t1 = 'double'; t2 = 'double'; end
        function [c1, c2] = isOutputComplexImpl(~), c1 = false; c2 = false; end
        function [f1, f2] = isOutputFixedSizeImpl(~), f1 = true; f2 = true; end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end
end
