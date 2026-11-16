classdef MyController < FurutaController
    %MYCONTROLLER Template: copy this folder to controllers/<yourname>/ and rename the class + file.
    %
    %   Then:   run_sim('YourClass', 'balance', 'Plot', true)
    %           compare_controllers({'LqrBalance', 'YourClass'}, {'balance', 'push'})
    %
    %   Rules of the slot (see FurutaController):
    %     inputs   y = [theta0; theta1; i]  (measured, theta1 wrapped, 0 = upright), r = arm reference
    %     outputs  duty in [-1, 1], mode (0 stopped, 1 swing-up, 2 stabilizing), dbg (4x1, logged)
    %   For the course deliverable (Stateflow supervisor, PID / pole placement / LQR) use the
    %   Simulink template instead: controllers/hybrid/ctrl_hybrid.slx. This class is for quick tests.
    %     called every obj.Ts (cfg.Ts); keep state in private properties, clear it in resetImpl.

    properties (Access = private)
        % your controller state, e.g. previous measurements, integrator, observer state
        prev
    end

    methods (Static)
        function P = design(cfg, P)
            % Defaults for your knobs; anything passed to run_sim('Params', ...) wins.
            if nargin < 2, P = struct(); end
            P = FurutaController.defaults(P, struct('gain', 0));
            % Derived values from the identified model, e.g.:
            % P.K = place(cfg.lin.A_up, cfg.lin.Bd_up, P.poles);
            P.example_model_pole = max(real(eig(cfg.lin.A_up)));   % +12.26 rad/s, report Table 7
        end
    end

    methods (Access = protected)
        function resetImpl(obj)
            obj.prev = [];
        end

        function [duty, mode, dbg] = stepImpl(obj, y, r) %#ok<INUSD>
            duty = obj.P.gain*y(2);      % <-- your control law here
            mode = 2;
            dbg = zeros(4, 1);
        end
    end
end
