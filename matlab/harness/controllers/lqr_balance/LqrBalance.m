classdef LqrBalance < FurutaController
    %LQRBALANCE Example controller: discrete LQR about the upright position.
    %   Model: report eq. 104 (upright, duty input), discretized at Ts as in eq. 108.
    %   Velocities: low-pass filtered finite differences of the measured angles.
    %   Friction: adds the duty that cancels the arm's Coulomb friction (report 5.7),
    %   switched smoothly by the sign of the LQR output.
    %   Only acts within catch_deg of upright; outputs zero duty elsewhere.
    %
    %   Knobs (P): q_th0 q_th1 q_w0 q_w1 r_duty (LQR weights), vel_fc [Hz], fric_gain,
    %              fric_band [duty], catch_deg.  Derived: K, fric.

    properties (Access = protected)
        prev   % previous measured [theta0; theta1]
        w      % velocity estimate [w0; w1]
    end

    methods (Static)
        function P = design(cfg, P)
            if nargin < 2, P = struct(); end
            P = FurutaController.defaults(P, struct( ...
                'q_th0', 5, 'q_th1', 100, 'q_w0', 0.05, 'q_w1', 0.1, 'r_duty', 300, ...
                'vel_fc', 40, 'fric_gain', 0.8, 'fric_band', 0.02, 'catch_deg', 25));
            % r_duty: much smaller (stiffer gains) balances with ideal sensors but, with the
            % rig's 7 ms pendulum lag and the slew limit, saturates and trips OVERCURRENT.
            sys = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), cfg.Ts);
            Q = diag([P.q_th0, P.q_th1, P.q_w0, P.q_w1]);
            P.K = dlqr(sys.A, sys.B, Q, P.r_duty);
            p = cfg.p;
            P.fric = [p.Rm*p.tcp, p.Rm*p.tcn]/p.Vs;   % ~[0.074 0.073] duty
        end
    end

    methods (Access = protected)
        function resetImpl(obj)
            obj.prev = [];
            obj.w = [0; 0];
        end

        function [duty, mode, dbg] = stepImpl(obj, y, r)
            [th0, th1, w0, w1] = obj.estimate(y);
            active = abs(th1) < deg2rad(obj.P.catch_deg);
            u = obj.balance(th0 - r, th1, w0, w1);
            duty = active*u;
            mode = 2*active;
            dbg = [w0; w1; u; active];
        end

        function [th0, th1, w0, w1] = estimate(obj, y)
            th0 = y(1);
            th1 = y(2);
            if isempty(obj.prev)
                obj.prev = [th0; th1];
            end
            a = exp(-2*pi*obj.P.vel_fc*obj.Ts);
            dth = [th0 - obj.prev(1); wrap_pi(th1 - obj.prev(2))]/obj.Ts;
            obj.w = a*obj.w + (1 - a)*dth;
            obj.prev = [th0; th1];
            w0 = obj.w(1);
            w1 = obj.w(2);
        end

        function u = balance(obj, e0, th1, w0, w1)
            u = -obj.P.K*[e0; th1; w0; w1];
            s = tanh(u/obj.P.fric_band);
            u = u + obj.P.fric_gain*(obj.P.fric(1)*max(s, 0) + obj.P.fric(2)*min(s, 0));
        end
    end
end
