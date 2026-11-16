classdef EnergySwingup < LqrBalance
    %ENERGYSWINGUP Example controller: energy-based swing-up, then LqrBalance near the top.
    %   Pendulum energy in the report's scaled units (eq. 89), 0 at rest upright:
    %       E = 1/2 Jp w1^2 + G (cos th1 - 1),      e = -E / (2G)   (1 hanging at rest, 0 upright)
    %   From the pendulum equation, dE/dt = -h cos(th1) w1 a0 (+ friction). The motor behaves like
    %   a velocity source (34 ms mechanical time constant, report Table 7), so the law commands
    %   an arm VELOCITY:
    %       w0_des = -k_swing * e * sin(th1)   =>   a0 ~ -k_swing e cos(th1) w1
    %                                          =>   dE/dt ~ h k_swing e cos^2(th1) w1^2 >= 0
    %   which pumps energy in until e = 0 and keeps the arm excursion bounded (a constant
    %   acceleration would run the arm into the 120 deg travel limit). Arm centering: -kp_arm*th0.
    %   The duty follows from the arm's steady state (eq. 88-89): V = Kb w + Rm (Beq w + tau_c),
    %   plus kv on the velocity error. A kick starts the swing from rest.
    %   Switches to the LQR when |th1| < catch_deg and back when |th1| > exit_deg.
    %
    %   Knobs (P): k_swing [rad/s], v_max [rad/s], kp_arm [1/s], kv [duty s/rad], kick [rad/s],
    %              exit_deg, plus the LqrBalance knobs.

    properties (Access = private)
        balancing
    end

    methods (Static)
        function P = design(cfg, P)
            if nargin < 2, P = struct(); end
            P = FurutaController.defaults(P, struct( ...
                'k_swing', 20, 'v_max', 6, 'kp_arm', 3, 'kv', 0.05, 'kick', 4, ...
                'catch_deg', 20, 'exit_deg', 35));
            P = LqrBalance.design(cfg, P);
            p = cfg.p;
            P.plant = struct('Jp', p.Jp, 'G', p.G, 'Beq', p.Beq, 'tc', (p.tcp + p.tcn)/2, ...
                             'Rm', p.Rm, 'Kb', p.Kb, 'Vs', p.Vs);
        end
    end

    methods (Access = protected)
        function resetImpl(obj)
            resetImpl@LqrBalance(obj);
            obj.balancing = false;
        end

        function [duty, mode, dbg] = stepImpl(obj, y, r)
            P = obj.P;
            [th0, th1, w0, w1] = obj.estimate(y);
            if obj.balancing && abs(th1) > deg2rad(P.exit_deg)
                obj.balancing = false;
            elseif ~obj.balancing && abs(th1) < deg2rad(P.catch_deg)
                obj.balancing = true;
            end
            m = P.plant;
            E = 0.5*m.Jp*w1^2 + m.G*(cos(th1) - 1);
            e = -E/(2*m.G);
            if obj.balancing
                duty = obj.balance(th0 - r, th1, w0, w1);
            else
                wd = -P.k_swing*e*sin(th1);
                if e > 0.9 && abs(w1) < 0.5
                    wd = P.kick;                             % start from rest
                end
                wd = min(max(wd, -P.v_max), P.v_max) - P.kp_arm*(th0 - r);
                V = m.Kb*wd + m.Rm*(m.Beq*wd + m.tc*tanh(wd/0.1));
                duty = V/m.Vs + P.kv*(wd - w0);
            end
            mode = 1 + obj.balancing;
            dbg = [w0; w1; E; obj.balancing];
        end
    end
end
