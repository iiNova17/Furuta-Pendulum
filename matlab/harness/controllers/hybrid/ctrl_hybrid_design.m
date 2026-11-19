function P = ctrl_hybrid_design(cfg, P)
%CTRL_HYBRID_DESIGN Every knob of the hybrid controller (ctrl_hybrid.slx), returned as ctrlP.
%   TEMPLATE: all values below are placeholders (TODO). Each owner fills in their own section.
%
%   P = ctrl_hybrid_design(cfg)        your values from this file
%   P = ctrl_hybrid_design(cfg, P)     overrides win, also nested ones, e.g.
%       run_sim('ctrl_hybrid.slx', 'push', 'Params', struct('stab', 1, 'pid', struct('Kp', 8)))
%
%   Design model available here: cfg.lin.A_up, cfg.lin.Bd_up (report eq. 104, duty input),
%   discretized with zero-order hold at the controller period cfg.Ts. cfg.p holds Table 6.
%   States everywhere: x = [theta0; theta1; dtheta0; dtheta1], theta1 = 0 upright.
if nargin < 2, P = struct(); end

% ---- which stabilizer runs in the Stabilize state ----------------------------------------------
D.stab = 3;                          % 1 = PID, 2 = pole placement, 3 = LQR

% ---- shared state estimator (ctrl_hybrid.slx / estimator) -- integration owner -----------------
D.est.vel_fc = 40;                   % TODO velocity filter cut-off [Hz]

% ---- supervisor, Stateflow (ctrl_hybrid.slx / Supervisor) -- Stateflow owner --------------------
D.sup.catch_th1 = deg2rad(20);       % TODO SwingUp -> Stabilize when |theta1| < catch_th1 ...
D.sup.catch_w1 = 8;                  % TODO ... and |dtheta1| < catch_w1 [rad/s]  (capture region)
D.sup.exit_th1 = deg2rad(35);        % TODO Stabilize -> SwingUp when |theta1| > exit_th1
D.sup.safe_th0 = deg2rad(100);       % TODO any -> SafeStop when |theta0| > safe_th0 (keep it below
                                     %      the firmware's 120 deg ARM_TRAVEL trip)
D.sup.rearm = 0;                     % 1: SafeStop -> SwingUp once the arm is back and the pendulum
                                     % hangs still; 0: stay stopped (e-stop behaviour)

% ---- PID owner (stab_pid.slx) -------------------------------------------------------------------
% u = PID(theta1) + Kp0 (theta0 - r) + Kd0 dtheta0. The integrator resets on entering Stabilize.
D.pid = struct('Kp', 0, 'Ki', 0, 'Kd', 0, 'N', 50, 'Kp0', 0, 'Kd0', 0);   % TODO your gains

% ---- pole-placement owner (stab_pole_placement.slx) ---------------------------------------------
% Continuous-time closed-loop poles [rad/s], mapped to z = exp(s Ts); K = place(Ad, Bd, z).
D.pp.poles = [];                     % TODO 4 poles from your specs; empty = K = 0

% ---- LQR owner (stab_lqr.slx) -------------------------------------------------------------------
% K = dlqr(Ad, Bd, Q, R). Bryson's rule: Q = diag(1/max_x^2), R = 1/max_duty^2 (report section 9).
% Largest acceptable theta0 30 deg, theta1 8 deg, w0 20 rad/s, w1 30 rad/s, duty 0.10:
% K = [-0.1795 -2.155 -0.1256 -0.1586]. Chosen for tolerance to delay (lqr_corner_test).
D.lqr.Q = diag(1./[deg2rad(30) deg2rad(8) 20 30].^2);
D.lqr.R = 1/0.10^2;

% ---- swing-up owner (swingup_energy.slx) --------------------------------------------------------
% Energy pumping with a current-commanding motor inverse (report section 10; the block reads
% ctrlP.swing.<name>). Starting values from physics: swingup_knobs.m. Energy e in units of 2G.
% a_push and wc chosen from the design map and the worst cases (swingup_sweep, swingup_corner_test):
% 48/48 worst-case runs, catch in 1.4-1.5 s, arm within 50 deg.
D.swing.E0 = 0.048;                  % energy target above the top: friction loss of the last quarter swing
D.swing.k_E = 21;                    % push fades within 1/k_E of the target (= 1/E0)
D.swing.a_push = 80;                 % pivot acceleration while pumping [rad/s^2] (physics start: 52)
D.swing.wc = 6.5;                    % arm centring natural frequency [rad/s] (physics start: 4.2)
D.swing.zc = 0.7;                    % arm centring damping
D.swing.a_kick = 69;                 % pivot acceleration of the kick from rest [rad/s^2]
D.swing.kick_E = 0.0019;             % kick while e < -1 + kick_E (pendulum swings < 5 deg)
D.swing.w_sign = 1;                  % smooth sign width on w1 cos(theta1) [rad/s]
D.swing.w_fric = 1;                  % smooth sign width on w0 for the friction term [rad/s]
D.swing.i_max = 1.0;                 % motor current command limit [A] (firmware trips at 1.5 A)
D.swing.u_max = 0.45;                % duty limit (firmware clips at 0.5)
% Breakaway feedforward in the motor inverse (firmware 1.8): with the arm at rest the friction term is the
% breakaway (13 % / 14 % duty, measured) in the direction of a0, fading to the Coulomb term as the arm moves.
% On the rig the kick from rest asked for 11 % duty without it and the arm never moved (2026-11-19).
D.swing.stick_gain = 1;              % 1 = on, 0 = off (the law before 1.8)
D.swing.w_stick = 0.2;               % arm speed over which the breakaway part fades, exp(-(w0/w_stick)^2) [rad/s]
D.swing.a_stick = 5;                 % smooth sign width on a0 for the breakaway part [rad/s^2]

% ---- friction compensation (ctrl_hybrid.slx), applied to the stabilizer output -----------------
D.fric.gain = 1;                     % fraction of the identified friction cancelled (was 0 for the first
                                     % hardware tests, report section 9.9). Set to 1 for firmware 1.9: with 0 the
                                     % LQR's corrections stuck below the 13 % breakaway and the arm lurched free
                                     % (run 20261119_205453); simulation: arm stuck 41-49 % of the time, 4 % with
                                     % gain 1 and stick 1.
D.fric.stick = 1;                    % 1.9: breakaway duty at rest, fading to Coulomb as the arm moves; 0 = Coulomb only
D.fric.band = 0.02;                  % width of the smooth sign [duty]

P = merge(D, P);

% ---- derived from your choices above (recomputed every run; do not edit) ------------------------
P.Ts = cfg.Ts;
sysd = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), cfg.Ts);
P.lqr.K = zeros(1, 4);
if ~isempty(P.lqr.Q), P.lqr.K = dlqr(sysd.A, sysd.B, P.lqr.Q, P.lqr.R); end
P.pp.K = zeros(1, 4);
if ~isempty(P.pp.poles), P.pp.K = place(sysd.A, sysd.B, exp(P.pp.poles(:)*cfg.Ts)); end
p = cfg.p;
P.fric.duty = [p.Rm*p.tcp, p.Rm*p.tcn]/p.Vs;     % identified Coulomb friction as duty (report 5.7)
P.plant = struct('Jp', p.Jp, 'G', p.G, 'h', p.h, 'Ja', p.Ja, 'Beq', p.Beq, ...
                 'tcp', p.tcp, 'tcn', p.tcn, 'Rm', p.Rm, 'Kb', p.Kb, 'Vs', p.Vs);
% arm breakaway friction (measured dead zone), also when the harness plant runs without it (harness_config)
bk = struct('tsp', 0, 'tsn', 0);
if isfield(cfg, 'breakaway'), bk = cfg.breakaway; end
P.plant.tsp = bk.tsp;
P.plant.tsn = bk.tsn;
P.fric.stick_duty = [p.Rm*bk.tsp, p.Rm*bk.tsn]/p.Vs;   % breakaway as duty (13 % / 14 %)
P = no_empty(P);     % Simulink/Stateflow parameters cannot hold empty fields
end

function S = no_empty(S)
for f = fieldnames(S)'
    if isstruct(S.(f{1})), S.(f{1}) = no_empty(S.(f{1}));
    elseif isempty(S.(f{1})), S.(f{1}) = NaN;
    end
end
end

function A = merge(A, B)
% Recursive struct merge: fields of B override A, nested structs merged field by field.
for f = fieldnames(B)'
    k = f{1};
    if isfield(A, k) && isstruct(A.(k)) && isstruct(B.(k))
        A.(k) = merge(A.(k), B.(k));
    else
        A.(k) = B.(k);
    end
end
end
