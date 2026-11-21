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
D.stab = 1;                          % 1 = PID, 2 = pole placement, 3 = LQR

% ---- model the controller is DESIGNED with (gains, observer model, friction numbers) -------------
% 20261120: the rig re-identified after it was moved (matlab/model/furuta_params_20261120.m; arm friction
% about halved, breakaway 6 %). 0: the report's Table 6 model (furuta_params.m). The SIMULATED plant is chosen
% separately: harness_config() simulates the 20 Nov model (the default), harness_config(..., 'Model', 'table6') Table 6.
D.design_model = 20261120;

% ---- shared state estimator (ctrl_hybrid.slx / estimator) -- estimator owner --------------------
D.est.vel_fc = 40;                   % velocity filter cut-off [Hz] of the filtered-difference estimator
% Model-assisted observer (est_observer.slx, est_observer_ref.m; study 2026-11-18, retune 2026-11-20).
% The pendulum reading lags the arm encoder by 9-15 ms on the rig (shaft coupling: play + flex, measured
% 2026-11-20); the observer estimates the state at the reading's time and predicts it forward to now through
% the duties it sent. Simulation on the 20 Nov model, 12 ms lag: 1.2 deg rms, no saturation, where the filtered
% differences chatter (4.7 deg, duty at the limit 50 %, as on the rig).
D.est.kind = 1;                      % 1 = model observer, 2 = its level 3 fallback (tracking loop, no model)
D.est.nd = 3;                        % pendulum lag compensated, in controller ticks (3 x 4 ms = 12 ms)
D.est.f_ab = 10;                     % bandwidth of the observer's alpha-beta correction [Hz]
D.est.f_trk = 25;                    % bandwidth of the level 3 tracking loop [Hz]
D.est.sub = 1;                       % RK4 substeps per tick in the observer's model. 1: the ESP32 computes a step in about
                                     % 0.5 ms (2 substeps: 0.93 ms worst, too close to the 1 ms firmware loop, measured 2026-11-20)

% ---- supervisor, Stateflow (ctrl_hybrid.slx / Supervisor) -- Stateflow owner --------------------
D.sup.catch_th1 = deg2rad(20);       % TODO SwingUp -> Stabilize when |theta1| < catch_th1 ...
D.sup.catch_w1 = 8;                  % TODO ... and |dtheta1| < catch_w1 [rad/s]  (capture region)
D.sup.exit_th1 = deg2rad(35);        % TODO Stabilize -> SwingUp when |theta1| > exit_th1
D.sup.safe_th0 = deg2rad(100);       % TODO any -> SafeStop when |theta0| > safe_th0 (keep it below
                                     %      the firmware's 120 deg ARM_TRAVEL trip)
D.sup.rearm = 0;                     % 1: SafeStop -> SwingUp once the arm is back and the pendulum
                                     % hangs still; 0: stay stopped (e-stop behaviour)

% ---- PID owner (stab_pid.slx) -------------------------------------------------------------------
% Two loops added together, on the observer's state xh = [theta0 theta1 w0 w1] (2026-11-21):
%   u = Kp1 theta1 + Kd1 w1  +  Kp0 (theta0 - r) + Ki0 sum(theta0 - r) Ts + Kd0 w0
%       PD on the pendulum      PID on the arm (discrete PI block, forward Euler, + Kd0 on the observer's w0)
% Starting point = the LQR gain (K = [-0.1789 -1.891 -0.1132 -0.1347]: Kp1 = -K(2), Kd1 = -K(4), Kp0 = -K(1),
% Kd0 = -K(3)), plus the arm integral Ki0. The arm integrator runs from the start of the run (no reset, no limit).
D.pid = struct('Kp1', 1.891, 'Kd1', 0.1347, 'Kp0', 0.1789, 'Ki0', 0.1, 'Kd0', 0.1132);

% ---- pole-placement owner (stab_pole_placement.slx) ---------------------------------------------
% Continuous-time closed-loop poles [rad/s], mapped to z = exp(s Ts); K = place(Ad, Bd, z).
D.pp.poles = [];                     % TODO 4 poles from your specs; empty = K = 0

% ---- LQR owner (stab_lqr.slx) -------------------------------------------------------------------
% K = dlqr(Ad, Bd, Q, R). Bryson's rule: Q = diag(1/max_x^2), R = 1/max_duty^2 (report section 9).
% Largest acceptable theta0 30 deg, theta1 8 deg, w0 20 rad/s, w1 30 rad/s, duty 0.10:
% K = [-0.1795 -2.155 -0.1256 -0.1586]. Chosen for tolerance to delay (lqr_corner_test).
% 2026-11-20: same limits, now designed on the 20 Nov model (D.design_model), with the model observer. Before
% (Table 6 model): K = [-0.1795 -2.155 -0.1256 -0.1586]; on the rig it chattered (lag margin 6 ms < 9-15 ms lag).
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
D.fric.gain = 0;                     % fraction of the identified friction cancelled. 0 since the retune of 2026-11-20
                                     % (firmware 1.12, model observer), from rig runs b3_new / b3_new_nocomp, two each:
                                     % pendulum 1.22-1.33 deg rms off vs 1.73-2.10 on, catch 1.4 s vs 1.8-2.2 s, same
                                     % current. On, the switching +-6 % term with the shaft-coupling play makes a 2.9 Hz
                                     % wobble (1-5 Hz band 2.0 vs 1.0 deg); the harness (no play) predicted the opposite.
                                     % It also depends on friction that drifted > 50 % in 5 days. Cost: the arm wanders
                                     % about +-25 deg around its mean (Bryson arm limit 30 deg) instead of +-5 deg.
                                     % 1 = full compensation (firmware 1.9-1.11 default).
D.fric.stick = 1;                    % 1.9: breakaway duty at rest, fading to Coulomb as the arm moves; 0 = Coulomb only
D.fric.band = 0.02;                  % width of the smooth sign [duty]

P = merge(D, P);

% ---- derived from your choices above (recomputed every run; do not edit) ------------------------
P.Ts = cfg.Ts;
dm = 'table6'; if P.design_model == 20261120, dm = '20261120'; end
if ~isfield(cfg, 'model') || ~strcmp(cfg.model, dm)          % plant simulated with another model: design on this one
    cm = harness_config(cfg.mode, 'Model', dm); cfg.lin = cm.lin; cfg.p = cm.p; cfg.breakaway = cm.breakaway;
end
sysd = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), cfg.Ts);
P.lqr.K = zeros(1, 4);
if ~isempty(P.lqr.Q), P.lqr.K = dlqr(sysd.A, sysd.B, P.lqr.Q, P.lqr.R); end
P.pp.K = zeros(1, 4);
if ~isempty(P.pp.poles), P.pp.K = place(sysd.A, sysd.B, exp(P.pp.poles(:)*cfg.Ts)); end
p = cfg.p;
P.fric.duty = [p.Rm*p.tcp, p.Rm*p.tcn]/p.Vs;     % identified Coulomb friction as duty (report 5.7)
P.plant = struct('Jp', p.Jp, 'G', p.G, 'h', p.h, 'Ja', p.Ja, 'Beq', p.Beq, ...
                 'tcp', p.tcp, 'tcn', p.tcn, 'Rm', p.Rm, 'Kb', p.Kb, 'Vs', p.Vs, ...
                 'b1', p.b1, 'tp', p.tp, 'eps', p.eps, 'eps1', p.eps1, ...         % the rest of furuta_rhs's
                 'stick_band', p.stick_band, 'stick_T', p.stick_T);                % fields: the observer's model
lam = exp(-2*pi*P.est.f_ab*P.Ts); al = 1 - lam^2; be = (1 - lam)^2;              % critically damped alpha-beta
P.est.L = [al 0; 0 al; be/P.Ts 0; 0 be/P.Ts];                                      % observer correction gain (4x2)
% arm breakaway friction (measured dead zone), also when the harness plant runs without it (harness_config)
bk = struct('tsp', 0, 'tsn', 0);
if isfield(cfg, 'breakaway'), bk = cfg.breakaway; end
P.plant.tsp = bk.tsp;
P.plant.tsn = bk.tsn;
P.fric.stick_duty = [p.Rm*bk.tsp, p.Rm*bk.tsn]/p.Vs;   % breakaway as duty (design model: 6 % / 6 % on 20 Nov, 13 / 14 % Table 6)
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
