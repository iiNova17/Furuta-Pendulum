function tests = test_harness
%TEST_HARNESS Checks that the harness plant is the report's model and that the slot works.
%   results = runtests('matlab/harness/tests/test_harness.m')
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.root = fileparts(fileparts(mfilename('fullpath')));
addpath(tc.TestData.root);
harness_setup();
tc.TestData.cfg = harness_config('ideal');
end

function test_linearization_matches_report_eq104(tc)
% Jacobian of the report equations (the reference the Simscape plant is tested against) at upright, Coulomb friction off (as in the report's linear model).
cfg = tc.TestData.cfg;
p = cfg.p; p.tcp = 0; p.tcn = 0; p.tp = 0;
x0 = zeros(4, 1); d0 = 1e-12;            % d ~= 0: driver on, back-EMF acts (eq. 88)
f = @(x, d) harness_plant_rhs(x, d, [0; 0], p);
A = zeros(4); h = 1e-7;
for k = 1:4
    e = zeros(4, 1); e(k) = h;
    A(:, k) = (f(x0 + e, d0) - f(x0 - e, d0))/(2*h);
end
Bd = (f(x0, 1e-4) - f(x0, -1e-4))/2e-4;
tc.verifyEqual(A, cfg.lin.A_up, 'RelTol', 1e-4, 'AbsTol', 1e-6);
tc.verifyEqual(Bd, cfg.lin.Bd_up, 'RelTol', 1e-4);
% report eq. 110: poles 0, +12.26, -12.63, -29.23
tc.verifyEqual(sort(real(eig(A))), sort([0; 12.26; -12.63; -29.23]), 'AbsTol', 0.01);
end

function test_simscape_plant_is_the_report_model(tc)
% The Simscape Multibody plant, driven by the report's 25 % / 1.4 Hz square wave plus disturbance
% pulses on both joints, against the report's equations (furuta_rhs, eq. 84/88) integrated with the
% same RK4 step and the same applied duty. Agreement to round-off means the equations are the same.
cfg = tc.TestData.cfg;
dist = @(t) [0.05*(t >= 1 & t < 1.05), 0.004*(t >= 2.5 & t < 2.52)];
R = run_sim('OpenLoopDuty', struct('name', 'openloop', 'T', 4, 'x0', [0; pi; 0; 0], ...
    'ref', @(t) zeros(size(t)), 'dist', dist, 'check', @(R) true, 'text', 'open loop'), 'Mode', 'ideal');
h = cfg.dt;
N = round(4/h);
x = zeros(N + 1, 4);
x(1, :) = [0 pi 0 0];
for n = 1:N
    tn = (n - 1)*h;
    d = R.d_fw(min(numel(R.d_fw), floor(tn/cfg.fw.Ts + 1e-9) + 1));   % duty the firmware applied
    w = dist(tn + 1e-12)';
    f = @(X) harness_plant_rhs(X, d, w, cfg.p);
    X = x(n, :)';
    k1 = f(X); k2 = f(X + h/2*k1); k3 = f(X + h/2*k2); k4 = f(X + h*k3);
    x(n + 1, :) = (X + h/6*(k1 + 2*k2 + 2*k3 + k4))';
end
x = x(1:round(cfg.Ts/h):end, :);
x = x(1:numel(R.t), :);
tc.verifyLessThan(max(abs(rad2deg(R.x(:, 1:2) - x(:, 1:2))), [], 'all'), 1e-5);   % degrees
tc.verifyLessThan(max(abs(R.x(:, 3:4) - x(:, 3:4)), [], 'all'), 1e-5);            % rad/s
tc.verifyGreaterThan(max(abs(rad2deg(wrap_pi(R.x(:, 2) - pi)))), 20);            % it did swing
end

function test_bodies_reproduce_lumped_parameters(tc)
% plant_bodies must give back exactly the report's lumped parameters (Table 6).
p = tc.TestData.cfg.p;
b = plant_bodies(p);
tc.verifyEqual(b.m*b.g*b.Lp, p.G, 'RelTol', 1e-12);
tc.verifyEqual(b.J1 + b.m*b.Lp^2, p.Jp, 'RelTol', 1e-12);
tc.verifyEqual(b.m*b.Larm*b.Lp, p.h, 'RelTol', 1e-12);
tc.verifyEqual(b.Ia + b.m*b.Larm^2, p.Ja, 'RelTol', 1e-12);
tc.verifyEqual(b.Larm, p.Larm, 'RelTol', 1e-6);              % 6.81 cm, report Table 6
tc.verifyGreaterThan([b.J1 b.Ia], 0);
end

function test_hanging_stays_hanging(tc)
R = run_sim('MyController', struct('name', 'rest', 'T', 1, 'x0', [0; pi; 0; 0], ...
    'ref', @(t) zeros(size(t)), 'dist', @(t) zeros(numel(t), 2), 'check', @(R) true, ...
    'text', 'rest'));
tc.verifyLessThan(max(abs(R.x(:, 3:4)), [], 'all'), 1e-9);
tc.verifyEqual(R.fault, 0);
end

function test_firmware_actuator(tc)
a = tc.TestData.cfg.act;
a.slew = 100; a.standby = 1e-3; a.duty_levels = 1023; a.max_duty = 0.5;
fw = FirmwareActuator('Ts', 1e-3, 'A', a);
ok = [0; 0; 0];
d = zeros(1, 20);
for k = 1:10, d(k) = fw(1, ok); end                 % clipped to 0.5, 0.1 per tick
tc.verifyEqual(d(1:6), [0.1 0.2 0.3 0.4 0.5 0.5], 'AbsTol', 1e-3);
for k = 11:20, d(k) = fw(-1, ok); end               % reversal goes through zero once
tc.verifyEqual(d(11:17), [0.4 0.3 0.2 0.1 0 -0.1 -0.2], 'AbsTol', 1e-3);
[dd, fault] = fw(0, [deg2rad(121); 0; 0]);          % travel trip, stays off
tc.verifyEqual([dd fault], [0 1]);
[dd, fault] = fw(0.3, ok);
tc.verifyEqual([dd fault], [0 1]);
end

function test_subsystem_controller_slot(tc)
name = 'ctrl_harness_selftest';
folder = fullfile(tc.TestData.root, 'controllers', name);
tc.addTeardown(@() rmdir(folder, 's'));
tc.addTeardown(@() rmpath(folder));
tc.addTeardown(@() bdclose('all'));
new_subsystem_controller(name);
R = run_sim([name '.slx'], 'balance', 'Mode', 'ideal');
tc.verifyEqual(max(abs(R.u)), 0);                   % template outputs zero duty
tc.verifyFalse(R.pass);                             % ... so the pendulum falls
end

function test_hybrid_template_plumbing(tc)
% The hybrid template (ctrl_hybrid.slx + its four law files) runs, the Stateflow supervisor switches
% modes, and every stabilizer path reaches the output. Placeholder gains, so nothing balances.
R = run_sim('ctrl_hybrid.slx', 'balance');
tc.verifyEqual(unique(R.mode(:)', 'stable'), [2 1]);     % starts in Stabilize, falls, SwingUp
for s = 1:3                                               % 1 PID, 2 pole placement, 3 LQR
    P = struct('stab', s, 'pid', struct('Kp', 1), 'pp', struct('poles', [-5 -6 -7 -8]), ...
        'lqr', struct('Q', eye(4), 'R', 1));
    R = run_sim('ctrl_hybrid.slx', 'balance', 'Params', P);
    tc.verifyNotEqual(R.u(2), 0, sprintf('stabilizer %d not wired', s));
end
end

function test_metrics_on_known_signal(tc)
% harness_metrics on a synthetic run: decaying theta1 after an event at 1 s.
cfg = tc.TestData.cfg;
t = (0:0.001:5)';
th1 = deg2rad(10)*exp(-(t - 1)/0.2).*(t >= 1);
R = struct('t', t, 'th1', th1, 'th0', zeros(size(t)), 'r', zeros(size(t)), 'u', 0.1*ones(size(t)), ...
    'mode', 2*ones(size(t)));
m = harness_metrics(R, cfg, struct('t_event', 1));
tc.verifyEqual(m.th1_max, 10, 'AbsTol', 1e-9);
tc.verifyEqual(m.th1_tsettle, 0.2*log(10/cfg.metrics.band_th1), 'AbsTol', 2e-3);   % 10 deg -> band
tc.verifyEqual(m.IAE, deg2rad(10)*0.2, 'RelTol', 1e-2);                           % integral of exp
tc.verifyEqual(m.u_peak, 0.1, 'AbsTol', 1e-12);
end

function test_replay_matches_report_table4(tc)
% Simulation vs measured: validation run 1 replayed through the harness plant reproduces the report's
% Table 4 open-loop fits (arm 89, pendulum 90, arm speed 92, current 88 %). Needs the export:
%   python matlab/harness/replay/export_runs.py
f = fullfile(tc.TestData.root, 'replay', 'runs', '20261115_100519_238300.mat');
tc.assumeTrue(isfile(f), 'recording not exported (run replay/export_runs.py)');
addpath(fullfile(tc.TestData.root, 'replay'));
C = compare_to_run('20261115_100519_238300', 'Plot', false);
tc.verifyEqual([C.fit.arm_angle C.fit.pendulum_angle C.fit.arm_speed C.fit.current], ...
    [89.0 89.6 92.0 88.3], 'AbsTol', 0.5);
end
