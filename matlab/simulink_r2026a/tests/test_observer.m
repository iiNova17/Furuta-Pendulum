function tests = test_observer
%TEST_OBSERVER The model-assisted observer (controllers/hybrid/est_observer.slx) and its reference implementation.
%   results = runtests('tests/test_observer.m')
%   1. closed loop with est_observer_ref on the 2026-11-20 model: 12 ms pendulum lag (shaft coupling), quantized
%      sensors, firmware slew/clip, LQR Bryson [30 8 20 30] / 0.10, push at 1 s: balances without saturating
%      (filtered differences chatter in the same loop: duty at the limit about half the time)
%   2. est_observer.slx fed with the same readings and duties equals est_observer_ref to round-off (kind 1 and 2)
tests = functiontests(localfunctions);
end

function setupOnce(tc)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root); harness_setup();
cfg = harness_config('realistic', 'Model', '20261120');
P.Ts = cfg.Ts; P.plant = cfg.p;
f_ab = 10; lam = exp(-2*pi*f_ab*P.Ts); al = 1 - lam^2; be = (1 - lam)^2;
P.est = struct('kind', 1, 'nd', 3, 'L', [al 0; 0 al; be/P.Ts 0; 0 be/P.Ts], 'f_trk', 25, 'vel_fc', 40, 'sub', 1);
sysd = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), P.Ts);
mx = [deg2rad(30) deg2rad(8) 20 30];
P.K = dlqr(sysd.A, sysd.B, diag(1./mx.^2), 1/0.10^2);
tc.TestData.P = P; tc.TestData.cfg = cfg;
end

function test_closed_loop_reference_observer(tc)
P = tc.TestData.P;
for play = [0 3]
    L = loop(P, 'model', 0.012, play);
    tc.verifyTrue(L.ok, sprintf('fell over (play %g deg)', play));
    tc.verifyLessThan(L.sat, 0.02, sprintf('duty at the limit %.1f %% (play %g deg)', 100*L.sat, play));
    tc.verifyLessThan(L.th1_rms, 2.5, sprintf('pendulum rms %.2f deg (play %g deg)', L.th1_rms, play));
end
F = loop(P, 'fd', 0.012, 0);           % the firmware 1.11 estimator in the same loop chatters (as on the rig)
tc.verifyGreaterThan(F.sat, 0.3);
fprintf('observer: rms %.2f deg, sat %.1f %% | filtered differences: rms %.2f deg, sat %.1f %%\n', ...
    L.th1_rms, 100*L.sat, F.th1_rms, 100*F.sat);
tc.TestData.L = loop(P, 'model', 0.012, 3);
end

function test_simulink_block_equals_reference(tc)
P = tc.TestData.P;
L = loop(P, 'model', 0.012, 3);                    % readings and duties of a closed-loop run
for kind = [1 2]
    P.est.kind = kind;
    S = []; ref = zeros(numel(L.t), 4);
    for k = 1:numel(L.t), [ref(k, :), S] = est_observer_ref(S, L.y(k, :)', L.u(k), P); end
    sim_xh = run_block(P, L);
    n = min(size(ref, 1), size(sim_xh, 1));
    err = max(abs(sim_xh(1:n, :) - ref(1:n, :)), [], 'all');
    fprintf('kind %d: max |Simulink - reference| = %.3g over %d ticks\n', kind, err, n);
    tc.verifyLessThan(err, 1e-9);
    tc.verifyGreaterThan(n, 900);
end
end

% ---------------------------------------------------------------------------------------------------
function L = loop(P, est, lag, play_deg)
% Plant furuta_rhs (2026-11-20 model, stick-slip), firmware tick 1 ms (slew 100/s, clip 0.5), encoder 1496,
% AS5600 4096 and `lag` late, encoder leading the arm by up to play/2 in the direction of the current.
p = P.plant; Ts = P.Ts; dt = 1e-3; nsub = round(Ts/dt); T = 4;
x = [0; deg2rad(5); 0; 0]; nl = round(lag/dt); hist = repmat(x(2), nl + 1, 1);
d = 0; u = 0; b = deg2rad(play_deg); S = []; w = [0; 0]; prev = []; a = exp(-2*pi*P.est.vel_fc*Ts);
N = round(T/Ts); L.t = (0:N-1)'*Ts; L.y = zeros(N, 3); L.u = zeros(N, 1); L.ok = true;
th1log = zeros(N*nsub, 1); dlog = th1log; q = 0;
for k = 1:N
    [~, i] = furuta_rhs(x, d, p);
    y = [round((x(1) + b/2*tanh(i/0.05))/(2*pi/1496))*2*pi/1496; round(hist(1)/(2*pi/4096))*2*pi/4096; i];
    if strcmp(est, 'model')
        [xh, S] = est_observer_ref(S, y, u, P);
    else                                            % filtered differences at vel_fc (firmware 1.11)
        th = [y(1); wrapf(y(2))]; if isempty(prev), prev = th; end
        dd = th - prev; dd(2) = wrapf(dd(2)); w = a*w + (1 - a)*dd/Ts; prev = th; xh = [th; w];
    end
    u = -P.K*xh;
    L.y(k, :) = y'; L.u(k) = u;
    for s = 1:nsub
        d = d + min(max(min(max(u, -0.5), 0.5) - d, -0.1), 0.1);
        for r = 1:2, x = rk4(x, d, p, dt/2); end
        if abs((k-1)*Ts + s*dt - 1) < dt/2, x(4) = x(4) + 1.5; end
        hist = [hist(2:end); x(2)];
        q = q + 1; th1log(q) = wrapf(x(2)); dlog(q) = d;
        if abs(wrapf(x(2))) > deg2rad(35), L.ok = false; end
    end
    if ~L.ok, break; end
end
n0 = round(2/dt);
L.th1_rms = rad2deg(sqrt(mean(th1log(n0:q).^2)));
L.sat = mean(abs(dlog(n0:q)) >= 0.49);
end

function xh = run_block(P, L)
mdl = 'est_observer_blocktest';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); c = onCleanup(@() close_system(mdl, 0));
add_block('simulink/Sources/From Workspace', [mdl '/y'], 'VariableName', 'y_in', 'SampleTime', 'ctrlP.Ts', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Sources/From Workspace', [mdl '/duty'], 'VariableName', 'u_in', 'SampleTime', 'ctrlP.Ts', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/obs'], 'ReferencedSubsystem', 'est_observer');
add_block('simulink/Sinks/To Workspace', [mdl '/out'], 'VariableName', 'xh_out', 'SaveFormat', 'Array', ...
    'SampleTime', 'ctrlP.Ts');
add_line(mdl, 'y/1', 'obs/1'); add_line(mdl, 'duty/1', 'obs/2'); add_line(mdl, 'obs/1', 'out/1');
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', 'ctrlP.Ts', 'StopTime', num2str(L.t(end)));
in = Simulink.SimulationInput(mdl);
in = in.setVariable('ctrlP', P);
in = in.setVariable('y_in', [L.t L.y]);
in = in.setVariable('u_in', [L.t L.u]);
out = sim(in);
xh = squeeze(out.xh_out);                          % To Workspace logs a 4x1 signal as 4x1xN
if size(xh, 1) == 4 && size(xh, 2) ~= 4, xh = xh'; end
end

function x = rk4(x, d, p, h)
k1 = furuta_rhs(x, d, p); k2 = furuta_rhs(x + h/2*k1, d, p);
k3 = furuta_rhs(x + h/2*k2, d, p); k4 = furuta_rhs(x + h*k3, d, p);
x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function a = wrapf(a)
a = mod(a + pi, 2*pi) - pi;
end
