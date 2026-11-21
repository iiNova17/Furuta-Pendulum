function V = validate_open_loop(opts)
%VALIDATE_OPEN_LOOP Phase-1 check (brief sec. 4.1, 8.1): Simscape model vs analytical models.
%   V = validate_open_loop()                 the report's 25 % / 1.4 Hz square wave, from hanging
%   V = validate_open_loop('Amp', 0.1, 'Freq', 1, 'T', 6, 'SaveDir', 'figs')
%
%   The same applied duty drives
%     1. the Simscape Multibody plant of the harness (ideal sensors and actuator),
%     2. the nonlinear equations, report eq. 84/88 (furuta_rhs), RK4 at the same step,
%     3. the linear state-space model about the hanging equilibrium, report eq. 106 (duty input).
%   It plots theta0 and theta1 for all three and returns the largest differences, plus the
%   open-loop eigenvalues of both equilibria (brief sec. 4.2). The linear model leaves out the
%   Coulomb friction (report 3.3), so it drifts in theta0 and is only valid for small swings.
arguments
    opts.Amp (1,1) double = 0.25
    opts.Freq (1,1) double = 1.4
    opts.T (1,1) double = 4
    opts.SaveDir char = ''
end
harness_setup();
cfg = harness_config('ideal');
t = (0:cfg.Ts:opts.T)';
duty = opts.Amp*sign(sin(2*pi*opts.Freq*t));
sc = struct('name', 'open_loop', 'T', opts.T, 'x0', [0; pi; 0; 0], 'ref', @(t) zeros(size(t)), ...
    'dist', @(t) zeros(numel(t), 2), 'check', @(R) true, 'text', 'open-loop validation', 't_event', 0);
R = run_sim('OpenLoopDuty', sc, 'Mode', 'ideal', 'Params', struct('t', t, 'd', duty));

% nonlinear model, same integrator and step, driven by the duty the firmware applied (1 ms ticks)
da = R.d_fw;
h = cfg.dt;
N = round(opts.T/h);
xn = zeros(N + 1, 4);
xn(1, :) = [0 pi 0 0];
for n = 1:N
    d = da(min(numel(da), floor((n - 1)*h/cfg.fw.Ts + 1e-9) + 1));
    f = @(X) furuta_rhs(X, d, cfg.p);
    X = xn(n, :)';
    k1 = f(X); k2 = f(X + h/2*k1); k3 = f(X + h/2*k2); k4 = f(X + h*k3);
    xn(n + 1, :) = (X + h/6*(k1 + 2*k2 + 2*k3 + k4))';
end
xn = xn(1:round(cfg.Ts/h):end, :);
xn = xn(1:numel(R.t), :);

% linear model about hanging: x = [theta0; phi; dtheta0; dphi], phi = theta1 - pi
sysl = ss(cfg.lin.A_down, cfg.lin.Bd_down, eye(4), 0);
tfw = (0:numel(da) - 1)'*cfg.fw.Ts;
xl = lsim(sysl, da, tfw, [0; 0; 0; 0], 'zoh');
xl = interp1(tfw, xl, R.t);

V.max_diff_simscape_vs_nonlinear_deg = max(abs(rad2deg(R.x(:, 1:2) - xn(:, 1:2))), [], 1);
V.max_diff_linear_vs_nonlinear_deg = max(abs(rad2deg([xl(:, 1), xl(:, 2) + pi] - xn(:, 1:2))), [], 1);
V.eig_upright = eig(cfg.lin.A_up);
V.eig_hanging = eig(cfg.lin.A_down);
V.ctrb_rank_upright = rank(ctrb(cfg.lin.A_up, cfg.lin.Bd_up));
V.obsv_rank_upright_angles = rank(obsv(cfg.lin.A_up, cfg.lin.C(1:2, :)));

fig = figure('Name', 'Open-loop validation');
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');
title(tl, sprintf('Open loop from hanging, duty %.0f %% square wave at %.2g Hz', 100*opts.Amp, opts.Freq));
ax = nexttile(tl);
plot(ax, R.t, rad2deg(R.x(:, 1)), R.t, rad2deg(xn(:, 1)), '--', R.t, rad2deg(xl(:, 1)), ':', 'LineWidth', 1.2);
ylabel(ax, '\theta_0 [deg]');
legend(ax, 'Simscape Multibody', 'nonlinear (eq. 84/88)', 'linear (eq. 106)', 'Location', 'best');
ax(2) = nexttile(tl);
plot(ax(2), R.t, rad2deg(R.x(:, 2) - pi), R.t, rad2deg(xn(:, 2) - pi), '--', R.t, rad2deg(xl(:, 2)), ':', ...
    'LineWidth', 1.2);
ylabel(ax(2), '\theta_1 - \pi [deg]');
ax(3) = nexttile(tl);
plot(ax(3), R.t, R.d);
ylabel(ax(3), 'duty'); xlabel(ax(3), 't [s]');
linkaxes(ax, 'x'); grid(ax, 'on');
if ~isempty(opts.SaveDir)
    if ~isfolder(opts.SaveDir), mkdir(opts.SaveDir); end
    exportgraphics(fig, fullfile(opts.SaveDir, 'open_loop_validation.png'), 'Resolution', 150);
end
fprintf('Simscape vs nonlinear: max |diff| theta0 %.2g deg, theta1 %.2g deg\n', V.max_diff_simscape_vs_nonlinear_deg);
fprintf('linear   vs nonlinear: max |diff| theta0 %.3g deg, theta1 %.3g deg\n', V.max_diff_linear_vs_nonlinear_deg);
fprintf('eig upright: %s\neig hanging: %s\n', mat2str(V.eig_upright', 4), mat2str(V.eig_hanging', 4));
fprintf('controllability rank %d/4, observability rank from both angles %d/4\n', ...
    V.ctrb_rank_upright, V.obsv_rank_upright_angles);
end
