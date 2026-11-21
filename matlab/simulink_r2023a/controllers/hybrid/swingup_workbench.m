function W = swingup_workbench(knobs, varargin)
%SWINGUP_WORKBENCH One set of swing-up knobs -> swing-up runs, numbers and an energy plot.
%   W = swingup_workbench()                         the knobs in ctrl_hybrid_design.m
%   W = swingup_workbench(struct('a_push', 60))     override some D.swing knobs (see swingup_knobs.m)
%   W = swingup_workbench(knobs, 'Seeds', 10, 'Plot', true, 'Cfg', cfg, 'Plant', plant, 'Show3D', true)
%
%   Runs the 'swingup' scenario (hanging at rest, 15 s, pass = |theta1| < 10 deg in the last 3 s,
%   no firmware trip) in realistic mode on the nominal plant, plus 'Seeds' perturbed plants (k = 2).
%   Prints time to catch (first entry into Stabilize), peak current, largest |theta0|, the state at
%   the hand-over, and plots energy, angles, current and duty (Plot = true).
%   Nothing is written to ctrl_hybrid_design.m: it prints the line to paste when you're happy.
if nargin < 1 || isempty(knobs), knobs = struct(); end
p = inputParser;
p.addParameter('Seeds', 0);
p.addParameter('Plot', true);
p.addParameter('Cfg', harness_config());
p.addParameter('Plant', struct());
p.addParameter('Show3D', false);
p.addParameter('Extra', struct());                  % other controller knobs, e.g. struct('sup', ...)
p.parse(varargin{:});
o = p.Results;

P = o.Extra;
P.swing = knobs;
w = warning('off', 'all'); cl = onCleanup(@() warning(w));
R = run_sim('ctrl_hybrid.slx', 'swingup', 'Params', P, 'Cfg', o.Cfg, 'Plant', o.Plant, 'Show3D', o.Show3D);
W = summarize(R);
W.R = R;
fprintf('\n=== swing-up: %s ===\n', knob_text(R.P.swing));
fprintf(['nominal: pass %d | catch at %.2f s | peak current %.2f A (trip 1.5) | max |theta0| %.0f deg ' ...
    '(SafeStop %g) | fault: %s\n'], W.pass, W.t_capture, W.i_peak, W.th0_max, ...
    rad2deg(R.P.sup.safe_th0), R.fault_name);
fprintf('hand-over state: theta0 %+.0f deg, theta1 %+.1f deg, w0 %+.1f rad/s, w1 %+.1f rad/s\n', W.x_catch);
if o.Seeds > 0
    S = zeros(o.Seeds, 4);
    for k = 1:o.Seeds
        Rk = run_sim('ctrl_hybrid.slx', 'swingup', 'Params', P, 'Cfg', o.Cfg, 'Perturb', 2, 'Seed', k);
        s = summarize(Rk);
        S(k, :) = [s.pass, s.t_capture, s.i_peak, s.th0_max];
    end
    W.seeds = S;
    fprintf('perturbed plants (k = 2) x %d: pass %d/%d, catch %.2f..%.2f s, peak current <= %.2f A, |theta0| <= %.0f deg\n', ...
        o.Seeds, nnz(S(:, 1)), o.Seeds, min(S(:, 2)), max(S(:, 2)), max(S(:, 3)), max(S(:, 4)));
end
fprintf('\nWhen you are happy, put this in ctrl_hybrid_design.m (swing-up owner section):\n%s\n\n', ...
    ['D.swing = struct(' knob_text(R.P.swing) ');']);
if o.Plot, plot_energy(R, W); end
end

function W = summarize(R)
W.pass = R.pass;
W.t_capture = R.metrics.t_capture;
W.i_peak = max(abs(R.i));
W.th0_max = rad2deg(max(abs(R.th0)));
W.fault = R.fault_name;
k = find(R.mode == 2, 1);
if isempty(k), W.x_catch = nan(1, 4);
else, W.x_catch = [rad2deg(R.th0(k)), rad2deg(R.th1(k)), R.x(k, 3), R.x(k, 4)];
end
end

function s = knob_text(k)
f = fieldnames(k);
c = cellfun(@(n) sprintf('''%s'', %.4g', n, k.(n)), f, 'UniformOutput', false);
s = strjoin(c', ', ');
end

function plot_energy(R, W)
m = R.P.plant;
E = R.dbg(:, 3)/(2*m.G);                          % estimated energy, units of 2G
Et = (0.5*m.Jp*R.x(:, 4).^2 + m.G*(cos(R.x(:, 2)) - 1))/(2*m.G);   % true energy
fig = figure('Name', 'swing-up', 'Color', 'w', 'Position', [100 100 760 720]);
try, fig.Theme = 'light'; catch, end        % R2025a+: keep the plot readable in dark mode
tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
ax(1) = nexttile; plot(R.t, Et, R.t, E, ':'); hold on
yline(R.P.swing.E0, '--', 'target E_0'); yline(0, ':', 'just reaches the top');
ylabel('energy / 2G'); legend('true', 'estimated', 'Location', 'southeast'); ylim([-1.1 0.3])
ax(2) = nexttile; plot(R.t, rad2deg(R.th1), R.t, rad2deg(R.th0)); hold on
yline(rad2deg(R.P.sup.catch_th1)*[-1 1], ':');
ylabel('angle [deg]'); legend('\theta_1 (0 = up)', '\theta_0 arm', 'Location', 'southeast');
ax(3) = nexttile; plot(R.t, R.i); yline([-1.5 1.5], 'r--', 'trip'); ylabel('current [A]');
ax(4) = nexttile; stairs(R.t, R.u); hold on; stairs(R.t, R.mode/4, 'k');
ylabel('duty'); legend('duty', 'mode/4 (1/4 swing-up, 1/2 stabilize)', 'Location', 'southeast');
xlabel(tl, 'time [s]'); linkaxes(ax, 'x');
if isfinite(W.t_capture), for a = ax, xline(a, W.t_capture, 'g-', 'HandleVisibility', 'off'); end, end
title(tl, sprintf('Swing-up: catch at %.2f s, peak current %.2f A, max |\\theta_0| %.0f deg, pass %d', ...
    W.t_capture, W.i_peak, W.th0_max, W.pass));
end
