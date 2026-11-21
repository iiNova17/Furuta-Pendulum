function fig = plot_run(R, fig)
%PLOT_RUN Plot one harness run: arm, pendulum, duty, current.
%   plot_run(R)        new figure
%   plot_run(R, fig)   reuse a figure (e.g. while tuning)
if nargin < 2 || ~isvalid(fig), fig = figure('Name', 'Furuta harness'); else, clf(fig); end
tl = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact');
pass = {'FAIL', 'PASS'};
title(tl, sprintf('%s on %s: %s  (%s)', R.controller, R.scenario, pass{R.pass + 1}, R.text), ...
    'Interpreter', 'none');

ax = nexttile(tl);
plot(ax, R.t, rad2deg(R.th0), R.t, rad2deg(R.r), '--', R.t, rad2deg(R.y(:, 1)), ':');
ylabel(ax, '\theta_0 [deg]'); legend(ax, 'true', 'reference', 'measured', 'Location', 'best');
yline(ax, rad2deg(R.cfg.act.travel)*[-1 1], 'r:', 'HandleVisibility', 'off');

ax(2) = nexttile(tl);
plot(ax(2), R.t, rad2deg(R.th1), R.t, rad2deg(R.y(:, 2)), ':');
ylabel(ax(2), '\theta_1 [deg]  (0 = up)'); legend(ax(2), 'true', 'measured', 'Location', 'best');

ax(3) = nexttile(tl);
plot(ax(3), R.t, R.u, R.t, R.d);
ylabel(ax(3), 'duty'); legend(ax(3), 'commanded', 'applied', 'Location', 'best');
yline(ax(3), R.cfg.act.max_duty*[-1 1], 'r:', 'HandleVisibility', 'off');

ax(4) = nexttile(tl);
plot(ax(4), R.t, R.i, R.t, R.y(:, 3), ':');
ylabel(ax(4), 'i [A]'); legend(ax(4), 'true', 'measured', 'Location', 'best');
xlabel(ax(4), 't [s]');
linkaxes(ax, 'x'); grid(ax, 'on');
if R.fault > 0
    for a = ax, xline(a, R.fault_time, 'r', R.fault_name); end
end
end
