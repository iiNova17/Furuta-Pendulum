function C = compare_to_run(run, opts)
%COMPARE_TO_RUN Simulation vs measured: replay a recorded hardware run through the harness plant.
%   C = compare_to_run('20261115_100519_238300')      a run exported by export_runs.py
%   C = compare_to_run(run, 'Resync', 2)              restart from the measured state every 2 s
%   C = compare_to_run(run, 'Plant', struct('h', ...)) try other plant parameters
%   C = compare_to_run(run, 'Model', 'table6')         the report's Table 6 model instead of the 20 Nov one
%
%   The Simscape plant is driven by the duty the real firmware applied (recorded), starting from the
%   measured state 0.5 s into the run, like the report's validation (estimation.simulate). The
%   harness firmware model is set to pass the duty through unchanged (no clip, slew, reversal tick or
%   trips: the recording already contains them), sensors ideal: the comparison is plant vs rig.
%   Measured signals are the team's prepared ones (zero-phase 40 Hz filter, 7 ms pendulum timing
%   correction). Fit % as in the report (eq. 103): 100 (1 - |y - yhat| / |y - mean(y)|).
%
%   Export the recordings first:  python replay/export_runs.py
arguments
    run (1,:) char
    opts.Resync (1,1) double = 0
    opts.Plant struct = struct()
    opts.Plot (1,1) logical = true
    opts.Model (1,:) char = '20261120'          % plant model (harness_config 'Model'); 'table6' = report Table 6
end
root = harness_setup();
here = fullfile(root, 'replay');
f = fullfile(here, 'runs', [run '.mat']);
if ~isfile(f)
    error('compare_to_run:missing', '%s not found. Run: python replay/export_runs.py %s', f, run);
end
M = load(f);
M.t = M.t(:); M.th0 = M.th0(:); M.th1 = M.th1(:); M.w0 = M.w0(:); M.w1 = M.w1(:); M.i = M.i(:); M.app = M.app(:);

cfg = harness_config('ideal', 'Model', opts.Model);
cfg.Ts = cfg.fw.Ts*max(1, round(1/M.fs/cfg.fw.Ts));      % recording's rate, on the 1 ms firmware grid
cfg.act.max_duty = 1; cfg.act.slew = Inf; cfg.act.standby = 0; cfg.act.duty_levels = Inf;
cfg.act.reverse_through_zero = false;
cfg.act.travel = Inf; cfg.act.i_trip = Inf;

s0 = min(round(0.5*M.fs), floor(numel(M.t)/10)) + 1;     % same start as estimation.simulate
n = numel(M.t);
L = n - s0 + 1;
if opts.Resync > 0, L = round(opts.Resync*M.fs); end
sim = struct('th0', nan(n, 1), 'th1', nan(n, 1), 'w0', nan(n, 1), 'w1', nan(n, 1), 'i', nan(n, 1));
for a = s0:L:n
    b = min(n, a + L - 1);
    T = M.t(b) - M.t(a);
    tl = M.t(a:b) - M.t(a);
    sc = struct('name', run, 'T', T, 'x0', [M.th0(a); M.th1(a); M.w0(a); M.w1(a)], ...
        'ref', @(t) zeros(size(t)), 'dist', @(t) zeros(numel(t), 2), 'check', @(R) true, ...
        'text', 'replay of a recorded run', 't_event', 0);
    R = run_sim('OpenLoopDuty', sc, 'Cfg', cfg, 'Plant', opts.Plant, ...
        'Params', struct('t', tl, 'd', M.app(a:b)));
    k = a:b;
    m = min(numel(k), numel(R.t));
    sim.th0(k(1:m)) = R.x(1:m, 1);
    sim.th1(k(1:m)) = R.x(1:m, 2);
    sim.w0(k(1:m)) = R.x(1:m, 3);
    sim.w1(k(1:m)) = R.x(1:m, 4);
    sim.i(k(1:m)) = R.i(1:m);
end

fit = @(y, yh) 100*(1 - norm(yh(isfinite(yh)) - y(isfinite(yh)))/norm(y(isfinite(yh)) - mean(y(isfinite(yh)))));
C.run = run;
C.role = M.role;
C.resync_s = opts.Resync;
C.fit.arm_angle = fit(M.th0, sim.th0);
C.fit.pendulum_angle = fit(M.th1, sim.th1);
C.fit.arm_speed = fit(M.w0, sim.w0);
C.fit.current = fit(M.i, sim.i);
ok = isfinite(sim.th0);
C.rms.arm_angle_deg = rad2deg(sqrt(mean((sim.th0(ok) - M.th0(ok)).^2)));
C.rms.pendulum_angle_deg = rad2deg(sqrt(mean((sim.th1(ok) - M.th1(ok)).^2)));
C.rms.current_A = sqrt(mean((sim.i(ok) - M.i(ok)).^2));
C.t = M.t; C.meas = M; C.sim = sim;

if opts.Plot
    fig = figure('Name', ['Simulation vs measured: ' run]);
    tl = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact');
    rs = 'open loop'; if opts.Resync > 0, rs = sprintf('restarted every %g s', opts.Resync); end
    title(tl, sprintf('%s (%s run), %s:  fit arm %.0f %%, pendulum %.0f %%, current %.0f %%', ...
        run, M.role, rs, C.fit.arm_angle, C.fit.pendulum_angle, C.fit.current), 'Interpreter', 'none');
    ax = nexttile(tl);
    plot(ax, M.t, rad2deg(M.th0), M.t, rad2deg(sim.th0), '--', 'LineWidth', 1.1);
    ylabel(ax, '\theta_0 [deg]'); legend(ax, 'measured (rig)', 'simulated (harness)', 'Location', 'best');
    ax(2) = nexttile(tl);
    plot(ax(2), M.t, rad2deg(M.th1), M.t, rad2deg(sim.th1), '--', 'LineWidth', 1.1);
    ylabel(ax(2), '\theta_1 [deg] (180 = hanging)');
    ax(3) = nexttile(tl);
    plot(ax(3), M.t, M.i, M.t, sim.i, '--', 'LineWidth', 1.1);
    ylabel(ax(3), 'i [A]');
    ax(4) = nexttile(tl);
    plot(ax(4), M.t, M.app);
    ylabel(ax(4), 'applied duty'); xlabel(ax(4), 't [s]');
    linkaxes(ax, 'x'); grid(ax, 'on');
end
fprintf('%s (%s): fit arm %.0f %%, pendulum %.0f %%, arm speed %.0f %%, current %.0f %%  |  RMS arm %.1f deg, pendulum %.1f deg, current %.3f A\n', ...
    run, M.role, C.fit.arm_angle, C.fit.pendulum_angle, C.fit.arm_speed, C.fit.current, ...
    C.rms.arm_angle_deg, C.rms.pendulum_angle_deg, C.rms.current_A);
end
