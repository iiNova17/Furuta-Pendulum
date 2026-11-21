function T = lqr_corner_test(max_x, max_duty, k, extra)
%LQR_CORNER_TEST Run an LQR design through the 12 physically consistent worst cases.
%   T = lqr_corner_test(max_x, max_duty)      max_x = [theta0 theta1 (deg) w0 w1 (rad/s)], Bryson
%   T = lqr_corner_test(..., k)               corner size in standard errors (default 2)
%   T = lqr_corner_test(..., k, extra)        extra controller knobs, e.g. struct('fric', struct('gain', 1))
%   4 plant corners (pendulum strength +-k sigma with Jp, h, b1, tp moving with G; arm friction
%   +-k sigma) x 3 timing cases (nominal; +1 controller tick of command delay; pendulum sensor lag
%   10 ms instead of 7), each on 'balance' and 'push'. Realistic sensors and firmware.
%   Example: T = lqr_corner_test([30 4.5 20 30], 0.16);
if nargin < 3 || isempty(k), k = 2; end
if nargin < 4, extra = struct(); end
cfg0 = harness_config();
mx = [deg2rad(max_x(1:2)) max_x(3:4)];
P = struct('lqr', struct('Q', diag(1./mx.^2), 'R', 1/max_duty^2), 'stab', 3);
for f = fieldnames(extra)', P.(f{1}) = extra.(f{1}); end
C = corner_plants(cfg0.p, k);
timing = {'nominal timing', '+1 tick delay', 'sensor lag 10 ms'};
w = warning('off', 'all'); c = onCleanup(@() warning(w));
rows = {};
for j = 1:numel(timing)
    cfg = cfg0;
    if j == 2, cfg.link.u_delay = 1; end
    if j == 3, cfg.sens.pend_delay = 10e-3; end
    for i = 1:numel(C)
        r = cell(1, 4);
        for s = {'balance', 'push'}
            R = run_sim('ctrl_hybrid.slx', s{1}, 'Params', P, 'Plant', C(i).plant, 'Cfg', cfg);
            if strcmp(s{1}, 'balance'), r(1:2) = {R.pass, max(abs(R.i))};
            else, r(3:4) = {R.pass, max(abs(R.i))}; itae = R.metrics.ITAE; f = R.fault_name; end
        end
        rows(end+1, :) = {timing{j}, C(i).name, r{:}, itae, f}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'timing', 'plant', 'balance_pass', 'balance_i_peak', ...
    'push_pass', 'push_i_peak', 'push_ITAE', 'push_fault'});
n = height(T);
fprintf('\nLQR max_x = [%g %g %g %g], max_duty = %g: balance %d/%d, push %d/%d, worst peak current %.2f A\n', ...
    max_x, max_duty, nnz(T.balance_pass), n, nnz(T.push_pass), n, max([T.balance_i_peak; T.push_i_peak]));
end
