function T = swingup_corner_test(knobs, k, extra, seeds)
%SWINGUP_CORNER_TEST Run a swing-up design through the 12 physically consistent worst cases.
%   T = swingup_corner_test(knobs)          knobs = struct of D.swing overrides (empty = design file)
%   T = swingup_corner_test(knobs, k)       corner size in standard errors (default 2)
%   T = swingup_corner_test(..., k, extra)  other controller knobs, e.g. struct('sup', struct(...))
%   T = swingup_corner_test(..., k, extra, seeds)  repeat every case with these noise seeds (default 1)
%   4 plant corners (pendulum strength +-k sigma with Jp, h, b1, tp moving with G; arm friction
%   +-k sigma) x 3 timing cases (nominal; +1 controller tick of command delay; pendulum sensor lag
%   10 ms instead of 7), each on the 'swingup' scenario. Realistic sensors and firmware.
%   Example: T = swingup_corner_test(struct('a_push', 70, 'wc', 4.2));
if nargin < 1 || isempty(knobs), knobs = struct(); end
if nargin < 2 || isempty(k), k = 2; end
if nargin < 3 || isempty(extra), extra = struct(); end
if nargin < 4 || isempty(seeds), seeds = 1; end
cfg0 = harness_config();
P = extra;
P.swing = knobs;
C = corner_plants(cfg0.p, k);
timing = {'nominal timing', '+1 tick delay', 'sensor lag 10 ms'};
w = warning('off', 'all'); c = onCleanup(@() warning(w));
rows = {};
for j = 1:numel(timing)
    cfg = cfg0;
    if j == 2, cfg.link.u_delay = 1; end
    if j == 3, cfg.sens.pend_delay = 10e-3; end
    for i = 1:numel(C)
        for sd = seeds
            R = run_sim('ctrl_hybrid.slx', 'swingup', 'Params', P, 'Plant', C(i).plant, 'Cfg', cfg, 'Seed', sd);
            k0 = find(R.mode == 2, 1);
            if isempty(k0), w0c = NaN; else, w0c = R.x(k0, 3); end
            rows(end+1, :) = {timing{j}, C(i).name, sd, R.pass, R.metrics.t_capture, max(abs(R.i)), ...
                rad2deg(max(abs(R.th0))), w0c, R.fault_name}; %#ok<AGROW>
        end
    end
end
T = cell2table(rows, 'VariableNames', {'timing', 'plant', 'seed', 'pass', 't_catch', 'i_peak', ...
    'th0_max_deg', 'w0_catch', 'fault'});
fprintf('\nswing-up %s: pass %d/%d, catch %.2f..%.2f s, worst peak current %.2f A, worst |theta0| %.0f deg\n', ...
    jsonencode(knobs), nnz(T.pass), height(T), min(T.t_catch), max(T.t_catch), max(T.i_peak), ...
    max(T.th0_max_deg));
end
