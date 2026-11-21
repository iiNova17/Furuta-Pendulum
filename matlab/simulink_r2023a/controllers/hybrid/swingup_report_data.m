function swingup_report_data()
%SWINGUP_REPORT_DATA Collect the swing-up results for report section 10 into report/figures/swingup_data.mat.
%   Needs (next to this file): swingup_catch.mat (swingup_catch_region), the one-at-a-time sweeps
%   swingup_sweep_<knob>.mat, the 2D map swingup_sweep_a_push_wc.mat (swingup_sweep) and the
%   candidate worst cases swingup_corners_<j>.mat. Runs the final design (ctrl_hybrid_design.m) once
%   nominally and through the 12 worst cases, keeping the time series and the hand-over states.
%   Then: python report/figures/make_swingup_figures.py
here = fileparts(mfilename('fullpath'));
out = fullfile(here, '..', '..', '..', '..', 'report', 'figures', 'swingup_data.mat');
w = warning('off', 'all'); cl = onCleanup(@() warning(w));

L = load(fullfile(here, 'swingup_catch.mat'));  D.catch = L.C;

knobs = {'a_push', 'E0', 'wc', 'k_E', 'i_max', 'a_kick', 'zc'};
for k = knobs
    L = load(fullfile(here, ['swingup_sweep_' k{1} '.mat']));
    D.oat.(k{1}) = rmfield(L.S, intersect(fieldnames(L.S), {'idx', 'done'}));
end
L = load(fullfile(here, 'swingup_sweep_a_push_wc.mat'));  D.map = L.S;

f = dir(fullfile(here, 'swingup_corners_*.mat'));
for j = 1:numel(f)
    L = load(fullfile(f(j).folder, f(j).name));
    T = L.T;
    s = regexp(f(j).name, '_(\d+)\.mat$', 'tokens', 'once');
    D.cand(str2double(s{1})) = struct('a', L.knobs.a_push, 'wc', L.knobs.wc, 'pass', T.pass', ...
        'tcap', T.t_catch', 'ipk', T.i_peak', 'th0', T.th0_max_deg', 'timing', {T.timing'}, ...
        'plant', {T.plant'});
end
% the finalists repeated with noise seeds 2..4 (swingup_seeds_<j>.mat)
f = dir(fullfile(here, 'swingup_seeds_*.mat'));
for j = 1:numel(f)
    L = load(fullfile(f(j).folder, f(j).name));
    T = L.T;
    D.seeds(j) = struct('a', L.knobs.a_push, 'wc', L.knobs.wc, 'pass', T.pass', 'tcap', T.t_catch', ...
        'ipk', T.i_peak', 'th0', T.th0_max_deg', 'w0c', T.w0_catch', 'timing', {T.timing'});
end

% final design: nominal run (time series) and the 12 worst cases (hand-over states)
cfg = harness_config();
R = run_sim('ctrl_hybrid.slx', 'swingup', 'Cfg', cfg);
m = R.P.plant;
D.run = struct('t', R.t, 'th0', rad2deg(R.th0), 'th1', rad2deg(R.th1), 'w0', R.x(:, 3), ...
    'w1', R.x(:, 4), 'i', R.i, 'u', R.u, 'mode', R.mode, 'E_est', R.dbg(:, 3)/(2*m.G), ...
    'E', (0.5*m.Jp*R.x(:, 4).^2 + m.G*(cos(R.x(:, 2)) - 1))/(2*m.G), 'tcap', R.metrics.t_capture);
D.final = R.P.swing;
D.sup = R.P.sup;
C = corner_plants(cfg.p, 2);
timing = {'nominal', '+1 tick', 'lag 10 ms'};
H = zeros(0, 8);
for j = 1:3
    c = cfg;
    if j == 2, c.link.u_delay = 1; end
    if j == 3, c.sens.pend_delay = 10e-3; end
    for i = 1:numel(C)
        Ri = run_sim('ctrl_hybrid.slx', 'swingup', 'Plant', C(i).plant, 'Cfg', c);
        k = find(Ri.mode == 2, 1);
        if isempty(k), xc = nan(1, 4); else, xc = [rad2deg(Ri.th0(k)), rad2deg(Ri.th1(k)), Ri.x(k, 3:4)]; end
        H(end+1, :) = [j, i, Ri.pass, Ri.metrics.t_capture, max(abs(Ri.i)), xc(2), xc(4), ...
            rad2deg(max(abs(Ri.th0)))]; %#ok<AGROW>
    end
end
D.final_corners = array2table(H, 'VariableNames', {'timing', 'plant', 'pass', 'tcap', 'ipk', ...
    'th1_catch', 'w1_catch', 'th0_max'});
D.final_corners = table2struct(D.final_corners, 'ToScalar', true);
D.timing_names = timing;
D.plant_names = {C.name};
D.p = struct('G', m.G, 'Jp', m.Jp);
save(out, '-struct', 'D');
fprintf('saved %s\n', out);
end
