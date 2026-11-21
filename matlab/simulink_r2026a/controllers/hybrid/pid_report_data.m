function pid_report_data()
%PID_REPORT_DATA Collect the PID tuning results for the report into report/figures/pid_data.mat.
%   Needs pid_tune_results.mat next to this file (written by pid_tune). Flattens the search history, the
%   sensitivity sweep, the time series of the start / tuned / LQR runs, the Monte Carlo push tests and the
%   linear check into plain arrays. Then: python report/figures/make_pid_figures.py
here = fileparts(mfilename('fullpath'));
out = fullfile(here, '..', '..', '..', '..', 'report', 'figures', 'pid_data.mat');
L = load(fullfile(here, 'pid_tune_results.mat')); S = L.S;
names = S.names;
D.names = names;
D.start = cellfun(@(n) S.start.(n), names);
D.tuned = cellfun(@(n) S.tuned.(n), names);
h = S.hist;
D.hist.J = [h.J]; D.hist.itae10 = [h.itae10]; D.hist.itae15 = [h.itae15];
D.hist.pass10 = [h.pass10]; D.hist.pass15 = [h.pass15]; D.hist.accepted = [h.accepted];
D.hist.arm = [h.arm]; D.hist.feasible = [h.feasible]; D.warm = S.warm; D.hist.pm = [h.pm]; D.hist.dm = [h.dm];
D.hist.g = cell2mat(arrayfun(@(x) cellfun(@(n) x.g.(n), names), h(:), 'UniformOutput', false));
D.sens = S.sens;
for c = {'start', 'tuned', 'lqr'}
    for r = fieldnames(S.runs.(c{1}))'
        R = S.runs.(c{1}).(r{1});
        D.runs.(c{1}).(r{1}) = struct('t', R.t, 'th0', R.th0, 'th1', R.th1, 'r', R.r, 'u', R.u, 'i', R.i, ...
            'mode', R.mode, 'pass', R.pass, 'fault', R.fault, 'itae', R.metrics.ITAE, 'sat', R.metrics.sat_pct, ...
            'tcap', R.metrics.t_capture, 'th1max', R.metrics.th1_max, 'trise', R.metrics.th0_trise, ...
            'overshoot', R.metrics.th0_overshoot, 'tsettle0', R.metrics.th0_tsettle, 'upeak', R.metrics.u_peak);
    end
    D.mc.(c{1}) = S.mc.(c{1});
end
D.lin = S.lin;
D.minpm = S.minpm; D.mindm = S.mindm;
% the first search, without the margin constraint (pid_tune_results_nomargin.mat), for comparison
f0 = fullfile(here, 'pid_tune_results_nomargin.mat');
if isfile(f0)
    L0 = load(f0); S0 = L0.S; a = [S0.hist.accepted];
    D.nomargin = struct('tuned', cellfun(@(n) S0.tuned.(n), names), 'J', S0.hist(find(a, 1, 'last')).J, ...
        'pm', S0.lin.tuned.pm, 'dm', S0.lin.tuned.dm, 'poles', S0.lin.tuned.poles, ...
        'itae10', S0.runs.tuned.push_10.metrics.ITAE, 'itae15', S0.runs.tuned.push_15.metrics.ITAE, ...
        'neval', numel(S0.hist), 'J0', S0.hist(1).J);
end
save(out, '-struct', 'D');
fprintf('wrote %s\n', out);
end
