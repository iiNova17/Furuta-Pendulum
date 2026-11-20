function T = observer_compare(varargin)
%OBSERVER_COMPARE Old controller vs the retuned one (model observer + LQR on the 20 Nov model), in the harness.
%   T = observer_compare()                      all cases, 20 Nov plant, realistic sensors and firmware
%   T = observer_compare('Lags', [9 12 15])     pendulum lag behind the encoder [ms] (shaft coupling, 9-15 measured)
%
%   Controllers (ctrl_hybrid.slx, knobs from ctrl_hybrid_design.m):
%     old          design_model 0 (Table 6), filtered differences (est.kind 0): the firmware 1.11 controller
%     new          design_model 20261120, model observer (est.kind 1), friction compensation as designed
%     new, fric 0  as new, friction compensation off
%   Scenarios: balance, push, swingup. Metrics after the catch + 1 s: pendulum rms, duty at the limit,
%   current rms; swing-up: catch time and arm range.
p = inputParser; p.addParameter('Lags', [9 12 15]); p.parse(varargin{:}); o = p.Results;
ctrls = {'old', struct('design_model', 0, 'est', struct('kind', 0)); ...
         'new', struct(); ...
         'new, fric 0', struct('fric', struct('gain', 0))};
scen = {'balance', 'push', 'swingup'};
w = warning('off', 'all'); c = onCleanup(@() warning(w));
rows = {};
for lag = o.Lags
    cfg = harness_config('realistic', 'Model', '20261120');
    cfg.sens.pend_delay = lag/1000;
    for i = 1:size(ctrls, 1)
        for s = scen
            R = run_sim('ctrl_hybrid.slx', s{1}, 'Cfg', cfg, 'Params', ctrls{i, 2});
            st = R.mode == 2; tc = R.metrics.t_capture;
            if isnan(tc), tc = 0; end
            k = st & R.t > tc + 1;
            if any(k)
                rmsd = rms(rad2deg(R.th1(k))); sat = 100*mean(abs(R.d(k)) >= 0.49); irms = rms(R.i(k));
            else
                rmsd = NaN; sat = NaN; irms = NaN;
            end
            rows(end+1, :) = {lag, ctrls{i, 1}, s{1}, R.pass, R.fault_name, R.metrics.t_capture, ...
                rad2deg(max(abs(R.th0))), rmsd, sat, irms}; %#ok<AGROW>
            fprintf('lag %2d ms  %-12s %-8s pass %d %-12s catch %5.2f s  arm max %5.1f deg  pend rms %5.2f deg  sat %5.1f %%  i rms %.2f A\n', ...
                rows{end, :});
        end
    end
end
T = cell2table(rows, 'VariableNames', {'lag_ms', 'controller', 'scenario', 'pass', 'fault', 't_catch', ...
    'arm_max_deg', 'pend_rms_deg', 'sat_pct', 'i_rms_A'});
end
