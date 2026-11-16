function T = compare_controllers(controllers, scenarios, opts)
%COMPARE_CONTROLLERS Score several controllers on the same scenarios.
%   T = compare_controllers({'LqrBalance', 'EnergySwingup'})              all scenarios
%   T = compare_controllers({'LqrBalance'}, {'balance', 'push'}, 'MonteCarlo', 20)
%
%   MonteCarlo N  also runs N plants drawn from the report's parameter uncertainty (Perturb 1)
%                 with different noise seeds; 'pass' is then the pass rate over nominal + N runs.
%   Params        cell array of Params structs, one per controller (default: design defaults)
%   Mode          'realistic' (default) | 'ideal'
%   Labels        names for the rows (e.g. {'PID', 'PP', 'LQR'} when the same controller file is
%                 run with different Params)
%   Columns: pass rate, fault, then every metric of harness_metrics (brief sec. 4.7).
%   PID vs pole placement vs LQR with the hybrid controller:
%     c = repmat({'ctrl_hybrid.slx'}, 1, 3);
%     T = compare_controllers(c, {'push'}, 'Params', {struct('stab', 1), struct('stab', 2), ...
%         struct('stab', 3)}, 'Labels', {'PID', 'PP', 'LQR'})
arguments
    controllers cell
    scenarios cell = furuta_scenario('list')
    opts.MonteCarlo (1,1) double = 0
    opts.Params cell = {}
    opts.Mode (1,:) char = 'realistic'
    opts.Labels cell = {}
end
rows = {};
for c = 1:numel(controllers)
    P = struct();
    if ~isempty(opts.Params), P = opts.Params{c}; end
    for s = 1:numel(scenarios)
        R = run_sim(controllers{c}, scenarios{s}, 'Params', P, 'Mode', opts.Mode);
        ok = R.pass;
        for n = 1:opts.MonteCarlo
            Rn = run_sim(controllers{c}, scenarios{s}, 'Params', P, 'Mode', opts.Mode, ...
                'Perturb', 1, 'Seed', n + 1);
            ok(end + 1) = Rn.pass; %#ok<AGROW>
        end
        m = R.metrics;
        name = controllers{c};
        if ~isempty(opts.Labels), name = opts.Labels{c}; end
        rows(end + 1, :) = [{name, scenarios{s}, mean(ok), R.fault_name}, struct2cell(m)']; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', [{'controller', 'scenario', 'pass', 'fault'}, fieldnames(m)']);
end
