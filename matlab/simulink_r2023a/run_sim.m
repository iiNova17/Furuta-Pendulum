function R = run_sim(controller, scenario, opts)
%RUN_SIM Simulate one controller on one scenario in the universal harness.
%   R = run_sim('ctrl_hybrid.slx', 'balance')
%   R = run_sim('ctrl_hybrid.slx', 'swingup', 'Plot', true)
%   R = run_sim('ctrl_hybrid.slx', 'push', 'Params', struct('stab', 1))   % PID stabilizer
%   R = run_sim('OpenLoopDuty', sc, 'Params', struct('t', [0; 4], 'd', [0.25; 0.25]))   % open loop
%
%   controller  a subsystem file name ending in .slx (ctrl_hybrid.slx), or a FurutaController
%               System object (OpenLoopDuty, used for open-loop validation)
%               (its design function, if any, is <name>_design(cfg, P))
%   scenario    name (see furuta_scenario('list')) or a scenario struct
%   Options:
%     Mode     'realistic' (default) | 'ideal'          sensors/firmware, see harness_config
%     Params   struct of tuning knobs, merged over the controller's design() defaults
%     Plant    struct of plant parameters to use instead of the identified ones (model mismatch)
%     Perturb  k > 0: draw plant parameters from the report's uncertainties (Table 6) times k
%     Seed     random seed for the current noise and Perturb (default 1)
%     Cfg      a full harness_config struct (overrides Mode), to change sensors/firmware settings
%     Plot     true to call plot_run
%     Show3D   true to open Mechanics Explorer and watch the Simscape pendulum (slower)
%   R: time series (t, x, th0, th1 true, r, y measured, u commanded duty, d applied duty, i true,
%      dbg), fault (code, time), metrics, pass, and the settings used.
arguments
    controller (1,:) char
    scenario
    opts.Mode (1,:) char = 'realistic'
    opts.Params struct = struct()
    opts.Plant struct = struct()
    opts.Perturb (1,1) double = 0
    opts.Seed (1,1) double = 1
    opts.Cfg struct = struct()
    opts.Plot (1,1) logical = false
    opts.Show3D (1,1) logical = false
end
root = harness_setup();
mdl = 'furuta_harness';
if ~bdIsLoaded(mdl), load_system(fullfile(root, [mdl '.slx'])); end
[V, P] = harness_prepare(mdl, controller, scenario, opts);
sc = V.sc;
cfg = V.hcfg;
hp = V.hp;
in = Simulink.SimulationInput(mdl);
for f = setdiff(fieldnames(V), {'sc'})'
    in = in.setVariable(f{1}, V.(f{1}));
end
onoff = {'off', 'on'};
in = in.setModelParameter('SimMechanicsOpenEditorOnUpdate', onoff{opts.Show3D + 1});
out = sim(in);
if ~isempty(out.ErrorMessage), error('run_sim:sim', '%s', out.ErrorMessage); end

R.controller = controller;
R.scenario = sc.name;
lg = out.logsout;
R.t = lg.get('x').Values.Time;
R.x = vals(lg, 'x');
R.th0 = R.x(:, 1);
R.th1 = wrap_pi(R.x(:, 2));
R.r = vals(lg, 'r');
R.y = vals(lg, 'y');
R.u = vals(lg, 'duty');
R.mode = vals(lg, 'mode');
R.t_fw = lg.get('d').Values.Time;      % firmware ticks
R.d_fw = vals(lg, 'd');                 % applied duty at every firmware tick
R.d = interp1(R.t_fw, R.d_fw, R.t, 'previous');
R.i = vals(lg, 'i');
R.dbg = vals(lg, 'dbg');
fl = vals(lg, 'fault');
R.fault = max(fl);
R.fault_time = R.t(find(fl > 0, 1));
R.fault_name = fault_name(R.fault);
R.metrics = harness_metrics(R, cfg, sc);
R.pass = logical(sc.check(R));
R.P = P;
R.cfg = cfg;
R.plant = hp;
R.text = sc.text;
if opts.Plot, plot_run(R); end
end

function v = vals(lg, name)
v = lg.get(name).Values.Data;
if ndims(v) == 3                      % [n 1 N] from column-vector signals
    v = reshape(permute(v, [3 1 2]), size(v, 3), []);
end
end

function s = fault_name(code)
names = {'none', 'ARM_TRAVEL', 'OVERCURRENT', 'NONFINITE_DUTY'};
s = names{code + 1};
end
