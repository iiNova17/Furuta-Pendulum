function [V, P] = harness_prepare(mdl, controller, scenario, opts)
%HARNESS_PREPARE Everything one run needs: put the controller in the slot and build the variables.
%   [V, P] = harness_prepare(mdl, controller, scenario, opts)
%   V is a struct of workspace variables (hcfg, hp, hb, hx0, hT, h_ref, h_dist, h_noise, ctrlP);
%   run_sim passes them to sim(), harness_load puts them in the base workspace.
%   opts: Mode, Params, Plant, Perturb, Seed, Cfg (see run_sim). Returns the controller params P.
if isempty(fieldnames(opts.Cfg)), cfg = harness_config(opts.Mode); else, cfg = opts.Cfg; end
if ischar(scenario) || isstring(scenario), sc = furuta_scenario(char(scenario), cfg); else, sc = scenario; end
isMult = @(a, b) abs(a/b - round(a/b)) < 1e-9 && round(a/b) >= 1;
assert(isMult(cfg.Ts, cfg.fw.Ts) && isMult(cfg.fw.Ts, cfg.dt), 'harness:rates', ...
    'cfg.Ts must be a multiple of cfg.fw.Ts, which must be a multiple of cfg.dt');
set_param([mdl '/Rig'], 'LabelModeActiveChoice', cfg.rig);   % 'Simulation' or 'Hardware'

% controller and its parameters (designed on the nominal model, whatever the plant is)
if endsWith(controller, '.slx')
    name = erase(controller, '.slx');
    set_controller(mdl, 'subsystem', name);
    P = opts.Params;
    if exist([name '_design'], 'file'), P = feval([name '_design'], cfg, P); end
else
    set_controller(mdl, 'system', controller);
    P = feval([controller '.design'], cfg, opts.Params);
end
set_param(mdl, 'Dirty', 'off');      % the slot is set per run; never save it into the file

hp = cfg.p;
if opts.Perturb > 0, hp = perturb_params(hp, opts.Perturb, opts.Seed); end
f = fieldnames(opts.Plant);
for k = 1:numel(f), hp.(f{k}) = opts.Plant.(f{k}); end

t = (0:cfg.Ts:sc.T)';
tf = (0:cfg.fw.Ts:sc.T)';
td = (0:cfg.dt:sc.T)';
rs = RandStream('mt19937ar', 'Seed', opts.Seed);
V.hcfg = cfg;
V.hp = hp;
V.hb = plant_bodies(hp);
V.hx0 = sc.x0(:);
V.hT = sc.T;
V.h_ref = [t, sc.ref(t)];
V.h_dist = [td, sc.dist(td)];
V.h_noise = [tf, randn(rs, numel(tf), 1)];       % current noise, one sample per firmware tick
V.ctrlP = P;
V.sc = sc;              % not a model variable; returned for run_sim
end
