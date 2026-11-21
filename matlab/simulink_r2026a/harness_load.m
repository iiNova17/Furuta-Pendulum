function harness_load(controller, scenario, opts)
%HARNESS_LOAD Open the harness ready for the Simulink Run button.
%   harness_load()                               the hybrid controller (ctrl_hybrid.slx) on 'balance'
%   harness_load('ctrl_hybrid.slx', 'swingup')   any controller / scenario, same options as run_sim
%   Puts the controller in the slot and every variable the model uses (hcfg, hp, hb, hx0, hT, h_ref,
%   h_dist, h_noise, ctrlP) in the base workspace, then opens the model. After pressing Run, the
%   logged signals are in out.logsout (Simulation Data Inspector shows them too).
%   run_sim does not need this; it passes the same variables to each run directly.
%   The model's PostLoadFcn calls harness_load('IfMissing', true) so it never opens with
%   unresolved parameters (red blocks).
arguments
    controller (1,:) char = 'ctrl_hybrid.slx'
    scenario = 'balance'
    opts.Mode (1,:) char = 'realistic'
    opts.Params struct = struct()
    opts.Plant struct = struct()
    opts.Perturb (1,1) double = 0
    opts.Seed (1,1) double = 1
    opts.Cfg struct = struct()
    opts.IfMissing (1,1) logical = false
    opts.Open (1,1) logical = true
end
% The saved model's PostLoadFcn still names LqrBalance, the example controller the model was saved with;
% the project's controller is the hybrid one, so that name opens it instead.
if strcmp(controller, 'LqrBalance'), controller = 'ctrl_hybrid.slx'; end
if opts.IfMissing && evalin('base', 'exist(''hcfg'', ''var'') && exist(''hb'', ''var'')')
    return
end
root = harness_setup();
mdl = 'furuta_harness';
if ~bdIsLoaded(mdl), load_system(fullfile(root, [mdl '.slx'])); end
[V, P] = harness_prepare(mdl, controller, scenario, rmfield(opts, {'IfMissing', 'Open'})); %#ok<ASGLU>
for f = setdiff(fieldnames(V), {'sc'})'
    assignin('base', f{1}, V.(f{1}));
end
if opts.Open, open_system(mdl); end
fprintf('Harness loaded: %s on %s (%s). Press Run in Simulink.\n', controller, V.sc.name, V.hcfg.mode);
end
