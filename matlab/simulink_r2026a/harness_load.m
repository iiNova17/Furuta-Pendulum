function harness_load(controller, scenario, opts)
%HARNESS_LOAD Open the harness ready for the Simulink Run button.
%   harness_load()                               the hybrid controller (ctrl_hybrid.slx) on 'swingup' (from hanging)
%   harness_load('ctrl_hybrid.slx', 'push')      any controller / scenario, same options as run_sim
%   Puts the controller in the slot and every variable the model uses (hcfg, hp, hb, hx0, hT, h_ref,
%   h_dist, h_noise, ctrlP) in the base workspace, then opens the model. After pressing Run, the
%   logged signals are in out.logsout (Simulation Data Inspector shows them too).
%   run_sim does not need this; it passes the same variables to each run directly.
%   The model's PostLoadFcn calls harness_load('ctrl_hybrid.slx', 'swingup') every time the model is
%   opened, so pressing Run starts from hanging at rest (theta0 = 0, theta1 = pi). For another scenario
%   call harness_load yourself after opening, e.g. harness_load('ctrl_hybrid.slx', 'push').
arguments
    controller (1,:) char = 'ctrl_hybrid.slx'
    scenario = 'swingup'
    opts.Mode (1,:) char = 'realistic'
    opts.Params struct = struct()
    opts.Plant struct = struct()
    opts.Perturb (1,1) double = 0
    opts.Seed (1,1) double = 1
    opts.Cfg struct = struct()
    opts.IfMissing (1,1) logical = false
    opts.Open (1,1) logical = true
end
% Models saved before 2026-11-21 (the R2026a copy) still call harness_load('LqrBalance', 'balance', 'IfMissing',
% true) on opening: treat that as the current callback, the hybrid controller on the swing-up from hanging.
if strcmp(controller, 'LqrBalance'), controller = 'ctrl_hybrid.slx'; scenario = 'swingup'; opts.IfMissing = false; end
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
