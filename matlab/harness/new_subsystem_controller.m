function file = new_subsystem_controller(name)
%NEW_SUBSYSTEM_CONTROLLER Create a Simulink-diagram controller with the harness's slot interface.
%   new_subsystem_controller('ctrl_ahmad')  creates controllers/ctrl_ahmad/ctrl_ahmad.slx with
%     inports  y (3x1: theta0, theta1, i)  and  r (arm reference),
%     outports duty, mode (0 stopped, 1 swing-up, 2 stabilizing) and dbg (4x1),
%   wired to zero output. Open it, draw your controller between the ports (discrete blocks at
%   sample time hcfg.Ts; read your knobs from the struct ctrlP, e.g. a Gain of 'ctrlP.K'),
%   then:  run_sim('ctrl_ahmad.slx', 'balance', 'Plot', true)
%   Optional: a function ctrl_ahmad_design(cfg, P) next to it that fills ctrlP defaults.
root = harness_setup();
folder = fullfile(root, 'controllers', name);
file = fullfile(folder, [name '.slx']);
if isfile(file), error('new_subsystem_controller:exists', '%s already exists', file); end
if ~isfolder(folder), mkdir(folder); end
addpath(folder);
new_system(name, 'Subsystem');
add_block('simulink/Sources/In1', [name '/y'], 'PortDimensions', '3', 'SampleTime', 'hcfg.Ts', ...
    'Position', [40 50 70 64]);
add_block('simulink/Sources/In1', [name '/r'], 'Position', [40 150 70 164]);
add_block('simulink/Sinks/Out1', [name '/duty'], 'Position', [500 50 530 64]);
add_block('simulink/Sinks/Out1', [name '/mode'], 'Position', [500 100 530 114]);
add_block('simulink/Sinks/Out1', [name '/dbg'], 'Position', [500 150 530 164]);
add_block('simulink/Sources/Constant', [name '/zero mode'], 'Value', '0', ...
    'SampleTime', 'hcfg.Ts', 'Position', [300 90 360 120]);
add_block('simulink/Sources/Constant', [name '/zero duty'], 'Value', '0', ...
    'SampleTime', 'hcfg.Ts', 'Position', [300 40 360 70]);
add_block('simulink/Sources/Constant', [name '/zero dbg'], 'Value', 'zeros(4,1)', ...
    'SampleTime', 'hcfg.Ts', 'Position', [300 140 360 170]);
add_block('simulink/Sinks/Terminator', [name '/unused y'], 'Position', [140 50 160 64]);
add_block('simulink/Sinks/Terminator', [name '/unused r'], 'Position', [140 150 160 164]);
add_line(name, 'y/1', 'unused y/1');
add_line(name, 'r/1', 'unused r/1');
add_line(name, 'zero duty/1', 'duty/1');
add_line(name, 'zero mode/1', 'mode/1');
add_line(name, 'zero dbg/1', 'dbg/1');
save_system(name, file);
close_system(name, 0);
fprintf('Created %s\n', file);
end
