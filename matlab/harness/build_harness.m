function build_harness()
%BUILD_HARNESS Generate furuta_harness.slx (the universal simulation) from scratch.
%   The model is generated, not hand-edited, so it can always be rebuilt and reviewed as code.
%
%   ref --> [Controller] --duty--> [Rig] --y--+        Controller: the slot (hybrid Stateflow template,
%                ^----------------------------+                    or any controller with the interface)
%   Rig is a Variant Subsystem (label = hcfg.rig):
%     Simulation : HIL link (delays, rate change)  ->  Firmware  ->  Plant  ->  ADC  ->  Sensors
%                  Firmware : FirmwareActuator at hcfg.fw.Ts (clip, slew, standby, PWM, trips)
%                  Plant    : the report's model. Motor + driver (eq. 71, 88) and friction (eq. 78)
%                             as Simulink blocks; arm and pendulum as Simscape Multibody bodies whose
%                             equations of motion are exactly eq. 89 (plant_bodies). Meshes visual only.
%                  Sensors  : FurutaSensors at hcfg.fw.Ts (quantization, pendulum lag, current filter)
%     Hardware   : the USB link to the ESP32 for HIL (placeholder until the bridge exists)
%   Rates: controller hcfg.Ts, firmware/sensors hcfg.fw.Ts, plant hcfg.dt (all in harness_config).
%   Logged signals (signal logging, every hcfg.Ts): r, duty, mode, dbg, y, x, i, fault; d every hcfg.fw.Ts.
%   Workspace variables used: hcfg, hp, hb, hx0, hT, h_ref, h_dist, h_noise, ctrlP (set by run_sim
%   or harness_load).
root = harness_setup();
mdl = 'furuta_harness';
if bdIsLoaded(mdl), close_system(mdl, 0); end
file = fullfile(root, [mdl '.slx']);
if isfile(file), delete(file); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'hcfg.dt', ...
    'StopTime', 'hT', 'SaveTime', 'off', 'SaveOutput', 'off', 'SaveState', 'off', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
    'AlgebraicLoopMsg', 'error');
% opening the model by hand: make sure every parameter resolves (no red blocks, Run works)
set_param(mdl, 'PostLoadFcn', sprintf(['addpath(fileparts(get_param(''%s'', ''FileName'')));\n' ...
    'harness_load(''LqrBalance'', ''balance'', ''IfMissing'', true, ''Open'', false);'], mdl));

blk = @(lib, name, pos, varargin) add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
area(mdl, 'Controller slot (yours)', [140 30 470 330], [0.86 0.95 0.86]);
area(mdl, 'Rig: Simulation (report model) or Hardware (ESP32, HIL)', [500 30 900 330], [0.88 0.92 1.0]);

blk('simulink/Sources/From Workspace', 'ref', [30 215 100 245], 'VariableName', 'h_ref', ...
    'SampleTime', 'hcfg.Ts', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
slot = [mdl '/Controller'];
blk('simulink/Ports & Subsystems/Subsystem', 'Controller', [210 130 340 280]);
delete_line(slot, 'In1/1', 'Out1/1');
delete_block([slot '/In1']); delete_block([slot '/Out1']);
add_block('simulink/Sources/In1', [slot '/y'], 'Position', [40 50 70 64]);
add_block('simulink/Sources/In1', [slot '/r'], 'Position', [40 110 70 124]);
add_block('simulink/Sinks/Out1', [slot '/duty'], 'Position', [400 40 430 54]);
add_block('simulink/Sinks/Out1', [slot '/mode'], 'Position', [400 80 430 94]);
add_block('simulink/Sinks/Out1', [slot '/dbg'], 'Position', [400 120 430 134]);
set_controller(mdl, 'system', 'LqrBalance');
blk('simulink/Sinks/Terminator', 'mode out', [400 197 415 213], 'ShowName', 'off');
blk('simulink/Sinks/Terminator', 'dbg out', [400 247 415 263], 'ShowName', 'off');

rig = [mdl '/Rig'];
blk('simulink/Ports & Subsystems/Variant Subsystem', 'Rig', [580 130 720 280]);
build_rig(rig);
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');   % run_sim('Show3D', true) opens it
for k = 1:4
    blk('simulink/Sinks/Terminator', sprintf('rig out %d', k + 1), ...
        [790 147 + 25*k 805 163 + 25*k], 'ShowName', 'off');
end
set_param(slot, 'ContentPreviewEnabled', 'off', 'BackgroundColor', '[0.75 0.92 0.75]');
set_param(rig, 'ContentPreviewEnabled', 'off', 'BackgroundColor', '[0.8 0.87 1.0]');

H = @(b) get_param([mdl '/' b], 'PortHandles');
W = @(src, k, dst, j) add_line(mdl, H(src).Outport(k), H(dst).Inport(j), 'autorouting', 'smart');
lr = W('ref', 1, 'Controller', 2);
lu = W('Controller', 1, 'Rig', 1);
lm = W('Controller', 2, 'mode out', 1);
lg = W('Controller', 3, 'dbg out', 1);
lo = cell(1, 4);
for k = 1:4
    lo{k} = W('Rig', k + 1, sprintf('rig out %d', k + 1), 1);
end
ys = get_param(H('Rig').Outport(1), 'Position');
yc = get_param(H('Controller').Inport(1), 'Position');
ly = add_line(mdl, [ys; 760, ys(2); 760, 80; 180, 80; 180, yc(2); yc]);
names = {lr, 'r'; lu, 'duty'; lm, 'mode'; lg, 'dbg'; ly, 'y'; lo{1}, 'x'; lo{2}, 'i'; lo{3}, 'd'; lo{4}, 'fault'};
for k = 1:size(names, 1)
    set_param(names{k, 1}, 'Name', names{k, 2});
    ph = get_param(names{k, 1}, 'SrcPortHandle');
    rate = 'hcfg.Ts';
    if strcmp(names{k, 2}, 'd'), rate = 'hcfg.fw.Ts'; end     % applied duty: every firmware tick
    set_param(ph, 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName', ...
        'DataLoggingSampleTime', rate);
end
note(mdl, sprintf(['mode: 0 = stopped / safe, 1 = swing-up, 2 = stabilizing.  Angles in rad (theta1 = 0 upright),\n' ...
    'duty in [-1, 1], current in A. Settings: harness_config.m (rig, rates), the controller design function (gains).\n' ...
    'Run: run_sim(controller, scenario) or harness_load(...) + Run. tests/test_harness.m checks the plant against the report.']), ...
    [140 350]);
save_system(mdl, file);
close_system(mdl, 0);
fprintf('Built %s\n', file);
end

% ------------------------------------------------------------------------------------------------
function build_rig(V)
% Variant Subsystem: label 'Simulation' (report model) / 'Hardware' (HIL placeholder).
% Ports: in duty; out y (measured, at hcfg.Ts), x (true state), i (true current), d (applied duty), fault.
kids = find_system(V, 'SearchDepth', 1, 'MatchFilter', @Simulink.match.allVariants);
for k = 1:numel(kids)
    if ~strcmp(kids{k}, V), delete_block(kids{k}); end
end
delete_line(find_system(V, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line'));
add_block('simulink/Sources/In1', [V '/duty'], 'Position', [20 100 50 114]);
outs = {'y', 'x', 'i', 'd', 'fault'};
for k = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [V '/' outs{k}], 'Position', [500 40 + 40*k 530 54 + 40*k]);
end
set_param(V, 'VariantControlMode', 'label');
S = [V '/Simulation'];
add_block('simulink/Ports & Subsystems/Subsystem', S, 'Position', [200 40 350 160]);
build_sim_rig(S, outs);
set_param(S, 'VariantControl', 'Simulation');
Hw = [V '/Hardware'];
add_block('simulink/Ports & Subsystems/Subsystem', Hw, 'Position', [200 200 350 320]);
build_hw_placeholder(Hw, outs);
set_param(Hw, 'VariantControl', 'Hardware');
set_param(V, 'LabelModeActiveChoice', 'Simulation');
end

function build_sim_rig(S, outs)
% Simulation choice of the Rig. Left to right: HIL link | firmware | plant | sensors | outputs.
% Signals that go backwards or leave the subsystem travel by Goto/From tags.
delete_line(S, 'In1/1', 'Out1/1');
delete_block([S '/In1']); delete_block([S '/Out1']);
B = @(lib, name, pos, varargin) add_block(lib, [S '/' name], 'Position', pos, varargin{:});
H = @(b) get_param([S '/' b], 'PortHandles');
W = @(src, k, dst, j) add_line(S, H(src).Outport(k), H(dst).Inport(j), 'autorouting', 'smart');
tag = @(kind, n, tg, y, x) B(['simulink/Signal Routing/' kind], n, [x y - 9 x + 50 y + 9], ...
    'GotoTag', tg, 'ShowName', 'off');
ufn = 'simulink/User-Defined Functions/';
fromws = {'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value'};

area(S, 'HIL link: USB delays and rate change', [10 10 330 330], [0.95 0.95 0.95]);
area(S, 'ESP32 firmware (fw.Ts)', [350 10 600 330], [1.0 0.95 0.85]);
area(S, 'Plant: report model (open it)', [610 10 810 330], [0.88 0.92 1.0]);
area(S, 'ESP32 sensors (fw.Ts)', [830 10 1150 330], [0.92 0.97 0.92]);

% measurements back to the controller (top row)
tag('From', 'y_fw to link', 'y_fw', 45, 20);
B('simulink/Signal Attributes/Rate Transition', 'to controller rate', [110 30 150 60], ...
    'OutPortSampleTime', 'hcfg.Ts');
B('simulink/Discrete/Delay', 'measurement delay', [190 30 230 60], 'DelayLength', 'hcfg.link.y_delay', ...
    'SampleTime', 'hcfg.Ts');
B('simulink/Sinks/Out1', 'y', [280 38 310 52]);
W('y_fw to link', 1, 'to controller rate', 1);
W('to controller rate', 1, 'measurement delay', 1);
W('measurement delay', 1, 'y', 1);

% commands to the firmware (second row, y = 122)
B('simulink/Sources/In1', 'duty', [20 115 50 129]);
B('simulink/Discrete/Delay', 'command delay', [110 107 150 137], 'DelayLength', 'hcfg.link.u_delay', ...
    'SampleTime', 'hcfg.Ts');
B('simulink/Signal Attributes/Rate Transition', 'to firmware rate', [190 107 230 137], ...
    'OutPortSampleTime', 'hcfg.fw.Ts');
W('duty', 1, 'command delay', 1);
W('command delay', 1, 'to firmware rate', 1);

% firmware: inputs at 122 (cmd) and 177 (y), outputs at 122 (d) and 177 (fault)
B([ufn 'MATLAB System'], 'Firmware', [420 95 530 205]);
set_param([S '/Firmware'], 'System', 'FirmwareActuator');
set_param([S '/Firmware'], 'Ts', 'hcfg.fw.Ts', 'A', 'hcfg.act', 'SimulateUsing', 'Interpreted execution');
tag('From', 'y_fw to firmware', 'y_fw', 177, 360);
W('to firmware rate', 1, 'Firmware', 1);
W('y_fw to firmware', 1, 'Firmware', 2);                 % trips use the firmware's own sensors
tag('Goto', 'd goto', 'd', 60, 540);
tag('Goto', 'fault goto', 'fault', 300, 540);
W('Firmware', 2, 'fault goto', 1);

% plant: inputs at 122 (d) and 177 (dist), outputs at 122 (x) and 177 (i)
B('simulink/Ports & Subsystems/Subsystem', 'Plant', [670 95 790 205]);
build_plant([S '/Plant']);
set_param([S '/Plant'], 'ContentPreviewEnabled', 'off', 'BackgroundColor', '[0.8 0.87 1.0]');
B('simulink/Sources/From Workspace', 'disturbance', [615 163 655 191], 'VariableName', 'h_dist', ...
    'SampleTime', 'hcfg.dt', fromws{:});
W('Firmware', 1, 'Plant', 1);
W('Firmware', 1, 'd goto', 1);
add_line(S, H('disturbance').Outport(1), H('Plant').Inport(2));   % ports level: straight
tag('Goto', 'x goto', 'x', 60, 850);
tag('Goto', 'i goto', 'i', 300, 850);

% sensors: inputs at 122 (x), 165 (i via ADC), 208 (noise)
B([ufn 'MATLAB System'], 'Sensors', [980 101 1080 229]);
set_param([S '/Sensors'], 'System', 'FurutaSensors');
set_param([S '/Sensors'], 'Ts', 'hcfg.fw.Ts', 'S', 'hcfg.sens', 'SimulateUsing', 'Interpreted execution');
B('simulink/Discrete/Unit Delay', 'ADC sample', [880 150 920 180], 'SampleTime', 'hcfg.fw.Ts', ...
    'InitialCondition', '0');
B('simulink/Sources/From Workspace', 'current noise', [870 193 940 223], 'VariableName', 'h_noise', ...
    'SampleTime', 'hcfg.fw.Ts', fromws{:});
W('Plant', 1, 'Sensors', 1);
W('Plant', 1, 'x goto', 1);
W('Plant', 2, 'ADC sample', 1);
W('Plant', 2, 'i goto', 1);
W('ADC sample', 1, 'Sensors', 2);
W('current noise', 1, 'Sensors', 3);
tag('Goto', 'y_fw goto', 'y_fw', 165, 1095);
W('Sensors', 1, 'y_fw goto', 1);

% outputs to the top level (truth for logging, plus the firmware's view)
for k = 2:numel(outs)
    y = 60 + 50*k;
    tag('From', [outs{k} ' out'], outs{k}, y, 1180);
    B('simulink/Sinks/Out1', outs{k}, [1260 y - 7 1290 y + 7]);
    W([outs{k} ' out'], 1, outs{k}, 1);
end
end

function build_hw_placeholder(Hw, outs)
delete_line(Hw, 'In1/1', 'Out1/1');
delete_block([Hw '/In1']); delete_block([Hw '/Out1']);
add_block('simulink/Sources/In1', [Hw '/duty'], 'Position', [30 100 60 114]);
add_block('simulink/Sinks/Terminator', [Hw '/to ESP32 (TODO)'], 'Position', [120 97 140 117]);
add_line(Hw, 'duty/1', 'to ESP32 (TODO)/1');
dims = {'zeros(3,1)', 'zeros(4,1)', '0', '0', '0'};
for k = 1:numel(outs)
    add_block('simulink/Sources/Constant', [Hw '/' outs{k} ' (TODO)'], 'Value', dims{k}, ...
        'SampleTime', 'hcfg.Ts', 'Position', [300 40 + 50*k 360 60 + 50*k]);
    add_block('simulink/Sinks/Out1', [Hw '/' outs{k}], 'Position', [420 43 + 50*k 450 57 + 50*k]);
    add_line(Hw, [outs{k} ' (TODO)/1'], [outs{k} '/1']);
end
note(Hw, sprintf(['HIL bridge (next step): send duty to the ESP32 over USB, receive y = [theta0; theta1; i],\n' ...
    'paced in real time by Simulink Desktop Real-Time. The controller slot does not change.']), [30 20]);
end

% ------------------------------------------------------------------------------------------------
function build_plant(S)
% Plant subsystem: inputs d, dist; outputs x = [th0 th1 w0 w1], i.
% Layout: ports of connected blocks share a y coordinate so lines are straight; signals that
% cross sections travel by Goto/From tags ([w0], [i], ...).
delete_line(S, 'In1/1', 'Out1/1');
delete_block([S '/In1']); delete_block([S '/Out1']);
B = @(lib, name, pos, varargin) add_block(lib, [S '/' name], 'Position', pos, varargin{:});
H = @(b) get_param([S '/' b], 'PortHandles');
L = @(a, b) add_line(S, a, b, 'autorouting', 'smart');
W = @(src, k, dst, j) L(H(src).Outport(k), H(dst).Inport(j));
tag = @(kind, name, tg, y, x) B(['simulink/Signal Routing/' kind], name, [x y - 9 x + 50 y + 9], ...
    'GotoTag', tg, 'ShowName', 'off');

area(S, 'Motor + driver (report eq. 71, 88):  i = (d Vs - Kb w0) / Rm,  i = 0 when d = 0', ...
    [10 5 600 212], [1.0 0.95 0.85]);
area(S, 'Friction (eq. 78; Coulomb smoothed as in 3.2) and joint torques', ...
    [10 220 600 570], [1.0 0.9 0.9]);
area(S, 'Arm + pendulum: report eq. 89 as Simscape Multibody (parameters from plant_bodies; meshes visual only)', ...
    [630 10 1560 570], [0.88 0.92 1.0]);

% --- motor + driver ---------------------------------------------------------------------------
B('simulink/Sources/In1', 'd', [30 70 60 84]);
B('simulink/Math Operations/Gain', 'Vs', [120 62 170 92], 'Gain', 'hp.Vs');
tag('From', 'w0 to motor', 'w0', 135, 40);
B('simulink/Math Operations/Gain', 'Kb', [120 120 170 150], 'Gain', 'hp.Kb');
B('simulink/Math Operations/Sum', 'V - Kb w0', [220 48 250 164], 'Inputs', '+-', 'IconShape', 'rectangular');
B('simulink/Math Operations/Gain', 'divide by Rm', [290 91 350 121], 'Gain', '1/hp.Rm');
tag('Goto', 'd goto', 'd', 40, 120);
tag('From', 'd to switch', 'd', 146, 360);
B('simulink/Sources/Constant', 'standby: i = 0', [360 176 410 196], 'Value', '0');
B('simulink/Signal Routing/Switch', 'driver on?', [450 86 480 206], 'Criteria', 'u2 ~= 0');
B('simulink/Sinks/Out1', 'i', [540 139 570 153]);
tag('Goto', 'i goto', 'i', 180, 520);
W('d', 1, 'Vs', 1);
L(H('d').Outport(1), H('d goto').Inport(1));
W('w0 to motor', 1, 'Kb', 1);
W('Vs', 1, 'V - Kb w0', 1);
W('Kb', 1, 'V - Kb w0', 2);
W('V - Kb w0', 1, 'divide by Rm', 1);
W('divide by Rm', 1, 'driver on?', 1);
W('d to switch', 1, 'driver on?', 2);
W('standby: i = 0', 1, 'driver on?', 3);
W('driver on?', 1, 'i', 1);
W('driver on?', 1, 'i goto', 1);

% --- friction and torques -----------------------------------------------------------------------
tag('From', 'w0 to friction', 'w0', 270, 40);
B('simulink/Math Operations/Gain', 'divide by eps', [110 255 185 285], 'Gain', '1/hp.eps');
B('simulink/Math Operations/Trigonometric Function', 'tanh', [200 255 240 285], 'Operator', 'tanh');
B('simulink/Sources/Constant', 'tau_c+', [120 302 170 318], 'Value', 'hp.tcp');
tag('From', 'w0 to sign', 'w0', 330, 120);
B('simulink/Sources/Constant', 'tau_c-', [120 342 170 358], 'Value', 'hp.tcn');
B('simulink/Signal Routing/Switch', 'w0 > 0 ?', [210 300 240 360], 'Criteria', 'u2 > Threshold', ...
    'Threshold', '0');
B('simulink/Math Operations/Product', 'arm Coulomb', [300 240 330 360]);
W('w0 to friction', 1, 'divide by eps', 1);
W('divide by eps', 1, 'tanh', 1);
W('tanh', 1, 'arm Coulomb', 1);
W('tau_c+', 1, 'w0 > 0 ?', 1);
W('w0 to sign', 1, 'w0 > 0 ?', 2);
W('tau_c-', 1, 'w0 > 0 ?', 3);
W('w0 > 0 ?', 1, 'arm Coulomb', 2);

tag('From', 'w1 to friction', 'w1', 430, 40);
B('simulink/Math Operations/Gain', 'divide by eps1', [110 415 185 445], 'Gain', '1/hp.eps1');
B('simulink/Math Operations/Trigonometric Function', 'tanh (pendulum)', [200 415 240 445], 'Operator', 'tanh');
B('simulink/Math Operations/Gain', 'pendulum dry', [300 415 350 445], 'Gain', 'hp.tp');
W('w1 to friction', 1, 'divide by eps1', 1);
W('divide by eps1', 1, 'tanh (pendulum)', 1);
W('tanh (pendulum)', 1, 'pendulum dry', 1);

B('simulink/Sources/In1', 'dist', [30 513 60 527]);
B('simulink/Signal Routing/Demux', 'dist demux', [110 495 115 545], 'Outputs', '2');
tag('Goto', 'dist0 goto', 'dist0', 507, 150);
tag('Goto', 'dist1 goto', 'dist1', 533, 150);
W('dist', 1, 'dist demux', 1);
W('dist demux', 1, 'dist0 goto', 1);
W('dist demux', 2, 'dist1 goto', 1);

tag('From', 'i to torque', 'i', 270, 380);
tag('From', 'dist0 to torque', 'dist0', 330, 380);
B('simulink/Math Operations/Sum', 'arm torque', [460 255 490 345], 'Inputs', '+-+', 'IconShape', 'rectangular');
tag('From', 'dist1 to torque', 'dist1', 460, 380);
B('simulink/Math Operations/Sum', 'pendulum torque', [460 415 490 475], 'Inputs', '-+', 'IconShape', 'rectangular');
tag('Goto', 'tau0 goto', 'tau0', 300, 520);
tag('Goto', 'tau1 goto', 'tau1', 445, 520);
W('i to torque', 1, 'arm torque', 1);
W('arm Coulomb', 1, 'arm torque', 2);
W('dist0 to torque', 1, 'arm torque', 3);
W('pendulum dry', 1, 'pendulum torque', 1);
W('dist1 to torque', 1, 'pendulum torque', 2);
W('arm torque', 1, 'tau0 goto', 1);
W('pendulum torque', 1, 'tau1 goto', 1);

% --- Simscape Multibody -------------------------------------------------------------------------
B('sm_lib/Utilities/Mechanism Configuration', 'Gravity', [660 60 700 100], 'GravityVector', '[0 0 -hb.g]');
B('sm_lib/Frames and Transforms/World Frame', 'World', [660 270 700 310]);
B('nesl_utility/Solver Configuration', 'Solver', [660 470 700 510]);
B('sm_lib/Frames and Transforms/Rigid Transform', 'base centre', [750 60 790 100], ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[0 0 0.05]', ...
    'TranslationCartesianOffsetUnits', 'm');
B('sm_lib/Body Elements/Brick Solid', 'Base (visual)', [840 50 890 110], ...
    'BrickDimensions', '[0.0872 0.0872 0.1]', 'BrickDimensionsUnits', 'm', 'InertiaType', 'PointMass', ...
    'Mass', '1e-9', 'GraphicDiffuseColor', '[0.6 0.62 0.65]', 'GraphicOpacity', '0.4');
B('sm_lib/Frames and Transforms/Rigid Transform', 'motor axis at arm height', [750 270 790 310], ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[0 0 0.105]', ...
    'TranslationCartesianOffsetUnits', 'm');
jointArgs = {'TorqueActuationMode', 'InputTorque', 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'PositionTargetSpecify', 'on', 'PositionTargetValueUnits', 'rad', 'PositionTargetPriority', 'High', ...
    'VelocityTargetSpecify', 'on', 'VelocityTargetValueUnits', 'rad/s', 'VelocityTargetPriority', 'High', ...
    'DampingCoefficientUnits', 'N*m/(rad/s)'};   % Simscape's default is per deg/s
tag('From', 'tau0 from', 'tau0', 370, 700);
B('nesl_utility/Simulink-PS Converter', 'tau0', [790 360 820 380], 'Unit', 'N*m');
B('sm_lib/Joints/Revolute Joint', 'Motor joint (theta0)', [860 260 930 340], jointArgs{:}, ...
    'DampingCoefficient', 'hp.Beq', 'PositionTargetValue', 'hx0(1)', 'VelocityTargetValue', 'hx0(3)');
mesh(S, 'Arm (visual)', 'body_arm_jumper3.stl', '[0.15 0.15 0.17]', [990 50 1040 110]);
B('sm_lib/Body Elements/Inertia', 'Arm inertia Ia', [990 150 1040 200], 'InertiaType', 'Custom', ...
    'Mass', 'hb.m_arm', 'MassUnits', 'kg', 'CenterOfMass', '[0 0 0]', 'CenterOfMassUnits', 'm', ...
    'MomentsOfInertia', '[hb.Ia/2 hb.Ia/2 hb.Ia]', 'ProductsOfInertia', '[0 0 0]', ...
    'MomentsOfInertiaUnits', 'kg*m^2', 'ProductsOfInertiaUnits', 'kg*m^2');
B('sm_lib/Frames and Transforms/Rigid Transform', 'pivot at Larm', [990 270 1030 310], ...
    'RotationMethod', 'RotationMatrix', 'RotationMatrix', '[0 0 -1; 0 1 0; 1 0 0]', ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[hb.Larm 0 0.014]', ...
    'TranslationCartesianOffsetUnits', 'm');
tag('From', 'tau1 from', 'tau1', 370, 1000);
B('nesl_utility/Simulink-PS Converter', 'tau1', [1080 360 1110 380], 'Unit', 'N*m');
B('sm_lib/Joints/Revolute Joint', 'Pendulum joint (theta1)', [1150 260 1220 340], jointArgs{:}, ...
    'DampingCoefficient', 'hp.b1', 'PositionTargetValue', 'hx0(2)', 'VelocityTargetValue', 'hx0(4)');
B('sm_lib/Frames and Transforms/Rigid Transform', 'mesh pose', [1280 60 1320 100], ...
    'RotationMethod', 'RotationMatrix', 'RotationMatrix', '[0 0 1; 1 0 0; 0 1 0]');
mesh(S, 'Pendulum (visual)', 'body_pendulum_egp.stl', '[0.8 0.1 0.1]', [1370 50 1420 110]);
B('sm_lib/Body Elements/Inertia', 'Pendulum m, Lp, J1', [1280 150 1330 200], 'InertiaType', 'Custom', ...
    'Mass', 'hb.m', 'MassUnits', 'kg', 'CenterOfMass', '[hb.Lp 0 0]', 'CenterOfMassUnits', 'm', ...
    'MomentsOfInertia', '[1e-12 hb.J1 hb.J1]', 'ProductsOfInertia', '[0 0 0]', ...
    'MomentsOfInertiaUnits', 'kg*m^2', 'ProductsOfInertiaUnits', 'kg*m^2');
sensors = {'th0', 'Motor joint (theta0)', 2, 'rad', 950, 430; 'w0', 'Motor joint (theta0)', 3, 'rad/s', 950, 490; ...
           'th1', 'Pendulum joint (theta1)', 2, 'rad', 1240, 430; 'w1', 'Pendulum joint (theta1)', 3, 'rad/s', 1240, 490};
for k = 1:4
    [n, j, port, u, x, y] = sensors{k, :};
    B('nesl_utility/PS-Simulink Converter', [n ' PS'], [x y - 10 x + 30 y + 10], 'Unit', u);
    tag('Goto', [n ' goto'], n, y, x + 60);
    L(subsref(get_param([S '/' j], 'PortHandles'), struct('type', {'.', '()'}, 'subs', {'RConn', {port}})), ...
      H([n ' PS']).LConn(1));
    W([n ' PS'], 1, [n ' goto'], 1);
end
order = {'th0', 'th1', 'w0', 'w1'};
B('simulink/Signal Routing/Mux', 'x = [th0 th1 w0 w1]', [1470 400 1475 520], 'Inputs', '4');
for k = 1:4
    tag('From', [order{k} ' to x'], order{k}, 400 + 15*(2*k - 1), 1390);   % Mux input heights
    W([order{k} ' to x'], 1, 'x = [th0 th1 w0 w1]', k);
end
B('simulink/Sinks/Out1', 'x', [1510 453 1540 467]);
W('x = [th0 th1 w0 w1]', 1, 'x', 1);
set_param([S '/x'], 'Port', '1');          % x is output 1, i output 2 (created in the other order)

P = @(b, side, k) subsref(get_param([S '/' b], 'PortHandles'), struct('type', {'.', '()'}, 'subs', {side, {k}}));
W('tau0 from', 1, 'tau0', 1);
W('tau1 from', 1, 'tau1', 1);
L(P('World', 'RConn', 1), P('Gravity', 'RConn', 1));
L(P('World', 'RConn', 1), P('Solver', 'RConn', 1));
L(P('World', 'RConn', 1), P('base centre', 'LConn', 1));
L(P('base centre', 'RConn', 1), P('Base (visual)', 'RConn', 1));
L(P('World', 'RConn', 1), P('motor axis at arm height', 'LConn', 1));
L(P('motor axis at arm height', 'RConn', 1), P('Motor joint (theta0)', 'LConn', 1));
L(P('tau0', 'RConn', 1), P('Motor joint (theta0)', 'LConn', 2));
L(P('Motor joint (theta0)', 'RConn', 1), P('pivot at Larm', 'LConn', 1));
L(P('Motor joint (theta0)', 'RConn', 1), P('Arm inertia Ia', 'RConn', 1));
L(P('Motor joint (theta0)', 'RConn', 1), P('Arm (visual)', 'RConn', 1));
L(P('pivot at Larm', 'RConn', 1), P('Pendulum joint (theta1)', 'LConn', 1));
L(P('tau1', 'RConn', 1), P('Pendulum joint (theta1)', 'LConn', 2));
L(P('Pendulum joint (theta1)', 'RConn', 1), P('Pendulum m, Lp, J1', 'RConn', 1));
L(P('Pendulum joint (theta1)', 'RConn', 1), P('mesh pose', 'LConn', 1));
L(P('mesh pose', 'RConn', 1), P('Pendulum (visual)', 'RConn', 1));
end

function mesh(S, name, stl, rgb, pos)
add_block('sm_lib/Body Elements/File Solid', [S '/' name], 'Position', pos);
set_param([S '/' name], 'ExtGeomFileName', stl, 'UnitType', 'Custom', 'ExtGeomFileUnits', 'mm', ...
    'InertiaType', 'PointMass', 'Mass', '1e-9', 'GraphicType', 'FromGeometry', 'GraphicDiffuseColor', rgb);
end

function area(sys, txt, pos, rgb)
h = add_block('built-in/Area', [sys '/' matlab.lang.makeValidName(strtok(txt, ':,(-'))], 'Position', pos);
set_param(h, 'Text', txt, 'BackgroundColor', mat2str(rgb), 'FontWeight', 'bold');
end

function note(sys, txt, xy)
a = Simulink.Annotation([sys '/' strtok(txt, ':')]);
a.Text = txt;
a.Position = [xy xy + [700 50]];
a.HorizontalAlignment = 'left';
end
