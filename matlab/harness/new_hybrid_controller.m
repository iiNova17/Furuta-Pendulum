function new_hybrid_controller(opts)
%NEW_HYBRID_CONTROLLER Create the course's hybrid controller template in controllers/hybrid/.
%   new_hybrid_controller()                      creates the files (refuses to overwrite)
%   new_hybrid_controller('Overwrite', true)     regenerates them (your edits are lost!)
%
%   Files (each owner edits only their own; .slx files do not merge in git):
%     ctrl_hybrid.slx          integration: estimator, stabilizer selector, friction compensation,
%                              Stateflow Supervisor (SwingUp / Stabilize / SafeStop)
%     stab_pid.slx             PID stabilizer            inputs: xh, r, active   output: u
%     stab_pole_placement.slx  pole-placement stabilizer   (same interface)
%     stab_lqr.slx             LQR stabilizer              (same interface)
%     swingup_energy.slx       energy swing-up           inputs: xh, r   outputs: u, E
%     ctrl_hybrid_design.m     every knob (ctrlP), one section per owner (not generated)
%   xh = [theta0; theta1 (wrapped, 0 up); dtheta0; dtheta1] from the shared estimator;
%   active = 1 while the Supervisor is in Stabilize (one period late), for integrator resets.
%   Use:  run_sim('ctrl_hybrid.slx', 'swingup', 'Plot', true)
arguments
    opts.Overwrite (1,1) logical = false
end
root = harness_setup();
folder = fullfile(root, 'controllers', 'hybrid');
names = {'ctrl_hybrid', 'stab_pid', 'stab_pole_placement', 'stab_lqr', 'swingup_energy'};
for k = 1:numel(names)
    f = fullfile(folder, [names{k} '.slx']);
    if isfile(f) && ~opts.Overwrite
        error('new_hybrid_controller:exists', '%s exists. Use ''Overwrite'', true to regenerate.', f);
    end
    if bdIsLoaded(names{k}), close_system(names{k}, 0); end
    if isfile(f), delete(f); end
end
addpath(folder);
make_stabilizer(folder, 'stab_pid', @fill_pid);
make_stabilizer(folder, 'stab_pole_placement', @(s) fill_state_feedback(s, 'ctrlP.pp.K', ...
    'Pole placement: u = -K (xh - [r 0 0 0]),  K = place(Ad, Bd, exp(poles Ts))'));
make_stabilizer(folder, 'stab_lqr', @(s) fill_state_feedback(s, 'ctrlP.lqr.K', ...
    'LQR: u = -K (xh - [r 0 0 0]),  K = dlqr(Ad, Bd, Q, R)'));
make_swingup(folder);
make_top(folder);
fprintf('Hybrid controller template created in %s\n', folder);
end

% ------------------------------------------------------------------------------------------------
function make_stabilizer(folder, name, fill)
new_system(name, 'Subsystem');
add_block('simulink/Sources/In1', [name '/xh'], 'PortDimensions', '4', 'Position', [30 60 60 74]);
add_block('simulink/Sources/In1', [name '/r'], 'Position', [30 140 60 154]);
add_block('simulink/Sources/In1', [name '/active'], 'Position', [30 220 60 234]);
add_block('simulink/Sinks/Out1', [name '/u'], 'Position', [620 133 650 147]);
fill(name);
save_system(name, fullfile(folder, [name '.slx']));
close_system(name, 0);
end

function fill_state_feedback(s, K, txt)
B = @(lib, n, pos, varargin) add_block(lib, [s '/' n], 'Position', pos, varargin{:});
B('simulink/Sources/Constant', 'zeros', [100 175 160 195], 'Value', 'zeros(3,1)');
B('simulink/Signal Routing/Mux', 'x_ref = [r 0 0 0]', [200 135 205 195], 'Inputs', '[1 3]');
B('simulink/Math Operations/Sum', 'error', [270 60 300 200], 'Inputs', '+-', 'IconShape', 'rectangular');
B('simulink/Math Operations/Gain', 'state feedback', [360 115 460 165], 'Gain', ['-' K], ...
    'Multiplication', 'Matrix(K*u)');
B('simulink/Sinks/Terminator', 'active unused', [100 220 120 240]);
add_line(s, 'r/1', 'x_ref = [r 0 0 0]/1', 'autorouting', 'smart');
add_line(s, 'zeros/1', 'x_ref = [r 0 0 0]/2', 'autorouting', 'smart');
add_line(s, 'xh/1', 'error/1', 'autorouting', 'smart');
add_line(s, 'x_ref = [r 0 0 0]/1', 'error/2', 'autorouting', 'smart');
add_line(s, 'error/1', 'state feedback/1', 'autorouting', 'smart');
add_line(s, 'state feedback/1', 'u/1', 'autorouting', 'smart');
add_line(s, 'active/1', 'active unused/1', 'autorouting', 'smart');
note(s, [txt sprintf('\nGains from ctrl_hybrid_design.m. Keep the interface: in xh, r, active; out u.')], [30 10]);
end

function fill_pid(s)
B = @(lib, n, pos, varargin) add_block(lib, [s '/' n], 'Position', pos, varargin{:});
B('simulink/Signal Routing/Demux', 'xh -> th0 th1 w0 w1', [100 40 105 120], 'Outputs', '4');
B('simulink/Discrete/Discrete PID Controller', 'PID on theta1', [250 50 340 110], ...
    'P', 'ctrlP.pid.Kp', 'I', 'ctrlP.pid.Ki', 'D', 'ctrlP.pid.Kd', 'N', 'ctrlP.pid.N', ...
    'SampleTime', 'hcfg.Ts', 'IntegratorMethod', 'Forward Euler', 'FilterMethod', 'Backward Euler', ...
    'ExternalReset', 'rising');
B('simulink/Math Operations/Sum', 'theta0 - r', [180 125 210 155], 'Inputs', '+-');
B('simulink/Math Operations/Gain', 'Kp0', [260 125 310 155], 'Gain', 'ctrlP.pid.Kp0');
B('simulink/Math Operations/Gain', 'Kd0', [260 175 310 205], 'Gain', 'ctrlP.pid.Kd0');
B('simulink/Math Operations/Sum', 'u = PID + arm PD', [500 105 530 175], 'Inputs', '+++', ...
    'IconShape', 'rectangular');
B('simulink/Sinks/Terminator', 'w1 unused', [180 95 200 110]);
L = @(a, b) add_line(s, a, b, 'autorouting', 'smart');
L('xh/1', 'xh -> th0 th1 w0 w1/1');
L('xh -> th0 th1 w0 w1/2', 'PID on theta1/1');
L('active/1', 'PID on theta1/2');                      % rising edge = entering Stabilize: reset
L('xh -> th0 th1 w0 w1/1', 'theta0 - r/1');
L('r/1', 'theta0 - r/2');
L('theta0 - r/1', 'Kp0/1');
L('xh -> th0 th1 w0 w1/3', 'Kd0/1');
L('xh -> th0 th1 w0 w1/4', 'w1 unused/1');
L('PID on theta1/1', 'u = PID + arm PD/1');
L('Kp0/1', 'u = PID + arm PD/2');
L('Kd0/1', 'u = PID + arm PD/3');
L('u = PID + arm PD/1', 'u/1');
note(s, sprintf(['PID: u = Kp e + Ki sum(e) Ts + Kd de/dt (filtered, N),  e = theta1  (+ arm PD: Kp0 (theta0 - r) + Kd0 dtheta0).\n' ...
    'The integrator resets when Stabilize is entered (active rises). Gains: ctrlP.pid in ctrl_hybrid_design.m.']), [30 0]);
end

function make_swingup(folder)
name = 'swingup_energy';
new_system(name, 'Subsystem');
add_block('simulink/Sources/In1', [name '/xh'], 'PortDimensions', '4', 'Position', [30 60 60 74]);
add_block('simulink/Sources/In1', [name '/r'], 'Position', [30 120 60 134]);
add_block('simulink/Sinks/Out1', [name '/u'], 'Position', [420 63 450 77]);
add_block('simulink/Sinks/Out1', [name '/E'], 'Position', [420 113 450 127]);
add_block('simulink/User-Defined Functions/MATLAB Function', [name '/energy law'], ...
    'Position', [150 50 330 140]);
mfb([name '/energy law'], sprintf([ ...
    'function [u, E] = energy_law(xh, r, ctrlP)\n' ...
    '%% TEMPLATE: swing-up owner writes the law here. Knobs: ctrlP.swing (ctrl_hybrid_design.m).\n' ...
    '%% Inputs: xh = [th0; th1 (0 up, wrapped); w0; w1], r = arm reference. Output u = duty.\n' ...
    '%% Plant constants for your law: ctrlP.plant (Jp, G, h, Ja, Beq, tcp, tcn, Rm, Kb, Vs).\n' ...
    '%% Pendulum energy in the report''s scaled units (eq. 89): 0 at rest upright, -2G hanging.\n' ...
    'm = ctrlP.plant;\n' ...
    'th1 = xh(2); w1 = xh(4);\n' ...
    'E = 0.5*m.Jp*w1^2 + m.G*(cos(th1) - 1);\n' ...
    'u = 0*ctrlP.swing.k*r;                          %% TODO your swing-up law\n']), {'ctrlP'});
add_line(name, 'xh/1', 'energy law/1', 'autorouting', 'smart');
add_line(name, 'r/1', 'energy law/2', 'autorouting', 'smart');
add_line(name, 'energy law/1', 'u/1', 'autorouting', 'smart');
add_line(name, 'energy law/2', 'E/1', 'autorouting', 'smart');
save_system(name, fullfile(folder, [name '.slx']));
close_system(name, 0);
end

% ------------------------------------------------------------------------------------------------
function make_top(folder)
% Layout: three areas left to right; signals that cross areas travel by Goto/From tags
% ([xh], [r], [active], [th0], [th1], [w0], [w1], [u_swing], [E], [u_stab], [mode]).
name = 'ctrl_hybrid';
new_system(name, 'Subsystem');
B = @(lib, n, pos, varargin) add_block(lib, [name '/' n], 'Position', pos, varargin{:});
H = @(b) get_param([name '/' b], 'PortHandles');
W = @(src, k, dst, j) add_line(name, H(src).Outport(k), H(dst).Inport(j), 'autorouting', 'smart');
tag = @(kind, n, tg, y, x) B(['simulink/Signal Routing/' kind], n, [x y - 9 x + 50 y + 9], ...
    'GotoTag', tg, 'ShowName', 'off');

area(name, 'Measurements -> state estimate (shared)', [10 10 330 470], [0.95 0.95 0.95]);
area(name, 'Control laws: one file per owner (Subsystem Reference)', [350 10 890 470], [1.0 0.95 0.85]);
area(name, 'Hybrid supervisor (Stateflow): picks the law, safety stop', [910 10 1370 470], [0.88 0.92 1.0]);

% --- estimate -------------------------------------------------------------------------------------
B('simulink/Sources/In1', 'y', [30 103 60 117], 'PortDimensions', '3', 'SampleTime', 'hcfg.Ts');
B('simulink/User-Defined Functions/MATLAB Function', 'estimator', [100 80 220 140]);
mfb([name '/estimator'], sprintf([ ...
    'function xh = estimator(y, ctrlP)\n' ...
    '%% xh = [theta0; theta1 wrapped to [-pi, pi) (0 = up); dtheta0; dtheta1]\n' ...
    '%% Velocities: finite differences low-pass filtered at ctrlP.est.vel_fc. Replace with an\n' ...
    '%% observer (e.g. Kalman) if you like; keep the output.\n' ...
    'persistent prev w\n' ...
    'th = [y(1); mod(y(2) + pi, 2*pi) - pi];\n' ...
    'if isempty(prev)\n' ...
    '    prev = th; w = [0; 0];\n' ...
    'end\n' ...
    'a = exp(-2*pi*ctrlP.est.vel_fc*ctrlP.Ts);\n' ...
    'd = th - prev;\n' ...
    'd(2) = mod(d(2) + pi, 2*pi) - pi;                %% no jump when theta1 wraps\n' ...
    'w = a*w + (1 - a)*d/ctrlP.Ts;\n' ...
    'prev = th;\n' ...
    'xh = [th; w];\n']), {'ctrlP'});
tag('Goto', 'xh goto', 'xh', 110, 250);
W('y', 1, 'estimator', 1);
W('estimator', 1, 'xh goto', 1);
tag('From', 'xh to split', 'xh', 230, 30);
B('simulink/Signal Routing/Demux', 'xh split', [110 190 115 270], 'Outputs', '4');
W('xh to split', 1, 'xh split', 1);
sig = {'th0', 'th1', 'w0', 'w1'};
for k = 1:4
    tag('Goto', [sig{k} ' goto'], sig{k}, 190 + 10*(2*k - 1), 160);
    W('xh split', k, [sig{k} ' goto'], 1);
end
B('simulink/Sources/In1', 'r', [30 313 60 327]);
tag('Goto', 'r goto', 'r', 320, 100);
W('r', 1, 'r goto', 1);
tag('From', 'mode to delay', 'mode', 410, 30);
B('simulink/Discrete/Unit Delay', 'mode (previous)', [100 395 140 425], 'SampleTime', 'hcfg.Ts');
B('simulink/Logic and Bit Operations/Compare To Constant', 'in Stabilize?', [170 395 230 425], ...
    'const', '2', 'relop', '==');
tag('Goto', 'active goto', 'active', 410, 260);
W('mode to delay', 1, 'mode (previous)', 1);
W('mode (previous)', 1, 'in Stabilize?', 1);
W('in Stabilize?', 1, 'active goto', 1);

% --- laws (each block: inputs at y0 + 70*[1 3 5]/6) -----------------------------------------------
laws = {'PID', 'stab_pid', 50; 'Pole placement', 'stab_pole_placement', 150; 'LQR', 'stab_lqr', 250};
for k = 1:3
    y0 = laws{k, 3};
    n = laws{k, 1};
    B('simulink/Ports & Subsystems/Subsystem Reference', n, [480 y0 580 y0 + 70], ...
        'ReferencedSubsystem', laws{k, 2});
    set_param([name '/' n], 'ContentPreviewEnabled', 'off');
    ins = {'xh', 'r', 'active'};
    for j = 1:3
        tag('From', sprintf('%s to %s', ins{j}, n), ins{j}, round(y0 + 70*(2*j - 1)/6), 390);
        W(sprintf('%s to %s', ins{j}, n), 1, n, j);
    end
end
B('simulink/Ports & Subsystems/Subsystem Reference', 'Swing-up', [480 360 580 430], ...
    'ReferencedSubsystem', 'swingup_energy');
set_param([name '/Swing-up'], 'ContentPreviewEnabled', 'off');
tag('From', 'xh to swing', 'xh', 378, 390);
tag('From', 'r to swing', 'r', 413, 390);
W('xh to swing', 1, 'Swing-up', 1);
W('r to swing', 1, 'Swing-up', 2);
tag('Goto', 'u_swing goto', 'u_swing', 378, 620);
tag('Goto', 'E goto', 'E', 413, 620);
W('Swing-up', 1, 'u_swing goto', 1);
W('Swing-up', 2, 'E goto', 1);
B('simulink/Sources/Constant', 'stabilizer choice (ctrlP.stab)', [610 56 680 76], 'Value', 'ctrlP.stab');
B('simulink/Signal Routing/Multiport Switch', 'select stabilizer', [700 30 720 320], 'Inputs', '3');
W('stabilizer choice (ctrlP.stab)', 1, 'select stabilizer', 1);
for k = 1:3
    W(laws{k, 1}, 1, 'select stabilizer', k + 1);
end
B('simulink/User-Defined Functions/MATLAB Function', 'friction compensation', [750 155 810 195]);
mfb([name '/friction compensation'], sprintf([ ...
    'function u = friction_comp(u0, ctrlP)\n' ...
    '%% Adds ctrlP.fric.gain x the identified Coulomb friction (report 5.7), smooth sign of u0.\n' ...
    's = tanh(u0/ctrlP.fric.band);\n' ...
    'u = u0 + ctrlP.fric.gain*(ctrlP.fric.duty(1)*max(s, 0) + ctrlP.fric.duty(2)*min(s, 0));\n']), {'ctrlP'});
W('select stabilizer', 1, 'friction compensation', 1);
tag('Goto', 'u_stab goto', 'u_stab', 175, 830);
W('friction compensation', 1, 'u_stab goto', 1);

% --- supervisor (chart inputs at 100 + 200*[1 3 5 7 9]/10) --------------------------------------
B('sflib/Chart', 'Supervisor', [1050 100 1210 300]);
build_chart([name '/Supervisor']);
chin = {'th0', 'th1', 'w1', 'u_stab', 'u_swing'};
for k = 1:5
    tag('From', [chin{k} ' to chart'], chin{k}, 100 + 20*(2*k - 1), 960);
    W([chin{k} ' to chart'], 1, 'Supervisor', k);
end
B('simulink/Sinks/Out1', 'duty', [1310 143 1340 157]);
B('simulink/Sinks/Out1', 'mode', [1310 243 1340 257]);
tag('Goto', 'mode goto', 'mode', 290, 1290);
W('Supervisor', 1, 'duty', 1);
W('Supervisor', 2, 'mode', 1);
W('Supervisor', 2, 'mode goto', 1);
B('simulink/Signal Routing/Mux', 'dbg = [w0 w1 E u_stab]', [1210 340 1215 440], 'Inputs', '4');
dbgin = {'w0', 'w1', 'E', 'u_stab'};
for k = 1:4
    tag('From', [dbgin{k} ' to dbg'], dbgin{k}, 340 + 25*k - 12.5, 1130);
    W([dbgin{k} ' to dbg'], 1, 'dbg = [w0 w1 E u_stab]', k);
end
B('simulink/Sinks/Out1', 'dbg', [1310 383 1340 397]);
W('dbg = [w0 w1 E u_stab]', 1, 'dbg', 1);
note(name, sprintf(['Hybrid controller (brief sec. 4.4). mode: 0 SafeStop, 1 SwingUp, 2 Stabilize. ' ...
    'Knobs: ctrlP from controllers/hybrid/ctrl_hybrid_design.m;\nstabilizer used in Stabilize: ctrlP.stab ' ...
    '(1 PID, 2 pole placement, 3 LQR). active = Stabilize one period ago (integrator resets).']), [10 490]);
save_system(name, fullfile(folder, [name '.slx']));
close_system(name, 0);
end

function build_chart(path)
% Supervisor: SwingUp <-> Stabilize, any -> SafeStop. Guards read ctrlP.sup (Parameter data).
% Execution order 1 on the SafeStop transitions: the safety check wins over everything else.
ch = sfroot().find('-isa', 'Stateflow.Chart', 'Path', path);
ch.ActionLanguage = 'MATLAB';
ins = {'th0', 'th1', 'w1', 'u_stab', 'u_swing'};
for k = 1:numel(ins)
    d = Stateflow.Data(ch); d.Name = ins{k}; d.Scope = 'Input'; d.Port = k;
end
outs = {'duty', 'mode'};
for k = 1:numel(outs)
    d = Stateflow.Data(ch); d.Name = outs{k}; d.Scope = 'Output'; d.Port = k; d.DataType = 'double';
end
d = Stateflow.Data(ch); d.Name = 'ctrlP'; d.Scope = 'Parameter';

swing = mkstate(ch, sprintf('SwingUp\nen, du:\nmode = 1;\nduty = u_swing;'), [40 150 200 110]);
stab = mkstate(ch, sprintf('Stabilize\nen, du:\nmode = 2;\nduty = u_stab;'), [640 150 200 110]);
stop = mkstate(ch, sprintf('SafeStop\nen, du:\nmode = 0;\nduty = 0;'), [340 430 200 100]);

capture = '[abs(th1) < ctrlP.sup.catch_th1 && abs(w1) < ctrlP.sup.catch_w1]';
safe = '[abs(th0) > ctrlP.sup.safe_th0]';
j = Stateflow.Junction(ch);
j.Position.Center = [440 70];
j.Position.Radius = 7;
t = Stateflow.Transition(ch);
t.Destination = j;
t.SourceEndpoint = [440 20];
t.MidPoint = [440 45];
tr(ch, j, stab, capture, 1, 3, 12, [470 40]);                   % start upright: Stabilize
tr(ch, j, swing, '', 2, 9, 12, []);                             % otherwise SwingUp
tr(ch, swing, stop, safe, 1, 6, 9, [60 340]);
tr(ch, swing, stab, capture, 2, 2, 10, [250 165]);
tr(ch, stab, stop, safe, 1, 6, 3, [640 340]);
tr(ch, stab, swing, '[abs(th1) > ctrlP.sup.exit_th1]', 2, 8, 4, [350 215]);
tr(ch, stop, swing, ['[ctrlP.sup.rearm && abs(th0) < ctrlP.sup.safe_th0/2 && ' ...
    'abs(th1) > pi - 0.2 && abs(w1) < 0.5]'], 1, 7, 7, [20 560]);
end

function s = mkstate(ch, label, pos)
s = Stateflow.State(ch);
s.LabelString = label;
s.Position = pos;
end

function t = tr(ch, a, b, label, order, srcClock, dstClock, labelXY)
t = Stateflow.Transition(ch);
t.Source = a;
t.Destination = b;
t.SourceOClock = srcClock;
t.DestinationOClock = dstClock;
t.LabelString = label;
t.ExecutionOrder = order;
if ~isempty(labelXY)
    lp = t.LabelPosition;
    t.LabelPosition = [labelXY lp(3:4)];
end
end
% ------------------------------------------------------------------------------------------------
function mfb(blk, code, params)
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for k = 1:numel(params)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

function area(sys, txt, pos, rgb)
h = add_block('built-in/Area', [sys '/' matlab.lang.makeValidName(strtok(txt, ':,(-'))], 'Position', pos);
set_param(h, 'Text', txt, 'BackgroundColor', mat2str(rgb), 'FontWeight', 'bold');
end

function note(sys, txt, xy)
a = Simulink.Annotation([sys '/' strtok(txt, ':.')]);
a.Text = txt;
a.Position = [xy xy + [700 40]];
a.HorizontalAlignment = 'left';
end
