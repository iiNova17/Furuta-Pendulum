% FURUTA_PARAMS  Final identified Furuta pendulum model (report section 6).
% Run this script before simulating: it creates the parameter struct p used by
% furuta_rhs.m (nonlinear model) and the linear models about both equilibria.
%
% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.
% State x = [theta0; theta1; dtheta0; dtheta1] in rad and rad/s; theta1 = 0 upright, pi hanging.
% Input: duty d in [-1, 1] (armature voltage V = d*Vs). Outputs: theta0, theta1, current i (A).
% Source: data/estimation/claude_loop (refined h, b1, tp), identified 2026-11-15.
p = struct();
p.Rm = 3.425409421;     % ohm, armature resistance incl. driver
p.Kb = 0.4813691365;    % V*s/rad, back-EMF constant at the arm shaft
p.Ja = 0.005634348234;  % A*s^2/rad, arm-side inertia
p.Jp = 0.0001853092537; % A*s^2/rad, pendulum inertia about its pivot
p.h = 0.0002012270497;  % A*s^2/rad, inertial coupling
p.G = 0.02898730616;    % A, gravity torque coefficient
p.Beq = 0.01612225308;  % A*s/rad, arm viscous friction
p.b1 = 0.0001208246778; % A*s/rad, pendulum viscous friction
p.tcp = 0.2581443496;   % A, arm Coulomb friction, dtheta0 > 0
p.tcn = 0.2568942722;   % A, arm Coulomb friction, dtheta0 < 0
p.tp = 0.0006378819055; % A, pendulum joint dry friction
p.Vs = 12;              % V, supply voltage
p.eps = 0.1;   % rad/s, arm friction smoothing tanh(w/eps)
p.eps1 = 0.05; % rad/s, pendulum friction smoothing
p.Larm = 0.06807680699;  % m, arm length from h/G
x0_hanging = [0; pi; 0; 0];
x0_upright = [0; 0; 0; 0];

% Linear models (input V in volts; *_duty versions take the duty cycle)
A_up = [0 0 1 0; 0 0 0 1; 0 -5.812080219 -28.92465458 0.02422587031; 0 162.7379816 31.40924044 -0.6783232662];
B_up = [0; 0; 53.90412509; -58.53441123];  % per volt
Bd_up = [0; 0; 646.8495011; -702.4129348];  % per unit duty
C_up = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_up = [0; 0; 0.2919359052];
sys_up = ss(A_up, B_up, C_up, D_up, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_up_duty = ss(A_up, Bd_up, C_up, D_up*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_up = minreal(tf(sys_up));
A_down = [0 0 1 0; 0 0 0 1; 0 -5.812080219 -28.92465458 -0.02422587031; 0 -162.7379816 -31.40924044 -0.6783232662];
B_down = [0; 0; 53.90412509; 58.53441123];  % per volt
Bd_down = [0; 0; 646.8495011; 702.4129348];  % per unit duty
C_down = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_down = [0; 0; 0.2919359052];
sys_down = ss(A_down, B_down, C_down, D_down, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_down_duty = ss(A_down, Bd_down, C_down, D_down*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_down = minreal(tf(sys_down));
% Nonlinear model: see furuta_rhs.m and README.md, e.g.
%   ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], x0_hanging)
