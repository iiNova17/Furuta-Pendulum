% FURUTA_PARAMS_20261120  Furuta pendulum model re-identified on 2026-11-20, after the rig was moved.
% Same struct, fields and linear models as furuta_params.m (report Table 6, 2026-11-15), which stays the
% report's model and the harness default. Select this one with harness_config(..., 'Model', '20261120').
%
% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.
% State x = [theta0; theta1; dtheta0; dtheta1] in rad and rad/s; theta1 = 0 upright, pi hanging.
% Input: duty d in [-1, 1] (armature voltage V = d*Vs). Outputs: theta0, theta1, current i (A).
% Source: data/estimation/suite_20261120 (desktop/reidentify_variants.py, variant E), the claude_loop chain
% with the arm terms (Ja, Beq, tc+, tc-) also refined by simulation and a 10 ms pendulum timing correction.
% Validation (3 runs never fitted): arm angle 62.6 %, pendulum 78.8 %,
% arm speed 90.0 %, current 72.0 % (Table 6 on the same runs: 54.7 / 45.3 / 76.0 / 71.4 %).
% Not modelled: the shaft coupling (about 1-3 deg of play plus flex between motor encoder and arm; the
% pendulum reading lags the encoder by 9-15 ms). Treat it as a delay in the controller design.
p = struct();
p.Rm = 2.691593207;     % ohm, armature resistance incl. driver
p.Kb = 0.4427439143;    % V*s/rad, back-EMF constant at the arm shaft
p.Ja = 0.005488376463;  % A*s^2/rad, arm-side inertia
p.Jp = 0.0002800612978; % A*s^2/rad, pendulum inertia about its pivot
p.h = 0.0003111539202;  % A*s^2/rad, inertial coupling
p.G = 0.0437796245;    % A, gravity torque coefficient
p.Beq = 0.00542157176;  % A*s/rad, arm viscous friction
p.b1 = 0.0002736933118; % A*s/rad, pendulum viscous friction
p.tcp = 0.2797879156;   % A, arm Coulomb friction, dtheta0 > 0
p.tcn = 0.2783580302;   % A, arm Coulomb friction, dtheta0 < 0
p.tp = 0.0003147328153; % A, pendulum joint dry friction
p.Vs = 12;              % V, supply voltage
p.eps = 0.1;   % rad/s, arm friction smoothing tanh(w/eps)
p.eps1 = 0.05; % rad/s, pendulum friction smoothing
% Arm breakaway (static) friction: 6 % duty in both directions, the first pulse that moved the arm (median
% over arm positions +2 / -24 / +31 deg, runs b1_breakaway_low of 2026-11-20); 13 % / 14 % on 2026-11-07.
p.tsp = 0.2674995606;  % A, positive direction
p.tsn = 0.2674995606;  % A, negative direction
p.stick_band = 0.05;     % rad/s, the arm can stick inside |dtheta0| < stick_band
p.stick_T = 0.002;       % s, time constant that brings a stuck arm to rest
p.Larm = 0.06969857842;  % m, arm length from h/G
p.pend_delay = 0.010;   % s, pendulum timing correction used in the identification (15 Nov: 7 ms)
x0_hanging = [0; pi; 0; 0];
x0_upright = [0; 0; 0; 0];

% Linear models (input V in volts; *_duty versions take the duty cycle)
A_up = [0 0 1 0; 0 0 0 1; 0 -9.458121318 -33.03978285 0.05912852329; 0 166.8297491 36.70788516 -1.042955189];
B_up = [0; 0; 72.24391026; -80.2644852];  % per volt
Bd_up = [0; 0; 866.9269232; -963.1738224];  % per unit duty
C_up = [1 0 0 0; 0 1 0 0; 0 0 -0.1644913924 0];
D_up = [0; 0; 0.3715271674];
sys_up = ss(A_up, B_up, C_up, D_up, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_up_duty = ss(A_up, Bd_up, C_up, D_up*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_up = minreal(tf(sys_up));
A_down = [0 0 1 0; 0 0 0 1; 0 -9.458121318 -33.03978285 -0.05912852329; 0 -166.8297491 -36.70788516 -1.042955189];
B_down = [0; 0; 72.24391026; 80.2644852];  % per volt
Bd_down = [0; 0; 866.9269232; 963.1738224];  % per unit duty
C_down = [1 0 0 0; 0 1 0 0; 0 0 -0.1644913924 0];
D_down = [0; 0; 0.3715271674];
sys_down = ss(A_down, B_down, C_down, D_down, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_down_duty = ss(A_down, Bd_down, C_down, D_down*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_down = minreal(tf(sys_down));
% Nonlinear model: furuta_rhs.m (the same function works with both parameter sets), e.g.
%   ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], x0_hanging)
