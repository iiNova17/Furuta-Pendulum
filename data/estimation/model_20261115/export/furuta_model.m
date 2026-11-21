% Furuta pendulum model identified with Furuta Lab.
% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.
% State x = [theta0; theta1; dtheta0; dtheta1]; theta1 = 0 upright. Input V (volts) or duty (B*Vs).
p = struct();
p.Rm = 3.425409421;
p.Kb = 0.4813691365;
p.Ja = 0.005634348234;
p.Jp = 0.0001853092537;
p.h = 0.0002012270497;
p.G = 0.02898730616;
p.Beq = 0.01612225308;
p.b1 = 0.0001208246778;
p.tcp = 0.2581443496;
p.tcn = 0.2568942722;
p.tp = 0.0006378819055;
p.Vs = 12;
p.eps = 0.1;  % arm friction smoothing (rad/s) used by furuta_rhs
p.eps1 = 0.05;  % pendulum friction smoothing (rad/s)
p.tsp = 0.455420012;  % arm breakaway friction, positive (A): measured dead zone
p.tsn = 0.4904523207;  % arm breakaway friction, negative (A)
p.stick_band = 0.05;  % rad/s, the arm can stick inside |dtheta0| < stick_band
p.stick_T = 0.002;  % s, time constant that brings a stuck arm to rest
p.Larm = 0.06807680699;  % m, from the data
p.Ktau = 0.4813691365;  % N*m/A (nominal scale); SI value = per-Ktau value * Ktau
p.eta_g = 1;
Ts = 0.001;
A_up = [0 0 1 0; 0 0 0 1; 0 -5.812080219 -28.92465458 0.02422587031; 0 162.7379816 31.40924044 -0.6783232662];
B_up = [0; 0; 53.90412509; -58.53441123];  % per volt
Bd_up = [0; 0; 646.8495011; -702.4129348];  % per unit duty
C_up = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_up = [0; 0; 0.2919359052];
sys_up = ss(A_up, B_up, C_up, D_up, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_up_duty = ss(A_up, Bd_up, C_up, D_up*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_up = minreal(tf(sys_up));
sysd_up = c2d(sys_up, Ts, 'zoh');
A_down = [0 0 1 0; 0 0 0 1; 0 -5.812080219 -28.92465458 -0.02422587031; 0 -162.7379816 -31.40924044 -0.6783232662];
B_down = [0; 0; 53.90412509; 58.53441123];  % per volt
Bd_down = [0; 0; 646.8495011; 702.4129348];  % per unit duty
C_down = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_down = [0; 0; 0.2919359052];
sys_down = ss(A_down, B_down, C_down, D_down, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_down_duty = ss(A_down, Bd_down, C_down, D_down*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_down = minreal(tf(sys_down));
sysd_down = c2d(sys_down, Ts, 'zoh');
% Nonlinear model: see furuta_rhs.m, e.g. ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], [0; pi; 0; 0])
