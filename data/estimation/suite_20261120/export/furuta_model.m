% Furuta pendulum model identified with Furuta Lab.
% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.
% State x = [theta0; theta1; dtheta0; dtheta1]; theta1 = 0 upright. Input V (volts) or duty (B*Vs).
p = struct();
p.Rm = 2.691593207;
p.Kb = 0.4427439143;
p.Ja = 0.005488376463;
p.Jp = 0.0002800612978;
p.h = 0.0003111539202;
p.G = 0.0437796245;
p.Beq = 0.00542157176;
p.b1 = 0.0002736933118;
p.tcp = 0.2797879156;
p.tcn = 0.2783580302;
p.tp = 0.0003147328153;
p.Vs = 12;
p.eps = 0.1;  % arm friction smoothing (rad/s) used by furuta_rhs
p.eps1 = 0.05;  % pendulum friction smoothing (rad/s)
p.tsp = 0.2674995606;  % arm breakaway friction, positive (A): measured dead zone
p.tsn = 0.2674995606;  % arm breakaway friction, negative (A)
p.stick_band = 0.05;  % rad/s, the arm can stick inside |dtheta0| < stick_band
p.stick_T = 0.002;  % s, time constant that brings a stuck arm to rest
p.Larm = 0.06969857842;  % m, from the data
Ts = 0.001;
A_up = [0 0 1 0; 0 0 0 1; 0 -9.458121318 -33.03978285 0.05912852329; 0 166.8297491 36.70788516 -1.042955189];
B_up = [0; 0; 72.24391026; -80.2644852];  % per volt
Bd_up = [0; 0; 866.9269232; -963.1738224];  % per unit duty
C_up = [1 0 0 0; 0 1 0 0; 0 0 -0.1644913924 0];
D_up = [0; 0; 0.3715271674];
sys_up = ss(A_up, B_up, C_up, D_up, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_up_duty = ss(A_up, Bd_up, C_up, D_up*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_up = minreal(tf(sys_up));
sysd_up = c2d(sys_up, Ts, 'zoh');
A_down = [0 0 1 0; 0 0 0 1; 0 -9.458121318 -33.03978285 -0.05912852329; 0 -166.8297491 -36.70788516 -1.042955189];
B_down = [0; 0; 72.24391026; 80.2644852];  % per volt
Bd_down = [0; 0; 866.9269232; 963.1738224];  % per unit duty
C_down = [1 0 0 0; 0 1 0 0; 0 0 -0.1644913924 0];
D_down = [0; 0; 0.3715271674];
sys_down = ss(A_down, B_down, C_down, D_down, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_down_duty = ss(A_down, Bd_down, C_down, D_down*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_down = minreal(tf(sys_down));
sysd_down = c2d(sys_down, Ts, 'zoh');
% Nonlinear model: see furuta_rhs.m, e.g. ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], [0; pi; 0; 0])
