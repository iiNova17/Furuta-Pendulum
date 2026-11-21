% Furuta pendulum model identified with Furuta Lab.
% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.
% State x = [theta0; theta1; dtheta0; dtheta1]; theta1 = 0 upright. Input V (volts) or duty (B*Vs).
p = struct();
p.Rm = 3.425409421;
p.Kb = 0.4813691365;
p.Ja = 0.005634348708;
p.Jp = 0.0001853004251;
p.h = 0.0001966758515;
p.G = 0.02898004761;
p.Beq = 0.01612224809;
p.b1 = 0.0001085136941;
p.tcp = 0.258144365;
p.tcn = 0.2568942873;
p.tp = 0.0005086190454;
p.Vs = 12;
p.eps = 0.1;  % arm friction smoothing (rad/s) used by furuta_rhs
p.eps1 = 0.05;  % pendulum friction smoothing (rad/s)
p.Larm = 0.06655376363;  % m, from the data
p.Ktau = 0.4813691365;  % N*m/A (nominal scale); SI value = per-Ktau value * Ktau
p.eta_g = 1;
Ts = 0.001;
A_up = [0 0 1 0; 0 0 0 1; 0 -5.669255336 -28.87260553 0.02122811693; 0 162.412219 30.64506881 -0.608140818];
B_up = [0; 0; 53.80712796; -57.11029913];  % per volt
Bd_up = [0; 0; 645.6855355; -685.3235895];  % per unit duty
C_up = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_up = [0; 0; 0.2919359052];
sys_up = ss(A_up, B_up, C_up, D_up, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_up_duty = ss(A_up, Bd_up, C_up, D_up*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_up = minreal(tf(sys_up));
sysd_up = c2d(sys_up, Ts, 'zoh');
A_down = [0 0 1 0; 0 0 0 1; 0 -5.669255336 -28.87260553 -0.02122811693; 0 -162.412219 -30.64506881 -0.608140818];
B_down = [0; 0; 53.80712796; 57.11029913];  % per volt
Bd_down = [0; 0; 645.6855355; 685.3235895];  % per unit duty
C_down = [1 0 0 0; 0 1 0 0; 0 0 -0.1405289346 0];
D_down = [0; 0; 0.2919359052];
sys_down = ss(A_down, B_down, C_down, D_down, 'InputName', 'V', 'OutputName', {'theta0','theta1','i'});
sys_down_duty = ss(A_down, Bd_down, C_down, D_down*p.Vs, 'InputName', 'duty', 'OutputName', {'theta0','theta1','i'});
G_down = minreal(tf(sys_down));
sysd_down = c2d(sys_down, Ts, 'zoh');
% Nonlinear model: see furuta_rhs.m, e.g. ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], [0; pi; 0; 0])
