%% LQR gains for the Furuta pendulum.  duty = -K*x,  x = [theta0; theta1; w0; w1]
cfg = harness_config();                                   % run harness_setup once first

% 1. model at 4 ms
sys = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), cfg.Ts);

% 2. how much you allow (change these)
max_x    = [deg2rad(30), deg2rad(5), 20, 30];             % theta0, theta1 [rad], w0, w1 [rad/s]
max_duty = 0.15;

% 3. weights and gain
Q = diag(1./max_x.^2);
R = 1/max_duty^2;
K = dlqr(sys.A, sys.B, Q, R)

% 4. stable if all < 1
abs(eig(sys.A - sys.B*K))'
