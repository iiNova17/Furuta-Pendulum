% Streams a simulated open-loop run into the Unreal twin. Start the twin first (twin/run_twin.ps1, see README.md).
% Plant: the identified model (matlab/model/furuta_rhs.m with furuta_params_20261120.m). Input: a bang-bang
% duty at the pendulum's natural frequency, so the pendulum swings up and over (open loop, no controller).
% Conventions: furuta_rhs uses theta1 = 0 upright; the twin packet uses q_pend = 0 hanging -> q_pend = theta1 - pi.
here = fileparts(mfilename('fullpath'));
model = fullfile(here, '..', '..', 'matlab', 'model');
addpath(model);
run(fullfile(model, 'furuta_params_20261120.m'));                  % p, x0_hanging
w0 = 12.5;                                                         % hanging natural frequency [rad/s]
ufun = @(t) 0.5*sign(sin(w0*t)).*(t < 2.5);
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', 2e-3);
[t, X] = ode15s(@(t, x) furuta_rhs(x, ufun(t), p), [0 8], x0_hanging, opts);
q_arm = X(:, 1);
q_pend = X(:, 2) - pi;
fprintf('max |q_pend| %.0f deg, max |q_arm| %.0f deg
', max(abs(q_pend))*180/pi, max(abs(q_arm))*180/pi);
twin_play(t, q_arm, q_pend);
