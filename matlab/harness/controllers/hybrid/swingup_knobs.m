%% Swing-up knobs from physics.  Law (swingup_energy.slx / energy law):
%   pump  a0 = -a_push * sat(k_E (E0 - E)) * tanh(w1 cos(theta1) / w_sign)    push the pivot against the bob
%   kick  a0 = a_kick while the pendulum hangs still (E < -1 + kick_E)          start from rest
%   arm   a0 = a0 - wc^2 (theta0 - r) - 2 zc wc w0                              soft spring to centre
%   motor i = Ja a0 + Beq w0 + friction(w0), |i| <= i_max;  duty = (Rm i + Kb w0) / Vs
%         -> the law commands CURRENT: back-EMF is compensated, so reversing never spikes the current
% Energy E in units of 2G: -1 hanging at rest, 0 = just reaches the top.
cfg = harness_config();                                   % run harness_setup once first
p = cfg.p;
wn = sqrt(p.G/p.Jp);                                      % pendulum natural frequency [rad/s]

% 1. energy target: what friction takes over the last quarter swing (horizontal -> top),
%    where cos(theta1) -> 0 and the pivot can no longer push. Dry: tp*pi/2. Viscous: b1 * int w1 dtheta
%    with w1 = 2 wn cos(phi/2) (E = 0 swing): int_{pi/2}^{pi} = 4 wn (1 - sin(pi/4)).
E0 = (p.tp*pi/2 + p.b1*4*wn*(1 - sin(pi/4)))/(2*p.G)

% 2. push size from the budgets (change these)
i_max = 1.0;                                              % [A] current command limit: 1/3 below the 1.5 A trip
u_max = 0.45;                                             % duty limit: below the firmware's 0.5 clip
Tq = pi/(2*wn);                                           % one push lasts ~ a quarter period
a_cur = (i_max - p.tcp)/p.Ja                              % current budget: Ja a + Coulomb <= i_max
a_duty = (u_max*p.Vs - p.Rm*p.tcp)/(p.Rm*p.Ja + (p.Rm*p.Beq + p.Kb)*Tq)  % duty budget, arm speed a*Tq
a_push = min(a_cur, a_duty)

% 3. how sharply the push fades near the target: full push until within one E0 of it
k_E = 1/E0

% 4. arm centring: ~3x slower than the pendulum so the two motions do not fight, well damped
wc = wn/3
zc = 0.7

% 5. kick from rest: the motor current must beat the arm's static friction with 50 % margin;
%    it stops once the pendulum swings ~5 deg (energy (1 - cos 5deg)/2)
a_kick = 1.5*max(p.tcp, p.tcn)/p.Ja
kick_E = (1 - cosd(5))/2

% 6. smoothing: a few counts of velocity noise (AS5600 4096 cpr, encoder 1496 cpr at 4 ms)
w_sign = 1.0;                                             % [rad/s] sign of w1 cos(theta1)
w_fric = 1.0;                                             % [rad/s] sign of w0 for the friction term

fprintf(['\nD.swing = struct(''E0'', %.3f, ''k_E'', %.1f, ''a_push'', %.0f, ''wc'', %.2f, ''zc'', %.1f, ...\n' ...
    '    ''a_kick'', %.0f, ''kick_E'', %.4f, ''w_sign'', %.1f, ''w_fric'', %.1f, ''i_max'', %.2f, ''u_max'', %.2f);\n'], ...
    E0, k_E, a_push, wc, zc, a_kick, kick_E, w_sign, w_fric, i_max, u_max);
