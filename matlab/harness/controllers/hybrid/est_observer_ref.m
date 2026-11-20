function [xh, S] = est_observer_ref(S, y, duty, P)
%EST_OBSERVER_REF One tick of est_observer.slx in plain MATLAB (reference for the block test and the firmware port).
%   S = [];  then every controller tick:  [xh, S] = est_observer_ref(S, y, duty, P)
%   y = [theta0; theta1; i] measured this tick, duty = the duty commanded THIS tick (the block delays it one tick,
%   so the model is stepped with the duty that actually acted), P = ctrlP (P.Ts, P.plant, P.est).
%   Returns xh = [theta0; theta1 (wrapped, 0 up); w0; w1] for the controller, the state at "now".
%
%   Model-assisted observer (Design Board part 2, level 5; study 2026-11-18, est_study.m 'MODEL'):
%   the pendulum reading is P.est.nd ticks old. xd = estimate at the moment of that reading:
%     predict : step xd one tick with the model (furuta_rhs, P.est.sub RK4 substeps) and the duty that acted then
%     compare : miss = [arm reading from nd ticks ago; pendulum reading] - prediction (pendulum miss wrapped)
%     correct : xd = prediction + L*miss          (L = alpha-beta gains, P.est.L, 4x2)
%     roll    : copy of xd stepped through the nd newer duties to "now"  -> xh
%   Start-up: xd starts from the first measurement (speeds 0); the arm miss is ignored for the first nd ticks
%   (the arm reading "nd ticks ago" does not exist yet).
%   P.est.kind = 2: level 3 fallback (alpha-beta tracking loop, theta1 pushed forward by the lag) instead.
nd = P.est.nd; Ts = P.Ts; p = P.plant;
th0 = y(1); th1 = wrapf(y(2));
if isempty(S)
    S.u_prev = 0;                        % duty unit delay (IC 0)
    S.uh = zeros(nd + 1, 1);             % duty history, oldest first (IC 0)
    S.th0b = zeros(nd, 1);               % theta0 nd ago: delay line (IC 0), oldest first
    S.xd = [0; pi; 0; 0];                % xd memory (IC)
    S.started = false; S.n = 0;          % predict / compare start-up
    S.trk = [];                          % fallback state
end
% duty that acted during the last tick, saturated, into the history
u_now = min(max(S.u_prev, -0.5), 0.5);
S.uh = [S.uh(2:end); u_now];
S.u_prev = duty;
% arm reading nd ticks ago (delay line output before it takes this tick's reading)
th0_then = S.th0b(1);
S.th0b = [S.th0b(2:end); th0];
% predict
if ~S.started
    S.started = true;
    xp = [th0; th1; 0; 0];
else
    xp = S.xd; xp(2) = wrapf(xp(2));
    for q = 1:P.est.sub, xp = rk4(xp, S.uh(1), p, Ts/P.est.sub); end
end
% compare
S.n = S.n + 1;
r = [th0_then - xp(1); wrapf(th1 - xp(2))];
if S.n <= nd, r(1) = 0; end
% correct
xd = xp + P.est.L*r;
S.xd = xd;
% roll forward
x = xd;
for j = 2:nd + 1
    for q = 1:P.est.sub, x = rk4(x, S.uh(j), p, Ts/P.est.sub); end
end
% wrap + NaN guard
x(2) = wrapf(x(2));
if any(~isfinite(x)), x = [th0; th1; 0; 0]; end
xh_model = x;
% level 3 fallback (always updated, so switching kind needs no restart)
[xh_trk, S.trk] = fallback_step(S.trk, th0, th1, P);
if P.est.kind == 2, xh = xh_trk; else, xh = xh_model; end
end

function [xh, T] = fallback_step(T, th0, th1, P)
lam = exp(-2*pi*P.est.f_trk*P.Ts); al = 1 - lam^2; be = (1 - lam)^2;
if isempty(T), T.th = [th0; th1]; T.w = [0; 0]; end
thp = T.th + P.Ts*T.w;
r = [th0; th1] - thp; r(2) = wrapf(r(2));
T.th = thp + al*r; T.th(2) = wrapf(T.th(2));
T.w = T.w + be/P.Ts*r;
xh = [T.th(1); wrapf(T.th(2) + T.w(2)*P.est.nd*P.Ts); T.w];
end

function x = rk4(x, d, p, h)
k1 = furuta_rhs(x, d, p); k2 = furuta_rhs(x + h/2*k1, d, p);
k3 = furuta_rhs(x + h/2*k2, d, p); k4 = furuta_rhs(x + h*k3, d, p);
x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function a = wrapf(a)
a = mod(a + pi, 2*pi) - pi;
end
