function W = lqr_workbench(max_x, max_u, varargin)
%LQR_WORKBENCH Design an LQR from what you can tolerate (Bryson's rule) and test it in one go.
%   W = lqr_workbench(max_x, max_u)
%   W = lqr_workbench(max_x, max_u, 'Seeds', 20, 'Scenarios', {'balance', 'push'}, 'Plot', true)
%
%   max_x  largest acceptable [theta0 theta1 w0 w1] while balancing, in [deg deg rad/s rad/s]
%   max_u  largest acceptable duty (keep <= 0.4: headroom below the firmware's 0.5 clip)
%
%   Bryson's rule: Q = diag(1./max_x.^2) (angles in rad), R = 1/max_u^2.
%   Prints K, the closed-loop poles, the loop margins at the duty (with perfect state), then runs
%   the harness scenarios in realistic mode, nominal plant and 'Seeds' perturbed plants (k = 2).
%   Nothing is written to ctrl_hybrid_design.m: it prints the two lines to paste when you're happy.
%
%   Example:  W = lqr_workbench([30 3 5 5], 0.4);
p = inputParser;
p.addParameter('Seeds', 0);
p.addParameter('Scenarios', {'balance', 'push'});
p.addParameter('Plot', false);
p.parse(varargin{:});
o = p.Results;

cfg = harness_config();
mx = max_x(:)';  mx(1:2) = deg2rad(mx(1:2));
Q = diag(1./mx.^2);
R = 1/max_u^2;
sysd = c2d(ss(cfg.lin.A_up, cfg.lin.Bd_up, eye(4), 0), cfg.Ts);
[K, ~, ev] = dlqr(sysd.A, sysd.B, Q, R);
s = log(ev)/cfg.Ts;                                   % closed-loop poles in rad/s

L = ss(sysd.A, sysd.B, K, 0, cfg.Ts);                 % loop broken at the duty, perfect state
[Gm, Pm, ~, Wcp] = margin(L);
dm = (Pm*pi/180)/Wcp;                                 % delay margin [s]

fprintf('\n=== LQR from Bryson: max_x = [%g deg, %g deg, %g rad/s, %g rad/s], max_u = %g ===\n', max_x, max_u);
fprintf('K = [%s]   (duty = -K*(x - [r;0;0;0]))\n', num2str(K, ' %.4g'));
fprintf('closed-loop poles [rad/s]: %s\n', num2str(s.', ' %.3g'));
fprintf('open-loop unstable pole: %+.3g rad/s -> fastest closed-loop pole should be well beyond it\n', ...
    max(real(eig(cfg.lin.A_up))));
fprintf('margins at the duty (perfect state): gain %.1f dB, phase %.0f deg, delay margin %.1f ms\n', ...
    20*log10(Gm), Pm, 1000*dm);
fprintf('  rig lag to cover: ~7 ms AS5600 + one %.0f ms tick -> want the delay margin well above ~11 ms\n', ...
    1000*cfg.Ts);
fprintf(['  current ~ (12*duty - %.2f*w0)/%.2f A; the firmware trips at %.1f A (10 ms): reversing hard ' ...
    'while the arm moves trips it\n'], cfg.p.Kb, cfg.p.Rm, cfg.act.i_trip);
fprintf('  duty if the state sat at max_x on every axis at once: %.2f (each axis alone: %s)\n', ...
    sum(abs(K.*mx)), num2str(abs(K.*mx), ' %.2f'));

Ptry = struct('lqr', struct('Q', Q, 'R', R), 'stab', 3);
rows = {};
w = warning('off', 'all');
cleanup = onCleanup(@() warning(w));
for sc = o.Scenarios
    Rn = run_sim('ctrl_hybrid.slx', sc{1}, 'Params', Ptry, 'Plot', o.Plot);
    rows(end+1, :) = {sc{1}, 'nominal', Rn.pass, Rn.metrics.th1_max, Rn.metrics.u_peak, ...
        max(abs(Rn.i)), Rn.metrics.th1_tsettle, Rn.fault_name}; %#ok<AGROW>
    W.runs.(sc{1}) = Rn;
    if o.Seeds > 0
        ok = 0; up = 0; ip = 0; nf = 0;
        for k = 1:o.Seeds
            Rk = run_sim('ctrl_hybrid.slx', sc{1}, 'Params', Ptry, 'Perturb', 2, 'Seed', k);
            ok = ok + Rk.pass;  up = max(up, Rk.metrics.u_peak);  ip = max(ip, max(abs(Rk.i)));
            nf = nf + (Rk.fault ~= 0);
        end
        rows(end+1, :) = {sc{1}, sprintf('Perturb 2 x %d', o.Seeds), ok, NaN, up, ip, NaN, ...
            sprintf('%d faults', nf)}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'scenario', 'plant', 'pass', 'th1_max_deg', 'u_peak', ...
    'i_peak_A', 'th1_settle_s', 'fault'});
disp(T)
fprintf(['pass criteria: balance = |th1| < 3 deg after 2 s; push = |th1| < 3 deg after 4 s ' ...
    '(see furuta_scenario.m)\n']);
fprintf('\nWhen you are happy, put this in ctrl_hybrid_design.m (LQR owner section):\n');
fprintf('D.lqr.Q = diag(1./[%s].^2);   %% Bryson: max theta0, theta1 [rad], w0, w1 [rad/s]\n', ...
    num2str(mx, ' %.4g'));
fprintf('D.lqr.R = 1/%g^2;                  %% Bryson: max duty\n\n', max_u);

W.Q = Q; W.R = R; W.K = K; W.poles = s; W.Gm = Gm; W.Pm = Pm; W.delay_margin = dm; W.table = T;
end
