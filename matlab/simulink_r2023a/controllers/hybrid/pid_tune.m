function S = pid_tune(opts)
%PID_TUNE Local search of the PID stabilizer's gains around the values in ctrl_hybrid_design.m.
%   S = pid_tune()                     search, then characterize start / tuned / LQR, save pid_tune_results.mat
%   S = pid_tune('Steps', [1.1 1.05])  other step sizes (multiplicative, coarse to fine)
%
%   Structure (stab_pid.slx): u = Kp1 th1 + Kd1 w1 + Kp0 (th0 - r) + Ki0 sum(th0 - r) Ts + Kd0 w0.
%   Cost of a gain set:
%     J = (ITAE_push(10 ms) + ITAE_push(15 ms))/2 + w_arm * A + (1 for each lag at which the push test fails)
%   ITAE of the push test (pendulum after the tap at 2 s) at the nominal pendulum sensor lag (10 ms) and the
%   rig's worst case (15 ms); push pass = |th1| < 3 deg after 4 s. A = mean |th0 - r| [rad] in the arm step
%   over 2-4 s and 5-7 s (1 s after each reference step), weighted by w_arm (WArm, default 0.25 so that a 20 deg
%   arm error counts about as much as the pendulum ITAE). The arm step's own 5 deg criterion is not required:
%   with friction compensation off (D.fric.gain 0) the arm sticks and slips, and even the LQR misses it.
%   A step is accepted only if it lowers J and the candidate also keeps linear stability margins (loop broken at
%   the duty, perfect state, no lag: phase margin >= MinPM and delay margin >= MinDM; a first search without them
%   reached a 17.5 deg phase margin and a 7.6 ms delay margin, too little for the unmodeled play and flex of the
%   rig), passes the swing-up at both lags, keeps the pendulum within 20 deg in the arm step, never trips the
%   firmware and keeps the duty at its limit for less than 5 % of the push tests.
%   Coordinate search: each gain in turn is multiplied by s and 1/s; the first improving change is kept;
%   passes repeat until no gain improves, then s shrinks (Steps).
%   Afterwards: one-at-a-time sensitivity around the result, Monte Carlo push tests on perturbed plants,
%   and the same scenarios for the starting gains, the tuned gains and the LQR. Results in the .mat file.
arguments
    opts.Steps (1,:) double = [1.25 1.1 1.05]
    opts.MaxPasses (1,1) double = 3
    opts.File (1,:) char = fullfile(fileparts(mfilename('fullpath')), 'pid_tune_results.mat')
    opts.MonteCarlo (1,1) double = 10
    opts.WArm (1,1) double = 0.25
    opts.MinPM (1,1) double = 45        % deg
    opts.MinDM (1,1) double = 0.020     % s
end
w = warning('off', 'all'); cw = onCleanup(@() warning(w));
cfg10 = harness_config();                                 % nominal: 10 ms pendulum sensor lag
cfg15 = cfg10; cfg15.sens.pend_delay = 0.015;             % rig worst case (9-15 ms measured)
D = ctrl_hybrid_design(cfg10);
names = {'Kp1', 'Kd1', 'Kp0', 'Ki0', 'Kd0'};
g0 = rmfield(D.pid, setdiff(fieldnames(D.pid), names));
S.start = g0; S.names = names; S.warm = opts.WArm; S.minpm = opts.MinPM; S.mindm = opts.MinDM;
S.hist = struct('g', {}, 'J', {}, 'itae10', {}, 'itae15', {}, 'pass10', {}, 'pass15', {}, 'arm', {}, 'sat', {}, ...
    'pm', {}, 'dm', {}, 'feasible', {}, 'accepted', {});
Ts = cfg10.Ts; z = tf('z', Ts); Pd = c2d(ss(cfg10.lin.A_up, cfg10.lin.Bd_up, eye(4), 0), Ts);
t0 = tic;

[J, e] = cost(g0); best = g0; Jb = J; addhist(g0, J, e, true, true);
fprintf('start: J %.4f (ITAE %.4f / %.4f, push pass %d / %d, arm error %.1f deg)\n', J, e.itae10, e.itae15, e.pass10, e.pass15, rad2deg(e.arm));
for s = opts.Steps
    for pass = 1:opts.MaxPasses
        improved = false;
        for n = names
            for f = [s, 1/s]
                g = best; g.(n{1}) = best.(n{1})*f;
                [J, e] = cost(g);
                ok = false;
                if J < Jb - 1e-4, ok = feasible(g, e); end
                addhist(g, J, e, ok, ok);
                tag = ''; if ok, tag = '  accepted'; end
                fprintf('[%5.0f s] step %.2f pass %d  %s x%.3f -> J %.4f%s\n', toc(t0), s, pass, n{1}, f, J, tag);
                if ok, best = g; Jb = J; improved = true; break, end
            end
        end
        if ~improved, break, end
    end
end
S.tuned = best;
save(opts.File, 'S');                                     % the search result, before the longer characterization
fprintf('tuned: %s  J %.4f (start %.4f)\n', gstr(best), Jb, S.hist(1).J);

% one-at-a-time sensitivity around the tuned gains (nominal lag)
S.sens.mult = [0.6 0.8 1 1.25 1.6];
for i = 1:numel(names)
    for j = 1:numel(S.sens.mult)
        g = best; g.(names{i}) = best.(names{i})*S.sens.mult(j);
        R = sim1(g, 'push', cfg10);
        S.sens.itae(i, j) = R.metrics.ITAE; S.sens.pass(i, j) = R.pass; S.sens.fault(i, j) = R.fault;
    end
end

% characterization: starting gains, tuned gains, LQR
ctrl = {'start', struct('stab', 1, 'pid', g0); 'tuned', struct('stab', 1, 'pid', best); 'lqr', struct('stab', 3)};
for k = 1:size(ctrl, 1)
    P = ctrl{k, 2};
    for sc = {'push', 'swingup', 'arm_step'}
        for lag = [10 15]
            cfg = cfg10; if lag == 15, cfg = cfg15; end
            R = run_sim('ctrl_hybrid.slx', sc{1}, 'Params', P, 'Cfg', cfg);
            S.runs.(ctrl{k, 1}).(sprintf('%s_%d', sc{1}, lag)) = keep(R);
        end
    end
    for m = 1:opts.MonteCarlo                             % perturbed plants (2 standard errors), new noise
        R = run_sim('ctrl_hybrid.slx', 'push', 'Params', P, 'Perturb', 2, 'Seed', m);
        S.mc.(ctrl{k, 1}).pass(m) = R.pass; S.mc.(ctrl{k, 1}).itae(m) = R.metrics.ITAE;
    end
    fprintf('[%5.0f s] characterized %s\n', toc(t0), ctrl{k, 1});
end

% linear check of the tuned loop (no lag, no integrator reset): closed-loop poles and margins
for k = {'start', 'tuned'}
    g = S.(k{1});
    C = [g.Kp0 + g.Ki0*Ts/(z - 1), g.Kp1, g.Kd0, g.Kd1];
    S.lin.(k{1}).poles = log(eig(feedback(Pd, C, +1)))/Ts;
    m = allmargin(-C*Pd);
    S.lin.(k{1}).pm = min(m.PhaseMargin); S.lin.(k{1}).gm = m.GainMargin; S.lin.(k{1}).dm = min(m.DelayMargin)*Ts;
end
S.elapsed = toc(t0);
save(opts.File, 'S');
fprintf('saved %s (%.0f s)\n', opts.File, S.elapsed);

    function R = sim1(g, sc, cfg)
        R = run_sim('ctrl_hybrid.slx', sc, 'Params', struct('stab', 1, 'pid', g), 'Cfg', cfg);
    end
    function [J, e] = cost(g)
        if any(structfun(@(v) v <= 0, g)), J = Inf; e = struct('itae10', NaN, 'itae15', NaN, 'pass10', 0, 'pass15', 0, 'arm', NaN, 'th1arm', NaN, 'sat', NaN, 'fault', 1); return, end
        a = sim1(g, 'push', cfg10); b = sim1(g, 'push', cfg15); c = sim1(g, 'arm_step', cfg10);
        win = (c.t >= 2 & c.t < 4) | (c.t >= 5 & c.t <= 7);
        e = struct('itae10', a.metrics.ITAE, 'itae15', b.metrics.ITAE, 'pass10', a.pass, 'pass15', b.pass, ...
                   'arm', mean(abs(c.th0(win) - c.r(win))), 'th1arm', max(abs(c.th1)), ...
                   'sat', max(a.metrics.sat_pct, b.metrics.sat_pct), 'fault', a.fault | b.fault | c.fault);
        J = (e.itae10 + e.itae15)/2 + opts.WArm*e.arm + ~e.pass10 + ~e.pass15;
        if e.fault, J = Inf; end
    end
    function ok = feasible(g, e)
        [pm, dm] = margins(g);
        ok = pm >= opts.MinPM && dm >= opts.MinDM && e.sat < 5 && e.th1arm < deg2rad(20);   % margins; arm step: pendulum up
        for flag = [10 15]                                % swing-up must pass at both lags (own loop names: nested
            if ~ok, break, end                            % functions share the outer ones)
            fcfg = cfg10; if flag == 15, fcfg = cfg15; end
            FR = sim1(g, 'swingup', fcfg); ok = ok && FR.pass && ~FR.fault;
        end
    end
    function addhist(g, J, e, feas, acc)
        [pm, dm] = margins(g);
        S.hist(end + 1) = struct('g', g, 'J', J, 'itae10', e.itae10, 'itae15', e.itae15, 'pass10', e.pass10, ...
            'pass15', e.pass15, 'arm', e.arm, 'sat', e.sat, 'pm', pm, 'dm', dm, 'feasible', feas, 'accepted', acc);
    end
    function [pm, dm] = margins(g)
        C = [g.Kp0 + g.Ki0*Ts/(z - 1), g.Kp1, g.Kd0, g.Kd1];   % u = C x, loop broken at the duty
        if any(abs(eig(feedback(Pd, C, +1))) >= 1), pm = -Inf; dm = 0; return, end
        mm = allmargin(-C*Pd); pm = min(mm.PhaseMargin); dm = min(mm.DelayMargin)*Ts;
        if isempty(pm), pm = Inf; dm = Inf; end
    end
end

function r = keep(R)
r = struct('t', R.t, 'th0', R.th0, 'th1', R.th1, 'r', R.r, 'u', R.u, 'd', R.d, 'i', R.i, 'mode', R.mode, ...
           'pass', R.pass, 'fault', R.fault, 'metrics', R.metrics);
end

function s = gstr(g)
s = sprintf('Kp1 %.4g  Kd1 %.4g  Kp0 %.4g  Ki0 %.4g  Kd0 %.4g', g.Kp1, g.Kd1, g.Kp0, g.Ki0, g.Kd0);
end
