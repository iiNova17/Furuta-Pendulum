function m = harness_metrics(R, cfg, sc)
%HARNESS_METRICS Performance metrics of one run, as listed in the project brief (sec. 4.7).
%   Angles in degrees, times in seconds. Measured from the scenario's event time t_ev:
%   sc.t_event (disturbance / step time), or for swing-up the capture time (first mode == 2).
%
%   theta1 after the event (transient, disturbance rejection):
%     th1_max      max |theta1| after t_ev            (maximum deviation)
%     th1_tpeak    time of that maximum after t_ev    (peak time)
%     th1_tsettle  time after t_ev until |theta1| stays inside cfg.metrics.band_th1 (recovery time)
%     IAE, ISE, ITAE  of theta1 [rad] from t_ev to the end (ITAE weighted by t - t_ev)
%   theta0 step (scenarios with sc.step = [t0 amplitude]):
%     th0_trise    10-90 % rise time,  th0_overshoot [%],  th0_tpeak,  th0_tsettle (cfg.metrics.band_th0 %)
%   steady state (last cfg.metrics.ss_window s):  ess_th1, ess_th0 = mean |theta1|, mean |theta0 - r|
%   control effort (after t_ev): u_peak, u_rms (commanded duty), sat_pct (% of ticks above max_duty)
%   capture: t_capture (first entry into Stabilize, NaN if never); t_up: |theta1| < 10 deg to the end
M = cfg.metrics;
t = R.t;
th1 = R.th1;
th0 = R.th0;
m.t_capture = first(t, R.mode == 2);
if isfield(sc, 't_event') && ~isempty(sc.t_event), tev = sc.t_event; else, tev = m.t_capture; end
if isnan(tev), tev = t(end); end
m.t_event = tev;
a = t >= tev;
ta = t(a) - tev;
e1 = th1(a);

[pk, k] = max(abs(e1));
m.th1_max = rad2deg(pk);
m.th1_tpeak = ta(k);
m.th1_tsettle = settle(ta, abs(e1) <= deg2rad(M.band_th1));
dt = mean(diff(t));
m.IAE = sum(abs(e1))*dt;
m.ISE = sum(e1.^2)*dt;
m.ITAE = sum(ta.*abs(e1))*dt;

m.th0_trise = NaN; m.th0_overshoot = NaN; m.th0_tpeak = NaN; m.th0_tsettle = NaN;
if isfield(sc, 'step') && ~isempty(sc.step)
    s0 = sc.step(1); A = sc.step(2);
    b = t >= s0;
    if isfield(sc, 'step_end'), b = b & t < sc.step_end; end
    tb = t(b) - s0;
    y = th0(b)/A;                                   % normalized step response
    m.th0_trise = first(tb, y >= 0.9) - first(tb, y >= 0.1);
    [ymax, k] = max(y);
    m.th0_overshoot = 100*max(0, ymax - 1);
    m.th0_tpeak = tb(k);
    m.th0_tsettle = settle(tb, abs(y - 1) <= M.band_th0/100);
end

ss = t >= t(end) - M.ss_window;
m.ess_th1 = rad2deg(mean(abs(th1(ss))));
m.ess_th0 = rad2deg(mean(abs(th0(ss) - R.r(ss))));
m.u_peak = max(abs(R.u(a)));
m.u_rms = sqrt(mean(R.u(a).^2));
m.sat_pct = 100*mean(abs(R.u) > cfg.act.max_duty);
up = abs(th1) < deg2rad(10);
m.t_up = settle(t, up);
end

function v = first(t, c)
k = find(c, 1);
if isempty(k), v = NaN; else, v = t(k); end
end

function v = settle(t, inside)
% time after which the condition holds to the end of the run (NaN if it does not hold at the end)
if isempty(inside) || ~inside(end), v = NaN; return, end
k = find(~inside, 1, 'last');
if isempty(k), v = t(1); else, v = t(k + 1); end
end
