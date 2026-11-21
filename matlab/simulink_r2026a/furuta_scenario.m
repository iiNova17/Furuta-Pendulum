function sc = furuta_scenario(name, cfg)
%FURUTA_SCENARIO The shared test cases every controller is scored on.
%   sc = furuta_scenario(name, cfg)      names = furuta_scenario('list')
%
%   balance   start 5 deg from upright, hold it.              pass: |th1| < 3 deg after 2 s
%   push      balance, tap the pendulum (+1.5 rad/s) at 2 s.   pass: |th1| < 3 deg after 4 s
%   arm_step  balance while the arm reference steps to 45 deg at 1 s and back at 4 s.
%             pass: never past 20 deg, |th0 - r| < 5 deg at 3.9 s and 6.9 s
%   swingup   start hanging at rest, swing up and balance.    pass: |th1| < 10 deg in the last 3 s
%   Every scenario also fails on any firmware trip (travel, overcurrent).
%
%   A scenario is a struct: T, x0 (true initial state), ref(t) -> r, dist(t) -> [arm pend]
%   disturbance torque [A], check(R) -> pass, text. Add your own the same way.
names = {'balance', 'push', 'arm_step', 'swingup'};
if strcmp(name, 'list'), sc = names; return, end
if nargin < 2, cfg = harness_config(); end
zero_ref = @(t) zeros(size(t));
no_dist = @(t) zeros(numel(t), 2);
d = @(x) deg2rad(x);
sc.name = name;
sc.ref = zero_ref;
sc.dist = no_dist;
switch name
    case 'balance'
        sc.T = 5;
        sc.x0 = [0; d(5); 0; 0];
        sc.t_event = 0;
        sc.check = @(R) R.fault == 0 && max(abs(R.th1(R.t >= 2))) < d(3);
        sc.text = 'Start 5 deg from upright; |th1| < 3 deg after 2 s';
    case 'push'
        sc.T = 6;
        sc.x0 = [0; d(2); 0; 0];
        dur = 0.02;
        amp = cfg.p.Jp*1.5/dur;                       % ~ +1.5 rad/s on the pendulum
        sc.dist = @(t) [zeros(numel(t), 1), amp*(t(:) >= 2 & t(:) < 2 + dur)];
        sc.t_event = 2;                               % the standardized disturbance test (brief 4.7)
        sc.check = @(R) R.fault == 0 && max(abs(R.th1(R.t >= 4))) < d(3);
        sc.text = 'Tap the pendulum at 2 s (+1.5 rad/s); |th1| < 3 deg after 4 s';
    case 'arm_step'
        sc.T = 7;
        sc.x0 = [0; 0; 0; 0];
        sc.ref = @(t) d(45)*(t >= 1 & t < 4);
        sc.t_event = 1;
        sc.step = [1 d(45)];                          % step metrics on [1 s, 4 s)
        sc.step_end = 4;
        at = @(R, tq) interp1(R.t, R.th0 - R.r, tq, 'previous');
        sc.check = @(R) R.fault == 0 && max(abs(R.th1)) < d(20) && ...
            abs(at(R, 3.9)) < d(5) && abs(at(R, 6.9)) < d(5);
        sc.text = 'Arm reference 0 -> 45 deg at 1 s -> 0 at 4 s; within 5 deg by 3.9 s and 6.9 s';
    case 'swingup'
        sc.T = 15;
        sc.x0 = [0; pi; 0; 0];
        sc.check = @(R) R.fault == 0 && max(abs(R.th1(R.t >= sc.T - 3))) < d(10);
        sc.text = 'Start hanging at rest; |th1| < 10 deg during the last 3 s';
    otherwise
        error('furuta_scenario:name', 'Unknown scenario "%s". Known: %s', name, strjoin(names, ', '));
end
end
