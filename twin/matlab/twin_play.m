function twin_play(t, q_arm, q_pend, source, speed)
% TWIN_PLAY  Stream a trajectory to the Unreal twin in real time (60 packets/s).
%   twin_play(t, q_arm, q_pend)                 as the model (translucent ghost, or solid if no rig data)
%   twin_play(t, q_arm, q_pend, 'real')         as the real rig
%   twin_play(..., source, speed)               speed = 0.25 plays at quarter speed
arguments
    t (:, 1) double
    q_arm (:, 1) double
    q_pend (:, 1) double
    source (1, :) char = 'model'
    speed (1, 1) double {mustBePositive} = 1
end
u = udpport('datagram', 'IPV4');
c = onCleanup(@() delete(u));
clk = tic; T = t(end) - t(1); next = 0;
while true
    s = t(1) + toc(clk) * speed;
    if s > t(end), break; end
    twin_send(interp1(t, q_arm, s), interp1(t, q_pend, s), source, u);
    next = next + 1 / 60;                       % deadline pacing: pause(1/60) alone gives ~33 Hz on Windows
    while toc(clk) < next, pause(0.001); end
end
twin_send(q_arm(end), q_pend(end), source, u);
fprintf('streamed %.1f s of trajectory to the twin\n', T);
end
