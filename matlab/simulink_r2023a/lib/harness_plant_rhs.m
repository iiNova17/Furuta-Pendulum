function dx = harness_plant_rhs(x, d, dist, p)
%HARNESS_PLANT_RHS Report plant (furuta_rhs, eq. 84 with eq. 88 and the arm breakaway) plus external
%   disturbance torques dist = [arm; pendulum] in the model's units (A, i.e. torque / K_tau).
%   The disturbances enter the generalized forces (eq. 82), also inside the stick test of the arm
%   friction (arm_friction), so a stuck arm stays stuck under a small push. With dist = 0 this is
%   exactly furuta_rhs (tests/test_harness.m checks it, including stuck states).
dist = dist(:);
if ~any(dist ~= 0)
    dx = furuta_rhs(x, d, p);
    return
end
s = sin(x(2)); c = cos(x(2)); w0 = x(3); w1 = x(4);
if d ~= 0
    i = (d*p.Vs - p.Kb*w0)/p.Rm;
else
    i = 0;
end
fr = arm_friction(w0, x(2), w1, i, dist, p);
f0 = i - p.Beq*w0 - fr - 2*p.Jp*s*c*w0*w1 + p.h*s*w1^2 + dist(1);
f1 = p.Jp*s*c*w0^2 - p.b1*w1 - p.tp*tanh(w1/p.eps1) + p.G*s + dist(2);
M = [p.Ja + p.Jp*s^2, p.h*c; p.h*c, p.Jp];
a = M \ [f0; f1];
dx = [w0; w1; a(1); a(2)];
end
