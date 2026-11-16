function dx = harness_plant_rhs(x, d, dist, p)
%HARNESS_PLANT_RHS Report plant (furuta_rhs, eq. 84 with eq. 88) plus external disturbance torques.
%   dist = [arm; pendulum] disturbance torque in the model's units (A, i.e. torque / K_tau).
%   The accelerations are linear in the generalized forces, so the disturbance is added through
%   the same mass matrix (eq. 82). With dist = 0 this is exactly furuta_rhs.
dx = furuta_rhs(x, d, p);
if any(dist ~= 0)
    s = sin(x(2));
    c = cos(x(2));
    M = [p.Ja + p.Jp*s^2, p.h*c; p.h*c, p.Jp];
    dx(3:4) = dx(3:4) + M \ dist(:);
end
end
