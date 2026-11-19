function [dx, i] = furuta_rhs(x, d, p)
%FURUTA_RHS Nonlinear Furuta model identified with Furuta Lab.
%   x = [theta0; theta1; dtheta0; dtheta1] (theta1 = 0 upright), d = duty in [-1, 1].
%   Torque-like parameters are divided by K_tau (torque in amperes of motor current).
%   At zero duty the driver is in standby: the motor is open-circuit and i = 0.
%   Arm friction: Coulomb (smoothed, tanh) + viscous while slipping. With p.tsp > 0 the arm also sticks
%   (Karnopp): inside |dtheta0| < p.stick_band the friction holds it, up to the breakaway p.tsp / p.tsn
%   (the measured 13 % / 14 % breakaway dead zone), and brings its speed to zero with time constant p.stick_T.
%   Same physics as desktop/furuta_lab/estimation.py (rhs).
s = sin(x(2)); c = cos(x(2)); w0 = x(3); w1 = x(4);
if d ~= 0
    i = (d*p.Vs - p.Kb*w0)/p.Rm;
else
    i = 0;
end
if w0 > 0, tc = p.tcp; else, tc = p.tcn; end
fr = tc*tanh(w0/p.eps);
f0 = i - p.Beq*w0 - 2*p.Jp*s*c*w0*w1 + p.h*s*w1^2;          % without the arm dry friction
f1 = p.Jp*s*c*w0^2 - p.b1*w1 - p.tp*tanh(w1/p.eps1) + p.G*s;
M = [p.Ja + p.Jp*s^2, p.h*c; p.h*c, p.Jp];
if isfield(p, 'tsp') && p.tsp > 0 && abs(w0) < p.stick_band
    dt_ = M(1,1)*p.Jp - M(1,2)^2;
    need = f0 - M(1,2)*f1/p.Jp + dt_*w0/(p.Jp*p.stick_T);   % friction that holds the arm
    if need <= p.tsp && need >= -p.tsn, fr = need; end
end
f0 = f0 - fr;
a = M \ [f0; f1];
dx = [w0; w1; a(1); a(2)];
end
