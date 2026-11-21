function fr = arm_friction(w0, th1, w1, i, dist, p)
%ARM_FRICTION Arm dry friction torque of the report's model (furuta_rhs.m, report eq. stick), in A.
%   fr = arm_friction(w0, th1, w1, i, dist, p)
%   Sliding: identified Coulomb friction, smoothed tanh(w0/eps). With p.tsp > 0 the arm can also stick
%   (Karnopp): inside |w0| < p.stick_band the friction takes the value that holds the arm (and brings
%   its speed to zero with time constant p.stick_T), as long as that stays within the breakaway
%   p.tsp / -p.tsn (the measured 13 % / 14 % duty dead zone).
%   dist = [arm; pendulum] external torques (harness disturbances, A); [0; 0] gives furuta_rhs exactly.
%   Used by harness_plant_rhs and by the Simscape plant's "arm friction" block, so both are the same.
s = sin(th1); c = cos(th1);
if w0 > 0, tc = p.tcp; else, tc = p.tcn; end
fr = tc*tanh(w0/p.eps);
if p.tsp > 0 && abs(w0) < p.stick_band
    f0 = i + dist(1) - p.Beq*w0 - 2*p.Jp*s*c*w0*w1 + p.h*s*w1^2;          % arm force without dry friction
    f1 = p.Jp*s*c*w0^2 - p.b1*w1 - p.tp*tanh(w1/p.eps1) + p.G*s + dist(2);
    m11 = p.Ja + p.Jp*s^2; m12 = p.h*c;
    need = f0 - m12*f1/p.Jp + (m11*p.Jp - m12^2)*w0/(p.Jp*p.stick_T);      % friction that holds the arm
    if need <= p.tsp && need >= -p.tsn, fr = need; end
end
end
