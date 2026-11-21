function b = plant_bodies(p)
%PLANT_BODIES Rigid bodies whose multibody equations are exactly the report's model (eq. 89).
%   The report identifies only lumped, K_tau-scaled parameters (section 4.1.1): Ja, Jp, h, G, ...
%   in A*s^2/rad and A. Taking K_tau = 1 N*m/A makes them ordinary SI values (kg*m^2, N*m) and the
%   motor current the joint torque. Any arm + slender pendulum with
%       G  = m g Lp,     Jp = J1 + m Lp^2,     h = m Larm Lp,     Ja = Ia + m Larm^2
%   and the pendulum's inertia about its own long axis = 0 (report eq. 9) has eq. 89 as its
%   equations of motion, so the Simscape model reproduces the report by construction.
%   m and Lp cannot be separated from the data (only m*Lp = G/g is identified); Lp is taken from
%   the CAD centre of mass (51.2 mm) unless that would make J1 negative. The split does not
%   change the dynamics. Larm = g h / G is the identified arm length (6.81 cm, report Table 6).
g = 9.80665;                                % the value the report used for Larm = g h / G
b.g = g;
b.Larm = g*p.h/p.G;
b.Lp = min(0.0512, 0.8*g*p.Jp/p.G);
b.m = p.G/(g*b.Lp);                         % "kg" for K_tau = 1
b.J1 = p.Jp - b.m*b.Lp^2;                   % about the pendulum's centre of mass
b.Ia = p.Ja - b.m*b.Larm^2;                 % arm + reflected rotor about the motor axis
b.m_arm = 0.03;                             % arm mass on the motor axis: no effect on the dynamics
end
