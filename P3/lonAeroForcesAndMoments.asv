function [X, Z, M] = lonAeroForcesAndMoments(x, u, aircraft_parameters)
%lonAeroForcesAndMoments - Calculate the longitudinal aerodynamic forces and moments
%
% Inputs:
%    x - state vector [x, y, z, phi, theta, psi, u, v, w, p, q, r] in SI units and radians
%    u - control vector [de, da, dr, dt] in radians
%    aircraft_parameters - struct containing nondimensional derivatives and other aircraft parameters
%
% Outputs:
%    X - force in x direction in N
%    Z - force in z direction in N
%    M - moment in y direction in Nm

ap = aircraft_parameters;
S = ap.S;
c = ap.c;

CL0 = ap.CL0;
CLalpha = ap.CLalpha;
CLq = ap.CLq;
CLde = ap.CLde;

CDmin = ap.CDmin;
CLmin = ap.CLmin;
K = ap.K;

Cm0 = ap.Cm0;
Cmalpha = ap.Cmalpha;
Cmq = ap.Cmq;
Cmde = ap.Cmde;

rho = stdatmo(-x(3));

dt = u(4);
de = u(1);

V = norm(x(7:9));
q = x(11);


Thrust = rho*ap.Sprop*ap.Cprop*(V + dt*(ap.kmotor - V))*dt*(ap.kmotor-V);

alpha = atan2(x(9),x(7));
qhat = (c*q/(2*V));

CL = CL0 + CLalpha*alpha + CLq*qhat + CLde*de;
CD = CDmin + K*((CL - CLmin)^2);
Cm = Cm0 + Cmalpha*alpha + Cmq*qhat + Cmde*de;

L = 0.5*rho*(V^2)*S*CL;
D = 0.5*rho*(V^2)*S*CD;

% Replace the following with your own code
X = -(D)*cos(alpha) + L*sin(alpha) + Thrust;
Z = -(D)*sin(alpha) - L*cos(alpha);
M = 0.5*rho*(V^2)*S*c*Cm;

end
