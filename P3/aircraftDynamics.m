function xdot = aircraftDynamics(x, u, aircraft_parameters)
%aircraftDynamics - computes the state derivative of a conventional aircraft
%
% Inputs:
%    x - state vector [x, y, z, phi, theta, psi, u, v, w, p, q, r] in SI units and radians
%    u - control vector [de, da, dr, dt] in radians
%    aircraft_parameters - struct containing nondimensional derivatives and other aircraft parameters
%
% Outputs:
%    xdot - state derivative vector [xdot, ydot, zdot, phidot, thetadot, psidot, udot, vdot, wdot, pdot, qdot, rdot] in SI units and radians

[X, Z, M] = lonAeroForcesAndMoments(x, u, aircraft_parameters);
[Y, L, N] = latAeroForcesAndMoments(x, u, aircraft_parameters);

m = aircraft_parameters.m;

p = x(1:3); % Position (m)
o = x(4:6); % Orientation (rad)
v_E_B = x(7:9); % Velocity (m/s)
omega_B = x(10:12); % Angular velocity (rad/s)
omega_B_mat = [0, -omega_B(3), omega_B(2); omega_B(3), 0, -omega_B(1); -omega_B(2), omega_B(1), 0];
Imat = [aircraft_parameters.Ix, 0, aircraft_parameters.Ixz; 0, aircraft_parameters.Iy, 0; aircraft_parameters.Ixz, 0, aircraft_parameters.Iz];
G = [L,M,N]';

% Replace with your code
xdot = zeros(12, 1);
xdot(1:3,1) = rotation321(o)'*v_E_B;
xdot(4:6,1) = attitudeInfluence321(o)*omega_B;
xdot(7:9,1) = [X/m;Y/m;Z/m] + rotation321(o)*[0;0;9.81] - cross(omega_B,v_E_B);
xdot(10:12,1) = Imat\(G - cross(omega_B,(Imat*omega_B)));

end


function T = attitudeInfluence321(phiThetaPsi)
% ATTITUDEINFLUENCE321  Calculate the attitude influence matrix for a 3-2-1 rotation sequence.
% Inputs:
%   phiThetaPsi - 3x1 vector of Euler angles [phi; theta; psi] (rad)
% Outputs:
%   T - 3x3 attitude influence matrix

phi = phiThetaPsi(1);
theta = phiThetaPsi(2);
psi = phiThetaPsi(3);

T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); 0, cos(phi), -sin(phi); 0, sin(phi)/cos(theta), cos(phi)/cos(theta)]; % Replace with your code

end
