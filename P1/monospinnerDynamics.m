function xdot = monospinnerDynamics(t, x, u)
% MONOSPINNERDYNAMICS Dynamics of a monospinner aircraft to be integrated with ODE45
% Inputs:
%   t = time (s)
%   x = 12 dimensional state vector
%   u = 1 dimensional control input
% Outputs:
%   xdot = time derivative of state vector x

p = x(1:3); % Position (m)
o = x(4:6); % Orientation (rad)
o = o;
v_E_B = x(7:9); % Velocity (m/s)
omega_B = x(10:12); % Angular velocity (rad/s)
omega_B_mat = [0, -omega_B(3), omega_B(2); omega_B(3), 0, -omega_B(1); -omega_B(2), omega_B(1), 0];

I_B = [102 -24  -9;
       -24 318   0;
        -9   0 414]*1e-5; % Moment of inertia matrix (kg-m^2) % Note: different from paper because of different coordinate system
m = 0.217; % Mass (kg)

rotorThrust = u(1); % N
k_m = 0.0024; % Control moment coefficient (N-m / N)
rotorTorque = k_m*rotorThrust; % Aerodynamic torque about the rotor axis (N-m)
l = 0.12; % Distance from mass center to rotor (m)

f_vec = [0,0,-rotorThrust]';
vdot_E_B = f_vec/m - cross(omega_B,v_E_B) + rotation321(o)*[0,0,9.81]';

g_vec = [0,rotorThrust*l,-rotorTorque]';
omegaDot_B = I_B\(g_vec - omega_B_mat*I_B*omega_B); 

xdot = zeros(12, 1); % Replace with your code
xdot(1:3,1) = inv(rotation321(o))*v_E_B;
xdot(4:6,1) = attitudeInfluence321(o)*omega_B;
xdot(7:9,1) = vdot_E_B;
xdot(10:12,1) = omegaDot_B;
end
