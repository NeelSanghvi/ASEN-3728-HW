function R = rotation321(phiThetaPsi)
% ROTATION321 Creates a rotation matrix from Euler angles for a 3-2-1 rotation sequence.
% Inputs:
%   phiThetaPsi = 3x1 vector of Euler angles [phi; theta; psi] (rad)
% Outputs:
%   R = 3x3 direction cosine matrix

phi = phiThetaPsi(1);
theta = phiThetaPsi(2);
psi = phiThetaPsi(3);

R_1 = [1, 0, 0; 0, cos(phi), sin(phi); 0, -sin(phi), cos(phi)];
R_2 = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)];
R_3 = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1];

R = R_1*R_2*R_3; 

end
