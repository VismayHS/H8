function dx = quad_dynamics(~, x, U, P, dist)
%QUAD_DYNAMICS  Nonlinear 6-DOF quadcopter dynamics (Task 1).
%
%   dx = QUAD_DYNAMICS(t, x, U, P, dist)
%
%   State vector (12 states):
%       x(1)  = X        inertial position along x        [m]
%       x(2)  = Xdot     inertial velocity along x        [m/s]
%       x(3)  = Y                                          [m]
%       x(4)  = Ydot                                       [m/s]
%       x(5)  = Z        altitude                          [m]
%       x(6)  = Zdot                                       [m/s]
%       x(7)  = phi      roll  (about body x)              [rad]
%       x(8)  = phidot                                     [rad/s]
%       x(9)  = theta    pitch (about body y)              [rad]
%       x(10) = thetadot                                   [rad/s]
%       x(11) = psi      yaw   (about body z)              [rad]
%       x(12) = psidot                                     [rad/s]
%
%   Control vector U (the standard four virtual inputs):
%       U(1) = U1  total thrust        [N]
%       U(2) = U2  roll  moment input  [N]   -> torque = l*U2
%       U(3) = U3  pitch moment input  [N]   -> torque = l*U3
%       U(4) = U4  yaw   torque        [N.m]
%       U(5) = Omega_r  residual rotor speed [rad/s]  (optional, default 0)
%
%   dist (optional) = [Fx Fy Fz] external disturbance force in the inertial
%   frame [N], e.g. a wind gust. Defaults to zeros.
%
%   Rotor geometry follows Fig. 1 of the problem statement: a cross ("plus")
%   configuration with rotor 1 on +x_B, rotor 2 on -y_B, rotor 3 on -x_B and
%   rotor 4 on +y_B. See ROTOR2U for the mapping from rotor speeds to U.

if nargin < 5 || isempty(dist), dist = [0 0 0]; end
if numel(U) < 5, U(5) = 0; end

U1 = U(1);  U2 = U(2);  U3 = U(3);  U4 = U(4);  Omr = U(5);

phi = x(7);  theta = x(9);  psi = x(11);
p   = x(8);  q     = x(10); r   = x(12);      % Euler rates

sphi = sin(phi); cphi = cos(phi);
sth  = sin(theta); cth = cos(theta);
spsi = sin(psi);  cpsi = cos(psi);

dx = zeros(12,1);

% ---- Translational dynamics (inertial frame) ----------------------------
% Thrust acts along the body z axis; rotate it into the inertial frame.
dx(1) = x(2);
dx(2) = (U1/P.m)*(cphi*sth*cpsi + sphi*spsi) - (P.Ax/P.m)*x(2) + dist(1)/P.m;

dx(3) = x(4);
dx(4) = (U1/P.m)*(cphi*sth*spsi - sphi*cpsi) - (P.Ay/P.m)*x(4) + dist(2)/P.m;

dx(5) = x(6);
dx(6) = -P.g + (U1/P.m)*(cphi*cth)          - (P.Az/P.m)*x(6) + dist(3)/P.m;

% ---- Rotational dynamics ------------------------------------------------
% Includes the coupled inertia terms and the gyroscopic term from the rotors.
dx(7)  = p;
dx(8)  = ((P.Iyy - P.Izz)/P.Ixx)*q*r - (P.IM/P.Ixx)*q*Omr + (P.l/P.Ixx)*U2;

dx(9)  = q;
dx(10) = ((P.Izz - P.Ixx)/P.Iyy)*p*r + (P.IM/P.Iyy)*p*Omr + (P.l/P.Iyy)*U3;

dx(11) = r;
dx(12) = ((P.Ixx - P.Iyy)/P.Izz)*p*q                      + (1/P.Izz)*U4;
end
