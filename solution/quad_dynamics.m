function dx = quad_dynamics(~, x, U, P, dist)
%QUAD_DYNAMICS  Nonlinear 6-DOF quadcopter dynamics (Task 1).
%
%   dx = QUAD_DYNAMICS(t, x, U, P, dist)
%
%   Implements the model of Mien, T. & Tu, T. (2024), "Design and Quality
%   Evaluation of the Position and Attitude Control System for 6-DOF UAV
%   Quadcopter Using Heuristic PID Tuning Methods", IJRCS 4(4), 1712-1730 -
%   the reference cited by the problem statement.
%
%   TRANSLATIONAL - the paper's Eq. (7), implemented exactly:
%       xddot = -Ax*xdot/m + (Cpsi*Sth*Cphi + Spsi*Sphi)*F/m
%       yddot = -Ay*ydot/m + (Spsi*Sth*Cphi - Cpsi*Sphi)*F/m
%       zddot = -Az*zdot/m - g + (Cphi*Cth)*F/m
%
%   ROTATIONAL - the paper's Eq. (11), Euler-Lagrange form:
%       etaddot = J(eta)^-1 * [ tau_B - C(eta,etadot)*etadot ]
%   with J(eta) from Eq. (8) and C(eta,etadot) from Eq. (12). This is a
%   configuration-dependent inertia matrix plus a full Coriolis matrix - NOT
%   the simplified body-frame Euler equations. The two agree at hover and
%   differ by roughly 22% at 17 degrees of tilt, so matching the paper
%   requires the Euler-Lagrange form.
%
%   Set P.rot_model = 'euler' to use the simplified body-frame equations
%   instead (they carry a rotor gyroscopic term the paper's Eq. (11) omits).
%   Default is 'lagrange', which reproduces the paper exactly.
%
%   State vector (12 states):
%       x(1:2)   = X,  Xdot        inertial position/velocity along x  [m, m/s]
%       x(3:4)   = Y,  Ydot                                            [m, m/s]
%       x(5:6)   = Z,  Zdot        altitude                            [m, m/s]
%       x(7:8)   = phi,   phidot   roll  (about x)                     [rad, rad/s]
%       x(9:10)  = theta, thetadot pitch (about y)                     [rad, rad/s]
%       x(11:12) = psi,   psidot   yaw   (about z)                     [rad, rad/s]
%
%   Control vector U:
%       U(1) = U1  total thrust F   [N]        (paper Eq. 18)
%       U(2) = U2  roll  input      [N]        tau_phi   = l*U2  (Eq. 19)
%       U(3) = U3  pitch input      [N]        tau_theta = l*U3  (Eq. 19)
%       U(4) = U4  yaw torque       [N.m]      tau_psi   = U4    (Eq. 19)
%       U(5) = Omega_r  residual rotor speed [rad/s], optional; used only by
%              the 'euler' model, ignored by 'lagrange' (the paper's Eq. (11)
%              contains no rotor gyroscopic term)
%
%   dist (optional) = [Fx Fy Fz] external disturbance force in the inertial
%   frame [N], e.g. a wind gust. Defaults to zeros.

if nargin < 5 || isempty(dist), dist = [0 0 0]; end
if numel(U) < 5, U(5) = 0; end
if ~isfield(P,'rot_model'), P.rot_model = 'lagrange'; end

F  = U(1);  U2 = U(2);  U3 = U(3);  U4 = U(4);  Omr = U(5);

phi = x(7);  theta = x(9);  psi = x(11);
pd  = x(8);  td    = x(10); yd  = x(12);      % Euler rates

Sphi = sin(phi); Cphi = cos(phi);
Sth  = sin(theta); Cth = cos(theta);
Spsi = sin(psi);  Cpsi = cos(psi);

dx = zeros(12,1);

%% ---- Translational dynamics: the paper's Eq. (7), exactly ------------
dx(1) = x(2);
dx(2) = -P.Ax*x(2)/P.m + (Cpsi*Sth*Cphi + Spsi*Sphi)*F/P.m + dist(1)/P.m;

dx(3) = x(4);
dx(4) = -P.Ay*x(4)/P.m + (Spsi*Sth*Cphi - Cpsi*Sphi)*F/P.m + dist(2)/P.m;

dx(5) = x(6);
dx(6) = -P.Az*x(6)/P.m - P.g + (Cphi*Cth)*F/P.m             + dist(3)/P.m;

%% ---- Rotational dynamics ---------------------------------------------
tau = [P.l*U2; P.l*U3; U4];      % body torques, paper Eq. (19)

dx(7)  = pd;
dx(9)  = td;
dx(11) = yd;

switch lower(P.rot_model)

case 'lagrange'
    % ---- Paper Eq. (11): etaddot = J^-1 (tau - C*etadot) --------------
    Ixx = P.Ixx; Iyy = P.Iyy; Izz = P.Izz;

    % Eq. (8): the configuration-dependent inertia matrix J(eta)
    J = [ Ixx,                    0,                          -Ixx*Sth;
          0,      Iyy*Cphi^2 + Izz*Sphi^2,      (Iyy-Izz)*Cphi*Sphi*Cth;
         -Ixx*Sth,  (Iyy-Izz)*Cphi*Sphi*Cth, ...
          Ixx*Sth^2 + Iyy*Sphi^2*Cth^2 + Izz*Cphi^2*Cth^2 ];

    % Eq. (12): the Coriolis matrix C(eta,etadot)
    C = zeros(3);
    C(1,1) = 0;
    C(1,2) = (Iyy-Izz)*(td*Cphi*Sphi + yd*Sphi^2*Cth) ...
             + (Izz-Iyy)*yd*Cphi^2*Cth - Ixx*yd*Cth;
    C(1,3) = (Izz-Iyy)*yd*Cphi*Sphi*Cth^2;
    C(2,1) = (Izz-Iyy)*(td*Cphi*Sphi + yd*Sphi*Cth) ...
             + (Iyy-Izz)*yd*Cphi^2*Cth + Ixx*yd*Cth;
    C(2,2) = (Izz-Iyy)*pd*Cphi*Sphi;
    C(2,3) = -Ixx*yd*Sth*Cth + Iyy*yd*Sphi^2*Sth*Cth + Izz*yd*Cphi^2*Sth*Cth;
    C(3,1) = (Iyy-Izz)*yd*Cth^2*Sphi*Cphi - Ixx*td*Cth;
    C(3,2) = (Izz-Iyy)*(td*Cphi*Sphi*Sth + pd*Sphi^2*Cth) ...
             + (Iyy-Izz)*pd*Cphi^2*Cth + Ixx*yd*Sth*Cth ...
             - Iyy*yd*Sphi^2*Sth*Cth - Izz*yd*Cphi^2*Sth*Cth;
    C(3,3) = (Iyy-Izz)*pd*Cphi*Sphi*Cth^2 - Iyy*td*Sphi^2*Cth*Sth ...
             - Izz*td*Cphi^2*Cth*Sth + Ixx*td*Cth*Sth;

    etadot = [pd; td; yd];
    rhs    = tau - C*etadot;

    % J(eta) becomes singular at theta = +/- pi/2 (gimbal lock of the ZYX
    % Euler parameterisation). Fall back to the hover-diagonal inertia there
    % rather than produce Inf - this is outside any sane flight envelope.
    if abs(Cth) < 1e-6 || rcond(J) < 1e-12
        etaddot = [rhs(1)/Ixx; rhs(2)/Iyy; rhs(3)/Izz];
    else
        etaddot = J \ rhs;
    end

    dx(8)  = etaddot(1);
    dx(10) = etaddot(2);
    dx(12) = etaddot(3);

case 'euler'
    % ---- Simplified body-frame Euler equations (small-angle equivalent) -
    %  Includes the rotor gyroscopic term, which the paper's Eq. (11) omits.
    dx(8)  = ((P.Iyy - P.Izz)/P.Ixx)*td*yd - (P.IM/P.Ixx)*td*Omr + tau(1)/P.Ixx;
    dx(10) = ((P.Izz - P.Ixx)/P.Iyy)*pd*yd + (P.IM/P.Iyy)*pd*Omr + tau(2)/P.Iyy;
    dx(12) = ((P.Ixx - P.Iyy)/P.Izz)*pd*td                       + tau(3)/P.Izz;

otherwise
    error('quad_dynamics:badModel', ...
          'P.rot_model must be ''lagrange'' or ''euler'', got ''%s''.', P.rot_model);
end
end
