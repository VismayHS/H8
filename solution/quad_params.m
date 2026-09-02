function P = quad_params()
%QUAD_PARAMS  Parameters of the 6-DOF UAV quadcopter (HACKSIMUL8 2026, Table 1).
%
%   All values taken directly from Table 1 of the problem statement.
%   Reference: Mien, T. & Tu, T. (2024), IJRCS 4(4), 1712-1730.

P.m   = 0.516;      % quadcopter mass                       [kg]
P.l   = 0.225;      % arm length (centre to rotor)          [m]
P.g   = 9.81;       % gravitational acceleration            [m/s^2]
P.IM  = 3.368e-5;   % inertia moment of the rotor           [kg.m^2]
P.k   = 2.996e-6;   % thrust factor of rotor                [N.s^2]
P.b   = 1.260e-7;   % drag coefficient                      [N.m.s^2]
P.Ixx = 4.984e-3;   % inertia about body x                  [kg.m^2]
P.Iyy = 4.984e-3;   % inertia about body y                  [kg.m^2]
P.Izz = 8.958e-3;   % inertia about body z                  [kg.m^2]

% ---- Derived quantities -------------------------------------------------
P.W      = P.m * P.g;                 % weight [N] = 5.0620 N
P.w_hover = sqrt(P.W / (4*P.k));      % hover speed of each rotor [rad/s]

% Aerodynamic drag on the airframe (small; set to 0 to match the pure
% Table 1 model, raise it if you want a more realistic demo)
P.Ax = 0.0; P.Ay = 0.0; P.Az = 0.0;   % translational drag [N.s/m]

% ---- Actuator limits ----------------------------------------------------
% Real rotors saturate. Modelling this is what makes the demo credible and
% is why anti-windup is needed in Task 2.
P.w_min = 0;                          % [rad/s]
P.w_max = 2 * P.w_hover;              % allows ~4x hover thrust

P.U1_min = 4 * P.k * P.w_min^2;       % [N]
P.U1_max = 4 * P.k * P.w_max^2;       % [N]

% ---- Rotational dynamics formulation ------------------------------------
%  'lagrange' = the reference paper's Eq. (11): etaddot = J(eta)^-1(tau - C*etadot)
%               with J from Eq. (8) and C from Eq. (12). This reproduces
%               Mien & Tu (2024) EXACTLY and is the default.
%  'euler'    = simplified body-frame Euler equations. Equivalent at hover,
%               diverges ~22% at 17 deg of tilt. Carries a rotor gyroscopic
%               term that the paper's Eq. (11) does not contain.
P.rot_model = 'lagrange';

% ---- Simulation defaults ------------------------------------------------
P.Ts   = 0.01;                        % controller sample time [s] (100 Hz)
P.Tend = 10;                          % default simulation stop time [s]
end
