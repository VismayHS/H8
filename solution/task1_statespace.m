%% TASK 1 - Dynamic mathematical model of the 6-DOF UAV quadcopter
%  HACKSIMUL8 2026, PES University
%
%  Deliverable: the nonlinear model AND its state-space representation,
%  with a numerical check that the two agree.
%
%  Run this first. It produces:
%     sys_full  - 12-state linear model about hover
%     sys_alt   - the altitude subsystem used in Task 2
%     A,B,C,D   - matrices to show on your slide

clear; clc; close all;
P = quad_params();

fprintf('=== TASK 1: 6-DOF Quadcopter Model ===\n\n');
fprintf('Mass          m   = %.3f kg\n',   P.m);
fprintf('Arm length    l   = %.3f m\n',    P.l);
fprintf('Ixx = Iyy         = %.4e kg.m^2\n', P.Ixx);
fprintf('Izz               = %.4e kg.m^2\n', P.Izz);
fprintf('Thrust factor k   = %.4e N.s^2\n',  P.k);
fprintf('Drag coeff.   b   = %.4e N.m.s^2\n',P.b);
fprintf('Hover thrust  W   = %.4f N\n',      P.W);
fprintf('Hover rotor speed = %.1f rad/s each\n\n', P.w_hover);

%% ------------------------------------------------------------------------
%  1. THE NONLINEAR MODEL  (see quad_dynamics.m for the full equations)
%
%  Translational:
%     xddot = (U1/m)(cos(phi)sin(th)cos(psi) + sin(phi)sin(psi))
%     yddot = (U1/m)(cos(phi)sin(th)sin(psi) - sin(phi)cos(psi))
%     zddot = -g + (U1/m)cos(phi)cos(th)
%
%  Rotational:
%     phiddot = ((Iyy-Izz)/Ixx)*thdot*psidot - (IM/Ixx)*thdot*Om_r + (l/Ixx)*U2
%     thddot  = ((Izz-Ixx)/Iyy)*phidot*psidot + (IM/Iyy)*phidot*Om_r + (l/Iyy)*U3
%     psiddot = ((Ixx-Iyy)/Izz)*phidot*thdot                        + (1/Izz)*U4
%
%  Control mapping (cross configuration, Fig. 1):
%     U1 = k(w1^2+w2^2+w3^2+w4^2)      U2 = k(w4^2-w2^2)
%     U3 = k(w3^2-w1^2)                U4 = b(w1^2+w3^2-w2^2-w4^2)
% -------------------------------------------------------------------------

%% 2. EQUILIBRIUM (HOVER) OPERATING POINT
x_hover = zeros(12,1);            % at rest, level, at the origin
U_hover = [P.W; 0; 0; 0; 0];      % thrust exactly balances weight

dx_check = quad_dynamics(0, x_hover, U_hover, P);
fprintf('Residual at hover ||dx|| = %.3e  (should be ~0)\n\n', norm(dx_check));

%% 3. ANALYTICAL LINEARISATION ABOUT HOVER
%  With phi, th, psi small and U1 = W + dU1, the model decouples into four
%  independent channels. This is the state-space form to put on your slide.
%
%  States  x = [X Xdot Y Ydot Z Zdot phi phidot th thdot psi psidot]'
%  Inputs  u = [dU1 U2 U3 U4]'

A = zeros(12,12);
B = zeros(12,4);

A(1,2)  = 1;
A(2,9)  =  P.g;          % xddot  =  g*theta
A(3,4)  = 1;
A(4,7)  = -P.g;          % yddot  = -g*phi
A(5,6)  = 1;
B(6,1)  = 1/P.m;         % zddot  =  dU1/m
A(7,8)  = 1;
B(8,2)  = P.l/P.Ixx;     % phiddot = (l/Ixx)*U2
A(9,10) = 1;
B(10,3) = P.l/P.Iyy;     % thddot  = (l/Iyy)*U3
A(11,12)= 1;
B(12,4) = 1/P.Izz;       % psiddot = (1/Izz)*U4

C = eye(12);
D = zeros(12,4);

stateNames = {'X','Xdot','Y','Ydot','Z','Zdot', ...
              'phi','phidot','theta','thetadot','psi','psidot'};
inputNames = {'dU1','U2','U3','U4'};

sys_full = ss(A,B,C,D, 'StateName',stateNames, ...
                       'InputName',inputNames, ...
                       'OutputName',stateNames);

%% 4. NUMERICAL CHECK OF THE LINEARISATION
%  Finite-difference the nonlinear model and compare with A and B.
%  This is the evidence that your model is right - show this to the judges.
eps_x = 1e-6;
An = zeros(12,12); Bn = zeros(12,4);
for i = 1:12
    dxp = zeros(12,1); dxp(i) = eps_x;
    An(:,i) = (quad_dynamics(0,x_hover+dxp,U_hover,P) - ...
               quad_dynamics(0,x_hover-dxp,U_hover,P)) / (2*eps_x);
end
for i = 1:4
    dup = zeros(5,1); dup(i) = eps_x;
    Bn(:,i) = (quad_dynamics(0,x_hover,U_hover+dup,P) - ...
               quad_dynamics(0,x_hover,U_hover-dup,P)) / (2*eps_x);
end
fprintf('Linearisation check:  max|A-Anum| = %.3e\n', max(abs(A(:)-An(:))));
fprintf('                      max|B-Bnum| = %.3e\n\n', max(abs(B(:)-Bn(:))));

%% 5. THE FOUR DECOUPLED SUBSYSTEMS
s = tf('s');
G_alt   = 1/(P.m*s^2);        % altitude   : dU1 -> Z      <-- Task 2 plant
G_roll  = P.l/(P.Ixx*s^2);    % roll       : U2  -> phi
G_pitch = P.l/(P.Iyy*s^2);    % pitch      : U3  -> theta
G_yaw   = 1/(P.Izz*s^2);      % yaw        : U4  -> psi

sys_alt = ss(G_alt);

fprintf('Altitude plant  G(s) = 1/(m*s^2) = 1/(%.4f s^2)\n', P.m);
fprintf('  -> a DOUBLE INTEGRATOR: two poles at the origin.\n');
fprintf('  -> marginally stable, so proportional control alone CANNOT\n');
fprintf('     stabilise it. Derivative action is mandatory.\n\n');

fprintf('Open-loop poles of the full model:\n');
disp(pole(sys_full).');
fprintf('All 12 poles at the origin => the quadcopter is inherently\n');
fprintf('unstable/marginally stable and REQUIRES feedback control.\n\n');

%% 6. CONTROLLABILITY
Co = ctrb(A,B);
fprintf('rank(ctrb) = %d of %d  ->  %s\n', rank(Co), size(A,1), ...
        string(rank(Co)==size(A,1)) + " fully controllable");
fprintf('(A rank of 12 means all four channels can be commanded\n');
fprintf(' independently with the four available inputs.)\n\n');

%% 7. NONLINEAR SIMULATION - open-loop hover, then a small thrust bump
%  Demonstrates that the nonlinear model behaves physically.
tspan = [0 3];
odefun = @(t,x) quad_dynamics(t, x, ...
                 [P.W*(1 + 0.02*(t>1)); 0; 0; 0; 0], P);
[t_ol, x_ol] = ode45(odefun, tspan, x_hover);

figure('Name','Task 1 - Open-loop response','Color','w');
subplot(2,1,1);
plot(t_ol, x_ol(:,5), 'LineWidth', 1.6); grid on;
ylabel('Altitude Z [m]'); title('Open loop: +2% thrust step at t = 1 s');
subplot(2,1,2);
plot(t_ol, x_ol(:,6), 'LineWidth', 1.6); grid on;
ylabel('Zdot [m/s]'); xlabel('Time [s]');
fprintf('Open-loop: a 2%% thrust error makes altitude diverge as t^2.\n');
fprintf('That divergence is the reason Task 2 exists.\n\n');

save('task1_model.mat','P','A','B','C','D','sys_full','sys_alt','G_alt');
fprintf('Saved -> task1_model.mat\n');
