%% TASK 2 - Altitude control system for the 6-DOF quadcopter using PID
%  HACKSIMUL8 2026, PES University
%
%  The problem statement asks for "a neat description of the methodology used
%  for tuning" and to "use the best method available". This script therefore
%  implements FOUR tuning methods, compares them on identical test cases, and
%  produces the numbers you quote to the judges.
%
%  Run task1_statespace.m first.

clear; clc; close all;
P = quad_params();
s = tf('s');

fprintf('=== TASK 2: Altitude PID Control ===\n\n');

%% ------------------------------------------------------------------------
%  STEP 1 - FEEDBACK LINEARISATION  (this is the key design decision)
%
%  The true altitude dynamics are NONLINEAR and coupled to attitude:
%       zddot = -g + (U1/m)*cos(phi)*cos(theta)
%
%  Instead of linearising and hoping the angles stay small, cancel the
%  nonlinearity exactly. Choose the thrust command as
%
%       U1 = m*(g + u_z) / (cos(phi)*cos(theta))
%
%  Substituting gives, EXACTLY and for any attitude:
%       zddot = u_z
%
%  So the plant seen by the PID is a clean double integrator 1/s^2, and the
%  design stays valid at large roll/pitch angles - not just near hover.
%  The m*g term is gravity feed-forward; the 1/(cos*cos) term compensates the
%  thrust lost to tilt.
%
%  THIS IS THE HEADLINE OF YOUR TASK 2 PRESENTATION.
% -------------------------------------------------------------------------

G = 1/s^2;                  % plant after feedback linearisation
G_raw = 1/(P.m*s^2);        % plant WITHOUT feedback linearisation

fprintf('After feedback linearisation the altitude plant is 1/s^2.\n');
fprintf('Poles: %s  -> double integrator, marginally stable.\n\n', ...
        mat2str(pole(G).'));

%% ------------------------------------------------------------------------
%  STEP 2 - WHEN ZIEGLER-NICHOLS IS AND IS NOT WELL-POSED
%
%  The ZN ultimate-gain method needs a gain Ku at which the loop sustains a
%  constant-amplitude oscillation. For a PURE double integrator under
%  proportional-only control the closed loop is
%       m*s^2 + Kp = 0   ->   poles at +/- j*sqrt(Kp/m)
%  purely imaginary for EVERY Kp > 0. The loop oscillates at any gain, so no
%  unique Ku exists and classical ZN is degenerate here. The open-loop
%  reaction-curve variant fails too: a double integrator has no S-curve.
%
%  IMPORTANT QUALIFICATION - do not overstate this to the judges.
%  The reference the organisers cite, Mien & Tu (2024) IJRCS 4(4), DOES apply
%  ZN successfully to this same quadcopter. The difference is in the model:
%  their Eq. (6) includes air resistance,  m*z'' + Az*z' + m*g = F,  which
%  puts one pole at -Az/m instead of the origin, so a finite ultimate gain
%  exists. They measure kth = 124.99 and tau_th = 3.52 s (their Fig. 5).
%
%  Table 1 - in our problem statement AND in the paper - lists no values for
%  Ax, Ay, Az. Setting them to zero is the only consistent reading of the
%  data we were given, and that is precisely what makes our altitude channel
%  a pure double integrator.
%
%  Honest statement for the judges: "for the plant as specified by Table 1
%  the ZN ultimate-gain method is degenerate; the reference avoids this
%  because its model retains aerodynamic drag."
% -------------------------------------------------------------------------
fprintf('Ziegler-Nichols on THIS plant (as specified by Table 1):\n');
fprintf('  Table 1 gives no air-resistance coefficients, so Az = 0 and the\n');
fprintf('  altitude channel is a PURE double integrator. P-only control gives\n');
fprintf('  m*s^2 + Kp = 0, purely imaginary roots for every Kp > 0, so the\n');
fprintf('  loop oscillates at any gain and no unique Ku exists.\n');
fprintf('  QUALIFICATION: Mien & Tu (2024) DO apply ZN successfully. Their\n');
fprintf('  Eq. (6) carries drag terms Ax,Ay,Az which move one pole off the\n');
fprintf('  origin, making Ku finite - they measure kth=124.99, tau=3.52 s.\n');
fprintf('  See task2b_zn_tyreus_luyben.m for the like-for-like benchmark.\n\n');

%% ------------------------------------------------------------------------
%  STEP 3 - METHOD A: ANALYTICAL POLE PLACEMENT (full 3rd-order PID)
%
%  A PID acting on the double integrator zddot = u_z gives
%       zddot = Kp*e + Ki*int(e) - Kd*zdot
%  so the closed-loop characteristic polynomial is THIRD order:
%       s^3 + Kd*s^2 + Kp*s + Ki = 0
%
%  IMPORTANT: do NOT design Kp and Kd as a PD and then add Ki afterwards.
%  Integral action shifts the closed-loop poles, so the damping you designed
%  for is destroyed. (Doing exactly that gave 20% overshoot and no settling.)
%  Instead place all three poles at once.
%
%  Choose a dominant complex pair plus one faster real pole at s = -p:
%       (s^2 + 2*zeta*wn*s + wn^2)(s + p)
%     =  s^3 + (2*zeta*wn + p)s^2 + (wn^2 + 2*zeta*wn*p)s + wn^2*p
%
%  Matching coefficients gives the gains in closed form:
%       Kd = 2*zeta*wn + p
%       Kp = wn^2 + 2*zeta*wn*p
%       Ki = wn^2 * p
%
%  Taking p = 2*zeta*wn puts the real pole twice as fast as the pair's
%  envelope: fast enough not to dominate the response, slow enough to keep
%  the gains inside what the rotors can actually deliver.
% -------------------------------------------------------------------------
zeta_d = 0.9;                    % damping ratio of the dominant pair
Ts_d   = 2.0;                    % desired 2% settling time [s]
wn_d   = 4/(zeta_d*Ts_d);        % -> wn from Ts = 4/(zeta*wn)
p_d    = 2*zeta_d*wn_d;          % the extra real pole

Kd_A = 2*zeta_d*wn_d + p_d;
Kp_A = wn_d^2 + 2*zeta_d*wn_d*p_d;
Ki_A = wn_d^2 * p_d;

% Sanity check: the initial acceleration demand must be inside what the
% rotors can produce, otherwise the design saturates on every step.
a_max    = P.U1_max/P.m - P.g;           % max upward acceleration [m/s^2]
a_demand = Kp_A * 1.0;                   % Kp*e for a 1 m step command

fprintf('METHOD A - Analytical pole placement (3rd-order PID)\n');
fprintf('  spec: zeta = %.2f, Ts = %.1f s -> wn = %.3f rad/s, p = %.3f\n', ...
        zeta_d, Ts_d, wn_d, p_d);
fprintf('  Kp = %.4f   Ki = %.4f   Kd = %.4f\n', Kp_A, Ki_A, Kd_A);
fprintf('  closed-loop poles: %s\n', mat2str(roots([1 Kd_A Kp_A Ki_A]).', 4));
fprintf('  initial accel demand %.1f m/s^2 vs %.1f available -> %s\n\n', ...
        a_demand, a_max, string(a_demand < a_max));

%% ------------------------------------------------------------------------
%  STEP 4 - METHOD B: MATLAB AUTOMATIC TUNING (pidtune)
% -------------------------------------------------------------------------
[C_B, info_B] = pidtune(G, 'PIDF');
Kp_B = C_B.Kp; Ki_B = C_B.Ki; Kd_B = C_B.Kd;

fprintf('METHOD B - pidtune (automatic)\n');
fprintf('  Kp = %.4f   Ki = %.4f   Kd = %.4f\n', Kp_B, Ki_B, Kd_B);
fprintf('  crossover = %.3f rad/s, phase margin = %.1f deg, stable = %d\n\n', ...
        info_B.CrossoverFrequency, info_B.PhaseMargin, info_B.Stable);

%% ------------------------------------------------------------------------
%  STEP 5 - METHOD C: pidtune WITH A TARGET BANDWIDTH
%  Push the loop faster by specifying the crossover frequency directly.
% -------------------------------------------------------------------------
wc_target = 2.5;
[C_C, info_C] = pidtune(G, 'PIDF', wc_target);
Kp_C = C_C.Kp; Ki_C = C_C.Ki; Kd_C = C_C.Kd;

fprintf('METHOD C - pidtune with target wc = %.1f rad/s\n', wc_target);
fprintf('  Kp = %.4f   Ki = %.4f   Kd = %.4f\n', Kp_C, Ki_C, Kd_C);
fprintf('  phase margin = %.1f deg\n\n', info_C.PhaseMargin);

%% ------------------------------------------------------------------------
%  STEP 6 - METHOD D: OPTIMISATION AGAINST ITAE  <-- "the best method"
%
%  Directly minimise a performance index on the FULL NONLINEAR model, with
%  actuator saturation and anti-windup active. This is the honest answer to
%  "use the best method available": the other three optimise a linear
%  approximation, this one optimises what actually flies.
%
%  Cost = ITAE + penalties on overshoot and control effort.
%  Uses fminsearch, so no Optimization Toolbox licence is required.
% -------------------------------------------------------------------------
fprintf('METHOD D - ITAE optimisation on the nonlinear model...\n');

z_ref  = 1.0;                       % 1 m step command
Tsim   = 8;
gains0 = [Kp_A Ki_A Kd_A];

costfun = @(g) alt_cost(abs(g), P, z_ref, Tsim);
opts    = optimset('Display','off','MaxIter',260,'MaxFunEvals',520, ...
                   'TolX',1e-4,'TolFun',1e-4);
g_opt   = abs(fminsearch(costfun, gains0, opts));

Kp_D = g_opt(1); Ki_D = g_opt(2); Kd_D = g_opt(3);
fprintf('  Kp = %.4f   Ki = %.4f   Kd = %.4f\n\n', Kp_D, Ki_D, Kd_D);

%% ------------------------------------------------------------------------
%  STEP 7 - COMPARE ALL FOUR ON THE NONLINEAR MODEL
% -------------------------------------------------------------------------
methods = { 'A: Pole placement', [Kp_A Ki_A Kd_A]
            'B: pidtune',        [Kp_B Ki_B Kd_B]
            'C: pidtune @ wc',   [Kp_C Ki_C Kd_C]
            'D: ITAE optimised', [Kp_D Ki_D Kd_D] };

figure('Name','Task 2 - Altitude PID tuning comparison','Color','w', ...
       'Position',[80 80 1000 620]);

results = struct([]);
for i = 1:size(methods,1)
    g = methods{i,2};
    R = sim_altitude(g, P, z_ref, Tsim);
    M = perf_metrics(R.t, R.z, z_ref, R.U1);

    results(i).name = methods{i,1};
    results(i).gains = g;
    results(i).M = M;

    subplot(2,1,1); hold on; grid on;
    plot(R.t, R.z, 'LineWidth', 1.6, 'DisplayName', methods{i,1});
    subplot(2,1,2); hold on; grid on;
    plot(R.t, R.U1, 'LineWidth', 1.4, 'DisplayName', methods{i,1});
end

subplot(2,1,1);
yline(z_ref,'k--','DisplayName','reference');
ylabel('Altitude Z [m]'); legend('Location','southeast');
title('Task 2: altitude step response, nonlinear model with actuator limits');

subplot(2,1,2);
yline(P.U1_max,'r--','DisplayName','saturation');
yline(P.W,'k:','DisplayName','hover thrust');
ylabel('Thrust U1 [N]'); xlabel('Time [s]'); legend('Location','northeast');

%% ------------------------------------------------------------------------
%  STEP 8 - RESULTS TABLE  (these are the numbers you quote)
% -------------------------------------------------------------------------
fprintf('\n%-20s %8s %8s %8s %9s %9s %9s %9s\n', ...
        'Method','Kp','Ki','Kd','Tr[s]','Ts[s]','OS[%]','ITAE');
fprintf('%s\n', repmat('-',1,92));
for i = 1:numel(results)
    fprintf('%-20s %8.3f %8.3f %8.3f %9.3f %9.3f %9.2f %9.4f\n', ...
        results(i).name, results(i).gains(1), results(i).gains(2), ...
        results(i).gains(3), results(i).M.Tr, results(i).M.Ts, ...
        results(i).M.OS, results(i).M.ITAE);
end

[~,best] = min(arrayfun(@(r) r.M.ITAE, results));
fprintf('\nBest by ITAE: %s\n', results(best).name);

PID_alt = results(best).gains;
fprintf('Selected gains: Kp = %.4f, Ki = %.4f, Kd = %.4f\n\n', PID_alt);

%% ------------------------------------------------------------------------
%  STEP 9 - ROBUSTNESS: WHY TASK 3 IS NEEDED
%  Fixed gains tuned at the nominal mass degrade when the mass changes
%  (payload pickup) or wind hits. Show this - it motivates the ML self-tuning.
% -------------------------------------------------------------------------
figure('Name','Task 2 - Fixed gains lose performance off-nominal','Color','w');
mass_ratios = [1.0 1.4 1.8 2.2];
hold on; grid on;
for mr = mass_ratios
    P2 = P; P2.m = P.m*mr; P2.W = P2.m*P2.g;
    R = sim_altitude(PID_alt, P2, z_ref, Tsim);
    M = perf_metrics(R.t, R.z, z_ref, R.U1);
    plot(R.t, R.z, 'LineWidth',1.6, ...
        'DisplayName',sprintf('m x%.1f  (OS %.1f%%, Ts %.2fs)', mr, M.OS, M.Ts));
end
yline(z_ref,'k--','HandleVisibility','off');
xlabel('Time [s]'); ylabel('Altitude Z [m]'); legend('Location','southeast');
title('Fixed PID gains: performance degrades as mass changes -> motivates Task 3');

save('task2_pid.mat','PID_alt','results','P');
fprintf('Saved -> task2_pid.mat\n');
fprintf('\nMETHODOLOGY SUMMARY FOR THE JUDGES:\n');
fprintf(' 1. Feedback-linearise the altitude channel -> exact double integrator\n');
fprintf(' 2. Note that classical Ziegler-Nichols is DEGENERATE for this plant\n');
fprintf('    as specified by Table 1 (no drag given -> pure double integrator),\n');
fprintf('    while the reference paper avoids this by retaining drag terms\n');
fprintf(' 3. Get a principled starting point from analytical pole placement\n');
fprintf(' 4. Refine with pidtune (linear, fast) \n');
fprintf(' 5. Finish with ITAE optimisation on the NONLINEAR model including\n');
fprintf('    actuator saturation and anti-windup  <- the best method\n');
fprintf(' 6. Validate across mass and disturbance variation -> motivates Task 3\n');
