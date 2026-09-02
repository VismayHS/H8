%% VERIFY_ALL - independent re-verification of everything claimed so far.
%  Reloads saved results and re-derives them from scratch, rather than
%  trusting the run that produced them.

fprintf('\n########## INDEPENDENT VERIFICATION ##########\n');
ok = 0; bad = 0;
function_handle_dummy = [];  %#ok<NASGU>

%% A - parameters against Table 1 of the problem statement
fprintf('\n[A] Table 1 parameters\n');
P = quad_params();
tbl = {'m',0.516,P.m; 'l',0.225,P.l; 'g',9.81,P.g; 'IM',3.368e-5,P.IM; ...
       'k',2.996e-6,P.k; 'b',1.260e-7,P.b; 'Ixx',4.984e-3,P.Ixx; ...
       'Iyy',4.984e-3,P.Iyy; 'Izz',8.958e-3,P.Izz};
for i = 1:size(tbl,1)
    match = abs(tbl{i,2}-tbl{i,3}) < 1e-12;
    fprintf('    %-5s expected %-12.6g got %-12.6g  %s\n', ...
        tbl{i,1}, tbl{i,2}, tbl{i,3}, string(match));
    if match, ok=ok+1; else, bad=bad+1; end
end

%% B - re-derive the linearisation from scratch
fprintf('\n[B] Re-derive A,B by finite differences and compare to saved\n');
S = load('task1_model.mat');
x0 = zeros(12,1); U0 = [P.W;0;0;0;0]; h = 1e-6;
An = zeros(12); Bn = zeros(12,4);
for i=1:12
    d=zeros(12,1); d(i)=h;
    An(:,i)=(quad_dynamics(0,x0+d,U0,P)-quad_dynamics(0,x0-d,U0,P))/(2*h);
end
for i=1:4
    d=zeros(5,1); d(i)=h;
    Bn(:,i)=(quad_dynamics(0,x0,U0+d,P)-quad_dynamics(0,x0,U0-d,P))/(2*h);
end
eA = max(abs(S.A(:)-An(:))); eB = max(abs(S.B(:)-Bn(:)));
fprintf('    max|A_saved - A_fresh| = %.3e   %s\n', eA, string(eA<1e-6));
fprintf('    max|B_saved - B_fresh| = %.3e   %s\n', eB, string(eB<1e-6));
if eA<1e-6 && eB<1e-6, ok=ok+2; else, bad=bad+2; end

%% C - key structural entries of A and B (the physics)
fprintf('\n[C] Structural entries carry the right physics\n');
chk = {'A(2,9)  = +g   (Xddot = g*theta)',  S.A(2,9),  P.g;
       'A(4,7)  = -g   (Yddot = -g*phi)',   S.A(4,7), -P.g;
       'B(6,1)  = 1/m  (Zddot = dU1/m)',    S.B(6,1),  1/P.m;
       'B(8,2)  = l/Ixx',                   S.B(8,2),  P.l/P.Ixx;
       'B(10,3) = l/Iyy',                   S.B(10,3), P.l/P.Iyy;
       'B(12,4) = 1/Izz',                   S.B(12,4), 1/P.Izz};
for i=1:size(chk,1)
    m = abs(chk{i,2}-chk{i,3}) < 1e-9;
    fprintf('    %-34s %10.4f  %s\n', chk{i,1}, chk{i,2}, string(m));
    if m, ok=ok+1; else, bad=bad+1; end
end

%% D - controllability and poles
fprintf('\n[D] Controllability and open-loop poles\n');
r = rank(ctrb(S.A,S.B));
pol = eig(S.A);
fprintf('    rank(ctrb) = %d of 12          %s\n', r, string(r==12));
fprintf('    all poles at origin?           %s\n', string(max(abs(pol))<1e-12));
if r==12, ok=ok+1; else, bad=bad+1; end

%% E - Task 2 gains reproduce the claimed performance
fprintf('\n[E] Task 2 gains re-simulated\n');
T2 = load('task2_pid.mat');
g = T2.PID_alt;
fprintf('    saved gains: Kp=%.4f Ki=%.4f Kd=%.4f\n', g);
R = sim_altitude(g, P, 1.0, 8);
M = perf_metrics(R.t, R.z, 1.0, R.U1);
fprintf('    Tr=%.3f  Ts=%.3f  OS=%.2f%%  ITAE=%.4f  z_end=%.4f\n', ...
        M.Tr, M.Ts, M.OS, M.ITAE, R.z(end));
good = abs(R.z(end)-1) < 0.02 && M.OS < 2;
fprintf('    tracks to 1 m with <2%% overshoot:  %s\n', string(good));
if good, ok=ok+1; else, bad=bad+1; end

%% F - feedback linearisation under large tilt
fprintf('\n[F] Feedback linearisation at large tilt\n');
for tilt = [0 0.15 0.30 0.45]
    o = struct('tilt_theta', @(t) tilt*(t>3));
    Rt = sim_altitude(g, P, 1.0, 8, o);
    sag = max(abs(Rt.z(Rt.t>3.5)-1));
    fprintf('    pitch %.2f rad (%4.1f deg): max sag = %.6f m\n', ...
            tilt, rad2deg(tilt), sag);
    if sag < 0.01, ok=ok+1; else, bad=bad+1; end
end

%% G - saturation actually engages and is respected
fprintf('\n[G] Actuator saturation respected\n');
viol = any(R.U1 > P.U1_max+1e-9) || any(R.U1 < P.U1_min-1e-9);
fprintf('    U1 range [%.3f, %.3f], limits [%.3f, %.3f]\n', ...
        min(R.U1), max(R.U1), P.U1_min, P.U1_max);
fprintf('    no limit violation:            %s\n', string(~viol));
if ~viol, ok=ok+1; else, bad=bad+1; end

%% H - Simulink model exists, loads and simulates
fprintf('\n[H] Simulink model\n');
try
    if isfile('quad_altitude.slx')
        out = sim('quad_altitude');
        zl = out.z_log; t = zl.time; z = zl.signals.values;
        fprintf('    quad_altitude.slx simulates: %d samples\n', numel(t));
        fprintf('    z final = %.4f m               %s\n', z(end), ...
                string(abs(z(end)-1)<0.01));
        sagS = max(abs(z(t>4.2)-1));
        fprintf('    sag after 0.3 rad pitch = %.6f m\n', sagS);
        ok = ok+1;
    else
        fprintf('    MISSING quad_altitude.slx\n'); bad=bad+1;
    end
catch ME
    fprintf('    FAIL %s\n', ME.message); bad=bad+1;
end

fprintf('\n########## VERIFICATION: %d passed, %d failed ##########\n\n', ok, bad);
