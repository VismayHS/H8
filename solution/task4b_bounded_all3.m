%% TASK 4b - EXPERIMENT: bound all three gains instead of freezing Kp,Kd
%
%  The shipped Task 4 freezes Kp and Kd at the Task 2 baseline because their
%  training R^2 is weak (0.2-0.5) and letting the model set them caused
%  catastrophic extrapolation (mean -168%). But the failure was specifically
%  NEGATIVE Kp, not "any deviation from baseline is bad". This experiment
%  tests a middle ground: let Kp and Kd move, but CLAMP them to the range
%  actually observed in training - same safety principle already used for Ki.
%
%  This is a non-destructive experiment. It does not touch task3_model.mat
%  or task4_test_ml_selftuning.m. Compare its printed summary against the
%  shipped result (mean +61.2%) before deciding whether to adopt it.

clear; clc; close all; rng(7);
P0 = quad_params();
Lm = load('task3_model.mat');
predict_gains = Lm.predict_gains;
PID_base      = Lm.PID_base;
D  = load('task3_dataset.mat');

lo = min(D.Y,[],1);  hi = max(D.Y,[],1);   % [Kp Ki Kd] training ranges
fprintf('Training ranges: Kp[%.2f,%.2f] Ki[%.2f,%.2f] Kd[%.2f,%.2f]\n\n', ...
        lo(1),hi(1), lo(2),hi(2), lo(3),hi(3));

t_id = 3.0; Tsim = 12;
nWorkers = 0;
try
    pp = gcp('nocreate'); if isempty(pp), pp = parpool('Processes'); end
    nWorkers = pp.NumWorkers;
catch
end

TC = struct( ...
 'name', {'TC1: Heavy payload (m x1.8)','TC2: Strong wind + gust', ...
          'TC3: Large step + sustained tilt','TC4: Combined worst case', ...
          'TC5: Sensor noise + payload'}, ...
 'mass_ratio', {1.8, 1.1, 1.3, 2.0, 1.6}, ...
 'wind',       {0.0, 1.2, 0.3, 1.0, 0.2}, ...
 'gust',       {0.0, 1.0, 0.0, 1.2, 0.3}, ...
 'zref',       {1.0, 1.0, 2.0, 1.5, 1.2}, ...
 'tilt',       {0.0, 0.10, 0.30, 0.25, 0.15}, ...
 'noise',      {0.0, 0.0,  0.0,  0.0,  0.010} );

nTC = numel(TC);
impr = zeros(1,nTC);
fprintf('%-34s %10s %10s %8s\n','Test case','ITAE fix','ITAE new','Impr%');
fprintf('%s\n', repmat('-',1,66));

for c = 1:nTC
    P = P0; P.m = P0.m*TC(c).mass_ratio; P.W = P.m*P.g;
    wv=TC(c).wind; gv=TC(c).gust; tl=TC(c).tilt; zref=TC(c).zref;
    opt = struct('dist',@(t) wv+gv*(t>6.0), 'tilt_theta',@(t) tl*(t>1.5), ...
                 'tilt_phi',@(t) 0, 'noise_std',TC(c).noise, 'm_ctrl',P0.m, 'nsub',4);

    R_fix = sim_altitude(PID_base, P, zref, Tsim, opt);
    M_fix = perf_metrics(R_fix.t, R_fix.z, zref, R_fix.U1);

    R1 = sim_altitude(PID_base, P, zref, t_id, opt);
    M1 = perf_metrics(R1.t, R1.z, zref, R1.U1);
    nSS = max(1, round(0.2*numel(R1.U1)));
    feat = [ M1.OS/100, safe(M1.Tr,3), safe(M1.Ts,t_id), M1.SSE/zref, M1.IAE/zref, ...
             mean(R1.U1(end-nSS+1:end))/P0.W, M1.Umax/P0.W, zref ];

    g_raw = predict_gains(feat);
    g_new = min(max(g_raw, lo), hi);         % bound ALL THREE, don't freeze Kp/Kd

    opt2 = opt; opt2.z0=R1.z(end); opt2.zdot0=R1.zdot(end); opt2.I0=R1.I_end; opt2.t0=t_id;
    R2 = sim_altitude(g_new, P, zref, Tsim-t_id, opt2);
    R_new.t=[R1.t(1:end-1);R2.t]; R_new.z=[R1.z(1:end-1);R2.z]; R_new.U1=[R1.U1(1:end-1);R2.U1];
    M_new = perf_metrics(R_new.t, R_new.z, zref, R_new.U1);

    impr(c) = 100*(M_fix.ITAE-M_new.ITAE)/M_fix.ITAE;
    fprintf('%-34s %10.3f %10.3f %+8.1f\n', TC(c).name, M_fix.ITAE, M_new.ITAE, impr(c));
    fprintf('   predicted (bounded): Kp=%.2f Ki=%.2f Kd=%.2f  (raw: Kp=%.2f Ki=%.2f Kd=%.2f)\n', ...
            g_new, g_raw);
end
fprintf('%s\n', repmat('-',1,66));
fprintf('MEAN IMPROVEMENT (bound all 3): %+.1f%%\n', mean(impr));
fprintf('(shipped Task 4, Ki-only:        +61.2%%  for comparison)\n');

function v = safe(v, fb)
if ~isfinite(v), v = fb; end
end
