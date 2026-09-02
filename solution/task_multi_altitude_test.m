%% MULTI-ALTITUDE TEST - does the design hold at more than one setpoint?
%
%  The reference paper (Mien & Tu, 2024) tests its controllers at exactly
%  ONE altitude command: zd = 2.0 m. task_final_comparison_z2m.m already
%  reproduces that exact point. This script checks FIVE altitude setpoints
%  - 0.5, 1.0, 1.5, 2.0 and 3.0 m - so the result is not a single lucky
%  point but demonstrated across a range.
%
%  NOTE ON THE ENVELOPE: Task 3's training data sampled reference steps of
%  0.5-2.0 m (task3_generate_data_train_ml.m, step_size variable). So:
%      0.5, 1.0, 1.5, 2.0 m  ->  INSIDE the range the ML model was trained on
%      3.0 m                  ->  OUTSIDE it (extrapolation) - included
%                                 deliberately to show what happens beyond
%                                 the trained envelope, and reported honestly
%                                 either way.
%
%  For each altitude, two designs are compared under a disturbed condition
%  (payload x1.6 + 0.5 N wind - the same scenario used in the z=2m
%  disturbed case, for consistency):
%      - OUR fixed PID   (Task 2, no adaptation)
%      - OUR ML self-tuner (Task 3/4, adapts after 3 s)
%  plus the paper's best published method (MATLAB PID Tuner, ITAE=0.3319
%  at their z=2m nominal test) as a reference line, run at the SAME
%  disturbed condition for a fair three-way comparison at every altitude.

clear; clc; close all;
P0 = quad_params();
Tsim = 20;
t_id = 3.0;

mass_ratio = 1.6; wind = 0.5; tilt = 0.15;    % same disturbed scenario throughout
P = P0; P.m = P0.m*mass_ratio; P.W = P.m*P.g;

PT = [57.0050, 0.0010, 23.1020];              % paper's best published method
L2 = load('task2_pid.mat'); OURS_FIXED = L2.PID_alt;
L3 = load('task3_model.mat'); predict_gains = L3.predict_gains;
LD = load('task3_dataset.mat'); Y_lo = min(LD.Y,[],1); Y_hi = max(LD.Y,[],1);

altitudes = [0.5 1.0 1.5 2.0 3.0];
fprintf('=== MULTI-ALTITUDE TEST (disturbed: mass x%.1f, wind %.1fN, tilt %.2f rad) ===\n\n', ...
        mass_ratio, wind, tilt);
fprintf('%-8s %-26s %8s %8s %8s\n','z_ref','Design','Ts[s]','OS[%]','ITAE');
fprintf('%s\n', repmat('-',1,64));

set(0,'DefaultFigureColor','w');
summary = struct([]);

for k = 1:numel(altitudes)
    zref = altitudes(k);
    inTraining = zref <= 2.0;

    opt_plain = struct('use_fbl', false, 'dist',@(t) wind, ...
                        'tilt_theta',@(t) tilt*(t>3), 'm_ctrl', P0.m);
    opt_fbl   = struct('use_fbl', true,  'dist',@(t) wind, ...
                        'tilt_theta',@(t) tilt*(t>3), 'm_ctrl', P0.m);

    R_pt = sim_altitude(PT, P, zref, Tsim, opt_plain);
    M_pt = perf_metrics(R_pt.t, R_pt.z, zref, R_pt.U1);
    fprintf('%-8.2f %-26s %8.3f %8.2f %8.4f\n', zref, 'PID Tuner (paper, ref.)', M_pt.Ts, M_pt.OS, M_pt.ITAE);

    R_fix = sim_altitude(OURS_FIXED, P, zref, Tsim, opt_fbl);
    M_fix = perf_metrics(R_fix.t, R_fix.z, zref, R_fix.U1);
    fprintf('%-8.2f %-26s %8.3f %8.2f %8.4f\n', zref, 'OURS: fixed PID', M_fix.Ts, M_fix.OS, M_fix.ITAE);

    R1 = sim_altitude(OURS_FIXED, P, zref, t_id, opt_fbl);
    M1 = perf_metrics(R1.t, R1.z, zref, R1.U1);
    nSS = max(1, round(0.2*numel(R1.U1)));
    tr1 = M1.Tr; if ~isfinite(tr1), tr1 = 3; end
    ts1 = M1.Ts; if ~isfinite(ts1), ts1 = t_id; end
    feat = [M1.OS/100, tr1, ts1, M1.SSE/zref, M1.IAE/zref, ...
            mean(R1.U1(end-nSS+1:end))/P0.W, M1.Umax/P0.W, zref];
    g_raw = predict_gains(feat);
    g_ml = OURS_FIXED;
    g_ml(1) = min(max(g_raw(1), OURS_FIXED(1)), Y_hi(1));
    g_ml(2) = min(max(g_raw(2), Y_lo(2)), Y_hi(2));
    opt2 = opt_fbl; opt2.z0=R1.z(end); opt2.zdot0=R1.zdot(end);
    opt2.I0=R1.I_end; opt2.t0=t_id;
    R2 = sim_altitude(g_ml, P, zref, Tsim-t_id, opt2);
    Rt=[R1.t(1:end-1);R2.t]; Rz=[R1.z(1:end-1);R2.z]; Ru=[R1.U1(1:end-1);R2.U1];
    M_ml = perf_metrics(Rt, Rz, zref, Ru);
    tag = ''; if ~inTraining, tag = ' [OUTSIDE trained range]'; end
    fprintf('%-8.2f %-26s %8.3f %8.2f %8.4f%s\n\n', zref, 'OURS: ML self-tuner', M_ml.Ts, M_ml.OS, M_ml.ITAE, tag);

    impr = 100*(M_fix.ITAE - M_ml.ITAE)/M_fix.ITAE;
    summary(k).zref = zref; summary(k).inTraining = inTraining;
    summary(k).ITAE_fix = M_fix.ITAE; summary(k).ITAE_ml = M_ml.ITAE;
    summary(k).ITAE_pt = M_pt.ITAE; summary(k).impr = impr;

    % ---- individual figure for this altitude ----
    f = figure('Color','w','Position',[60 60 850 420]);
    hold on; grid on;
    plot(R_pt.t, R_pt.z, 'LineWidth', 1.3, 'Color',[0.6 0.6 0.6], ...
         'DisplayName', sprintf('PID Tuner, paper (ITAE %.3f)', M_pt.ITAE));
    plot(R_fix.t, R_fix.z, 'LineWidth', 1.6, 'Color',[0.85 0.33 0.10], ...
         'DisplayName', sprintf('OURS fixed PID (ITAE %.3f)', M_fix.ITAE));
    plot(Rt, Rz, 'LineWidth', 1.9, 'Color',[0 0.45 0.74], ...
         'DisplayName', sprintf('OURS ML self-tuner (ITAE %.3f)', M_ml.ITAE));
    yline(zref, 'k--', 'DisplayName', 'reference');
    xline(t_id, 'm-.', 'HandleVisibility','off');
    xlabel('Time [s]'); ylabel('Altitude [m]');
    envTag = 'inside trained range (0.5-2.0 m)';
    if ~inTraining, envTag = 'OUTSIDE trained range - extrapolation'; end
    title(sprintf('z_{ref} = %.1f m  (%s)  |  ML improves %+.1f%% over our own fixed PID', ...
          zref, envTag, impr));
    legend('Location','southeast','FontSize',8);
    if ~isfolder('figures'), mkdir('figures'); end
    fname = sprintf('figures/10_altitude_%.1fm.png', zref);
    exportgraphics(f, fname, 'Resolution', 150);
    fprintf('Saved -> %s\n\n', fname);
end

fprintf('=== SUMMARY ACROSS ALL ALTITUDES ===\n');
fprintf('%-8s %-12s %10s %10s %10s %9s\n','z_ref','in-range?','ITAE fix','ITAE ML','ITAE paper','Impr%');
for k = 1:numel(summary)
    fprintf('%-8.2f %-12s %10.4f %10.4f %10.4f %+9.1f\n', summary(k).zref, ...
        string(summary(k).inTraining), summary(k).ITAE_fix, summary(k).ITAE_ml, ...
        summary(k).ITAE_pt, summary(k).impr);
end

save('task_multi_altitude_results.mat','summary','altitudes','mass_ratio','wind','tilt');
fprintf('\nSaved -> task_multi_altitude_results.mat\n');
