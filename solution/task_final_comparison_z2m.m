%% FINAL COMPARISON - everything, at the paper's exact z = 2.0 m benchmark
%
%  The reference paper (Mien & Tu, 2024) designs and evaluates its three
%  heuristic controllers at a single setpoint: zd = 2.0 m. task2b_zn_tyreus_
%  luyben.m already benchmarks our FIXED PID against their three methods at
%  that exact setpoint, in undisturbed flight.
%
%  What was missing: our ML SELF-TUNER, at the same z = 2.0 m setpoint,
%  under a realistic disturbed condition (payload + wind) - because that is
%  precisely the situation where self-tuning earns its value over a fixed
%  PID. This script puts every design head to head, on identical terms:
%
%      1. Ziegler-Nichols       (Mien & Tu, published gains)
%      2. Tyreus-Luyben         (Mien & Tu, published gains)
%      3. MATLAB PID Tuner      (Mien & Tu, published gains)
%      4. OUR fixed PID         (Task 2, ITAE-optimised, feedback-linearised)
%      5. OUR ML self-tuner     (Task 3/4, same starting gains as #4)
%
%  Two scenarios, both at zd = 2.0 m:
%      (a) NOMINAL   - nominal mass, no wind (matches the paper's own test)
%      (b) DISTURBED - moderate payload + wind (the case that motivates ML)

clear; clc; close all;
P0 = quad_params();
zref = 2.0;      % the paper's exact benchmark setpoint
Tsim = 20;

fprintf('=== FINAL COMPARISON: all five designs at z = %.1f m ===\n\n', zref);

% ---- the paper's published gains (reproduced and verified in task2b) ----
ZN = [74.9940, 42.6102, 32.9974];
TL = [56.8136,  7.3365, 31.7435];
PT = [57.0050,  0.0010, 23.1020];

L2 = load('task2_pid.mat'); OURS_FIXED = L2.PID_alt;
L3 = load('task3_model.mat'); predict_gains = L3.predict_gains;
LD = load('task3_dataset.mat'); Y_lo = min(LD.Y,[],1); Y_hi = max(LD.Y,[],1);

scenarios = { ...
  'NOMINAL (matches the paper''s own test)',  1.0, 0.0, 0.0, 0.0;
  'DISTURBED (payload x1.6 + 0.5N wind)',     1.6, 0.5, 0.0, 0.15 };

for s = 1:size(scenarios,1)
    fprintf('\n########## SCENARIO %d: %s ##########\n', s, scenarios{s,1});
    mass_ratio = scenarios{s,2}; wind = scenarios{s,3};
    gust = scenarios{s,4}; tilt = scenarios{s,5};

    P = P0; P.m = P0.m*mass_ratio; P.W = P.m*P.g;
    opt_plain = struct('use_fbl', false, 'dist',@(t) wind+gust*(t>10), ...
                        'tilt_theta',@(t) tilt*(t>3), 'm_ctrl', P0.m);
    opt_fbl   = struct('use_fbl', true,  'dist',@(t) wind+gust*(t>10), ...
                        'tilt_theta',@(t) tilt*(t>3), 'm_ctrl', P0.m);

    designs = { 'Ziegler-Nichols (Mien & Tu)',  ZN,         opt_plain;
                'Tyreus-Luyben (Mien & Tu)',    TL,         opt_plain;
                'MATLAB PID Tuner (Mien & Tu)', PT,         opt_plain;
                'OURS: fixed PID',              OURS_FIXED, opt_fbl };

    fprintf('%-30s %8s %8s %8s %8s %8s %9s\n', ...
            'Design','Kp','Ki','Kd','Ts[s]','OS[%]','ITAE');
    fprintf('%s\n', repmat('-',1,84));

    results = {};
    for i = 1:size(designs,1)
        g = designs{i,2}; o = designs{i,3};
        R = sim_altitude(g, P, zref, Tsim, o);
        M = perf_metrics(R.t, R.z, zref, R.U1);
        fprintf('%-30s %8.3f %8.3f %8.3f %8.3f %8.2f %9.4f\n', ...
                designs{i,1}, g, M.Ts, M.OS, M.ITAE);
        results{end+1} = struct('name',designs{i,1},'R',R,'M',M); %#ok<SAGROW>
    end

    % ---- OUR ML SELF-TUNER: identify for 3s, then adapt ----
    t_id = 3.0;
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
    fprintf('%-30s %8.3f %8.3f %8.3f %8.3f %8.2f %9.4f\n', ...
            'OURS: ML self-tuner', g_ml, M_ml.Ts, M_ml.OS, M_ml.ITAE);
    results{end+1} = struct('name','OURS: ML self-tuner', ...
                             'R',struct('t',Rt,'z',Rz,'U1',Ru),'M',M_ml);
    fprintf('%s\n', repmat('-',1,84));

    if s == 2
        impr = 100*(results{4}.M.ITAE - M_ml.ITAE)/results{4}.M.ITAE;
        fprintf('ML self-tuner vs OUR OWN fixed PID, same disturbed scenario: %+.1f%%\n', impr);
    end

    % ---- figure ----
    set(0,'DefaultFigureColor','w');
    f = figure('Color','w','Position',[60 60 950 500]);
    hold on; grid on;
    for i = 1:numel(results)
        plot(results{i}.R.t, results{i}.R.z, 'LineWidth', 1.6, ...
             'DisplayName', sprintf('%s (ITAE %.3f)', results{i}.name, results{i}.M.ITAE));
    end
    yline(zref,'k--','DisplayName','reference');
    xlabel('Time [s]'); ylabel('Altitude [m]'); legend('Location','southeast','FontSize',8);
    title(sprintf('All five designs at z=%.1f m: %s', zref, scenarios{s,1}));
    if ~isfolder('figures'), mkdir('figures'); end
    fname = sprintf('figures/09_final_comparison_scenario%d.png', s);
    exportgraphics(f, fname, 'Resolution', 150);
    fprintf('Saved -> %s\n', fname);
end

fprintf('\n=== DONE ===\n');
