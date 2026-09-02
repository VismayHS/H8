%% TASK 4 STRESS TEST - how does the self-tuner do across MANY conditions?
%
%  The 5 curated test cases in task4_test_ml_selftuning.m are useful for a
%  presentation, but 5 points cannot tell you whether the model generalises
%  or whether those 5 happened to land well. This script draws N random
%  conditions from across the operating envelope - held out from BOTH the
%  Task 3 training draw (rng 42) and the Task 4 test-case draw (rng 7) - and
%  compares Fixed PID vs ML self-tuner on every one.
%
%  The oracle is DELIBERATELY OMITTED here: a per-case multi-start
%  optimisation over N=100+ conditions would take far longer than is
%  worthwhile for a broad sweep. This script answers "does it generalise
%  and how often does it help", not "how close to optimal" - that question
%  is already answered precisely for the 5 curated cases in Task 4.
%
%  ADAPTATION RULE - kept in exact sync with task4_test_ml_selftuning.m:
%      Kp: may rise above baseline, never fall below it, capped at the
%          training-observed maximum
%      Ki: free within the full training-observed range
%      Kd: held at the Task 2 baseline

clear; clc; close all; rng(99);   % disjoint from Task 3 (42) and Task 4 (7)

N = 120;
P0 = quad_params();
Lm = load('task3_model.mat');
predict_gains = Lm.predict_gains;
PID_base      = Lm.PID_base;
D  = load('task3_dataset.mat');
Y_lo = min(D.Y,[],1);  Y_hi = max(D.Y,[],1);

t_id = 3.0; Tsim = 12;
fprintf('=== TASK 4 STRESS TEST: %d random held-out conditions ===\n', N);
fprintf('Envelope: mass x1.0-2.4, wind +/-1N, gust 0-1.5N, step 0.5-2.0m, tilt 0-0.35rad\n\n');

mass_ratio = 1.0 + 1.4*rand(N,1);
wind_amp   =      2.0*rand(N,1) - 1.0;
gust_amp   =      1.5*rand(N,1);
step_size  = 0.5 + 1.5*rand(N,1);
tilt_amp   =      0.35*rand(N,1);
noise_amp  =      0.02*rand(N,1);          % 0-2 cm sensor noise, broader than TC5's fixed 1 cm

nWorkers = 0;
try
    pp = gcp('nocreate'); if isempty(pp), pp = parpool('Processes'); end
    nWorkers = pp.NumWorkers;
catch
end
fprintf('workers: %d\n\n', nWorkers);

impr  = nan(N,1);
osFix = nan(N,1);  osML = nan(N,1);
itaeFix = nan(N,1); itaeML = nan(N,1);
gains_ml = nan(N,3);
ok = false(N,1);

t0 = tic;
parfor (i = 1:N, nWorkers)
    P = P0; P.m = P0.m*mass_ratio(i); P.W = P.m*P.g;
    zref = step_size(i);
    opt = struct('dist',@(t) wind_amp(i) + gust_amp(i)*(t>6.0), ...
                 'tilt_theta',@(t) tilt_amp(i)*(t>1.5), 'tilt_phi',@(t) 0, ...
                 'noise_std', noise_amp(i), 'm_ctrl', P0.m, 'nsub', 4);

    R_fix = sim_altitude(PID_base, P, zref, Tsim, opt);
    if any(~isfinite(R_fix.z)) || max(abs(R_fix.z)) > 50*max(zref,1), continue; end
    M_fix = perf_metrics(R_fix.t, R_fix.z, zref, R_fix.U1);

    R1 = sim_altitude(PID_base, P, zref, t_id, opt);
    if any(~isfinite(R1.z)), continue; end
    M1 = perf_metrics(R1.t, R1.z, zref, R1.U1);
    nSS = max(1, round(0.2*numel(R1.U1)));
    tr1 = M1.Tr; if ~isfinite(tr1), tr1 = 3; end
    ts1 = M1.Ts; if ~isfinite(ts1), ts1 = t_id; end
    feat = [ M1.OS/100, tr1, ts1, M1.SSE/zref, M1.IAE/zref, ...
             mean(R1.U1(end-nSS+1:end))/P0.W, M1.Umax/P0.W, zref ];

    g_raw = predict_gains(feat);
    g_ml = PID_base;
    g_ml(1) = min(max(g_raw(1), PID_base(1)), Y_hi(1));   % Kp: up only
    g_ml(2) = min(max(g_raw(2), Y_lo(2)),      Y_hi(2));   % Ki: full range
    % Kd stays at baseline

    opt2 = opt; opt2.z0=R1.z(end); opt2.zdot0=R1.zdot(end); opt2.I0=R1.I_end; opt2.t0=t_id;
    R2 = sim_altitude(g_ml, P, zref, Tsim-t_id, opt2);
    if any(~isfinite(R2.z)), continue; end
    Rt=[R1.t(1:end-1);R2.t]; Rz=[R1.z(1:end-1);R2.z]; Ru=[R1.U1(1:end-1);R2.U1];
    M_ml = perf_metrics(Rt, Rz, zref, Ru);

    impr(i)    = 100*(M_fix.ITAE - M_ml.ITAE)/M_fix.ITAE;
    osFix(i)   = M_fix.OS;   osML(i) = M_ml.OS;
    itaeFix(i) = M_fix.ITAE; itaeML(i) = M_ml.ITAE;
    gains_ml(i,:) = g_ml;
    ok(i) = true;
end
fprintf('finished in %.0f s  (%d/%d valid runs)\n\n', toc(t0), sum(ok), N);

impr = impr(ok); osFix = osFix(ok); osML = osML(ok);
itaeFix = itaeFix(ok); itaeML = itaeML(ok); gains_ml = gains_ml(ok,:);
mr = mass_ratio(ok); wa = wind_amp(ok);

%% ---------------------------------------------------------------------
%  SUMMARY STATISTICS
% ----------------------------------------------------------------------
fprintf('=== IMPROVEMENT DISTRIBUTION (%d conditions) ===\n', numel(impr));
fprintf('  mean    : %+.1f%%\n', mean(impr));
fprintf('  median  : %+.1f%%\n', median(impr));
fprintf('  std dev : %.1f%%\n', std(impr));
fprintf('  min     : %+.1f%%\n', min(impr));
fprintf('  max     : %+.1f%%\n', max(impr));
fprintf('\n  deciles:\n');
pr = prctile(impr, [10 25 50 75 90]);
fprintf('    10th=%+.1f%%  25th=%+.1f%%  50th=%+.1f%%  75th=%+.1f%%  90th=%+.1f%%\n', pr);

nWorse = sum(impr < 0);
nNoChange = sum(abs(impr) < 1);
nBetter = sum(impr >= 1);
fprintf('\n  cases WORSE than fixed PID (impr < 0%%)  : %d / %d (%.1f%%)\n', ...
        nWorse, numel(impr), 100*nWorse/numel(impr));
fprintf('  cases ~unchanged (|impr| < 1%%)          : %d / %d (%.1f%%)\n', ...
        nNoChange, numel(impr), 100*nNoChange/numel(impr));
fprintf('  cases better by >=1%%                    : %d / %d (%.1f%%)\n', ...
        nBetter, numel(impr), 100*nBetter/numel(impr));

fprintf('\n=== OVERSHOOT ===\n');
fprintf('  ML max overshoot   : %.2f%%\n', max(osML));
fprintf('  ML mean overshoot  : %.2f%%\n', mean(osML));
fprintf('  cases with ML overshoot > 20%%: %d / %d\n', sum(osML>20), numel(osML));

%% Worst cases - inspect these by hand
[~, worstIdx] = sort(impr);
fprintf('\n=== 5 WORST CASES (lowest improvement) ===\n');
fprintf('%-6s %-6s %-8s %-8s %-9s\n','mass','wind','impr%','OS_ML%','Kp_used');
for k = 1:min(5, numel(worstIdx))
    j = worstIdx(k);
    fprintf('%-6.2f %-6.2f %-8.1f %-8.1f %-9.2f\n', mr(j), wa(j), impr(j), osML(j), gains_ml(j,1));
end

%% Does improvement correlate with mass or wind?
cc = @(a,b) subsref(corrcoef(a,b),struct('type','()','subs',{{1,2}}));
fprintf('\n=== WHAT DRIVES THE VARIATION? ===\n');
fprintf('  corr(improvement, mass ratio) = %+.3f\n', cc(mr, impr));
fprintf('  corr(improvement, wind)       = %+.3f\n', cc(wa, impr));
fprintf('  corr(overshoot,   Kp used)    = %+.3f\n', cc(gains_ml(:,1), osML));

%% Figure
set(0,'DefaultFigureColor','w');
f = figure('Color','w','Position',[80 80 1000 420]);
subplot(1,2,1);
histogram(impr, 20, 'FaceColor',[0.2 0.5 0.8]); grid on;
xlabel('ITAE improvement over fixed PID [%]'); ylabel('count');
title(sprintf('%d random held-out conditions', numel(impr)));
xline(0,'r--','LineWidth',1.5);
subplot(1,2,2);
scatter(mr, impr, 18, wa, 'filled'); grid on; colorbar;
xlabel('mass ratio'); ylabel('improvement [%]');
title('coloured by wind [N]'); yline(0,'r--');
if ~isfolder('figures'), mkdir('figures'); end
exportgraphics(f, 'figures/08_task4_stress_test.png', 'Resolution', 150);
fprintf('\nSaved -> figures/08_task4_stress_test.png\n');

save('task4_stress_results.mat','impr','osFix','osML','itaeFix','itaeML', ...
     'gains_ml','mr','wa','N');
fprintf('Saved -> task4_stress_results.mat\n');
