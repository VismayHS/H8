%% TASK 3c - Re-label the dataset under the corrected cost, warm-started.
%
%  WHY: the first label set was generated with an overshoot weight of 0.05.
%  ITAE then dominated the objective and the optimiser bought speed with
%  huge overshoot - the resulting controller flew to 1.62 m on a 1.00 m
%  command. Overshoot weight is now 0.30 in alt_cost.m.
%
%  Rather than repeat 32 minutes of cold-start optimisation, each condition
%  is re-optimised STARTING FROM ITS PREVIOUS OPTIMUM. The cost changed only
%  moderately, so the old optimum is close to the new one and fminsearch
%  converges in far fewer iterations.
%
%  The operating conditions are regenerated with the SAME rng seed as
%  task3_generate_data_train_ml.m, so condition i here is condition i there.

clear; clc; rng(42);
P0 = quad_params();
D  = load('task3_dataset.mat');
Xold = D.X;  Yold = D.Y;  featNames = D.featNames;  PID_base = D.PID_base;

N = 150;                       % must match the original run
mass_ratio = 1.0 + 1.4*rand(N,1);
wind_amp   =      2.0*rand(N,1) - 1.0;
gust_amp   =      1.5*rand(N,1);
step_size  = 0.5 + 1.5*rand(N,1);
tilt_amp   =      0.35*rand(N,1);

Tsim_id  = 5;
Tsim_opt = 8;
nsub_ds  = 4;
GAIN_CAP = 400;

nOld = size(Yold,1);
fprintf('=== TASK 3c: re-label under corrected cost (w_OS 0.05 -> 0.30) ===\n');
fprintf('warm-starting from %d previous optima\n\n', nOld);

X = zeros(N,8); Y = zeros(N,3); COND = zeros(N,5); ok = false(N,1);

% ---- parallel pool -----------------------------------------------------
nW = 0;
try
    pp = gcp('nocreate');
    if isempty(pp), pp = parpool('Processes'); end
    nW = pp.NumWorkers;
catch ME
    fprintf('parpool unavailable (%s) - running serially\n', ME.message);
end
fprintf('workers: %d\n\n', nW);

t0 = tic;

parfor (i = 1:N, nW)
    P = P0;
    P.m = P0.m * mass_ratio(i);
    P.W = P.m * P.g;

    wa = wind_amp(i); ga = gust_amp(i);
    opt = struct();
    opt.dist       = @(t) wa + ga*(t > 3.0);
    opt.tilt_theta = @(t) tilt_amp(i)*(t > 1.5);
    opt.tilt_phi   = @(t) 0;
    opt.nsub       = nsub_ds;
    opt.m_ctrl     = P0.m;
    zref = step_size(i);

    Rb = sim_altitude(PID_base, P, zref, Tsim_id, opt);
    if any(~isfinite(Rb.z)) || max(abs(Rb.z)) > 50, continue; end
    Mb = perf_metrics(Rb.t, Rb.z, zref, Rb.U1);

    nSS = max(1, round(0.2*numel(Rb.U1)));
    % inline the NaN guards: local functions are not reliably visible to
    % parfor workers when the file is a script
    tr_ = Mb.Tr; if ~isfinite(tr_), tr_ = 3;        end
    ts_ = Mb.Ts; if ~isfinite(ts_), ts_ = Tsim_id;  end
    X(i,:) = [ Mb.OS/100, tr_, ts_, Mb.SSE/zref, ...
               Mb.IAE/zref, mean(Rb.U1(end-nSS+1:end))/P0.W, ...
               Mb.Umax/P0.W, zref ];

    % warm start from the old optimum for this condition
    if i <= nOld, seed = Yold(i,:); else, seed = [PID_base(1) 5 PID_base(3)]; end
    seed = max(seed, [0.5 0.0 0.5]);

    cf = @(g) alt_cost(abs(g), P, zref, Tsim_opt, opt);
    o  = optimset('Display','off','MaxIter',45,'MaxFunEvals',90, ...
                  'TolX',1e-3,'TolFun',1e-3);
    g1 = abs(fminsearch(cf, seed, o));

    % one gentler alternative start, in case the old optimum was a bad basin
    g2 = abs(fminsearch(cf, [seed(1)*0.6, seed(2)*0.5, seed(3)], o));
    if cf(g2) < cf(g1), g1 = g2; end

    if all(isfinite(g1)) && all(g1 <= GAIN_CAP)
        Y(i,:) = g1;
        COND(i,:) = [mass_ratio(i) wind_amp(i) gust_amp(i) step_size(i) tilt_amp(i)];
        ok(i) = true;
    end

end
fprintf('  optimisation finished in %.0f s\n', toc(t0));

X = X(ok,:); Y = Y(ok,:); COND = COND(ok,:);
fprintf('\nusable: %d of %d in %.0f s\n', size(X,1), N, toc(t0));

%% how much did the labels improve?
fprintf('\nOvershoot of the LABELLED controllers (sampled 30 conditions):\n');
osOld = []; osNew = [];
for i = 1:min(30, size(Y,1))
    P = P0; P.m = P0.m*COND(i,1); P.W = P.m*P.g;
    wa=COND(i,2); ga=COND(i,3);
    opt = struct('dist',@(t) wa+ga*(t>3), 'tilt_theta',@(t) COND(i,5)*(t>1.5), ...
                 'nsub',4, 'm_ctrl',P0.m);
    zr = COND(i,4);
    Rn = sim_altitude(Y(i,:), P, zr, 8, opt);
    Mn = perf_metrics(Rn.t, Rn.z, zr, Rn.U1);
    osNew(end+1) = Mn.OS; %#ok<SAGROW>
    if i <= nOld
        Ro = sim_altitude(Yold(i,:), P, zr, 8, opt);
        Mo = perf_metrics(Ro.t, Ro.z, zr, Ro.U1);
        osOld(end+1) = Mo.OS; %#ok<SAGROW>
    end
end
fprintf('  old labels: mean OS %.1f%%, max %.1f%%\n', mean(osOld), max(osOld));
fprintf('  new labels: mean OS %.1f%%, max %.1f%%\n', mean(osNew), max(osNew));

save('task3_dataset.mat','X','Y','COND','featNames','PID_base','P0');
fprintf('\nSaved -> task3_dataset.mat (relabelled)\n');

