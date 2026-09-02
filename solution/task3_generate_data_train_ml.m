%% TASK 3 - Generate simulation data and train an ML model for PID self-tuning
%  HACKSIMUL8 2026, PES University
%
%  DESIGN DECISION - read this before you present.
%
%  There are two ways to frame "self-tuning PID with ML":
%
%   (a) Gain scheduling:  features = the operating condition (mass, wind)
%                         -> the controller must be TOLD the mass. Weak,
%                         because in flight you do not know the payload.
%
%   (b) Response-signature inference:  fly briefly with a baseline PID,
%                         MEASURE how the aircraft responded, and infer the
%                         right gains from that signature. The controller
%                         discovers the mass and disturbance from behaviour.
%                         This is genuine self-tuning.
%
%  We implement (b). The network learns the inverse map
%       observed closed-loop response  ->  optimal PID gains
%
%  Run task1_statespace.m and task2_altitude_pid.m first.

clear; clc; close all; rng(42);       % fixed seed = reproducible results
P0 = quad_params();

if isfile('task2_pid.mat')
    L = load('task2_pid.mat');  PID_base = L.PID_alt;
else
    PID_base = [4.94 1.65 4.00];      % fallback baseline gains
end
fprintf('=== TASK 3: Data generation + ML training ===\n');
fprintf('Baseline PID: Kp=%.3f Ki=%.3f Kd=%.3f\n\n', PID_base);

%% ------------------------------------------------------------------------
%  1. DEFINE THE OPERATING ENVELOPE TO SAMPLE
% -------------------------------------------------------------------------
% ---- SPEED SETTING -----------------------------------------------------
%  FAST = true  : ~150 conditions, lighter optimiser  -> about 2-4 minutes
%  FAST = false : ~300 conditions, full optimiser     -> about 6-12 minutes
%  FAST=true is what you want on a deadline. Accuracy drops a little; the
%  story and the conclusions do not change.
FAST = true;
% ------------------------------------------------------------------------

if FAST
    N = 150; optIter = 90;  optFun = 180; nsub_ds = 4;
else
    N = 300; optIter = 140; optFun = 280; nsub_ds = 8;
end
fprintf('Sampling %d operating conditions (FAST=%d)...\n', N, FAST);

mass_ratio = 1.0 + 1.4*rand(N,1);        % 1.0 .. 2.4  (payload)
wind_amp   =      2.0*rand(N,1) - 1.0;   % -1 .. +1 N  (steady vertical wind)
gust_amp   =      1.5*rand(N,1);         % 0 .. 1.5 N  (gust magnitude)
step_size  = 0.5 + 1.5*rand(N,1);        % 0.5 .. 2.0 m reference step
tilt_amp   =      0.35*rand(N,1);        % 0 .. 0.35 rad sustained tilt

Tsim_id  = 5;      % identification window (what the controller gets to see)
Tsim_opt = 8;      % window used for optimisation (long enough to show
                   % steady-state error, which is what drives Ki)

X = zeros(N, 8);   % features (response signature)
Y = zeros(N, 3);   % labels   (optimal Kp, Ki, Kd)
COND = zeros(N,5); % keep the true conditions for analysis/plots
ok = false(N,1);

GAIN_CAP = 400;    % reject only genuinely diverged optimisations
nrej_nonfinite = 0; nrej_toobig = 0; nrej_sim = 0;

% Seed the optimiser with a NON-ZERO Ki. fminsearch builds its initial
% simplex from the starting point, and a component that starts at exactly
% zero gets an almost invisible initial step - so it never explores integral
% action. Seeding Ki lets it find that integral action is the right answer
% for a mass mismatch, which is the physically correct result.
SEED = [PID_base(1), 5.0, PID_base(3)];

% Parallel across cores if Parallel Computing Toolbox is available.
usePar = license('test','Distrib_Computing_Toolbox');
if usePar
    try
        pp = gcp('nocreate');
        if isempty(pp), parpool('Processes'); end
    catch
        usePar = false;
    end
end
fprintf('Parallel: %d (%d cores)\n', usePar, feature('numcores'));

t_start = tic;
parfor (i = 1:N, 8*usePar)
    % --- build this operating condition --------------------------------
    P = P0;
    P.m = P0.m * mass_ratio(i);      % TRUE mass (plant)
    P.W = P.m * P.g;
    % NOTE: actuator limits stay at the NOMINAL airframe values, so a heavy
    % quadcopter genuinely has less thrust margin. That realism matters.
    % CRITICAL: the controller is NOT told the payload. It keeps using the
    % nominal mass in its feedback linearisation, so it is mis-scaled and
    % leaves a steady-state error - the signal the ML learns to read.

    wa = wind_amp(i); ga = gust_amp(i);
    opt = struct();
    opt.dist       = @(t) wa + ga*(t > 3.0);      % steady wind + gust at 3 s
    opt.tilt_theta = @(t) tilt_amp(i)*(t > 1.5);  % sustained pitch
    opt.tilt_phi   = @(t) 0;
    opt.nsub       = nsub_ds;                     % integration fidelity
    opt.m_ctrl     = P0.m;                        % controller assumes NOMINAL mass

    zref = step_size(i);

    % --- (i) fly with the BASELINE gains and measure the signature ------
    Rb = sim_altitude(PID_base, P, zref, Tsim_id, opt);
    if any(~isfinite(Rb.z)) || max(abs(Rb.z)) > 50
        nrej_sim = nrej_sim + 1; continue;
    end
    Mb = perf_metrics(Rb.t, Rb.z, zref, Rb.U1);

    % Feature vector - all of these are MEASURABLE in flight
    nSS   = max(1, round(0.2*numel(Rb.U1)));
    thrust_ratio = mean(Rb.U1(end-nSS+1:end)) / P0.W;  % reveals the mass
    X(i,:) = [ Mb.OS/100, ...
               safe(Mb.Tr, 3), ...
               safe(Mb.Ts, Tsim_id), ...
               Mb.SSE/zref, ...
               Mb.IAE/zref, ...
               thrust_ratio, ...
               Mb.Umax/P0.W, ...
               zref ];

    % --- (ii) find the OPTIMAL gains for this condition -----------------
    cf = @(g) alt_cost(abs(g), P, zref, Tsim_opt, opt);
    o  = optimset('Display','off','MaxIter',optIter,'MaxFunEvals',optFun, ...
                  'TolX',5e-4,'TolFun',5e-4);
    g_star = abs(fminsearch(cf, SEED, o));

    % keep only sane, converged results
    if ~all(isfinite(g_star))
        nrej_nonfinite = nrej_nonfinite + 1;
    elseif any(g_star > GAIN_CAP)
        nrej_toobig = nrej_toobig + 1;
    else
        Y(i,:)   = g_star;
        COND(i,:) = [mass_ratio(i) wind_amp(i) gust_amp(i) step_size(i) tilt_amp(i)];
        ok(i)    = true;
    end

end
fprintf('  data generation finished in %.0f s\n', toc(t_start));

X = X(ok,:);  Y = Y(ok,:);  COND = COND(ok,:);
fprintf('\nUsable samples: %d of %d  (%.0f s total)\n', size(X,1), N, toc(t_start));
fprintf('  rejected: %d diverged sim, %d non-finite gains, %d gains > %g\n\n', ...
        nrej_sim, nrej_nonfinite, nrej_toobig, GAIN_CAP);
if size(X,1) < 20
    error(['Only %d usable samples - cannot train. Check the rejection ' ...
           'counts above.'], size(X,1));
end

featNames = {'overshoot','riseTime','settleTime','SSEnorm','IAEnorm', ...
             'thrustRatio','peakThrust','stepSize'};
save('task3_dataset.mat','X','Y','COND','featNames','PID_base','P0');

%% ------------------------------------------------------------------------
%  2. TRAIN / TEST SPLIT
% -------------------------------------------------------------------------
n     = size(X,1);
idx   = randperm(n);
nTr   = round(0.70*n);
nVa   = round(0.15*n);
iTr   = idx(1:nTr);
iVa   = idx(nTr+1:nTr+nVa);
iTe   = idx(nTr+nVa+1:end);

fprintf('Split: %d train / %d validation / %d test\n\n', ...
        numel(iTr), numel(iVa), numel(iTe));

%% ------------------------------------------------------------------------
%  3. TRAIN THE MODEL
%  Primary: shallow feedforward neural network (Deep Learning Toolbox).
%  Fallback: linear least squares, so the script still runs without it.
% -------------------------------------------------------------------------
useNN = license('test','Neural_Network_Toolbox') && exist('fitnet','file')==2;

if useNN
    fprintf('Training feedforward neural network (Deep Learning Toolbox)...\n');
    hidden = [12 8];
    net = fitnet(hidden, 'trainlm');
    net.divideFcn      = 'divideind';
    net.divideParam.trainInd = iTr;
    net.divideParam.valInd   = iVa;
    net.divideParam.testInd  = iTe;
    net.trainParam.showWindow = false;
    net.trainParam.epochs     = 500;
    net.input.processFcns  = {'removeconstantrows','mapminmax'};
    net.output.processFcns = {'removeconstantrows','mapminmax'};

    [net, tr] = train(net, X', Y');
    predict_gains = @(xx) max(net(xx')', 0);
    fprintf('  architecture: %d -> %s -> %d\n', size(X,2), mat2str(hidden), size(Y,2));
    try
        fprintf('  best validation MSE %.5f at epoch %d\n\n', ...
                tr.best_vperf, tr.best_epoch);
    catch
        fprintf('  (training record unavailable)\n\n');
    end
    modelType = sprintf('feedforward NN %s', mat2str(hidden));
else
    fprintf('Deep Learning Toolbox not available - using linear regression.\n\n');
    Xa = [ones(numel(iTr),1) X(iTr,:)];
    Wls = Xa \ Y(iTr,:);
    predict_gains = @(xx) max([ones(size(xx,1),1) xx]*Wls, 0);
    net = Wls;
    modelType = 'linear least squares';
end

%% ------------------------------------------------------------------------
%  4. EVALUATE THE REGRESSION
% -------------------------------------------------------------------------
Yhat_te = predict_gains(X(iTe,:));
Yte     = Y(iTe,:);

gnames = {'Kp','Ki','Kd'};
fprintf('Test-set regression accuracy (%s):\n', modelType);
fprintf('%-6s %10s %10s %10s\n','gain','RMSE','MAE','R^2');
fprintf('%s\n', repmat('-',1,40));
for j = 1:3
    err  = Yhat_te(:,j) - Yte(:,j);
    rmse = sqrt(mean(err.^2));
    mae  = mean(abs(err));
    R2   = 1 - sum(err.^2)/sum((Yte(:,j)-mean(Yte(:,j))).^2);
    fprintf('%-6s %10.4f %10.4f %10.4f\n', gnames{j}, rmse, mae, R2);
end
fprintf('\n');

figure('Name','Task 3 - ML gain prediction accuracy','Color','w', ...
       'Position',[80 80 1100 340]);
for j = 1:3
    subplot(1,3,j);
    scatter(Yte(:,j), Yhat_te(:,j), 26, 'filled', 'MarkerFaceAlpha',0.6); hold on;
    lims = [min([Yte(:,j);Yhat_te(:,j)]) max([Yte(:,j);Yhat_te(:,j)])];
    plot(lims, lims, 'k--', 'LineWidth',1.2);
    grid on; axis square;
    xlabel(['true ' gnames{j}]); ylabel(['predicted ' gnames{j}]);
    title(gnames{j});
end
sgtitle('Task 3: predicted vs optimal PID gains (test set)');

%% ------------------------------------------------------------------------
%  5. SANITY PLOT - do the optimal gains actually vary with mass?
%  If they do not, the whole exercise is pointless. Show that they do.
% -------------------------------------------------------------------------
figure('Name','Task 3 - Optimal gains vs operating condition','Color','w', ...
       'Position',[80 80 1100 340]);
for j = 1:3
    subplot(1,3,j);
    scatter(COND(:,1), Y(:,j), 26, COND(:,4), 'filled'); grid on;
    xlabel('mass ratio m/m_{nom}'); ylabel(['optimal ' gnames{j}]);
    title(gnames{j}); cb = colorbar; cb.Label.String = 'step size [m]';
end
sgtitle('Optimal gains genuinely depend on the operating condition');

save('task3_model.mat','net','predict_gains','modelType','featNames', ...
     'PID_base','iTr','iVa','iTe','X','Y','COND','useNN');
fprintf('Saved -> task3_dataset.mat, task3_model.mat\n');

% ------------------------------------------------------------------------
function v = safe(v, fallback)
%SAFE  Replace NaN/Inf with a finite fallback so features stay usable.
if ~isfinite(v), v = fallback; end
end
