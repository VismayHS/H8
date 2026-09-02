%% TASK 2b - Benchmark against the reference paper's own published controllers
%
%  SOURCE: Mien, T. & Tu, T. (2024), IJRCS 4(4), 1712-1730 - the paper the
%  organisers cite. We read it and extracted:
%
%    * their measured ultimate gain for the altitude loop, Fig. 5:
%          kth = 124.99,  tau_th = 3.52 s
%    * their tuning formulas, Eq. (24) Ziegler-Nichols and Eq. (25)
%      Tyreus-Luyben
%    * their published altitude-controller gains, Tables 2, 3 and 4
%    * their altitude control law, Eq. (27) - a PLAIN PID with no gravity
%      feed-forward and no tilt compensation
%
%  VERIFICATION: applying their formulas to their kth and tau_th reproduces
%  their published Table 2/3 altitude gains to four decimal places. That
%  confirms our reading of the paper is correct before we compare anything.
%
%  FAIR-COMPARISON NOTE
%  Their Eq. (27) has no feedback linearisation - gravity is absorbed by the
%  integral term. Running their gains on OUR feedback-linearised loop would
%  not be the controller they designed. So each design is simulated in the
%  configuration its author intended:
%      their three designs -> plain PID          (use_fbl = false)
%      our Method D        -> feedback-linearised (use_fbl = true)
%  Both face the identical plant, actuator limits and disturbances.

clear; clc; close all;
P = quad_params();
zref = 2.0;      % the paper simulates zd = 2.0 m; we match it
Tsim = 20;

fprintf('=== TASK 2b: benchmark vs Mien & Tu (2024) ===\n\n');

%% ---------------------------------------------------------------------
%  1 - reproduce their published gains from their published kth, tau_th
% ----------------------------------------------------------------------
Ku = 124.99;   Tu = 3.52;          % their Fig. 5

kp_zn = 3*Ku/5;   ZN = [kp_zn, 2*kp_zn/Tu,        kp_zn*Tu/8   ];
kp_tl = 5*Ku/11;  TL = [kp_tl, 5*kp_tl/(11*Tu),  10*kp_tl*Tu/63];
PT = [57.005, 0.001, 23.1020];     % their Table 4, MATLAB PID Tuner

pubZN = [74.9940, 42.6102, 32.9974];   % their Table 3, altitude column
pubTL = [56.8136,  7.3365, 31.7435];   % their Table 2, altitude column

fprintf('Reproducing their published gains from kth=%.2f, tau_th=%.2f:\n', Ku, Tu);
fprintf('  ZN  computed [%9.4f %9.4f %9.4f]\n', ZN);
fprintf('      published[%9.4f %9.4f %9.4f]  max err %.2e\n', pubZN, max(abs(ZN-pubZN)));
fprintf('  TL  computed [%9.4f %9.4f %9.4f]\n', TL);
fprintf('      published[%9.4f %9.4f %9.4f]  max err %.2e\n\n', pubTL, max(abs(TL-pubTL)));

%% ---------------------------------------------------------------------
%  2 - simulate each design in its intended configuration
% ----------------------------------------------------------------------
if isfile('task2_pid.mat')
    L = load('task2_pid.mat'); OURS = L.PID_alt;
else
    OURS = [30.1847 0 10.3994];
end

designs = { ...
  'Ziegler-Nichols (Mien & Tu)',  ZN,   false;
  'Tyreus-Luyben (Mien & Tu)',    TL,   false;
  'MATLAB PID Tuner (Mien & Tu)', PT,   false;
  'OURS: ITAE + feedback lin.',   OURS, true };

fprintf('%-30s %9s %9s %9s %7s %7s %7s %9s\n', ...
        'Design','Kp','Ki','Kd','Tr[s]','Ts[s]','OS[%]','ITAE');
fprintf('%s\n', repmat('-',1,100));

res = struct([]);
for i = 1:size(designs,1)
    g   = designs{i,2};
    opt = struct('use_fbl', designs{i,3});
    R = sim_altitude(g, P, zref, Tsim, opt);
    M = perf_metrics(R.t, R.z, zref, R.U1);
    res(i).name = designs{i,1}; res(i).g = g; res(i).M = M; res(i).R = R;
    fprintf('%-30s %9.4f %9.4f %9.4f %7.3f %7.3f %7.2f %9.4f\n', ...
            designs{i,1}, g, M.Tr, M.Ts, M.OS, M.ITAE);
end
fprintf('%s\n\n', repmat('-',1,100));

%% steady-state error - the metric the paper reports ("less than 1%")
fprintf('Steady-state error (the paper reports < 1%% for Tyreus-Luyben):\n');
for i = 1:numel(res)
    fprintf('  %-30s %+8.5f m  =  %6.3f%% of setpoint\n', ...
            res(i).name, res(i).M.SSE, abs(res(i).M.SSE)/zref*100);
end

[~,b] = min(arrayfun(@(r) r.M.ITAE, res));
fprintf('\nBest by ITAE: %s\n', res(b).name);

%% ---------------------------------------------------------------------
%  3 - the differentiator: response to tilt
%  Their Eq. (27) has no 1/(cos phi cos theta) term, so tilting costs them
%  altitude. Ours cancels it. Same 0.30 rad pitch applied to both.
% ----------------------------------------------------------------------
fprintf('\nAltitude loss under a 0.30 rad (17.2 deg) pitch step at t = 10 s:\n');
sag = zeros(1,numel(res));
for i = 1:numel(res)
    opt = struct('use_fbl', designs{i,3}, 'tilt_theta', @(t) 0.30*(t>10));
    R = sim_altitude(res(i).g, P, zref, Tsim, opt);
    sag(i) = max(abs(R.z(R.t>10.5) - zref));
    res(i).Rtilt = R;
    fprintf('  %-30s sag = %.6f m\n', res(i).name, sag(i));
end

%% ---------------------------------------------------------------------
%  4 - figures
% ----------------------------------------------------------------------
set(0,'DefaultFigureColor','w');
f = figure('Color','w','Position',[60 60 980 640]);
subplot(2,1,1); hold on; grid on;
for i = 1:numel(res)
    plot(res(i).R.t, res(i).R.z, 'LineWidth',1.6, ...
        'DisplayName', sprintf('%s (ITAE %.3f)', res(i).name, res(i).M.ITAE));
end
yline(zref,'k--','DisplayName','reference 2.0 m');
ylabel('Altitude z [m]'); legend('Location','southeast','FontSize',8);
title('Altitude step to 2.0 m: reference-paper designs vs ours');

subplot(2,1,2); hold on; grid on;
for i = 1:numel(res)
    plot(res(i).Rtilt.t, res(i).Rtilt.z, 'LineWidth',1.6, 'DisplayName', res(i).name);
end
yline(zref,'k--','HandleVisibility','off');
xline(10,'m-.','HandleVisibility','off');
text(10.1, zref*0.6, ' 0.30 rad pitch', 'Color','m','FontSize',8);
ylabel('Altitude z [m]'); xlabel('Time [s]');
legend('Location','southwest','FontSize',8);
title('Same designs under a 17.2 deg pitch - only ours compensates the lift loss');

if ~isfolder('figures'), mkdir('figures'); end
exportgraphics(f,'figures/07_reference_paper_benchmark.png','Resolution',150);
fprintf('\nSaved -> figures/07_reference_paper_benchmark.png\n');

save('task2b_benchmark.mat','Ku','Tu','ZN','TL','PT','OURS','res','sag');
fprintf('Saved -> task2b_benchmark.mat\n');
