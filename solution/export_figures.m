%% EXPORT_FIGURES - regenerate every figure as a clean, presentation-ready PNG.
%
%  MATLAB R2026a defaults to a dark theme, which prints badly on slides and
%  handouts. This forces a white background, black axes and readable fonts,
%  then writes 150 dpi PNGs into a figures/ subfolder.
%
%  Run AFTER task1..task4 have completed.

clear; close all;
outdir = 'figures';
if ~isfolder(outdir), mkdir(outdir); end

% force light theme for everything we create here
set(0,'DefaultFigureColor','w');
set(0,'DefaultAxesColor','w');
set(0,'DefaultAxesXColor','k');
set(0,'DefaultAxesYColor','k');
set(0,'DefaultAxesGridColor',[0.15 0.15 0.15]);
set(0,'DefaultTextColor','k');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',1.7);

P = quad_params();
s = tf('s');
n = 0;

    function saveFig(f, name, outdir)
        set(f,'Color','w','InvertHardcopy','off');
        ax = findall(f,'Type','axes');
        for a = ax(:)'
            set(a,'Color','w','XColor','k','YColor','k','GridColor',[.15 .15 .15]);
        end
        lg = findall(f,'Type','legend');
        for L = lg(:)', set(L,'TextColor','k','Color','w'); end
        exportgraphics(f, fullfile(outdir,[name '.png']), 'Resolution', 150);
        fprintf('   saved figures/%s.png\n', name);
    end

fprintf('Exporting figures...\n');

%% FIG 1 - open-loop divergence (Task 1)
x0 = zeros(12,1);
od = @(t,x) quad_dynamics(t,x,[P.W*(1+0.02*(t>1));0;0;0;0],P);
[t1,x1] = ode45(od,[0 3],x0);
f = figure('Position',[100 100 820 420]);
plot(t1,x1(:,5)); grid on; hold on; xline(1,'r--');
xlabel('Time [s]'); ylabel('Altitude Z [m]');
title('Task 1: open loop - a 2% thrust error diverges as t^2');
legend('altitude','thrust step','Location','northwest');
saveFig(f,'01_task1_openloop_divergence',outdir); n=n+1;

%% FIG 2 - the four decoupled channels (Task 1)
f = figure('Position',[100 100 900 460]);
G = {1/(P.m*s^2), P.l/(P.Ixx*s^2), P.l/(P.Iyy*s^2), 1/(P.Izz*s^2)};
nm = {'Altitude (1/ms^2)','Roll (l/I_{xx}s^2)','Pitch (l/I_{yy}s^2)','Yaw (1/I_{zz}s^2)'};
for i=1:4
    subplot(2,2,i); pzmap(G{i}); grid on; title(nm{i});
end
sgtitle('Task 1: all four channels are double integrators (poles at origin)');
saveFig(f,'02_task1_four_channels_pzmap',outdir); n=n+1;

%% FIG 3 - Task 2 tuning comparison
if isfile('task2_pid.mat')
    L = load('task2_pid.mat');
    f = figure('Position',[100 100 950 620]);
    for i=1:numel(L.results)
        R = sim_altitude(L.results(i).gains, P, 1.0, 8);
        subplot(2,1,1); hold on; grid on;
        plot(R.t,R.z,'DisplayName',sprintf('%s (ITAE %.3f)', ...
             L.results(i).name, L.results(i).M.ITAE));
        subplot(2,1,2); hold on; grid on;
        plot(R.t,R.U1,'DisplayName',L.results(i).name);
    end
    subplot(2,1,1); yline(1,'k--','DisplayName','reference');
    ylabel('Altitude Z [m]'); legend('Location','southeast','FontSize',9);
    title('Task 2: four tuning methods on the nonlinear model');
    subplot(2,1,2); yline(P.U1_max,'r--','DisplayName','saturation');
    yline(P.W,'k:','DisplayName','hover weight');
    ylabel('Thrust U1 [N]'); xlabel('Time [s]');
    legend('Location','northeast','FontSize',9);
    saveFig(f,'03_task2_tuning_comparison',outdir); n=n+1;
end

%% FIG 4 - feedback linearisation holds under tilt
f = figure('Position',[100 100 900 460]);
g = [32.8366 0 10.4046];
if isfile('task2_pid.mat'), L=load('task2_pid.mat'); g=L.PID_alt; end
subplot(2,1,1); hold on; grid on;
for tilt = [0 0.15 0.30 0.45]
    o = struct('tilt_theta', @(t) tilt*(t>3));
    R = sim_altitude(g,P,1.0,8,o);
    plot(R.t,R.z,'DisplayName',sprintf('pitch %.2f rad (%.0f deg)',tilt,rad2deg(tilt)));
end
yline(1,'k--','HandleVisibility','off'); xline(3,'m-.','HandleVisibility','off');
ylabel('Altitude Z [m]'); legend('Location','southeast','FontSize',9);
title('Feedback linearisation: altitude is unaffected by tilt');
subplot(2,1,2); hold on; grid on;
for tilt = [0 0.15 0.30 0.45]
    o = struct('tilt_theta', @(t) tilt*(t>3));
    R = sim_altitude(g,P,1.0,8,o);
    plot(R.t,(R.z-1)*1000,'DisplayName',sprintf('%.0f deg',rad2deg(tilt)));
end
ylabel('Error [mm]'); xlabel('Time [s]'); legend('Location','northeast','FontSize',9);
title('Same data, error in millimetres');
saveFig(f,'04_feedback_linearisation_tilt',outdir); n=n+1;

%% FIG 5 - fixed gains degrade with payload (bridge to Task 3)
f = figure('Position',[100 100 900 460]);
hold on; grid on;
for mr = [1.0 1.4 1.8 2.2]
    P2 = P; P2.m = P.m*mr; P2.W = P2.m*P2.g;
    o  = struct('m_ctrl', P.m);        % controller NOT told the payload
    R  = sim_altitude(g,P2,1.0,8,o);
    M  = perf_metrics(R.t,R.z,1.0,R.U1);
    plot(R.t,R.z,'DisplayName',sprintf('m x%.1f  (SSE %.3f m)',mr,abs(M.SSE)));
end
yline(1,'k--','HandleVisibility','off');
xlabel('Time [s]'); ylabel('Altitude Z [m]');
legend('Location','southeast','FontSize',10);
title('Fixed gains + unknown payload -> steady-state error. This motivates Task 3.');
saveFig(f,'05_fixed_gains_degrade',outdir); n=n+1;

%% FIG 6/7 - Task 3 and 4 outputs if available
if isfile('task3_dataset.mat')
    L3 = load('task3_dataset.mat');   % dataset holds COND; model does not
    gn = {'Kp','Ki','Kd'};
    f = figure('Position',[100 100 1050 340]);
    for j=1:3
        subplot(1,3,j);
        scatter(L3.COND(:,1), L3.Y(:,j), 28, 'filled'); grid on;
        xlabel('mass ratio'); ylabel(['optimal ' gn{j}]); title(gn{j});
    end
    sgtitle('Task 3: optimal gains vs payload - note Ki rises with mass');
    saveFig(f,'06_task3_gains_vs_mass',outdir); n=n+1;
end

fprintf('\nExported %d figures to %s\\\n', n, outdir);
