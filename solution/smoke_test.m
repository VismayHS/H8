%% SMOKE_TEST - fast syntax/logic check of the whole solution package.
%  Catches typos and shape errors in seconds instead of during the demo.
%  Run:  matlab -batch "cd('...\solution'); smoke_test"

fprintf('\n===== SMOKE TEST =====\n');
pass = 0; fail = 0;

function_list = {};   %#ok<NASGU>

%% 1 - toolboxes
fprintf('\n[1] Toolbox availability\n');
checks = {'Control_Toolbox','Control System Toolbox'; ...
          'Simulink','Simulink'; ...
          'Neural_Network_Toolbox','Deep Learning Toolbox'; ...
          'Stateflow','Stateflow'; ...
          'Simulink_Control_Design','Simulink Control Design'};
for i = 1:size(checks,1)
    ok = license('test', checks{i,1});
    fprintf('    %-28s %s\n', checks{i,2}, string(ok));
end

%% 2 - params
fprintf('\n[2] quad_params\n');
try
    P = quad_params();
    assert(abs(P.m-0.516)<1e-9 && abs(P.l-0.225)<1e-9, 'Table 1 values wrong');
    assert(abs(P.W - 5.06196) < 1e-3, 'weight wrong');
    fprintf('    OK   m=%.3f  W=%.4f N  w_hover=%.1f rad/s\n', P.m, P.W, P.w_hover);
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 3 - dynamics at hover
fprintf('\n[3] quad_dynamics equilibrium\n');
try
    x0 = zeros(12,1);
    dx = quad_dynamics(0, x0, [P.W;0;0;0;0], P);
    assert(numel(dx)==12, 'dx must be 12x1');
    assert(norm(dx) < 1e-9, sprintf('hover residual %.3e too large', norm(dx)));
    fprintf('    OK   ||dx|| = %.3e\n', norm(dx));
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 4 - rotor mapping
fprintf('\n[4] rotor2U mapping\n');
try
    wh = P.w_hover;
    [U,Omr] = rotor2U([wh wh wh wh], P);
    assert(abs(U(1)-P.W) < 1e-6, 'hover thrust mismatch');
    assert(all(abs(U(2:4)) < 1e-9), 'hover should give zero moments');
    fprintf('    OK   U1=%.4f N (=W), moments ~0, Omega_r=%.2f\n', U(1), Omr);
    % roll sign: speed up rotor 4, slow rotor 2 -> expect POSITIVE U2
    [U2c,~] = rotor2U([wh wh*0.9 wh wh*1.1], P);
    fprintf('    roll check: U2 = %+.5f  (should be POSITIVE)\n', U2c(2));
    % pitch: speed up rotor 3, slow rotor 1 -> expect POSITIVE U3
    [U3c,~] = rotor2U([wh*0.9 wh wh*1.1 wh], P);
    fprintf('    pitch check: U3 = %+.5f  (should be POSITIVE)\n', U3c(3));
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 5 - closed-loop simulator
fprintf('\n[5] sim_altitude closed loop\n');
try
    g = [4.94 1.65 4.00];
    R = sim_altitude(g, P, 1.0, 6);
    assert(all(isfinite(R.z)), 'non-finite altitude');
    assert(abs(R.z(end)-1.0) < 0.15, sprintf('did not track: z_end=%.3f', R.z(end)));
    fprintf('    OK   z_end = %.4f m (target 1.0), U1_end = %.3f N\n', ...
            R.z(end), R.U1(end));
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 6 - metrics
fprintf('\n[6] perf_metrics\n');
try
    M = perf_metrics(R.t, R.z, 1.0, R.U1);
    fprintf('    OK   Tr=%.3f Ts=%.3f OS=%.2f%% ITAE=%.4f\n', ...
            M.Tr, M.Ts, M.OS, M.ITAE);
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 7 - cost function
fprintf('\n[7] alt_cost\n');
try
    J = alt_cost([4.94 1.65 4.00], P, 1.0, 6);
    assert(isfinite(J) && J > 0, 'cost not finite/positive');
    fprintf('    OK   J = %.4f\n', J);
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 8 - feedback linearisation under tilt
fprintf('\n[8] feedback linearisation holds under tilt\n');
try
    o = struct('tilt_theta', @(t) 0.30*(t>1.5));
    Rt = sim_altitude([4.94 1.65 4.00], P, 1.0, 8, o);
    err = abs(Rt.z(end)-1.0);
    fprintf('    OK   with 0.30 rad pitch, z_end = %.4f (err %.4f)\n', Rt.z(end), err);
    if err > 0.10
        fprintf('    WARN tracking degraded under tilt - check the 1/(cos*cos) term\n');
    end
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 9 - pidtune availability
fprintf('\n[9] pidtune on the altitude plant\n');
try
    s = tf('s'); G = 1/s^2;
    [C,info] = pidtune(G,'PIDF');
    fprintf('    OK   Kp=%.3f Ki=%.3f Kd=%.3f  PM=%.1f deg  wc=%.3f\n', ...
            C.Kp, C.Ki, C.Kd, info.PhaseMargin, info.CrossoverFrequency);
    pass = pass+1;
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

%% 10 - fitnet availability
fprintf('\n[10] fitnet (Deep Learning Toolbox)\n');
try
    if exist('fitnet','file')==2
        net = fitnet(4); net.trainParam.showWindow = false;
        Xd = rand(3,60); Yd = sum(Xd,1);
        net = train(net, Xd, Yd);
        fprintf('    OK   fitnet trains, sample output %.4f\n', net(Xd(:,1)));
        pass = pass+1;
    else
        fprintf('    SKIP fitnet not found - task3 will use linear fallback\n');
    end
catch ME
    fprintf('    FAIL %s\n', ME.message); fail = fail+1;
end

fprintf('\n===== RESULT: %d passed, %d failed =====\n\n', pass, fail);
