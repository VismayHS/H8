function R = sim_altitude(gains, P, z_ref, Tsim, opt)
%SIM_ALTITUDE  Closed-loop altitude simulation on the NONLINEAR model.
%
%   R = SIM_ALTITUDE(gains, P, z_ref, Tsim, opt)
%
%   gains = [Kp Ki Kd]
%   opt (optional struct):
%       .tilt_phi, .tilt_theta  function handles @(t) giving roll/pitch [rad]
%                               (default: zero - the quadcopter stays level)
%       .dist                   function handle @(t) vertical disturbance
%                               force [N], e.g. a wind gust (default: 0)
%       .z0, .zdot0             initial conditions (default 0)
%       .use_fbl                true (default) = feedback linearisation on
%                               false = plain PID with a constant gravity
%                               feed-forward, so you can show the difference
%       .noise_std              std-dev of altitude measurement noise [m]
%
%   Returns R with fields t, z, zdot, U1, u_z, e.
%
%   The controller runs at P.Ts with derivative-on-measurement (no derivative
%   kick) and clamping anti-windup. The plant is integrated with RK4 at a
%   finer step, which is what a real fixed-step Simulink run would do.

if nargin < 5, opt = struct(); end
if ~isfield(opt,'tilt_phi'),   opt.tilt_phi   = @(t) 0;    end
if ~isfield(opt,'tilt_theta'), opt.tilt_theta = @(t) 0;    end
if ~isfield(opt,'dist'),       opt.dist       = @(t) 0;    end
if ~isfield(opt,'z0'),         opt.z0         = 0;         end
if ~isfield(opt,'zdot0'),      opt.zdot0      = 0;         end
if ~isfield(opt,'use_fbl'),    opt.use_fbl    = true;      end
if ~isfield(opt,'noise_std'),  opt.noise_std  = 0;         end
if ~isfield(opt,'I0'),         opt.I0         = 0;         end
if ~isfield(opt,'t0'),         opt.t0         = 0;         end
if ~isfield(opt,'nsub'),       opt.nsub       = 10;        end
% Mass the CONTROLLER believes it is flying. Defaults to the true mass
% (perfect knowledge). Set it to the NOMINAL mass while P.m holds the TRUE
% mass to model an unknown payload - the controller is then mis-scaled and
% leaves a steady-state error, which is exactly what the ML must detect.
if ~isfield(opt,'m_ctrl'),     opt.m_ctrl     = P.m;       end

Kp = gains(1); Ki = gains(2); Kd = gains(3);
m_c = opt.m_ctrl;            % controller's assumed mass
m_p = P.m;                   % true plant mass

Ts   = P.Ts;                 % controller period
nsub = opt.nsub;             % plant integration substeps per control period
                             % 10 = high fidelity; 4 is plenty for this
                             % system (RK4 at dt=2.5ms vs wn ~2 rad/s) and
                             % roughly halves dataset-generation time
dt   = Ts/nsub;
N    = round(Tsim/Ts);

t    = opt.t0 + (0:N)'*Ts;
z    = zeros(N+1,1);   zdot = zeros(N+1,1);
U1   = zeros(N+1,1);   uz   = zeros(N+1,1);   e = zeros(N+1,1);

z(1) = opt.z0;  zdot(1) = opt.zdot0;
I    = opt.I0;                             % integral accumulator (bumpless
                                           % transfer when handing over gains)

% reference can be a scalar or a function of time
if isa(z_ref,'function_handle'), rfun = z_ref; else, rfun = @(tt) z_ref; end

for kk = 1:N
    tk  = t(kk);
    ref = rfun(tk);

    zmeas = z(kk);
    if opt.noise_std > 0
        zmeas = zmeas + opt.noise_std*randn;
    end

    ek     = ref - zmeas;
    e(kk)  = ek;
    phi    = opt.tilt_phi(tk);
    th     = opt.tilt_theta(tk);
    ctilt  = cos(phi)*cos(th);
    ctilt  = max(ctilt, 0.30);             % guard: never divide by ~0

    % --- PID, derivative taken on the measurement (no derivative kick) ----
    u_unsat = Kp*ek + Ki*I - Kd*zdot(kk);

    % --- Feedback linearisation ------------------------------------------
    if opt.use_fbl
        U1cmd = m_c*(P.g + u_unsat)/ctilt;  % uses the ASSUMED mass
    else
        U1cmd = m_c*P.g + m_c*u_unsat;      % gravity feed-forward only
    end

    U1k = min(max(U1cmd, P.U1_min), P.U1_max);   % actuator saturation

    % --- Clamping anti-windup --------------------------------------------
    % Integrate only when not saturated, or when the error would unwind it.
    saturated = (U1cmd > P.U1_max) || (U1cmd < P.U1_min);
    if ~saturated || (sign(ek) ~= sign(U1cmd - U1k))
        I = I + ek*Ts;
    end

    U1(kk) = U1k;  uz(kk) = u_unsat;

    % --- Integrate the nonlinear vertical dynamics with RK4 --------------
    %     zddot = -g + (U1/m)*cos(phi)*cos(theta) + Fdist/m
    x = [z(kk); zdot(kk)];
    for ii = 1:nsub
        tt = tk + (ii-1)*dt;
        f  = @(tau, xx) [ xx(2);
                          -P.g + (U1k/m_p)*cos(opt.tilt_phi(tau))*cos(opt.tilt_theta(tau)) ...
                          + opt.dist(tau)/m_p ];   % TRUE mass in the plant
        k1 = f(tt,        x);
        k2 = f(tt+dt/2,   x + dt/2*k1);
        k3 = f(tt+dt/2,   x + dt/2*k2);
        k4 = f(tt+dt,     x + dt  *k3);
        x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    z(kk+1) = x(1);  zdot(kk+1) = x(2);
end

U1(end) = U1(end-1);  uz(end) = uz(end-1);
e(end)  = rfun(t(end)) - z(end);

R = struct('t',t,'z',z,'zdot',zdot,'U1',U1,'u_z',uz,'e',e,'I_end',I);
end
