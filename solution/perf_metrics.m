function M = perf_metrics(t, y, ref, u)
%PERF_METRICS  Standard control performance indices.
%
%   M = PERF_METRICS(t, y, ref, u)
%
%   Returns a struct with the numbers you quote to the judges:
%       Tr    10-90% rise time            [s]
%       Ts    2% settling time            [s]
%       OS    percent overshoot           [%]
%       SSE   steady-state error          [m]
%       ITAE  integral of time*|error|
%       IAE   integral of |error|
%       ISE   integral of error^2
%       Umax  peak control effort
%       Ueff  RMS control effort

if nargin < 4, u = []; end

t = t(:); y = y(:);
e = ref - y;

% --- Rise time (10% -> 90% of the final commanded value) -----------------
M.Tr = NaN;
if ref ~= 0
    i10 = find(y >= 0.10*ref, 1, 'first');
    i90 = find(y >= 0.90*ref, 1, 'first');
    if ~isempty(i10) && ~isempty(i90) && i90 >= i10
        M.Tr = t(i90) - t(i10);
    end
end

% --- Settling time: last time the response leaves the +/-2% band ----------
band  = 0.02*abs(ref);
outside = find(abs(e) > band, 1, 'last');
if isempty(outside)
    M.Ts = 0;
elseif outside >= numel(t)
    M.Ts = NaN;                 % never settled within the simulation
else
    M.Ts = t(outside+1);
end

% --- Overshoot -----------------------------------------------------------
if ref > 0
    M.OS = max(0, (max(y) - ref)/ref*100);
else
    M.OS = 0;
end

% --- Errors and integral indices -----------------------------------------
nSS   = max(1, round(0.10*numel(y)));       % average the last 10%
M.SSE = mean(e(end-nSS+1:end));

M.ITAE = trapz(t, t.*abs(e));
M.IAE  = trapz(t,   abs(e));
M.ISE  = trapz(t,   e.^2);
M.RMSE = sqrt(mean(e.^2));

% --- Control effort ------------------------------------------------------
if ~isempty(u)
    u = u(:);
    M.Umax = max(u);
    M.Ueff = sqrt(mean(u.^2));
    M.dU   = trapz(t(1:end-1), abs(diff(u)));   % total control variation
else
    M.Umax = NaN; M.Ueff = NaN; M.dU = NaN;
end
end
